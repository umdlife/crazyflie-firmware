/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 * Copyright (C) 2019 Unmanned Systems Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * comm_mavlink.c - Mavlink helper functions
 */

#include <stdbool.h>
#include "comm_mavlink.h"
#include "uart2.h" // uart2SendData, uart2Init
#include "physicalConstants.h" // GRAVITY_MAGNITUDE
#include "cf_math.h" // DEG_TO_RAD

/*  
 * MAVLink messages definition:
 * We are using MAVLink 1 but what if user compiled v2?
 * 
 * MAVLink1 Header: 6 bytes
 * MAVLink2 Header: 10 bytes
 * 
 * Payload: MAVLINK_MSG_ID_TDOA_MEASUREMENT_LEN 
 *          MAVLINK_MSG_ID_GYRO_ACC_TEMP_LEN
 * 
 * MAVLink2 Signature: 13 bytes
 * MAVLink CRC: 2 bytes
 * 
 * Therefore: Mavlink1 message = payload + 8;
 * Therefore: Mavlink2 message = payload + 25;
 */

#define MAVLINK_VERSION_1 strcmp(MAVLINK_WIRE_PROTOCOL_VERSION, "1.0")
#define MAVLINK_VERSION_2 strcmp(MAVLINK_WIRE_PROTOCOL_VERSION, "2.0")
#if $MAVLINK_VERSION_1 == 0
#define MAVLINK_PROTOCOL_OVERHEAD 8
#else
// Force overhead of 25 bytes if the protocol is unknown
#define MAVLINK_PROTOCOL_OVERHEAD 25
#endif

// Length of the buffers +1 to have a \0 at the end.
#define MAVLINK_TDOA_FULL_MSG_LEN MAVLINK_MSG_ID_TDOA_MEASUREMENT_LEN + MAVLINK_PROTOCOL_OVERHEAD + 1
#define MAVLINK_IMU_FULL_MSG_LEN MAVLINK_MSG_ID_GYRO_ACC_TEMP_LEN + MAVLINK_PROTOCOL_OVERHEAD + 1
#define MAVLINK_POSE_MSG_LEN MAVLINK_MSG_ID_POSE_LEN + MAVLINK_PROTOCOL_OVERHEAD + 1
static uint8_t buff_tdoa_msg[MAVLINK_TDOA_FULL_MSG_LEN];
static uint8_t buff_imu_msg[MAVLINK_IMU_FULL_MSG_LEN];
static uint8_t buff_pose_msg[MAVLINK_POSE_MSG_LEN];

#ifndef MAVLINK_MSG_GYRO_ACC_RATE_LIMITER
#define MAVLINK_MSG_GYRO_ACC_RATE_LIMITER 10
#endif
static unsigned int gyro_acc_rate_limiter = 0;

#ifndef MAVLINK_MSG_POSE_RATE_LIMITER
#define MAVLINK_MSG_POSE_RATE_LIMITER 10
#endif
static unsigned int pose_rate_limiter = 0;


static bool isInit;
void commMavlinkInit(void)
{
  if (isInit)
    return;
  gyro_acc_rate_limiter = 0;
  pose_rate_limiter = 0;

  uart2Init(921600);

  // Zeroing buffers
  for(unsigned int i = 0; i < MAVLINK_TDOA_FULL_MSG_LEN; i++) {
    buff_tdoa_msg[i] = 0;
  }

  for(unsigned int i = 0; i < MAVLINK_IMU_FULL_MSG_LEN; i++) {
    buff_imu_msg[i] = 0;
  }

  for(unsigned int i = 0; i < MAVLINK_POSE_MSG_LEN; i++) {
    buff_pose_msg[i] = 0;
  }

  isInit = true;
}

bool commMavlinkTest(void)
{
  bool pass=isInit;
  return pass;
}

int commMavlinkSendTDoA(const tdoaMeasurement_t * tdoa)
{
  if (!isInit)
    return -1;
  static mavlink_message_t mav_msg;
  static mavlink_tdoa_measurement_t msg;

  msg.dist_diff = tdoa->distanceDiff;
  msg.stddev = tdoa->stdDev;
  msg.anchor_ax = tdoa->anchorPosition[0].x;
  msg.anchor_ay = tdoa->anchorPosition[0].y;
  msg.anchor_az = tdoa->anchorPosition[0].z;
  msg.anchor_bx = tdoa->anchorPosition[1].x;
  msg.anchor_by = tdoa->anchorPosition[1].y;
  msg.anchor_bz = tdoa->anchorPosition[1].z;

  mavlink_msg_tdoa_measurement_encode(
    10, 200, &mav_msg, &msg
  );

  unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff_tdoa_msg, &mav_msg);
  uart2SendData(len , buff_tdoa_msg);
  return len;
}

int commMavlinkSendImuRateLimited(sensorData_t * sensors)
{
  if (!isInit)
    return -1;
  if(gyro_acc_rate_limiter == MAVLINK_MSG_GYRO_ACC_RATE_LIMITER) {
    static mavlink_message_t mav_msg;
    static mavlink_gyro_acc_temp_t msg;
    msg.xacc = GRAVITY_MAGNITUDE * sensors->acc.x;
    msg.yacc = GRAVITY_MAGNITUDE * sensors->acc.y;
    msg.zacc = GRAVITY_MAGNITUDE * sensors->acc.z;
    msg.xgyro = sensors->gyro.x * DEG_TO_RAD;
    msg.ygyro = sensors->gyro.y * DEG_TO_RAD;
    msg.zgyro = sensors->gyro.z * DEG_TO_RAD;

    mavlink_msg_gyro_acc_temp_encode(
      10, 200, &mav_msg, &msg
    );

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff_imu_msg, &mav_msg);
    uart2SendData(len , buff_imu_msg);
    gyro_acc_rate_limiter = 0;
    return len;
  }
  gyro_acc_rate_limiter += 1;
  return 0;
}

int commMavlinkSendPoseRateLimited(state_t * state)
{
  if (!isInit)
    return -1;
  if(pose_rate_limiter == MAVLINK_MSG_POSE_RATE_LIMITER) {
    static mavlink_message_t mav_msg;
    static mavlink_pose_t msg;

    msg.x = state->position.x;
    msg.y = state->position.y;
    msg.z = state->position.z;
    msg.vx = state->velocity.x;
    msg.vy = state->velocity.y;
    msg.vz = state->velocity.z;
    msg.qx = state->attitudeQuaternion.x;
    msg.qy = state->attitudeQuaternion.y;
    msg.qz = state->attitudeQuaternion.z;
    msg.qw = state->attitudeQuaternion.w;

    mavlink_msg_pose_encode(
      10, 200, &mav_msg, &msg
    );

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff_pose_msg, &mav_msg);
    uart2SendData(len , buff_pose_msg);
    pose_rate_limiter = 0;
    return len;
  }
  pose_rate_limiter += 1;
  return 0;
}
