/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * default_packets.h: Packet definition to be sent through UART.
 */

#include "uart2.h"
#include "stabilizer_types.h"

#define DEFAULT_PACKET_ID_STATUS 0
#define DEFAULT_PACKET_ID_POSE 1
#define DEFAULT_PACKET_ID_IMU_ALL_RAW 8
#define DEFAULT_PACKET_ID_TDOA3_RANGE 16

#define DEFAULT_PACKET_FILL(data) data->len = sizeof(data);

typedef struct vector3_t {
  float x;
  float y;
  float z;
} VECTOR3;

typedef struct vector4_t {
  float x;
  float y;
  float z;
  float w;
} VECTOR4;

typedef struct posePacket_t {
    uint8_t len;
    uint8_t id;
    uint32_t timestamp;
    VECTOR3 position;
    VECTOR4 orientation;
} POSEPACKET;

typedef struct imuPacket_t {
    uint8_t len;
    uint8_t id;
    uint32_t timestamp;
    VECTOR3 acc;
    VECTOR3 gyro;
    float baro;
} IMURAWPACKET;

typedef struct tdoa3Packet_t {
    uint8_t len;
    uint8_t id;
    tdoaMeasurement_t data;
} TDOA3PACKET;



