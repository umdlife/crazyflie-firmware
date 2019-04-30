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
 * comm_mavlink.h - Mavlink helper functions
 */

#ifndef __COMM_MAVLINK_H__
#define __COMM_MAVLINK_H__

#include "stabilizer_types.h"
#include "mavlink/roadrunner/mavlink.h"

void commMavlinkInit(void);
bool commMavlinkTest(void);

int commMavlinkSendTDoA(const tdoaMeasurement_t * tdoa);
int commMavlinkSendImuRateLimited(sensorData_t * sensors);
int commMavlinkSendPoseRateLimited(state_t * state);
int commMavlinkSendQuaternionRateLimited(state_t * state);

#endif //__COMM_MAVLINK_H__
