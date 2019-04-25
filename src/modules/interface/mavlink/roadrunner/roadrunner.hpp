/** @file
 *	@brief MAVLink comm protocol generated from roadrunner.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace roadrunner {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 4> MESSAGE_ENTRIES {{ {1, 107, 28, 0, 0, 0}, {2, 56, 40, 0, 0, 0}, {3, 182, 32, 0, 0, 0}, {4, 176, 32, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS




} // namespace roadrunner
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_gyro_acc_baro.hpp"
#include "./mavlink_msg_position.hpp"
#include "./mavlink_msg_quaternion.hpp"
#include "./mavlink_msg_tdoa_measurement.hpp"

// base include

