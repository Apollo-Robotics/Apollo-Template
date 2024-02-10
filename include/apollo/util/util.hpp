/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#include "pros/misc.hpp"
extern pros::Controller master;

namespace apollo {
namespace util {
enum chassis_tracker_type {
  DRIVE_MOTOR_ENCODER,
  DRIVE_ADI_ENCODER,
  DRIVE_ROTATION_SENSOR
};
enum chassis_control_type {
  NORMAL_SINGLE_JOYSTICK,
  NORMAL_SPLIT_JOYSTICK,
  FLIPPED_SINGLE_JOYSTICK,
  FLIPPED_SPLIT_JOYSTICK
};
bool is_reversed(double input);
} // namespace util
} // namespace apollo