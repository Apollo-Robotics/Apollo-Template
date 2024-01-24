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
enum drivetrain_type {
  TANK_DRIVE = 1,
  X_DRIVE = 2,
  MECCANUM_DRIVE = 3,
  H_DRIVE = 4
};
enum drivetrain_tracker_type {
  DRIVE_MOTOR_ENCODER = 1,
  DRIVE_ADI_ENCODER = 2,
  DRIVE_ROTATION_SENSOR = 3
};
enum drivetrain_control_type { SINGLE_JOYSTICK = 1, SPLIT_JOYSTICK = 2 };
bool is_reversed(double input);
} // namespace util
} // namespace apollo