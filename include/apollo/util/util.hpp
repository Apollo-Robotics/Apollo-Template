/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once
#include "pros/misc.hpp"
#include "pros/motors.hpp"
extern pros::Controller master;

namespace apollo {
  namespace util {
    enum chassis_tracker_type {
      DRIVE_MOTOR_ENCODER,
      DRIVE_ADI_ENCODER,
      DRIVE_ROTATION_SENSOR
    };
    enum default_control_type {
      NORMAL_TANK_JOYSTICK,
      NORMAL_SINGLE_ARCADE_JOYSTICK,
      NORMAL_SPLIT_ARCADE_JOYSTICK,
      FLIPPED_SINGLE_ARCADE_JOYSTICK,
      FLIPPED_SPLIT_ARCADE_JOYSTICK,
      NORMAL_STRAFE_JOYSTICK

    };
    bool is_reversed(double input);
    extern int convert_gear_ratio(pros::v5::MotorGears input);
  }  // namespace util
}  // namespace apollo