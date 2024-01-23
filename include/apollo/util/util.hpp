/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
#pragma once

namespace apollo {
namespace util {
enum drivetrain_type {
  TANK_DRIVE = 1,
  X_DRIVE = 2,
  MECCANUM_DRIVE = 3,
  H_DRIVE = 4
};
enum drivetrain_control_type { SINGLE_JOYSTICK = 1, SPLIT_JOYSTICK = 2 };
enum drivetrain_swing_type { LEFT_SWING = 1, RIGHT_SWING = 2 };
} // namespace util
} // namespace apollo