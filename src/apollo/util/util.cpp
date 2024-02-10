/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/util/util.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
namespace apollo {
namespace util {
int convert_gear_ratio(pros::v5::MotorGears input) {
  if (input == pros::v5::MotorGears::ratio_18_to_1 ||
      input == pros::v5::MotorGears::green ||
      input == pros::v5::MotorGears::rpm_200) {
    return 200;
  } else if (input == pros::v5::MotorGears::ratio_36_to_1 ||
             input == pros::v5::MotorGears::blue ||
             input == pros::v5::MotorGears::rpm_600) {
    return 600;
  } else if (input == pros::v5::MotorGears::ratio_6_to_1 ||
             input == pros::v5::MotorGears::red ||
             input == pros::v5::MotorGears::rpm_100) {
    return 100;
  } else {
    return INT32_MAX;
  }
}
} // namespace util
} // namespace apollo