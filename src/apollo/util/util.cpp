/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/util/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
namespace apollo {
namespace util {
bool is_reversed(double input) {
  if (input < 0)
    return true;
  return false;
}
} // namespace util
} // namespace apollo