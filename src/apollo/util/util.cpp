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