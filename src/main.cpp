#include "main.h"
Tank drivetrain(
  {1, 2},
  {3, 4},
  5,
  4,
  1,
  200
);
void initialize() { pros::lcd::initialize(); }
void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() {
  while (true) {
    pros::delay(20);
  }
}
