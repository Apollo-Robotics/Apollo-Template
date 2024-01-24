#include "main.h"
Drivetrain drivetrain(util::TANK_DRIVE, {1, 2}, {3, 4}, 5, 4, 1, 200);
void initialize() { pros::lcd::initialize(); }
void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() {
  while (true) {
    drivetrain.arcade_control(util::SPLIT_JOYSTICK);
    pros::delay(20);
  }
}
