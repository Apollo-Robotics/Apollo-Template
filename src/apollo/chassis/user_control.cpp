#include "apollo/chassis/drivetrain.hpp"
#include "apollo/util/util.hpp"
#include "pros/misc.h"
namespace apollo {
void Drivetrain::tank_control() {
  int left_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int right_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  if (abs(left_stick) > joytstick_deadzone ||
      abs(right_stick) > joytstick_deadzone) {
    for (auto i : left_motors) {
      i.move_voltage(left_stick * (12000.0 / 127.0));
    }
    for (auto i : right_motors) {
      i.move_voltage(right_stick * (12000.0 / 127.0));
    }
  } else {
    for (auto i : left_motors) {
      i.move_voltage(0);
    }
    for (auto i : right_motors) {
      i.move_voltage(0);
    }
  }
}
void Drivetrain::arcade_control(util::drivetrain_control_type control_type,
                                bool flip) {
  int forward_stick, turn_stick;
  if (!flip) {
    if (control_type == util::SPLIT_JOYSTICK) {
      forward_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      turn_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    } else if (control_type == util::SINGLE_JOYSTICK) {
      forward_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      turn_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    }
  } else if (flip) {
    if (control_type == util::SPLIT_JOYSTICK) {
      forward_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
      turn_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    } else if (control_type == util::SINGLE_JOYSTICK) {
      forward_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      turn_stick = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    }
  }
  if (abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) >
          joytstick_deadzone ||
      abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) >
          joytstick_deadzone) {
    for (auto i : left_motors) {
      i.move_voltage(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) *
                     (12000.0 / 127.0));
    }
    for (auto i : right_motors) {
      i.move_voltage(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) *
                     (12000.0 / 127.0));
    }
  } else {
    for (auto i : left_motors) {
      i.move_voltage(0);
    }
    for (auto i : right_motors) {
      i.move_voltage(0);
    }
  }
}
} // namespace apollo