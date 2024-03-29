/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "chassisModel.hpp"
namespace apollo {
  void ChassisModel::set_brake_mode(pros::motor_brake_mode_e_t brake_mode) {
    current_brake_mode = brake_mode;
  }
  pros::motor_brake_mode_e_t ChassisModel::get_brake_mode() {
    return current_brake_mode;
  }
  void ChassisModel::set_encoder_units(
      pros::motor_encoder_units_e_t encoder_units) {
    current_encoder_units = encoder_units;
  }
  pros::motor_encoder_units_e_t ChassisModel::get_encoder_units() {
    return current_encoder_units;
  }
  void ChassisModel::set_joystick_deadband(int input) {
    joystick_deadband = input;
  }
  int ChassisModel::get_joystick_deadband() { return joystick_deadband; }

  int get_scaled_voltage(pros::controller_analog_e_t input) {
    return (input * 12000) / 127;
  }

  void ChassisModel::set_tank_joysticks(
      pros::controller_analog_e_t left_input,
      pros::controller_analog_e_t right_input) {
    left_tank_joystick = left_input;
    right_tank_joystick = right_input;
  }
  void ChassisModel::set_arcade_joysticks(
      pros::controller_analog_e_t forward_input,
      pros::controller_analog_e_t turn_input) {
    forward_arcade_joystick = forward_input;
    turn_arcade_joystick = turn_input;
  }
  void ChassisModel::set_strafe_joysticks(
      pros::controller_analog_e_t strafe_input) {
    strafe_arcade_joystick = strafe_input;
  }
  void ChassisModel::set_rotate_joysticks(
      pros::controller_analog_e_t rotate_input) {
    rotate_arcade_joystick = rotate_input;
  }

}  // namespace apollo