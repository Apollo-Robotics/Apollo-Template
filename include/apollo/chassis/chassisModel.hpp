/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "apollo/util/util.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <vector>

namespace apollo {
class Chassis {
public:
  /*
    // Drivetrain
    std::vector<pros::Motor> left_motors;
    std::vector<pros::Motor> right_motors;
    std::vector<pros::Motor> front_left_motors;
    std::vector<pros::Motor> front_right_motors;
    std::vector<pros::Motor> back_left_motors;
    std::vector<pros::Motor> back_right_motors;
    std::vector<pros::Motor> center_horizontal_motors;
    pros::Imu inertial_sensor;
    pros::ADIEncoder left_adi_encoder_tracker;
    pros::ADIEncoder right_adi_encoder_tracker;
    pros::ADIEncoder center_adi_encoder_tracker;
    pros::Rotation left_rotation_tracker;
    pros::Rotation right_rotation_tracker;
    pros::Rotation center_rotation_tracker;
  */
  // Control
  virtual void tank_control();
  virtual void arcade_control();
  // Drive Motors
  virtual void get_scaled_joystick_output(pros::controller_analog_e_t input);
  // Drive Params
  virtual void set_brake_mode(pros::motor_brake_mode_e_t);
  virtual pros::motor_brake_mode_e_t get_brake_mode();
  virtual void set_encoder_units(pros::motor_encoder_units_e_t input);
  virtual pros::motor_encoder_units_e_t get_encoder_units();
  virtual void set_max_velocity(int input);
  virtual int get_max_velocity();
  virtual void set_max_voltage(double input);
  virtual double get_max_voltage();

protected:
  double max_drive_voltage = 11.0000;
  int max_drive_velocity = 127;
};
} // namespace apollo
