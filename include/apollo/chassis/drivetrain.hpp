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
namespace chassis {
class Drivetrain {
public:
  int joystick_threshold;
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
  pros::Rotation left_rotation_racker;
  pros::Rotation right_rotatiom_tracker;
  pros::Rotation center_rotation_tracker;
  // tank with no encoders
  Drivetrain(util::drivetrain_type, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge);
  // tank with adi
  Drivetrain(util::drivetrain_type, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with adi center
  Drivetrain(util::drivetrain_type, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with rotation
  Drivetrain(util::drivetrain_type, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge, int left_rotation_ports,
             int right_rotation_ports, double tracker_wheel_diameter,
             double tracker_gear_ratio);
  // tank with rotation center
  Drivetrain(util::drivetrain_type, std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge, int left_rotation_ports,
             int right_rotation_ports, int center_rotation_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);

  // meccanum, x
  Drivetrain(util::drivetrain_type, std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_gear_catridge);
  // meccanum with adi
  // meccanum with adi center
  // meccanum with rotation

  // x
  // x with adi
  // x with adi center
  // x with rotation
  // x with rotation center
  // x with rotation center

  // asterisk
  // asterisk with adi
  // asterisk with adi center
  // asterisk with rotation
  // asterisk with rotation center
  // asterisk with rotation center

  // h
  // h with adi
  // h with adi center
  // h with rotaion
  // h with rotation center
};
} // namespace chassis
} // namespace apollo
