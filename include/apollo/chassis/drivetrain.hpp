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
class Drivetrain {
public:
  /////
  // Drivetrain
  /////
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
  // tank with no encoders
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge);
  // tank with adi
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with adi center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with adi expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with adi center expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // tank with rotation
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, double tracker_wheel_diameter,
             double tracker_gear_ratio);
  // tank with rotation center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, int center_rotation_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);

  // meccanum, x
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge);
  // meccanum,x with adi
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // meccanum,x with adi center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // meccanum,x with adi expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // meccanum,x with adi center expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // meccanum,x with rotation
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, double tracker_wheel_diameter,
             double tracker_gear_ratio);
  // meccanum,x with rotation center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, int center_rotation_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // asterisk
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge);
  // asterisk with adi
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // asterisk with adi center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // asterisk with adi expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // asterisk with adi center expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // asterisk with rotation
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, double tracker_wheel_diameter,
             double tracker_gear_ratio);
  // asterisk with rotation center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> front_left_motor_ports,
             std::vector<int> left_motor_ports,
             std::vector<int> back_left_motor_ports,
             std::vector<int> front_right_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> back_right_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, int center_rotation_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);

  // h
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge);
  // h with adi
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // h with adi center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // h with adi expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // h with adi center expander
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge,
             std::vector<int> left_adi_encoder_ports,
             std::vector<int> right_adi_encoder_ports,
             std::vector<int> center_adi_encoder_ports, int expander_smart_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);
  // h with rotation
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, double tracker_wheel_diameter,
             double tracker_gear_ratio);
  // h with rotation center
  Drivetrain(util::drivetrain_type drivetrain_type,
             std::vector<int> left_motor_ports,
             std::vector<int> right_motor_ports,
             std::vector<int> center_motor_ports, int inertial_sensor_port,
             double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
             double drivetrain_motor_catridge, int left_rotation_port,
             int right_rotation_port, int center_rotation_port,
             double tracker_wheel_diameter, double tracker_gear_ratio);

  /////
  // User Control
  /////
  int joytstick_deadzone;
  void tank_control();
  void arcade_control(util::drivetrain_control_type control_type,
                      bool flip = false);
  void set_active_brake(int input);
  void set_joystick_deadzone(int input);
  void standard_curve_function(int input);
  /////
  // PTO
  /////
  /////
  // Telemetry
  /////
  int get_left_sensor_value();
  int get_left_sensor_velocity();
  double get_left_motor_voltage();
  double get_left_motor_current();
  int get_right_sensor_value();
  int get_right_sensor_velocity();
  double get_right_motor_voltage();
  double get_right_motor_current();
  double get_inertial_sensor_value();
  void reset_drive_sensors();
  void reset_inertial_sensor();
  void reset_left_drive_sensor();
  void reset_right_drive_sensor();
  /////
  // Autonomous
  /////
  void set_max_speed(int input);
  int get_max_speed();

private:
  /////
  // Drivetrain
  /////
  util::drivetrain_type current_drivetrain_type;
  util::drivetrain_tracker_type current_tracker_type;
  double drivetrain_tick_per_revolution;
  double drivetrain_tick_per_inch;

  double wheel_motor_cartridge;
  double wheel_circumference;
  double wheel_diameter;
  double wheel_gear_ratio;

  double tracker_circumference;
  double tracker_diameter;
  double tracker_gear_ratio;

  /////
  // User Control
  /////
};
} // namespace apollo
