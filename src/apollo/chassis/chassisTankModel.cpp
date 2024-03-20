/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/chassis/chassisTankModel.hpp"
#include "apollo/util/util.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include <cmath>

namespace apollo {
// Tank Drive - Motor encoders
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = wheel_diameter;
  tracker_circumference = wheel_circumference;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right)
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(left_adi_encoder_ports[0],
                               left_adi_encoder_ports[1],
                               util::is_reversed(left_adi_encoder_ports[0])),
      right_adi_encoder_tracker(right_adi_encoder_ports[0],
                                right_adi_encoder_ports[1],
                                util::is_reversed(right_adi_encoder_ports[0])),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right, Center)
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           std::vector<int> center_adi_encoder_ports,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(left_adi_encoder_ports[0],
                               left_adi_encoder_ports[1],
                               util::is_reversed(left_adi_encoder_ports[0])),
      right_adi_encoder_tracker(right_adi_encoder_ports[0],
                                right_adi_encoder_ports[1],
                                util::is_reversed(right_adi_encoder_ports[0])),
      center_adi_encoder_tracker(
          center_adi_encoder_ports[0], center_adi_encoder_ports[1],
          util::is_reversed(center_adi_encoder_ports[0])),
      left_rotation_tracker(-1), right_rotation_tracker(-1),
      center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right) in expander
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports, int expander_smart_port,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker({expander_smart_port, left_adi_encoder_ports[0],
                                left_adi_encoder_ports[1]},
                               util::is_reversed(left_adi_encoder_ports[0])),
      right_adi_encoder_tracker({expander_smart_port,
                                 right_adi_encoder_ports[0],
                                 right_adi_encoder_ports[1]},
                                util::is_reversed(right_adi_encoder_ports[0])),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right, Center) in expander
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           std::vector<int> center_adi_encoder_ports, int expander_smart_port,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker({expander_smart_port, left_adi_encoder_ports[0],
                                left_adi_encoder_ports[1]},
                               util::is_reversed(left_adi_encoder_ports[0])),
      right_adi_encoder_tracker({expander_smart_port,
                                 right_adi_encoder_ports[0],
                                 right_adi_encoder_ports[1]},
                                util::is_reversed(right_adi_encoder_ports[0])),
      center_adi_encoder_tracker(
          {expander_smart_port, center_adi_encoder_ports[0],
           center_adi_encoder_ports[1]},
          util::is_reversed(center_adi_encoder_ports[0])),
      left_rotation_tracker(-1), right_rotation_tracker(-1),
      center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - Rotation Sensors (Left, Right)
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           int left_rotation_port, int right_rotation_port,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(left_rotation_port),
      right_rotation_tracker(right_rotation_port), center_rotation_tracker(-1) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - Rotation Sensors (Left, Right, Center)
Tank::Tank(std::vector<int8_t> left_motor_ports,
           std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           int left_rotation_port, int right_rotation_port,
           int center_rotation_port, double tracker_wheel_diameter,
           double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(left_rotation_port),
      right_rotation_tracker(right_rotation_port),
      center_rotation_tracker(center_rotation_port) {
  pros::MotorGroup left_temp(left_motor_ports);
  left_motor_group().append(left_temp);
  pros::MotorGroup right_temp(right_motor_ports);
  right_motor_group().append(right_temp);
  wheel_motor_cartridge = util::convert_gear_ratio(drivetrain_motor_cartridge);
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = drivetrain_wheel_diameter * M_PI;
  wheel_diameter = drivetrain_wheel_diameter;

  tracker_diameter = tracker_wheel_diameter;
  tracker_circumference = tracker_diameter * M_PI;
  tracker_gear_ratio = tracker_gear_ratio;

  drivetrain_tick_per_revolution = wheel_motor_cartridge * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
/////
// Control
/////

// Setting Joysticks
void Tank::set_joystick_deadband(double input) { joystick_deadband = input; }
void Tank::set_left_drive_joystick(pros::controller_analog_e_t input) {
  left_tank_joystick = input;
}
void Tank::set_right_drive_joystick(pros::controller_analog_e_t input) {
  right_tank_joystick = input;
}
void Tank::set_forward_arcade_joystick(pros::controller_analog_e_t input) {
  forward_arcade_joystick = input;
}
void Tank::set_turn_arcade_joystick(pros::controller_analog_e_t input) {
  turn_arcade_joystick = input;
}

//  Getting Joysticks
double Tank::get_joystick_deadband() { return joystick_deadband; };
pros::controller_analog_e_t Tank::get_left_drive_joystick() {
  return left_tank_joystick;
}
pros::controller_analog_e_t Tank::get_right_drive_joystick() {
  return right_tank_joystick;
}
pros::controller_analog_e_t Tank::get_forward_arcade_joystick() {
  return forward_arcade_joystick;
}
pros::controller_analog_e_t Tank::get_turn_arcade_joystick() {
  return turn_arcade_joystick;
}

// Joystick Sub-Methods
int Tank::get_scaled_voltage_ouput(int input) {
  if (input > 127) {
    return 12000;
  } else {
    return input * 12000 / 127;
  }
}

// Tank and Arcade Control
void Tank::tank_control() {
  if (abs(master.get_analog(left_tank_joystick)) > joystick_deadband ||
      abs(master.get_analog(right_tank_joystick)) > joystick_deadband) {
    left_motor_group().move_velocity(master.get_analog(left_tank_joystick));
    right_motor_group().move_velocity(master.get_analog(right_tank_joystick));
  } else {
    left_motor_group().move(0);
    right_motor_group().move(0);
  }
}
void Tank::arcade_control() {
  if (abs(master.get_analog(forward_arcade_joystick)) > joystick_deadband ||
      abs(master.get_analog(turn_arcade_joystick)) > joystick_deadband) {
    left_motor_group().move_velocity(
        master.get_analog(forward_arcade_joystick) +
        master.get_analog(turn_arcade_joystick));
    right_motor_group().move_velocity(
        master.get_analog(forward_arcade_joystick) -
        master.get_analog(turn_arcade_joystick));

  } else {
    left_motor_group().move(0);
    right_motor_group().move(0);
  }
}
} // namespace apollo
