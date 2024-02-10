/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/chassis/chassisTankModel.hpp"
#include "apollo/util/util.hpp"
#include "pros/motors.h"
#include <cmath>

namespace apollo {
// Tank Drive - Motor encoders
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge,
           std::vector<int> left_adi_encoder_ports,
           std::vector<int> right_adi_encoder_ports,
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
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge,
           std::vector<int> left_adi_encoder_ports,
           std::vector<int> right_adi_encoder_ports,
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
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge,
           std::vector<int> left_adi_encoder_ports,
           std::vector<int> right_adi_encoder_ports, int expander_smart_port,
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
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge,
           std::vector<int> left_adi_encoder_ports,
           std::vector<int> right_adi_encoder_ports,
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
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge, int left_rotation_port,
           int right_rotation_port, double tracker_wheel_diameter,
           double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(left_rotation_port),
      right_rotation_tracker(right_rotation_port), center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
Tank::Tank(std::vector<int> left_motor_ports,
           std::vector<int> right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           double drivetrain_motor_cartridge, int left_rotation_port,
           int right_rotation_port, int center_rotation_port,
           double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(left_rotation_port),
      right_rotation_tracker(right_rotation_port),
      center_rotation_tracker(center_rotation_port) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), util::is_reversed(i));
    right_motors.push_back(temp);
  }
  wheel_motor_cartridge = drivetrain_motor_cartridge;
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
// Control
void Tank::tank_control() {}
void Tank::arcade_control() {}
void Tank::set_left_motors_voltage(int input) {
  for (auto i : left_motors) {
    i.move_voltage(input * (12000.0 / 127.0));
  }
}
void Tank::set_right_motors_voltage(int input) {
  for (auto i : right_motors) {
    i.move_voltage(input * (12000.0 / 127.0));
  }
}
void Tank::get_scaled_joystick_output(pros::controller_analog_e_t input) {}
// Drive Params
void Tank::set_brake_mode(pros::motor_brake_mode_e_t) {}
pros::motor_brake_mode_e_t Tank::get_brake_mode() {
  return pros::E_MOTOR_BRAKE_HOLD;
}
void Tank::set_encoder_units(pros::motor_encoder_units_e_t input) {}
pros::motor_encoder_units_e_t Tank::get_encoder_units() {
  return pros::E_MOTOR_ENCODER_ROTATIONS;
}
void Tank::set_max_velocity(int input) { max_drive_velocity = input; }
int Tank::get_max_velocity() { return max_drive_velocity; }
void Tank::set_max_voltage(double input) { max_drive_voltage = input; }
double Tank::get_max_voltage() { return max_drive_voltage; }
// Telemetry
int Tank::get_left_sensor_value() {
  if (current_tracker_type == util::DRIVE_MOTOR_ENCODER) {
    return left_motors[0].get_position();
  } else if (current_tracker_type == util::DRIVE_ADI_ENCODER) {
    return left_adi_encoder_tracker.get_value();
  } else if (current_tracker_type == util::DRIVE_ROTATION_SENSOR) {
    return left_rotation_tracker.get_position();
  } else {
    return 0;
  }
}
int Tank::get_left_sensor_velocity() {
  if (current_tracker_type == util::DRIVE_MOTOR_ENCODER) {
    return left_motors[0].get_actual_velocity();
  } else if (current_tracker_type == util::DRIVE_ADI_ENCODER) {
    return left_adi_encoder_tracker.get_value();
  } else if (current_tracker_type == util::DRIVE_ROTATION_SENSOR) {
    return left_rotation_tracker.get_position();
  } else {
    return 0;
  }
}
double Tank::get_left_motor_voltage() { return 0; }
double Tank::get_left_motor_current() { return 0; }
int Tank::get_right_sensor_value() {
  if (current_tracker_type == util::DRIVE_MOTOR_ENCODER) {
    return right_motors[0].get_position();
  } else if (current_tracker_type == util::DRIVE_ADI_ENCODER) {
    return right_adi_encoder_tracker.get_value();
  } else if (current_tracker_type == util::DRIVE_ROTATION_SENSOR) {
    return right_rotation_tracker.get_position();
  } else {
    return 0;
  }
}
int Tank::get_right_sensor_velocity() { return 0; }
double Tank::get_right_motor_voltage() { return 0; }
double Tank::get_right_motor_current() { return 0; }
double Tank::get_inertial_sensor_value() { return 0; }
void Tank::reset_drive_sensors() {}
void Tank::reset_inertial_sensor() {}
void Tank::reset_left_drive_sensor() {}
void Tank::reset_right_drive_sensor() {
}
} // namespace apollo
