/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/chassis/chassisTankModel.hpp"

#include <cmath>

#include "apollo/util/util.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

namespace apollo {
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(-1, -1, false),
        right_adi_encoder_tracker(-1, -1, false),
        center_adi_encoder_tracker(-1, -1, false),
        left_rotation_tracker(-1),
        right_rotation_tracker(-1),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge,
                       std::vector<int8_t> left_adi_encoder_ports,
                       std::vector<int8_t> right_adi_encoder_ports,
                       double tracker_wheel_diameter, double tracker_gear_ratio)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(left_adi_encoder_ports[0],
                                 left_adi_encoder_ports[1],
                                 util::is_reversed(left_adi_encoder_ports[0])),
        right_adi_encoder_tracker(
            right_adi_encoder_ports[0], right_adi_encoder_ports[1],
            util::is_reversed(right_adi_encoder_ports[0])),
        center_adi_encoder_tracker(-1, -1, false),
        left_rotation_tracker(-1),
        right_rotation_tracker(-1),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge,
                       std::vector<int8_t> left_adi_encoder_ports,
                       std::vector<int8_t> right_adi_encoder_ports,
                       std::vector<int> center_adi_encoder_ports,
                       double tracker_wheel_diameter, double tracker_gear_ratio)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(left_adi_encoder_ports[0],
                                 left_adi_encoder_ports[1],
                                 util::is_reversed(left_adi_encoder_ports[0])),
        right_adi_encoder_tracker(
            right_adi_encoder_ports[0], right_adi_encoder_ports[1],
            util::is_reversed(right_adi_encoder_ports[0])),
        center_adi_encoder_tracker(
            center_adi_encoder_ports[0], center_adi_encoder_ports[1],
            util::is_reversed(center_adi_encoder_ports[0])),
        left_rotation_tracker(-1),
        right_rotation_tracker(-1),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge,
                       std::vector<int8_t> left_adi_encoder_ports,
                       std::vector<int8_t> right_adi_encoder_ports,
                       int expander_smart_port, double tracker_wheel_diameter,
                       double tracker_gear_ratio)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(
            {expander_smart_port, left_adi_encoder_ports[0],
             left_adi_encoder_ports[1]},
            util::is_reversed(left_adi_encoder_ports[0])),
        right_adi_encoder_tracker(
            {expander_smart_port, right_adi_encoder_ports[0],
             right_adi_encoder_ports[1]},
            util::is_reversed(right_adi_encoder_ports[0])),
        center_adi_encoder_tracker(-1, -1, false),
        left_rotation_tracker(-1),
        right_rotation_tracker(-1),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge,
                       std::vector<int8_t> left_adi_encoder_ports,
                       std::vector<int8_t> right_adi_encoder_ports,
                       std::vector<int> center_adi_encoder_ports,
                       int expander_smart_port, double tracker_wheel_diameter,
                       double tracker_gear_ratio)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(
            {expander_smart_port, left_adi_encoder_ports[0],
             left_adi_encoder_ports[1]},
            util::is_reversed(left_adi_encoder_ports[0])),
        right_adi_encoder_tracker(
            {expander_smart_port, right_adi_encoder_ports[0],
             right_adi_encoder_ports[1]},
            util::is_reversed(right_adi_encoder_ports[0])),
        center_adi_encoder_tracker(
            {expander_smart_port, center_adi_encoder_ports[0],
             center_adi_encoder_ports[1]},
            util::is_reversed(center_adi_encoder_ports[0])),
        left_rotation_tracker(-1),
        right_rotation_tracker(-1),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       pros::v5::MotorGears drivetrain_motor_cartridge,
                       int left_rotation_port, int right_rotation_port,
                       double tracker_wheel_diameter, double tracker_gear_ratio)
      : inertial_sensor(inertial_sensor_port),
        left_adi_encoder_tracker(-1, -1, false),
        right_adi_encoder_tracker(-1, -1, false),
        center_adi_encoder_tracker(-1, -1, false),
        left_rotation_tracker(left_rotation_port),
        right_rotation_tracker(right_rotation_port),
        center_rotation_tracker(-1) {
    pros::MotorGroup left_temp(left_motor_ports);
    left_motor_group().append(left_temp);
    pros::MotorGroup right_temp(right_motor_ports);
    right_motor_group().append(right_temp);
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  TankModel::TankModel(std::vector<int8_t> left_motor_ports,
                       std::vector<int8_t> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
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
    wheel_motor_cartridge =
        util::convert_gear_ratio(drivetrain_motor_cartridge);
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
  void TankModel::tank_control() {
    if(abs(master.get_analog(left_tank_joystick)) > joystick_deadband) {
      left_motor_group().move(get_scaled_voltage(left_tank_joystick));
    } else if(abs(master.get_analog(right_tank_joystick)) > joystick_deadband) {
      right_motor_group().move(get_scaled_voltage(right_tank_joystick));
    } else {
      left_motor_group().move(0);
      right_motor_group().move(0);
    }
  }
  void TankModel::arcade_control(bool is_flipped, bool is_split) {
    if(abs(master.get_analog(forward_arcade_joystick)) > joystick_deadband ||
       abs(master.get_analog(turn_arcade_joystick)) > joystick_deadband) {
      left_motor_group().move(get_scaled_voltage(forward_arcade_joystick) +
                              get_scaled_voltage(turn_arcade_joystick));
      right_motor_group().move(get_scaled_voltage(forward_arcade_joystick) -
                               get_scaled_voltage(turn_arcade_joystick));
    } else {
      left_motor_group().move(0);
      right_motor_group().move(0);
    }
  }
}  // namespace apollo