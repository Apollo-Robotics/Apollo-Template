/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#include "apollo/util/util.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
namespace apollo {
  class ChassisModel {
   public:
    util::chassis_tracker_type current_tracker_type;
    pros::motor_brake_mode_e_t current_brake_mode = pros::E_MOTOR_BRAKE_COAST;
    pros::motor_encoder_units_e_t current_encoder_units =
        pros::E_MOTOR_ENCODER_ROTATIONS;
    int joystick_deadband;

    double drivetrain_tick_per_revolution;
    double drivetrain_tick_per_inch;

    double wheel_motor_cartridge;
    double wheel_circumference;
    double wheel_diameter;
    double wheel_gear_ratio;

    double tracker_circumference;
    double tracker_diameter;
    double tracker_gear_ratio;

    void get_chassis_parameters();

    void set_brake_mode(pros::motor_brake_mode_e_t brake_mode);
    pros::motor_brake_mode_e_t get_brake_mode();
    void set_encoder_units(pros::motor_encoder_units_e_t encoder_units);
    pros::motor_encoder_units_e_t get_encoder_units();

    void set_joystick_deadband(int input);
    int get_joystick_deadband();

    pros::controller_analog_e_t left_tank_joystick = pros::E_CONTROLLER_ANALOG_LEFT_Y;
    pros::controller_analog_e_t right_tank_joystick = pros::E_CONTROLLER_ANALOG_RIGHT_Y;
    pros::controller_analog_e_t forward_arcade_joystick;
    pros::controller_analog_e_t turn_arcade_joystick;
    pros::controller_analog_e_t strafe_arcade_joystick;
    pros::controller_analog_e_t rotate_arcade_joystick;

    int get_scaled_voltage(pros::controller_analog_e_t input);

    void set_tank_joysticks(pros::controller_analog_e_t left,
                            pros::controller_analog_e_t right);
    void set_arcade_joysticks(pros::controller_analog_e_t forward,
                              pros::controller_analog_e_t turn);
    void set_strafe_joysticks(pros::controller_analog_e_t strafe);
    void set_rotate_joysticks(pros::controller_analog_e_t rotate);
  };
}  // namespace apollo
