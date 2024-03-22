/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/util/util.hpp"
#include "chassisModel.hpp"
#include "pros/motor_group.hpp"
#include <vector>
namespace apollo {
class Tank {
public:
  pros::MotorGroup left_motor_group();
  pros::MotorGroup right_motor_group();
  pros::Imu inertial_sensor;
  pros::adi::Encoder left_adi_encoder_tracker;
  pros::adi::Encoder right_adi_encoder_tracker;
  pros::adi::Encoder center_adi_encoder_tracker;
  pros::Rotation left_rotation_tracker;
  pros::Rotation right_rotation_tracker;
  pros::Rotation center_rotation_tracker;
  // Tank Drive - Motor encoders
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge);
  // Tank Drive - ADI Encoders (Left, Right)
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge,
       std::vector<int8_t> left_adi_encoder_ports,
       std::vector<int8_t> right_adi_encoder_ports, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right, Center)
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge,
       std::vector<int8_t> left_adi_encoder_ports,
       std::vector<int8_t> right_adi_encoder_ports,
       std::vector<int> center_adi_encoder_ports, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right) in expander
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge,
       std::vector<int8_t> left_adi_encoder_ports,
       std::vector<int8_t> right_adi_encoder_ports, int expander_smart_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right, Center) in expander
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge,
       std::vector<int8_t> left_adi_encoder_ports,
       std::vector<int8_t> right_adi_encoder_ports,
       std::vector<int> center_adi_encoder_ports, int expander_smart_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);
  // Tank Drive - Rotation Sensors (Left, Right)
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge, int left_rotation_port,
       int right_rotation_port, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Tank Drive - Rotation Sensors (Left, Right, Center)
  Tank(std::vector<int8_t> left_motor_ports, std::vector<int8_t> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio,
       pros::v5::MotorGears drivetrain_motor_cartridge, int left_rotation_port,
       int right_rotation_port, int center_rotation_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);

private:
  // Chassis
  util::chassis_tracker_type current_tracker_type;
  pros::controller_analog_e_t left_tank_joystick;
  pros::controller_analog_e_t right_tank_joystick;
  pros::controller_analog_e_t forward_arcade_joystick;
  pros::controller_analog_e_t turn_arcade_joystick;
  double joystick_deadband;
  double drivetrain_tick_per_revolution;
  double drivetrain_tick_per_inch;

  double wheel_motor_cartridge;
  double wheel_circumference;
  double wheel_diameter;
  double wheel_gear_ratio;

  double tracker_circumference;
  double tracker_diameter;
  double tracker_gear_ratio;
};
} // namespace apollo