/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "apollo/util/util.hpp"
#include "chassisModel.hpp"
#include <vector>
namespace apollo {
class Tank : public Chassis {
public:
  std::vector<pros::Motor> left_motors;
  std::vector<pros::Motor> right_motors;
  pros::Imu inertial_sensor;
  pros::ADIEncoder left_adi_encoder_tracker;
  pros::ADIEncoder right_adi_encoder_tracker;
  pros::ADIEncoder center_adi_encoder_tracker;
  pros::Rotation left_rotation_tracker;
  pros::Rotation right_rotation_tracker;
  pros::Rotation center_rotation_tracker;
  // Tank Drive - Motor encoders
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge);
  // Tank Drive - ADI Encoders (Left, Right)
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       std::vector<int> left_adi_encoder_ports,
       std::vector<int> right_adi_encoder_ports, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right, Center)
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       std::vector<int> left_adi_encoder_ports,
       std::vector<int> right_adi_encoder_ports,
       std::vector<int> center_adi_encoder_ports, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right) in expander
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       std::vector<int> left_adi_encoder_ports,
       std::vector<int> right_adi_encoder_ports, int expander_smart_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);
  // Tank Drive - ADI Encoders (Left, Right, Center) in expander
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       std::vector<int> left_adi_encoder_ports,
       std::vector<int> right_adi_encoder_ports,
       std::vector<int> center_adi_encoder_ports, int expander_smart_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);
  // Tank Drive - Rotation Sensors (Left, Right)
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       int left_rotation_port, int right_rotation_port,
       double tracker_wheel_diameter, double tracker_gear_ratio);
  // Tank Drive - Rotation Sensors (Left, Right, Center)
  Tank(std::vector<int> left_motor_ports, std::vector<int> right_motor_ports,
       int inertial_sensor_port, double drivetrain_wheel_diameter,
       double drivetrain_gear_ratio, double drivetrain_motor_catridge,
       int left_rotation_port, int right_rotation_port,
       int center_rotation_port, double tracker_wheel_diameter,
       double tracker_gear_ratio);
  // Control
  void tank_control() override;
  void arcade_control() override;
  // Drive Motors
  void set_left_motors_voltage(int input);
  void set_right_motors_voltage(int input);
  void get_scaled_joystick_output(pros::controller_analog_e_t input) override;
  // Drive Params
  void set_brake_mode(pros::motor_brake_mode_e_t) override;
  pros::motor_brake_mode_e_t get_brake_mode() override;
  void set_encoder_units(pros::motor_encoder_units_e_t input) override;
  pros::motor_encoder_units_e_t get_encoder_units() override;
  void set_max_velocity(int input) override;
  int get_max_velocity() override;
  void set_max_voltage(double input) override;
  double get_max_voltage() override;
  // Telemetry
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

private:
  // Chassis
  util::chassis_tracker_type current_tracker_type;
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