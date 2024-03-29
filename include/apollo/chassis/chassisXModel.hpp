/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include <vector>

#include "chassisModel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace apollo {
  class XModel : public ChassisModel {
    /**
     * @brief The drivetrain's Front Left Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup front_left_motor_group();
    /**
     * @brief The drivetrain's Front Right Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup front_right_motor_group();
    /**
     * @brief The drivetrain's Back Left Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup back_left_motor_group();
    /**
     * @brief The drivetrain's Back Right Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup back_right_motor_group();
    /**
     * @brief The drivetrain's Inertial Sensor. Used for PID functions in
     * autonomous.
     *
     */
    pros::Imu inertial_sensor;
    /**
     * @brief Optional ADI Encoder tracking wheels for the left side of the
     * drivetrain.
     *
     */
    pros::adi::Encoder left_adi_encoder_tracker;
    /**
     * @brief Optional ADI Encoder tracking wheels for the right side of the
     * drivetrain.
     *
     */
    pros::adi::Encoder right_adi_encoder_tracker;
    /**
     * @brief Optional ADI Encoder tracking wheels for center-horizontal
     * tracking of the drivetrain.
     *
     */
    pros::adi::Encoder center_adi_encoder_tracker;
    /**
     * @brief Optional Rotation Sensor tracking wheels for the left side of the
     * drivetrain.
     *
     */
    pros::Rotation left_rotation_tracker;
    /**
     * @brief Optional Rotation Sensor tracking wheels for the right side of the
     * drivetrain.
     *
     */
    pros::Rotation right_rotation_tracker;
    /**
     * @brief Optional Rotation Sensor tracking wheels for center-horizontal
     * tracking of the drivetrain.
     *
     */
    pros::Rotation center_rotation_tracker;
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           double tracker_wheel_diameter, double tracker_gear_ratio);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           std::vector<int> center_adi_encoder_ports,
           double tracker_wheel_diameter, double tracker_gear_ratio);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports, int expander_smart_port,
           double tracker_wheel_diameter, double tracker_gear_ratio);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           std::vector<int8_t> left_adi_encoder_ports,
           std::vector<int8_t> right_adi_encoder_ports,
           std::vector<int> center_adi_encoder_ports, int expander_smart_port,
           double tracker_wheel_diameter, double tracker_gear_ratio);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           int left_rotation_port, int right_rotation_port,
           double tracker_wheel_diameter, double tracker_gear_ratio);
    XModel(std::vector<int8_t> front_left_motor_ports,
           std::vector<int8_t> front_right_motor_ports,
           std::vector<int8_t> back_left_motor_ports,
           std::vector<int8_t> back_right_motor_ports, int inertial_sensor_port,
           double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
           pros::v5::MotorGears drivetrain_motor_cartridge,
           int left_rotation_port, int right_rotation_port,
           int center_rotation_port, double tracker_wheel_diameter,
           double tracker_gear_ratio);
    void tank_control();
    void arcade_control();
  };

}  // namespace apollo