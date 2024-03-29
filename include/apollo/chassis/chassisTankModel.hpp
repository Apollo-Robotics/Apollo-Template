/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include <vector>

#include "apollo/util/util.hpp"
#include "chassisModel.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace apollo {
  class TankModel : public ChassisModel {
   public:
    /**
     * @brief The drivetrain's Left Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup left_motor_group();
    /**
     * @brief The drivetrain's Right Motor Group
     *
     * @return pros::MotorGroup
     */
    pros::MotorGroup right_motor_group();
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
    /**
     * @brief Construct a new Tank Drive Drivetrain using Motor Encoders
     *
     * @param left_motor_ports The ports that the Left Motors are connected to.
     * The first motor is the sensored motor.
     * @param right_motor_ports The ports that the Right Motors are connected
     * to. The first motor is the sensored motor.
     * @param inertial_sensor_port The Inertial Sensor's port
     * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels. Used
     * for PID functions.
     * @param drivetrain_gear_ratio Gear Ratio of your drivetrain. Used for PID
     * functions.
     * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain.
     * Used for PID functions.
     */
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge);
    /**
     * @brief Construct a new Tank Drive Drivetrain using Left and Right ADI
     * Encoders
     *
     * @param left_motor_ports The ports that the Left Motors are connected to.
     * The first motor is the sensored motor.
     * @param right_motor_ports The ports that the Right Motors are connected
     * to. The first motor is the sensored motor.
     * @param inertial_sensor_port The Inertial Sensor's port
     * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
     * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
     * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
     * @param left_adi_encoder_ports Ports of the Left ADI Encoder. First port
     * is assumed to be the 'top' port of the encoder.
     * @param right_adi_encoder_ports Ports of the Right ADI Encoder. First port
     * is assumed to be the 'top' port of the encoder.
     * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for
     * PID functions.
     * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for
     * PID functions.
     */
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              std::vector<int8_t> left_adi_encoder_ports,
              std::vector<int8_t> right_adi_encoder_ports,
              double tracker_wheel_diameter, double tracker_gear_ratio);
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              std::vector<int8_t> left_adi_encoder_ports,
              std::vector<int8_t> right_adi_encoder_ports,
              std::vector<int> center_adi_encoder_ports,
              double tracker_wheel_diameter, double tracker_gear_ratio);
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              std::vector<int8_t> left_adi_encoder_ports,
              std::vector<int8_t> right_adi_encoder_ports,
              int expander_smart_port, double tracker_wheel_diameter,
              double tracker_gear_ratio);
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              std::vector<int8_t> left_adi_encoder_ports,
              std::vector<int8_t> right_adi_encoder_ports,
              std::vector<int> center_adi_encoder_ports,
              int expander_smart_port, double tracker_wheel_diameter,
              double tracker_gear_ratio);
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              int left_rotation_port, int right_rotation_port,
              double tracker_wheel_diameter, double tracker_gear_ratio);
    TankModel(std::vector<int8_t> left_motor_ports,
              std::vector<int8_t> right_motor_ports, int inertial_sensor_port,
              double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
              pros::v5::MotorGears drivetrain_motor_cartridge,
              int left_rotation_port, int right_rotation_port,
              int center_rotation_port, double tracker_wheel_diameter,
              double tracker_gear_ratio);
    void tank_control();
    void arcade_control(bool is_flipped = true, bool is_split = true);
  };
}  // namespace apollo