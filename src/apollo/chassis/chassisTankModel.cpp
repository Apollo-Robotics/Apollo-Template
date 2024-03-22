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
/**
 * @brief Construct a new Tank Drive Drivetrain using Motor Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels. Used for
 * PID functions.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain. Used for PID
 * functions.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain. Used
 * for PID functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left and Right ADI
 * Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_adi_encoder_ports Ports of the Left ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param right_adi_encoder_ports Ports of the Right ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left, Right, and Center
 * Horizontal ADI Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_adi_encoder_ports Ports of the Left ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param right_adi_encoder_ports Ports of the Right ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param center_adi_encoder_ports Ports of the Center ADI Encoder. First port
 * is assumed to be the 'top' port of the encoder.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left, Right, and Center
 * Horizontal ADI Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_adi_encoder_ports Ports of the Left ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param right_adi_encoder_ports Ports of the Right ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param expander_smart_port 3-Wire Expanders smart port. Assumes all of your
 * tracking wheels are connected to the expander.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left, Right, and Center
 * Horizontal ADI Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_adi_encoder_ports Ports of the Left ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param right_adi_encoder_ports Ports of the Right ADI Encoder. First port is
 * assumed to be the 'top' port of the encoder.
 * @param center_adi_encoder_ports Ports of the Center ADI Encoder. First port
 * is assumed to be the 'top' port of the encoder.
 * @param expander_smart_port 3-Wire Expanders smart port. Assumes all of your
 * tracking wheels are connected to the expander.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left, Right, and Center
 * Horizontal ADI Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_rotation_port Port of the Left Rotation sensor used for tracking.
 * @param right_rotation_port Port of the Right Rotation sensor used for
 * tracking.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
/**
 * @brief Construct a new Tank Drive Drivetrain using Left, Right, and Center
 * Horizontal ADI Encoders
 *
 * @param left_motor_ports The ports that the Left Motors are connected to. The
 * first motor is the sensored motor.
 * @param right_motor_ports The ports that the Right Motors are connected to.
 * The first motor is the sensored motor.
 * @param inertial_sensor_port The Inertial Sensor's port
 * @param drivetrain_wheel_diameter Diameter of your drivetrain wheels.
 * @param drivetrain_gear_ratio Gear Ratio of your drivetrain.
 * @param drivetrain_motor_cartridge Motor Cartridge of your drivetrain
 * @param left_rotation_port Port of the Left Rotation sensor used for tracking.
 * @param right_rotation_port Port of the Right Rotation sensor used for
 * tracking.
 * @param center_rotation_port Port of the Center Rotation sensor used for
 * tracking.
 * @param tracker_wheel_diameter Diameter of your tracking wheels. Used for PID
 * functions.
 * @param tracker_gear_ratio Gear Ratio of your tracking wheels. Used for PID
 * functions.
 */
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
} // namespace apollo