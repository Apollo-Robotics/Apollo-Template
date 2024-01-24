#include "apollo/chassis/drivetrain.hpp"
#include "apollo/util/util.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <vector>
namespace apollo {
// Tank Drive - Motor encoders
Drivetrain::Drivetrain(util::drivetrain_type drivetrain_type,
                       std::vector<int> left_motor_ports,
                       std::vector<int> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       double drivetrain_motor_catridge)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right)
Drivetrain::Drivetrain(
    util::drivetrain_type drivetrain_type, std::vector<int> left_motor_ports,
    std::vector<int> right_motor_ports, int inertial_sensor_port,
    double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
    double drivetrain_motor_catridge, std::vector<int> left_adi_encoder_ports,
    std::vector<int> right_adi_encoder_ports, double tracker_wheel_diameter,
    double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(
          abs(left_adi_encoder_ports[0]), abs(left_adi_encoder_ports[1]),
          util::is_reversed(abs(left_adi_encoder_ports[0]))),
      right_adi_encoder_tracker(
          abs(right_adi_encoder_ports[0]), abs(right_adi_encoder_ports[1]),
          util::is_reversed(abs(right_adi_encoder_ports[0]))),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right, Center)
Drivetrain::Drivetrain(
    util::drivetrain_type drivetrain_type, std::vector<int> left_motor_ports,
    std::vector<int> right_motor_ports, int inertial_sensor_port,
    double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
    double drivetrain_motor_catridge, std::vector<int> left_adi_encoder_ports,
    std::vector<int> right_adi_encoder_ports,
    std::vector<int> center_adi_encoder_ports, double tracker_wheel_diameter,
    double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(
          abs(left_adi_encoder_ports[0]), abs(left_adi_encoder_ports[1]),
          util::is_reversed(abs(left_adi_encoder_ports[0]))),
      right_adi_encoder_tracker(
          abs(right_adi_encoder_ports[0]), abs(right_adi_encoder_ports[1]),
          util::is_reversed(abs(right_adi_encoder_ports[0]))),
      center_adi_encoder_tracker(
          abs(center_adi_encoder_ports[0]), abs(center_adi_encoder_ports[1]),
          util::is_reversed(abs(center_adi_encoder_ports[0]))),
      left_rotation_tracker(-1), right_rotation_tracker(-1),
      center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right) in expander
Drivetrain::Drivetrain(
    util::drivetrain_type drivetrain_type, std::vector<int> left_motor_ports,
    std::vector<int> right_motor_ports, int inertial_sensor_port,
    double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
    double drivetrain_motor_catridge, std::vector<int> left_adi_encoder_ports,
    std::vector<int> right_adi_encoder_ports, int expander_smart_port,
    double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(
          {expander_smart_port, abs(left_adi_encoder_ports[0]),
           abs(left_adi_encoder_ports[1])},
          util::is_reversed(abs(left_adi_encoder_ports[0]))),
      right_adi_encoder_tracker(
          {expander_smart_port, abs(right_adi_encoder_ports[0]),
           abs(right_adi_encoder_ports[1])},
          util::is_reversed(abs(right_adi_encoder_ports[0]))),
      center_adi_encoder_tracker(-1, -1, false), left_rotation_tracker(-1),
      right_rotation_tracker(-1), center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right, Center) in expander
Drivetrain::Drivetrain(
    util::drivetrain_type drivetrain_type, std::vector<int> left_motor_ports,
    std::vector<int> right_motor_ports, int inertial_sensor_port,
    double drivetrain_wheel_diameter, double drivetrain_gear_ratio,
    double drivetrain_motor_catridge, std::vector<int> left_adi_encoder_ports,
    std::vector<int> right_adi_encoder_ports,
    std::vector<int> center_adi_encoder_ports, int expander_smart_port,
    double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(
          {expander_smart_port, abs(left_adi_encoder_ports[0]),
           abs(left_adi_encoder_ports[1])},
          util::is_reversed(abs(left_adi_encoder_ports[0]))),
      right_adi_encoder_tracker(
          {expander_smart_port, abs(right_adi_encoder_ports[0]),
           abs(right_adi_encoder_ports[1])},
          util::is_reversed(abs(right_adi_encoder_ports[0]))),
      center_adi_encoder_tracker(
          {expander_smart_port, abs(center_adi_encoder_ports[0]),
           abs(center_adi_encoder_ports[1])},
          util::is_reversed(abs(center_adi_encoder_ports[0]))),
      left_rotation_tracker(-1), right_rotation_tracker(-1),
      center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}

// Tank Drive - Rotation Sensors (Left, Right)
Drivetrain::Drivetrain(util::drivetrain_type drivetrain_type,
                       std::vector<int> left_motor_ports,
                       std::vector<int> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       double drivetrain_motor_catridge, int left_rotation_port,
                       int right_rotation_port, double tracker_wheel_diameter,
                       double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(abs(left_rotation_port),
                            util::is_reversed(left_rotation_port)),
      right_rotation_tracker(abs(right_rotation_port),
                             util::is_reversed(right_rotation_port)),
      center_rotation_tracker(-1) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
// Tank Drive - ADI Encoders (Left, Right)
Drivetrain::Drivetrain(util::drivetrain_type drivetrain_type,
                       std::vector<int> left_motor_ports,
                       std::vector<int> right_motor_ports,
                       int inertial_sensor_port,
                       double drivetrain_wheel_diameter,
                       double drivetrain_gear_ratio,
                       double drivetrain_motor_catridge, int left_rotation_port,
                       int right_rotation_port, int center_rotation_port,
                       double tracker_wheel_diameter, double tracker_gear_ratio)
    : inertial_sensor(inertial_sensor_port),
      left_adi_encoder_tracker(-1, -1, false),
      right_adi_encoder_tracker(-1, -1, false),
      center_adi_encoder_tracker(-1, -1, false),
      left_rotation_tracker(abs(left_rotation_port),
                            util::is_reversed(left_rotation_port)),
      right_rotation_tracker(abs(right_rotation_port),
                             util::is_reversed(right_rotation_port)),
      center_rotation_tracker(abs(center_rotation_port),
                              util::is_reversed(center_rotation_port)) {
  for (auto i : left_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    left_motors.push_back(temp);
  }
  for (auto i : right_motor_ports) {
    pros::Motor temp(abs(i), apollo::util::is_reversed(i));
    right_motors.push_back(temp);
  }
  current_drivetrain_type = drivetrain_type;
  current_tracker_type = util::DRIVE_MOTOR_ENCODER;

  wheel_motor_cartridge = drivetrain_motor_catridge;
  wheel_diameter = drivetrain_wheel_diameter;
  wheel_circumference = wheel_diameter * M_PI;
  wheel_gear_ratio = drivetrain_gear_ratio;

  tracker_circumference = wheel_circumference;
  tracker_diameter = wheel_diameter;
  tracker_gear_ratio = wheel_gear_ratio;

  drivetrain_tick_per_revolution =
      (50.0 * (3600.0 / wheel_motor_cartridge)) * tracker_gear_ratio;
  drivetrain_tick_per_inch =
      (drivetrain_tick_per_revolution / tracker_circumference);
}
} // namespace apollo