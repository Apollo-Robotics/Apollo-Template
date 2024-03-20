#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <string>
#pragma onxw
namespace apollo {
struct motor {
  pros::Motor motor;
  std::string name;
  int port;
  pros::MotorGears cartridge;
  bool is_reversed;
};
struct button{
    int initial_x_postion;
    int initial_y_postion;
    int width;
    int height;
};
struct auton {
  std::string name;
  std::string description;
  void autonomous_function();
};
class GUI {
public:
  // create initializers using drive template params
  //  add extra motors params
  bool is_auton_selector_enabled;

private:
  int total_autons = 0;
  int selected_auton = 0;
  int curent_auton_page = 0;
};

} // namespace apollo