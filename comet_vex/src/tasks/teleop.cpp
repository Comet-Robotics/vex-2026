#include "tasks/teleop.h"
#include "nlohmann/json.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

using json = nlohmann::json;

void opcontrol_initialize() {}

void opcontrol() {  
    json j;
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Rotation odom(13);
    pros::Motor motor(16);

    odom.reset_position();

    while (true) {
        int odom_pos = odom.get_position();
        int motor_pos = motor.get_position();

        j = {
            {"odom", odom_pos},
            {"motor", motor_pos}
        };

        // send data
        printf("%s\n", j.dump().c_str());

        // receive data
        std::string input;
        std::cin >> input;


        if (!input.empty()) {
            json received = json::parse(input);
            if (received.contains("motor")) {
                int voltage = received["motor"];
                motor.move_voltage(voltage);
            }
        }

        pros::delay(10);
    }
}