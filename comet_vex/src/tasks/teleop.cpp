#include "tasks/teleop.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include <sstream>

extern "C" int32_t inp_buffer_read(uint32_t timeout);

uint32_t last_input = pros::millis();

std::string read_serial_nonblocking(size_t maxlen) {
    if (maxlen == 0) return std::string();

    std::string out;
    out.reserve(maxlen);

    for (size_t i = 0; i < maxlen; ++i) {
        int32_t c = inp_buffer_read(0); // 0 = non-blocking read
        if (c == -1) break; // no more data
        out.push_back(static_cast<char>(c));
        if (c == '\n') break; // stop at new line
    }

    last_input = pros::millis();

    return out;
}

std::vector<int> parse_csv_ints(const std::string& line) {
    std::vector<int> out;
    std::stringstream ss(line);
    int val;
    char comma;

    while (ss >> val) {
        out.push_back(val);
        ss >> comma;
    }
    return out;
}

// 0-3: left, 4-7: right
std::vector<int> motorNums = {-7, 8, -9, 14, 17, -18, 19, -20};

void opcontrol_initialize() {
    drivebase->calibrateIMU();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    std::vector<pros::Motor> motors;
    motors.reserve(motorNums.size());
    for (int port : motorNums) {
        motors.emplace_back(port);
    }

    last_input = pros::millis();

    while (true) {
        drivebase->update();
        // other subsystems update

        drivebase->errorDrive(
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
        );

        // // get drivetrain twist and send to serial
        // Twist2D twist = drivebase->getTwist();
        // std::string line = std::to_string(twist.vx) + "," +
        //                    std::to_string(twist.vy) + "," +
        //                    std::to_string(twist.w);

        // printf("%s\n", line.c_str());

        Pose2D currentPose = drivebase->getPose();
        printf("%f,%f,%f\n", currentPose.x, currentPose.y, currentPose.theta);

        // serial input
        std::string input = read_serial_nonblocking(512);
        auto pose = parse_csv_ints(input);
        if (pose.size() >= 3) {
            drivebase->setPose(Pose2D(pose[0], pose[1], pose[2]));
        }
        

        pros::delay(10);
    }
}