#include "tasks/teleop.h"
#include "liblvgl/llemu.hpp"
#include "messages/message.pb.h"

void opcontrol_initialize() {}

void opcontrol() {
    // Example usage of the generated protobuf code
    Person person;
    person.set_name("John Doe");
    person.set_id(123);
    person.set_email("john.doe@example.com");

    while (true) {
        pros::lcd::clear();
        pros::lcd::print(1, "Name: %s", person.name().c_str());
        pros::lcd::print(2, "ID: %d", person.id());
        pros::lcd::print(3, "Email: %s", person.email().c_str());
        pros::delay(20);
    }
}