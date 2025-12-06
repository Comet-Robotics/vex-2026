#include "msgs/msg/Brain.hpp"

struct Position {
    msgs::msg::Brain prev;
};
    
class WheelOdom {
    Position pos;
    void update(double left, double right, double theta);
};