#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/velocity.hpp"
#include "msgs/msg/brain.hpp"


class WheelOdomNode : public rclcpp::Node {
public:
    WheelOdomNode() : Node("wheel_odom_node") {
        brainSubscriber = this->create_subscription<msgs::msg::Brain>(
                "brain", 10, std::bind(&WheelOdomNode::brainCallback, this, std::placeholders::_1));
        velocityPublisher = this->create_publisher<msgs::msg::Velocity>("wheel_odometry", 10);            
    }
private:
    void brainCallback(const msgs::msg::Brain::SharedPtr msg) {
        msgs::msg::Velocity velocityMsg;

        velocityMsg.vx = 0;
        velocityMsg.vy = (msg->left_vel) + (msg->right_vel - msg->left_vel) / 2; 
        velocityMsg.w = msg->w;

        velocityPublisher->publish(velocityMsg);
        prev = std::make_shared<msgs::msg::Brain>(*msg);
    }

    msgs::msg::Brain::SharedPtr prev;
    rclcpp::Subscription<msgs::msg::Brain>::SharedPtr brainSubscriber;
    rclcpp::Publisher<msgs::msg::Velocity>::SharedPtr velocityPublisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}