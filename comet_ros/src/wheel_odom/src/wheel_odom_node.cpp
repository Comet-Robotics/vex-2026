#include "rclcpp/rclcpp.hpp"
#include "msgs/msg/Velocity.hpp"
#include "msgs/msg/Brain.hpp"

class WheelOdomNode : public rclcpp::Node {
public:
    WheelOdomNode() : Node("wheel_odom_node") {
        brainSubscriber = this->create_subscription<msgs::msg::Brain>(
                "brain", 10, std::bind(&ParticleFilterNode::brainCallback, this, std::placeholders::_1));
        velocityPublisher = this->create_publisher<msgs::msg::Velocity>("wheel_odometry", 10);            
    }
private:
    void brainCallback(const msgs::msg::Brain msg) {
        msgs::msg::Velocity velocityMsg;
        // left right
        velocityMsg.vx = 0
        // forward backwards
        rclcpp::Time t1 = rclcpp::Time(msg.header.stamp);
        rclcpp::Time t2 = rclcpp::Time(prev.header.stamp);
        
        rclcpp::Duration dt = t1 - t2;
        auto leftVel = (msg.left_pos - prev.left_pos) / dt;
        auto rightVel = (msg.right_pos - prev.right_pos) / dt;
        velocityMsg.vy = (leftVel) + (rightVel-leftVel) / 2;
        velocityMsg.w = msg.w;

        velocityPublisher->publish(velocityMsg);
    }

    rclcpp::Subscription<msgs::msg::Brain>::SharedPtr brainSubscriber;
    rclcpp::Publisher<msgs::msg::Velocity>::SharedPtr velocityPublisher;
    
}