#include "rclcpp/rclcpp.hpp"
#include "odom_test.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/duration.hpp"

class OdomTestNode : public rclcpp::Node
{
    public:
        OdomTestNode()
        : Node("odom_test_node")
        {
            particle_ = pf.initializeParticle(0.0, 0.0, 0.0);

            robotTwistSubscriber= this->create_subscription<geometry_msgs::msg::Twist>(
                "robot_twist", 10, std::bind(&OdomTestNode::odomCallback, this, std::placeholders::_1));
            pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>("odom_test_pose_geometry", 10);
            marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("odom_test_pose_marker", 10);
        }
    private:
        rclcpp::Time lastTwistTime_;
        bool firstTwistReceived_ = false;
        OdomTest::Odometry latestTwist_;

        /**
        * Callback for wheel odometry messages
        * @param msg The received odometry message
        */
        void odomCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received odom data:");
            
            double dt = 0.05; // seconds
            if (firstTwistReceived_) {
                dt = (this->now() - lastTwistTime_).seconds(); // seconds as double
            } else {
                firstTwistReceived_ = true;
            }

            lastTwistTime_ = this->now();

            latestTwist_.vx = msg->linear.x;
            latestTwist_.vy = msg->linear.y;
            latestTwist_.w = msg->angular.z;

            // update particles with actual dt
            particle_ = pf.predictParticle(particle_, latestTwist_, dt);
            
            // Publish the particle pose
            auto pose_msg = geometry_msgs::msg::Pose2D();
            pose_msg.x = particle_.x;
            pose_msg.y = particle_.y;
            pose_msg.theta = particle_.theta;
            pose_publisher->publish(pose_msg);

            visualization_msgs::msg::Marker markerMsg;
            markerMsg.header.frame_id = "map";
            markerMsg.ns = "pose_arrow";
            markerMsg.id = 0;
            markerMsg.type = visualization_msgs::msg::Marker::ARROW;
            markerMsg.action = visualization_msgs::msg::Marker::ADD;
            markerMsg.scale.x = 0.2;
            markerMsg.scale.y = 0.05;
            markerMsg.scale.z = 0.05;
            markerMsg.color.a = 1.0;
            markerMsg.color.r = 1.0;
            markerMsg.color.g = 0.0;
            markerMsg.color.b = 0.0;
            markerMsg.pose.position.x = particle_.x; // convert from inches to feet
            markerMsg.pose.position.y = particle_.y; // convert from inches to feet
            markerMsg.pose.position.z = 0.0;
            
            geometry_msgs::msg::Quaternion q;
            q.x = 0.0;
            q.y = 0.0;
            q.z = std::sin(particle_.theta / 2.0);
            q.w = std::cos(particle_.theta / 2.0);
            markerMsg.pose.orientation = q;

            markerMsg.lifetime = rclcpp::Duration(0, 0);
            marker_publisher->publish(markerMsg);
        }


        OdomTest pf;
        OdomTest::Particle particle_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robotTwistSubscriber;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTestNode>());
    rclcpp::shutdown();
    return 0;
}