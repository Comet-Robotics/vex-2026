#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "particle_filter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "msgs/msg/velocity.hpp"

class ParticleFilterNode : public rclcpp::Node
{
    public:
        ParticleFilterNode()
        : Node("particle_filter_node")
        {
            particles_ = pf.initializeParticles();

            odomSubscriber = this->create_subscription<msgs::msg::Velocity>(
                "wheel_odometry", 10, std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));
            scanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));
            publisher = this->create_publisher<nav_msgs::msg::Odometry>("particle_filter_estimate", 10);
        }
    private:
        rclcpp::Time lastOdomTime_;
        bool firstOdomReceived_ = false;
        ParticleFilter::Odometry latestOdom_;

        /**
        * Callback for wheel odometry messages
        * @param msg The received odometry message
        */
        void odomCallback(const msgs::msg::Velocity::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received odom data");

            rclcpp::Time currentTime = msg->header.stamp;

            double dt = 0.05;
            if (firstOdomReceived_) {
                dt = (currentTime - lastOdomTime_).seconds(); // seconds as double
            } else {
                firstOdomReceived_ = true;
            }

            lastOdomTime_ = currentTime;

            latestOdom_.vx = msg->vx;
            latestOdom_.vy = msg->vy;
            latestOdom_.w  = msg->w;

            // update particles with actual dt
            if (particles_.empty()) {
                particles_ = pf.initializeParticles();
            } else {
                particles_ = pf.predictParticles(particles_, latestOdom_, dt);
            }

            // log data if u want
            // double x = msg->pose.pose.position.x;
            // double y = msg->pose.pose.position.y;
            // double theta = pf.yawFromQuaternion(
            //     msg->pose.pose.orientation.x,
            //     msg->pose.pose.orientation.y,
            //     msg->pose.pose.orientation.z,
            //     msg->pose.pose.orientation.w);
            // RCLCPP_INFO(this->get_logger(), "Received odom position: x='%f', y='%f', theta='%f'", x, y, theta);

            // double dx = msg->twist.twist.linear.x;
            // double dy = msg->twist.twist.linear.y;
            // double w = msg->twist.twist.angular.z;
            // RCLCPP_INFO(this->get_logger(), "Received odom velocities: dx='%f', dy='%f', w='%f'", dx, dy, w);
        }

        /**
        * Callback for laser scan messages
        * @param msg The received laser scan message
        */
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received scan data");

            ParticleFilter::LaserScan pfScan;
            pfScan.ranges = msg->ranges;
            pfScan.angle_min = msg->angle_min;
            pfScan.angle_increment = msg->angle_increment;

            particles_ = pf.weightParticles(pfScan, particles_);
            particles_ = pf.resampleParticles(particles_);

            std::vector<double> poseEstimate = pf.estimatePose(particles_);

            nav_msgs::msg::Odometry odomMsg;
            odomMsg.header.stamp = msg->header.stamp;
            odomMsg.pose.pose.position.x = poseEstimate[0];
            odomMsg.pose.pose.position.y = poseEstimate[1];
            odomMsg.pose.pose.orientation.z = sin(poseEstimate[2] / 2.0);
            odomMsg.pose.pose.orientation.w = cos(poseEstimate[2] / 2.0);
            publisher->publish(odomMsg);
        }
        
        ParticleFilter pf;
        std::vector<ParticleFilter::Particle> particles_;
        rclcpp::Subscription<msgs::msg::Velocity>::SharedPtr odomSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}