#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "particle_filter.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ParticleFilterNode : public rclcpp::Node
{
    public:
        ParticleFilterNode()
        : Node("particle_filter_node")
        {
            particles_ = pf.initializeParticles();

            robotTwistSubscriber= this->create_subscription<geometry_msgs::msg::Twist>(
                "robot_twist", 10, std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));
            scanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));
            pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>("pf_pose_geometry", 10);
        }
    private:
        rclcpp::Time lastTwistTime_;
        bool firstTwistReceived_ = false;
        ParticleFilter::Odometry latestTwist_;

        /**
        * Callback for wheel odometry messages
        * @param msg The received odometry message
        */
        void odomCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received odom data:");

            rclcpp::Time currentTime = this->now();
            
            double dt = 0.05;
            if (firstTwistReceived_) {
                dt = (currentTime - lastTwistTime_).seconds(); // seconds as double
            } else {
                firstTwistReceived_ = true;
            }

            lastTwistTime_ = currentTime;

            latestTwist_.vx = msg->linear.x;
            latestTwist_.vy = msg->linear.y;
            latestTwist_.w = msg->angular.z;

            // update particles with actual dt
            if (particles_.empty()) {
                particles_ = pf.initializeParticles();
            } else {
                particles_ = pf.predictParticles(particles_, latestTwist_, dt);
            }
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

            geometry_msgs::msg::Pose2D pose2D;
            pose2D.x = poseEstimate[0];
            pose2D.y = poseEstimate[1];
            pose2D.theta = poseEstimate[2];
            pose_publisher->publish(pose2D);

        }

        ParticleFilter pf;
        std::vector<ParticleFilter::Particle> particles_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robotTwistSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}