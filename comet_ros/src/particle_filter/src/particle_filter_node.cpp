#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "particle_filter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ParticleFilterNode : public rclcpp::Node
{
    public:
        ParticleFilterNode()
        : Node("particle_filter_node")
        {
            odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));
            scanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));
            publisher = this->create_publisher<nav_msgs::msg::Odometry>("particle_filter_estimate", 10);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10);

            double hz = 20.0;                 // <<< choose your rate
            double period = 1.0 / hz;

            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(period),
                std::bind(&ParticleFilterNode::timerCallback, this)
            );
        }
    private:
        rclcpp::Time lastOdomTime_;
        bool firstOdomReceived_ = false;
        ParticleFilter::Odometry latestOdom_;
        void odomCallback(const nav_msgs::msg::Odometry::ConstPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Receieved odom data");
            
            rclcpp::Time currentTime = msg->header.stamp;

            double dt = 0.05; // default fallback
            if (firstOdomReceived_) {
                dt = (currentTime - lastOdomTime_).seconds(); // seconds as double
            } else {
                firstOdomReceived_ = true;
            }

            lastOdomTime_ = currentTime;

            latestOdom_.vx = msg->twist.twist.linear.x;
            latestOdom_.vy = msg->twist.twist.linear.y;
            latestOdom_.w  = msg->twist.twist.angular.z;

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

        void scanCallback(const sensor_msgs::msg::LaserScan::ConstPtr msg)
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

        void timerCallback()
        {
            auto poseEstimate = pf.estimatePose(particles_);

            geometry_msgs::msg::PoseStamped poseMsg;
            poseMsg.header.stamp = this->now();
            odomMsg.header.frame_id = "map";
            odomMsg.child_frame_id = "base_link";

            poseMsg.pose.position.x = poseEstimate[0];
            poseMsg.pose.position.y = poseEstimate[1];
            
            tf2::Quaternion q;
            q.setRPY(0, 0, poseEstimate[2]);
            poseMsg.pose.orientation = tf2::toMsg(q);
            
            pose_pub_->publish(poseMsg);
        }
        
        ParticleFilter pf;
        std::vector<ParticleFilter::Particle> particles_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}