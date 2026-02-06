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
            resampled_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("pf_resampled_scan", 10);
            simulated_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("pf_simulated_scan", 10);
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
            if (particles_.empty()) {
                particles_ = pf.initializeParticles(0.0, 0.0, 0.0);
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

            sensor_msgs::msg::LaserScan resampledScan = pf.resampleLaserScan(*msg, 0.0, 360.0, pf.getNumBeams());

            resampled_scan_publisher->publish(resampledScan);

            // Simulate laser scan and publish to visualize in Foxglove
            ParticleFilter::LaserScan simulatedScan = pf.lidar_scan(ParticleFilter::Particle{0.0, 0.0, 0.0, 0.0});
            // convert to ROS LaserScan message
            sensor_msgs::msg::LaserScan simulatedScanMsg;
            simulatedScanMsg.header.stamp = this->now();
            simulatedScanMsg.header.frame_id = "laser_frame";
            simulatedScanMsg.angle_min = resampledScan.angle_min;
            simulatedScanMsg.angle_max = resampledScan.angle_max + resampledScan.angle_increment * resampledScan.ranges.size(); // ensure we cover the full 360 degrees
            simulatedScanMsg.angle_increment = resampledScan.angle_increment;
            simulatedScanMsg.time_increment = msg->time_increment;
            simulatedScanMsg.scan_time = msg->scan_time;
            simulatedScanMsg.range_min = msg->range_min;
            simulatedScanMsg.range_max = msg->range_max;
            for (const auto &range : simulatedScan.ranges) {
                simulatedScanMsg.ranges.push_back(range);
            }
            simulated_scan_publisher->publish(simulatedScanMsg);

            ParticleFilter::LaserScan pfScan;
            for (const auto &range : resampledScan.ranges) {
                pfScan.ranges.push_back(pf.metersToFeet(range));
            }
            pfScan.angle_min = resampledScan.angle_min;
            pfScan.angle_increment = resampledScan.angle_increment;            

            particles_ = pf.weightParticles(pfScan, particles_);
            particles_ = pf.resampleParticles(particles_);

            std::vector<double> poseEstimate = pf.estimatePose(particles_);

            geometry_msgs::msg::Pose2D pose2D;
            pose2D.x = poseEstimate[0];
            pose2D.y = poseEstimate[1];
            pose2D.theta = poseEstimate[2] * (180.0 / M_PI); // convert to degrees
            pose_publisher->publish(pose2D);

        }

        ParticleFilter pf;
        std::vector<ParticleFilter::Particle> particles_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robotTwistSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr resampled_scan_publisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr simulated_scan_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}