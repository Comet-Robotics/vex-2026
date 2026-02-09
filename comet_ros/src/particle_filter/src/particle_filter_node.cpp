#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "particle_filter.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "msgs/msg/robot.hpp"

class ParticleFilterNode : public rclcpp::Node
{
    public:
        ParticleFilterNode()
        : Node("particle_filter_node"),
            x_dist(0.0, 0.03), // mean 0, stddev 0.03 feet
            y_dist(0.0, 0.02), // mean 0, stddev 0.02 feet
            theta_dist(0.0, 0.01), // mean 0, stddev 0.01 radians
            gen(std::random_device{}())
        {
            robotTwistSubscriber= this->create_subscription<msgs::msg::Robot>(
                "robot", 10, std::bind(&ParticleFilterNode::odomTest, this, std::placeholders::_1));
            scanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1));
            pose_publisher = this->create_publisher<geometry_msgs::msg::Pose2D>("pf_pose_geometry", 10);
            resampled_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("pf_resampled_scan", 10);
            simulated_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("pf_simulated_scan", 10);
            marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("pf_markers", 10);
        }
    private:
        rclcpp::Time lastTwistTime_;
        bool firstTwistReceived_ = false;
        ParticleFilter::Odometry latestTwist_;
        double x = 0.0, y = 0.0, theta = 0.0; // robot's estimated pose in feet and radians
        std::normal_distribution<double> x_dist;
        std::normal_distribution<double> y_dist;
        std::normal_distribution<double> theta_dist;
        std::mt19937 gen;

        /**
        * Callback for wheel odometry messages
        * @param msg The received odometry message
        */
        void odomCallback(const msgs::msg::Robot::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received odom data:");
            
            double dt = 0.05; // seconds
            if (firstTwistReceived_) {
                dt = (this->now() - lastTwistTime_).seconds(); // seconds as double
            } else {
                firstTwistReceived_ = true;
            }

            lastTwistTime_ = this->now();

            latestTwist_.vx = msg->twist.linear.x;
            latestTwist_.vy = msg->twist.linear.y;
            latestTwist_.w = msg->twist.angular.z;

            // update particles with actual dt
            if (particles_.empty()) {
                double startingX = msg->starting_pose.x;
                double startingY = msg->starting_pose.y;
                double startingTheta = msg->starting_pose.theta;
                particles_ = pf.initializeParticles(startingX, startingY, startingTheta);
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
            if (particles_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Particles not initialized, skipping scan callback");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Received scan data");

            sensor_msgs::msg::LaserScan resampledScan = pf.resampleLaserScan(*msg, 0.0, 360.0);

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

        // test odometry before implementing particle filter
        void odomTest(const msgs::msg::Robot::SharedPtr msg) {
            double dt = 0.05; // seconds
            if (firstTwistReceived_) {
                dt = (this->now() - lastTwistTime_).seconds(); // seconds as double
            } else {
                firstTwistReceived_ = true;
            }
            lastTwistTime_ = this->now();

            double noiseX = x_dist(gen);
            double noiseY = y_dist(gen);
            double noiseTheta = theta_dist(gen);

            double noisyOdomVx = msg->twist.linear.x + noiseX;
            double noisyOdomVy = msg->twist.linear.y + noiseY;
            double noisyOdomW = msg->twist.angular.z + noiseTheta;

            // Use a simple motion model with better integration for more accurate curves
            double theta_mid = theta + 0.5 * msg->twist.angular.z * dt; // Midpoint for better integration

            double cosT = std::cos(theta_mid);
            double sinT = std::sin(theta_mid);

            double dx = (noisyOdomVx * cosT - noisyOdomVy * sinT) * dt;
            double dy = (noisyOdomVx * sinT + noisyOdomVy * cosT) * dt;
            double dtheta = noisyOdomW * dt;

            x += dx;
            y += dy;
            theta = angleNormalize(theta + dtheta);

            RCLCPP_INFO(this->get_logger(), "Twist: vx=%.2f, vy=%.2f, w=%.2f | Pose: x=%.2f, y=%.2f, theta=%.2fdeg (%2frad)", 
                msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z, 
                x, y, theta * (180.0 / M_PI), theta);

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
            markerMsg.pose.position.x = x;
            markerMsg.pose.position.y = y;
            markerMsg.pose.position.z = 0.0;
            
            geometry_msgs::msg::Quaternion q;
            q.x = 0.0;
            q.y = 0.0;
            q.z = std::sin(theta / 2.0);
            q.w = std::cos(theta / 2.0);
            markerMsg.pose.orientation = q;

            markerMsg.lifetime = rclcpp::Duration(0, 0);
            marker_publisher->publish(markerMsg);
        }

        double angleNormalize(double a) {
            while (a < 0) a += 2.0*M_PI;
            while (a >= 2.0*M_PI) a -= 2.0*M_PI;
            return a;
        }

        ParticleFilter pf;
        std::vector<ParticleFilter::Particle> particles_;
        rclcpp::Subscription<msgs::msg::Robot>::SharedPtr robotTwistSubscriber;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr resampled_scan_publisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr simulated_scan_publisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}