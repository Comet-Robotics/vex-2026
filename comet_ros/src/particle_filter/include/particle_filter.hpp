#pragma once
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ParticleFilter {
    public:
        ParticleFilter();
        struct Particle {
            double x, y, theta, weight;
        };
        struct LineSegment {
            double x1, y1, x2, y2;
        };
        struct Map {
            std::vector<LineSegment> walls;
            Map();
        };
        struct Odometry {
            double vx, vy, w;
        };
        struct LaserScan {
            std::vector<float> ranges;
            double angle_min, angle_increment;
        };

        std::vector<Particle> initializeParticles();
        std::vector<Particle> predictParticles(const std::vector<Particle> &particles,
                                            const Odometry &odom,
                                            const double dt,
                                            const double xyNoiseMax = 0.01,
                                            const double thetaNoiseMax = 0.005,
                                            const double sigmaV = 0.01,
                                            const double sigmaW = 0.01,
                                            const double sigmaGamma = 0.002);
        std::vector<Particle> weightParticles(const LaserScan &scan, 
                                            const std::vector<Particle> &particles);
        std::vector<Particle> resampleParticles(const std::vector<Particle> &particles);
        std::vector<double> estimatePose(const std::vector<Particle> &particles);
        double yawFromQuaternion(double x, double y, double z, double w);
    
    private:
        const int32_t numParticles;
        const double maxScanRange;
        const Map map_;
        
        double gaussianDistribution(double mu, double sigma);
        std::vector<double> findIntersection(double startX, double startY, double endX, double endY,
                                            double wallX1, double wallY1, double wallX2, double wallY2);
        double simulateRay(const Particle &particle, const double angle);
        double metersToFeet(double meters);
        double gaussianWeight(double mu, double sigma, double x);
};