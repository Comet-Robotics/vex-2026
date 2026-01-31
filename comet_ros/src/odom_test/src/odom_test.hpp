#define _USE_MATH_DEFINES
#include <iostream>
#include <ostream>
#include <random>
#include <cmath>
#include <algorithm>

class OdomTest
{
    public:
        OdomTest() : numParticles(1) {}

        /**
         * @struct Particle
         * @brief Particle structure representing a single particle in the filter
         */
        struct Particle {
            double x;      ///< The x coordinate of the particle in feet
            double y;      ///< The y coordinate of the particle in feet
            double theta;  ///< The orientation of the particle in radians
            double weight; ///< The weight of the particle
        };

        /**
         * @struct Odometry
         * @brief Odometry structure representing robot motion data
         */
        struct Odometry {
            double vx;      ///< Linear velocity in the x direction (feet per second)
            double vy;      ///< Linear velocity in the y direction (feet per second)
            double w;       ///< Angular velocity (radians per second)
        };

        /**
        * Initialize particles randomly within the map boundaries
        * @param x  initial x position
        * @param y  initial y position
        * @param heading  initial heading in degrees
        * @return A vector of initialized particles
        */
        Particle initializeParticle(
            double x = 0.0, 
            double y = 0.0,
            double theta = 0.0
        ) {
            Particle p;
            p.x = x;
            p.y = y;
            p.theta = theta * (M_PI / 180.0); // convert to radians
            return p;
        }

        /**
        * Get the pose of a given particle
        * @param p The particle whose pose is to be retrieved
        * @return A vector containing the x, y, and theta of the particle
        * @note theta is in radians
        */
        std::vector<double> getParticlePose(const Particle &p) {
            std::vector<double> pose;
            pose.push_back(p.x);
            pose.push_back(p.y);
            pose.push_back(p.theta);
            return pose;
        }

        /**
        * Predict particle states based on odometry data
        * @param particles The vector of particles to predict
        * @param odom The odometry data
        * @param dt The time delta for prediction
        * @param xy_sigma The standard deviation for position noise
        * @param theta_sigma The standard deviation for orientation noise
        * @param vxy_sigma The standard deviation for linear velocity noise
        * @param gamma_sigma The standard deviation for angular velocity noise
        * @return A vector of predicted particles
        */
        Particle predictParticle(
            const Particle &particle,
            const Odometry &odom,
            const double dt
        ) {
            Particle pPred = particle;

            double cosT = std::cos(particle.theta);
            double sinT = std::sin(particle.theta);

            double dx = (odom.vx * cosT - odom.vy * sinT) * dt;
            double dy = (odom.vx * sinT + odom.vy * cosT) * dt;

            pPred.x = particle.x + dx;
            pPred.y = particle.y + dy;
            pPred.theta = angleNormalize(particle.theta + odom.w * dt);

            return pPred;
        }

        /**
         * Convert meters to feet
         * @param meters The value in meters
         * @return The value converted to feet
         */
        double metersToFeet(double meters)
        {
            return meters * 3.28084;
        }

    private:
        const int32_t numParticles;

        /**
         * Normalize an angle to the range [0, 2*pi)
         * @param a The angle to normalize
         * @return The normalized angle
         */
        double angleNormalize(double a) {
            while (a < 0) a += 2.0*M_PI;
            while (a >=  2.0*M_PI) a -= 2.0*M_PI;
            return a;
        }
};

