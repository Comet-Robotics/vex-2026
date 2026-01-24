#define _USE_MATH_DEFINES
#include <iostream>
#include <ostream>
#include <random>
#include <cmath>
#include <algorithm>

class ParticleFilter
{
    public:
        ParticleFilter() : numParticles(100), maxScanRange(metersToFeet(6.0)), numBeams(20), particleDropFraction(0.3) {}
                                                        // 6 meters max range
        struct Point {
            double x, y;
        };

        struct LineSegment {
            double x1, y1, x2, y2;
        };

        struct Map {
            std::vector<LineSegment> walls;

            Map() {
                // center is at (0,0), size is 12ft x 12ft
                walls = {
                    {-6.0, -6.0, 6.0, -6.0},   // bottom wall
                    {6.0, -6.0, 6.0, 6.0},     // right wall
                    {6.0, 6.0, -6.0, 6.0},     // top wall
                    {-6.0, 6.0, -6.0, -6.0}    // left wall
                };
            }
        };

        struct Particle {
            double x;
            double y;
            double theta;
            double weight;
        };

        struct Odometry {
            double vx;
            double vy;
            double w;
        };

        struct LaserScan {
            std::vector<float> ranges;
            double angle_min, angle_increment;
        };

        /**
        * Resample a laser scan to a new angular resolution
        * @param in The input laser scan
        * @param new_min_angle The minimum angle of the resampled scan
        * @param new_max_angle The maximum angle of the resampled scan
        * @param num_beams The number of beams in the resampled scan
        * @return The resampled laser scan
        */
        sensor_msgs::msg::LaserScan resampleLaserScan(
            const sensor_msgs::msg::LaserScan &in,
            float new_min_angle,
            float new_max_angle,
            int num_beams) 
        {

            std::cout << "Resampling laser scan from "
                 << in.angle_min << " to " << in.angle_max
                 << " with " << in.ranges.size() << " beams "
                 << "to new scan from " << new_min_angle << " to " << new_max_angle
                 << " with " << num_beams << " beams." << std::endl;

            sensor_msgs::msg::LaserScan out;

            // Copy basic metadata
            out.header = in.header;
            out.range_min = in.range_min;
            out.range_max = in.range_max;

            // Define output scan properties
            out.angle_min = new_min_angle;
            out.angle_max = new_max_angle;
            out.angle_increment = (new_max_angle - new_min_angle) / (num_beams - 1);

            out.ranges.resize(num_beams);
            out.intensities.resize(num_beams);

            // For each output beam, compute the corresponding input index
            for (int i = 0; i < num_beams; i++) {
                float angle = new_min_angle + i * out.angle_increment;

                // Convert angle -> original index
                float idx_f = (angle - in.angle_min) / in.angle_increment;
                int idx = static_cast<int>(std::round(idx_f));

                // Bounds check
                if (idx >= 0 && idx < (int)in.ranges.size()) {
                    out.ranges[i] = in.ranges[idx];
                    if (!in.intensities.empty())
                        out.intensities[i] = in.intensities[idx];
                    else
                        out.intensities[i] = 0.0f;
                } else {
                    // Out of original bounds — set max range or NaN
                    out.ranges[i] = out.range_max;
                    out.intensities[i] = 0.0f;
                }
            }

            return out;
        }

        /**
        * Initialize particles randomly within the map boundaries
        * @param heading Optional fixed heading for all particles; if -1.0, random headings are assigned [0, 360)
        * @return A vector of initialized particles
        */
        std::vector<Particle> initializeParticles(double heading = -1.0)
        {
            std::vector<Particle> particles;
            for (int i = 0; i < numParticles; i++) {
                Particle p;
                p.x = static_cast<double>(rand()) / RAND_MAX * 12.0 - 6.0;
                p.y = static_cast<double>(rand()) / RAND_MAX * 12.0 - 6.0;
                p.theta = (heading == -1.0) ? static_cast<double>(rand()) / RAND_MAX * 2.0 * M_PI : heading;
                p.weight = 1.0;
                particles.push_back(p);
            }
            return particles;
        }

        /**
        * Simulate a LIDAR scan from a given particle
        * @param p The particle from which the LIDAR scan is simulated
        * @param noise The measurement noise to be added
        * @return A LaserScan struct containing the simulated scan data
        */
        LaserScan lidar_scan(Particle p) {
            LaserScan scan;
            Point ray_start{p.x, p.y};
            double minAngle = 0.0;
            double angleIncrement = (2.0 * M_PI) / numBeams;
            scan.angle_min = minAngle;
            scan.angle_increment = angleIncrement;

            for (int i = 0; i < numBeams; i++) {
                double angle = p.theta + minAngle + i * angleIncrement;
                double min_dist = maxScanRange;
                Point ray_end{
                    p.x + std::cos(angle) * maxScanRange,
                    p.y + std::sin(angle) * maxScanRange
                };

                for (const auto &wall : map_.walls) {
                    auto intersection = findIntersection(ray_start.x, ray_start.y, ray_end.x, ray_end.y,
                                                        wall.x1, wall.y1, wall.x2, wall.y2);
                    if (!intersection.empty()) {
                        double d = std::hypot(intersection[0] - p.x, intersection[1] - p.y);
                        if (d < min_dist) {
                            min_dist = d;
                        }
                    }
                }

                scan.ranges.push_back(static_cast<float>(min_dist));
            }
            return scan;
        }

        /**
        * Predict particle states based on odometry data
        * @param particles The vector of particles to predict
        * @param odom The odometry data
        * @param dt The time interval
        * @param sigmaV The standard deviation of the linear velocity noise
        * @param sigmaW The standard deviation of the angular velocity noise
        * @return A vector of predicted particles
        */
        std::vector<Particle> predictParticles(const std::vector<Particle> &particles, 
                                                const Odometry &odom,
                                                const double dt,
                                                const double xy_sigma = 0.01,
                                                const double theta_sigma = 0.01,
                                                const double vxy_sigma = 0.05,
                                                const double gamma_sigma = 0.02)
        {
            std::vector<Particle> predictedParticles;
            predictedParticles.reserve(particles.size());
            for (const auto &p : particles) {
                Particle pPred = p;

                double vx_noise = gaussianDistribution(0, vxy_sigma);
                double vy_noise = gaussianDistribution(0, vxy_sigma);
                double gamma_noise = gaussianDistribution(0, gamma_sigma);
                double x_jitter = gaussianDistribution(0, xy_sigma);
                double y_jitter = gaussianDistribution(0, xy_sigma);
                double theta_jitter = gaussianDistribution(0, theta_sigma);

                double vxb = odom.vx + vx_noise;
                double vyb = odom.vy + vy_noise;
                double wb = odom.w + gamma_noise;

                // transform body-frame delta to world-frame using current heading
                // dx_body = vxb * dt ; dy_body = vyb * dt
                double dx_world = std::cos(p.theta) * (vxb * dt) - std::sin(p.theta) * (vyb * dt);
                double dy_world = std::sin(p.theta) * (vxb * dt) + std::cos(p.theta) * (vyb * dt);
                double dtheta = wb * dt;

                double x_new = p.x + dx_world + x_jitter;
                double y_new = p.y + dy_world + y_jitter;
                double theta_new = angleNormalize(p.theta + dtheta + theta_jitter);
                // double theta_new = angleNormalize(p.theta + odom.w * dt + theta_jitter);

                pPred.x = x_new;
                pPred.y = y_new;
                pPred.theta = theta_new;

                predictedParticles.push_back(pPred);
            }
            return predictedParticles;
        }

        /**
        * Weight particles based on laser scan measurements
        * @param scan The laser scan data
        * @param particles The vector of particles to weight
        * @return A vector of weighted particles
        */
        std::vector<Particle> weightParticles(const LaserScan &scan, 
                                            const std::vector<Particle> &particles)
        {

            double sigma = metersToFeet(0.067); // A1M8 lidar usually has a noise of 2-3 cm
            int N = particles.size();
            std::vector<double> logw(N, 0.0);
            std::vector<double> errs;
            std::vector<double> abs_errs;

            for (int i = 0; i < N; i++) {
                const auto& p = particles[i];
                auto z_hat_data = lidar_scan(p);
                errs.clear();
                abs_errs.clear();

                for (int k = 0; k < numBeams; k++) {
                    double e = metersToFeet(scan.ranges[k]) - z_hat_data.ranges[k];
                    errs.push_back(e);
                    abs_errs.push_back(std::abs(e));
                }

                std::vector<double> sorted_abs_errs = abs_errs;
                std::sort(sorted_abs_errs.begin(), sorted_abs_errs.end());
                int threshold_index = static_cast<int>(numBeams * (1 - particleDropFraction));
                double threshold = sorted_abs_errs[threshold_index];

                // Compute log weight
                double ll = 0.0;
                for (size_t k = 0; k < errs.size(); ++k) {
                    if (abs_errs[k] < threshold) {
                        ll += -0.5 * (errs[k] * errs[k]) / (sigma * sigma);
                    }
                }
                if (!inFreeSpace(p.x, p.y)) {
                    ll -= 1e9; // Heavy penalty for particles outside free space
                }
                logw[i] = ll;
            }

            // Reweight particles using log weights
            double m = *std::max_element(logw.begin(), logw.end());
            std::vector<Particle> weightedParticles(N);

            for (int i = 0; i < N; i++) {
                double w = std::exp(logw[i] - m);
                weightedParticles[i] = particles[i];
                weightedParticles[i].weight = w;
            }

            return normalizeParticles(weightedParticles);
        }

        /**
        * Normalize particle weights so that they sum to 1
        * @param particles The vector of particles to normalize
        * @return A vector of particles with normalized weights
        */
        std::vector<Particle> normalizeParticles(const std::vector<Particle> &particles)
        {
            std::vector<Particle> normalizedParticles = particles;
            double sumWeights = 0.0;
            for (const auto &p : normalizedParticles) {
                sumWeights += p.weight;
            }
            if (sumWeights > 0) {
                for (auto &p : normalizedParticles) {
                    p.weight /= sumWeights;
                }
            } else {
                double uniformWeight = 1.0 / normalizedParticles.size();
                for (auto &p : normalizedParticles) {
                    p.weight = uniformWeight;
                }
                std::cout << "Warning: All particle weights are zero or negative. Resetting to uniform weights." << std::endl;
            }
            return normalizedParticles;
        }

        /**
        * Resample particles based on their weights using low-variance resampling
        * @param particles The vector of particles to resample
        * @return A vector of resampled particles
        */
       std::vector<Particle> resampleParticles(const std::vector<Particle> &particles)
       {
            int N = static_cast<int>(particles.size());
            std::vector<Particle> new_particles;
            new_particles.reserve(N);

            // 1. Create a random starting point (r) between 0 and 1/N
            std::uniform_real_distribution<double> dist(0.0, 1.0 / N);
            double r = dist(gen);
            
            // 2. The "Wheel" logic
            double c = particles[0].weight; // Cumulative weight sum
            int i = 0;

            for (int m = 0; m < N; m++) {
                // U is the current "pointer" on the wheel
                double U = r + static_cast<double>(m) / N;
                
                // Move through the weights until we find the particle that spans U
                while (U > c && i < N - 1) {
                    i++;
                    c += particles[i].weight;
                }
                new_particles.push_back(particles[i]);
            }

            return new_particles;
        }
        // std::vector<Particle> resampleParticles(const std::vector<Particle> &particles)
        // {
        //     std::vector<Particle> new_particles;
            
        //     new_particles.reserve(particles.size());
        //     std::vector<double> weights;
        //     weights.reserve(particles.size());
        //     for (const auto &p : particles) {
        //         weights.push_back(p.weight);
        //     }

        //     /*
        //     Create a discrete distribution based on particle weights
        //     This generates indices according to the weights, where 
        //     particles with higher weights are more likely to be chosen
        //     */
        //     std::discrete_distribution<int> dist(
        //         weights.begin(), weights.end()
        //     );

        //     for (size_t i = 0; i < particles.size(); ++i) {
        //         new_particles.push_back(particles[dist(gen)]);
        //     }

        //     return new_particles;
        // }

        /**
        * Estimate the robot's pose based on the weighted particles
        * @param particles The vector of particles
        * @return A vector containing the estimated x, y, and theta
        */
        std::vector<double> estimatePose(const std::vector<Particle> &particles)
        {
            double weight_sum = 0.0;
            for (const auto& p : particles) {
                weight_sum += p.weight;
            }

            double x = 0.0;
            double y = 0.0;
            double sin = 0.0; 
            double cos = 0.0;
            for (const auto &p : particles) {
                double w = p.weight / weight_sum;
                x += p.x * w;
                y += p.y * w;
                sin += std::sin(p.theta) * w;
                cos += std::cos(p.theta) * w;
            }
            double theta = std::atan2(sin, cos);
            return {x, y, theta};
        }

    private:
        const int32_t numParticles;
        const double maxScanRange;
        const int32_t numBeams;
        const double particleDropFraction = 0.7; // Fraction of particles to drop
        const Map map_;

        std::mt19937 gen{std::random_device{}()};

        /**
         * Generate a random number based on a Gaussian distribution
         * @param mu The mean of the distribution
         * @param sigma The standard deviation of the distribution
         * @return A random number following the Gaussian distribution
         */
        double gaussianDistribution(double mu, double sigma) {
            std::normal_distribution<double> dist(mu, sigma);
            return dist(gen);
        }

        /**
        * Find the intersection point between a ray and a wall segment
        * @param startX The x coordinate of the ray start point
        * @param startY The y coordinate of the ray start point
        * @param endX The x coordinate of the ray end point
        * @param endY The y coordinate of the ray end point
        * @param wallX1 The x coordinate of the first wall endpoint
        * @param wallY1 The y coordinate of the first wall endpoint
        * @param wallX2 The x coordinate of the second wall endpoint
        * @param wallY2 The y coordinate of the second wall endpoint
        * @return A vector containing the intersection point coordinates if an intersection exists, otherwise an empty vector
        */
        std::vector<double> findIntersection(double startX, double startY, double endX, double endY,
                                            double wallX1, double wallY1, double wallX2, double wallY2)
        {
            double dxRay = endX - startX;
            double dyRay = endY - startY;
            double dxWall = wallX2 - wallX1;
            double dyWall = wallY2 - wallY1;

            double denominator = dxRay * dyWall - dyRay * dxWall;
            if (denominator == 0) {
                return {}; // Parallel lines
            }
            double t = ((startX - wallX1) * dyWall - (startY - wallY1) * dxWall) / denominator;
            double u = -((startX - wallX1) * dyRay - (startY - wallY1) * dxRay) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                double intersectionX = startX + t * dxRay;
                double intersectionY = startY + t * dyRay;
                return {intersectionX, intersectionY};
            }
            return {}; // No intersection within the segments
        }

        /**
         * Check if a point is within the free space of the map
         * @param x The x coordinate of the point
         * @param y The y coordinate of the point
         * @return True if the point is in free space, false otherwise
         */
        bool inFreeSpace(double x, double y) {
            return (x >= -6.0 && x <= 6.0 && y >= -6.0 && y <= 6.0);
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

        /**
         * Calculate the Gaussian weight for a given value
         * @param mu The mean of the distribution
         * @param sigma The standard deviation of the distribution
         * @param x The value for which the weight is calculated
         * @return The Gaussian weight
         */
        double gaussianWeight(double mu, double sigma, double x)
        {
            if (std::isinf(mu)) {
                mu = maxScanRange;
            }
            if (std::isinf(x)) {
                x = maxScanRange;
            }
            return (1.0 / (sigma * std::sqrt(2.0 * M_PI))) * 
                   std::exp(-0.5 * std::pow((x - mu) / sigma, 2));
        }

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

