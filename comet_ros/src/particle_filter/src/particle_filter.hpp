#define _USE_MATH_DEFINES
#include <iostream>
#include <ostream>
#include <random>
#include <cmath>
#include <algorithm>

class ParticleFilter
{
    public:
        ParticleFilter() : numParticles(100), maxScanRange(metersToFeet(6.0)), numBeams(20), particleDropFraction(0.7) {}
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
                // center is at (0,0)
                
                // real map (12ft x 12ft)
                // walls = {
                //     {-6.0, -6.0, 6.0, -6.0},   // bottom wall
                //     {6.0, -6.0, 6.0, 6.0},     // right wall
                //     {6.0, 6.0, -6.0, 6.0},     // top wall
                //     {-6.0, 6.0, -6.0, -6.0}    // left wall
                // };

                // smaller map for testing (6ft x 6ft)
                walls = {
                    {-3.0, -3.0, 3.0, -3.0},   // bottom wall
                    {3.0, -3.0, 3.0, 3.0},     // right wall
                    {3.0, 3.0, -3.0, 3.0},     // top wall
                    {-3.0, 3.0, -3.0, -3.0}    // left wall
                };
            }
        };

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
         * @struct LaserScan
         * @brief LaserScan structure representing a LIDAR scan
         */
        struct LaserScan {
            std::vector<float> ranges;  ///< Distance measurements from the LIDAR in feet
            double angle_min;           ///< Minimum angle of the scan in radians
            double angle_increment;     ///< Angle increment between measurements in radians
        };


        // this caused the resampled point to follow a point that was 2 points behind it, below is fix
        /**
        * Resample a laser scan to a new angular resolution
        * @param in The input laser scan
        * @param new_min_angle The minimum angle of the resampled scan in degrees
        * @param new_max_angle The maximum angle of the resampled scan in degrees
        * @param num_beams The number of beams in the resampled scan
        * @return The resampled laser scan
        * @note x axis is in the direction of the motor, and y axis is angled 90 degrees counter-clockwise from the motor
        * @note 0 degrees is in the direction of the x axis, or the motor, and angles increase counter-clockwise
        */
        // sensor_msgs::msg::LaserScan resampleLaserScan(
        //     const sensor_msgs::msg::LaserScan &in,
        //     float new_min_angle,
        //     float new_max_angle,
        //     int num_beams
        // ) {

        //     std::cout << "Resampling laser scan: ("
        //          << in.angle_min << " to " << in.angle_max
        //          << "), " << in.ranges.size() << " beams "
        //          << "→ (" << new_min_angle << " to " << new_max_angle
        //          << "), " << num_beams << " beams." << std::endl;

        //     sensor_msgs::msg::LaserScan out;
            
        //     new_min_angle *= (M_PI / 180.0); // Convert to radians for math
        //     new_max_angle *= (M_PI / 180.0);

        //     // Copy basic metadata
        //     out.header = in.header;
        //     out.range_min = in.range_min;
        //     out.range_max = in.range_max;

        //     // Define output scan properties
        //     out.angle_min = new_min_angle;
        //     out.angle_max = new_max_angle;
        //     out.angle_increment = (new_max_angle - new_min_angle) / (num_beams - 1);

        //     minAngle = new_min_angle;
        //     angleIncrement = out.angle_increment;

        //     out.ranges.resize(num_beams);
        //     out.intensities.resize(num_beams);

        //     // For each output beam, compute the corresponding input index
        //     for (int i = 0; i < num_beams; i++) {
        //         float outAngle = new_min_angle + i * out.angle_increment;
        //         // float idx_in_f = fmod((outAngle - in.angle_min) / in.angle_increment, in.ranges.size());


        //         // 1. Calculate how far through the input range the current output angle is (0.0 to 1.0)
        //         float unit_ratio = (outAngle - in.angle_min) / (in.angle_max - in.angle_min);
        //         // 2. Map that ratio to the input array indices (0 to size-1)
        //         float idx_in_f = unit_ratio * (in.ranges.size() - 1);
        //         // idx_in_f = ratio * (in_size - 1) + 0.5f;

        //         // Bounds check
        //         if (idx_in_f >= 0 && idx_in_f < (int)in.ranges.size()) {
        //             // linear interpolation for smoother resampling
        //             int i0 = std::floor(idx_in_f);
        //             int i1 = i0 + 1;
        //             if (i1 >= (int)in.ranges.size()) {
        //                 i1 = 0;
        //             }
        //             float t = idx_in_f - i0;

        //             float r0 = in.ranges[i0];
        //             float r1 = in.ranges[i1];

        //             float r_out = (1 - t) * r0 + t * r1;

        //             out.ranges[i] = r_out;

        //             if (!in.intensities.empty()) {
        //                 out.intensities[i] = in.intensities[i0];
        //             } else {
        //                 std::cerr << "Input scan has no intensities!" << std::endl;
        //             }
        //         } else {
        //             std::cerr << "Index " << idx_in_f << " out of bounds for input scan size " << in.ranges.size() << std::endl;
        //         }
        //     }

        //     // for (size_t i = 0; i < in.ranges.size(); i++) {
        //     //     std::cout << "Input beam " << i << ": angle " << (in.angle_min + i * in.angle_increment) 
        //     //               << ", range " << in.ranges[i] 
        //     //               << ", intensity " << (in.intensities.empty() ? 0.0f : in.intensities[i]) 
        //     //               << std::endl;
        //     // }
        //     return out;
        // }
        sensor_msgs::msg::LaserScan resampleLaserScan(
            const sensor_msgs::msg::LaserScan &in,
            float new_min_angle_deg,
            float new_max_angle_deg,
            int num_beams
        ) {
            sensor_msgs::msg::LaserScan out;
            
            // setup metadata
            out.header = in.header;
            out.range_min = in.range_min;
            out.range_max = in.range_max;
            out.angle_min = new_min_angle_deg * (M_PI / 180.0);
            out.angle_max = new_max_angle_deg * (M_PI / 180.0);

            // calculate output increment (n-1 for inclusive endpoints)
            if (num_beams > 1) {
                out.angle_increment = (out.angle_max - out.angle_min) / (num_beams - 1);
            } else {
                out.angle_increment = 0;
            }

            // initialize output ranges and intensities
            out.ranges.assign(num_beams, std::numeric_limits<float>::quiet_NaN());
            out.intensities.resize(num_beams, 0.0f);

            // cache input properties for efficiency
            float in_span = in.angle_max - in.angle_min;
            int in_size = static_cast<int>(in.ranges.size());

            for (int i = 0; i < num_beams; i++) {
                float outAngle = out.angle_min + i * out.angle_increment;

                // find where angle sits relative to input [0, 1]
                float ratio = (outAngle - in.angle_min) / in_span;
                
                // map ratio to input index space [0, size-1]
                float idx_in_f = fmod(ratio * (in_size - 1) + 0.5f, in_size); // Adding 0.5 for better rounding to nearest index

                // bounds check with a small epsilon for float jitter
                if (idx_in_f >= 0.0f && idx_in_f <= (in_size - 1.0001f)) {
                    int i0 = std::floor(idx_in_f);
                    int i1 = i0 + 1;
                    float t = idx_in_f - i0;

                    // linear Interpolation for smoother resampling
                    float r0 = in.ranges[i0];
                    float r1 = in.ranges[i1];

                    if (std::isfinite(r0) && std::isfinite(r1)) {
                        out.ranges[i] = (1.0f - t) * r0 + t * r1;
                        
                        if (!in.intensities.empty()) {
                            out.intensities[i] = (1.0f - t) * in.intensities[i0] + t * in.intensities[i1];
                        } else {
                            std::cerr << "Input scan has no intensities!" << std::endl;
                        }
                    }
                } else {
                    std::cerr << "Index " << idx_in_f << " out of bounds for input scan size " << in.ranges.size() << std::endl;
                    continue;
                }
            }
            return out;
        }

        /**
        * Initialize particles randomly within the map boundaries
        * @param x Optional initial x position
        * @param y Optional initial y position
        * @param heading Optional initial heading
        * @return A vector of initialized particles
        */
        // std::vector<Particle> initializeParticles(
        //     std::optional<double> x = std::nullopt,
        //     std::optional<double> y = std::nullopt,
        //     std::optional<double> heading = std::nullopt)
        // {
        //     std::vector<Particle> particles;
        //     for (int i = 0; i < numParticles; i++) {
        //         Particle p;
        //         p.x = x.value_or(((double)rand() / RAND_MAX) * 12.0 - 6.0);
        //         p.y = y.value_or(((double)rand() / RAND_MAX) * 12.0 - 6.0);
        //         p.theta = heading.value_or(((double)rand() / RAND_MAX) * 2.0 * M_PI);
        //         p.weight = 1.0;
        //         particles.push_back(p);
        //     }
        //     return particles;
        // }



        // needs testing
        /**
        * Initialize particles randomly within the map boundaries
        * @param x Optional initial x position
        * @param y Optional initial y position
        * @param heading Optional initial heading
        * @return A vector of initialized particles
        */
        std::vector<Particle> initializeParticles(
            std::optional<double> x = std::nullopt,
            std::optional<double> y = std::nullopt,
            std::optional<double> heading = std::nullopt
        ) {
            constexpr double MAP_HALF = 6.0;          // map bounds [-6, 6] ft
            constexpr double BOX_HALF = 0.5;          // 1 ft box → ±0.5
            constexpr double ANG_SPREAD = M_PI / 12;  // ±15°


            // returns 0-1
            auto urand = []() {
                return (double)rand() / RAND_MAX;
            };

            const bool hasXY = x && y;

            std::vector<Particle> particles;
            particles.reserve(numParticles);

            for (int i = 0; i < numParticles; i++) {
                Particle p;

                if (hasXY) {
                    // Uniform 1 ft box around provided pose
                    p.x = *x + (urand() * 2.0 - 1.0) * BOX_HALF;
                    p.y = *y + (urand() * 2.0 - 1.0) * BOX_HALF;
                } else {
                    // Uniform over entire map
                    p.x = (urand() * 2.0 - 1.0) * MAP_HALF;
                    p.y = (urand() * 2.0 - 1.0) * MAP_HALF;
                }

                if (heading) {
                    p.theta = *heading + (urand() * 2.0 - 1.0) * ANG_SPREAD;
                } else {
                    p.theta = urand() * 2.0 * M_PI;
                }

                p.weight = 1.0 / numParticles;
                particles.push_back(p);
            }

            return particles;
        }


        /**
        * Simulate a LIDAR scan from a given particle
        * @param p The particle from which the LIDAR scan is simulated
        * @param noise The measurement noise to be added
        * @return A LaserScan struct containing the simulated scan data
        * @note x axis is in the direction of the motor, and y axis is angled 90 degrees counter-clockwise from the motor
        * @note 0 degrees is in the direction of the x axis, or the motor, and angles increase clockwise
        */
        LaserScan lidar_scan(Particle p) {
            LaserScan scan;
            Point ray_start{p.x, p.y};
            scan.angle_min = minAngle;
            scan.angle_increment = angleIncrement;

            for (int i = 0; i < numBeams; i++) {
                double angle = angleNormalize(p.theta + minAngle + i * angleIncrement);
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
        * @param dt The time delta for prediction
        * @param xy_sigma The standard deviation for position noise
        * @param theta_sigma The standard deviation for orientation noise
        * @param vxy_sigma The standard deviation for linear velocity noise
        * @param gamma_sigma The standard deviation for angular velocity noise
        * @return A vector of predicted particles
        */
        std::vector<Particle> predictParticles(
            const std::vector<Particle> &particles,
            const Odometry &odom,
            const double dt,
            const double x_sigma = 0.03,
            const double y_sigma = 0.02,
            const double theta_sigma = 0.01
        ) {
            std::vector<Particle> predictedParticles;
            predictedParticles.reserve(particles.size());

            for (const auto &p : particles) {
                Particle pPred = p;

                // Noise
                double noiseX = gaussianDistribution(0.0, x_sigma);
                double noiseY = gaussianDistribution(0.0, y_sigma);
                double noiseTheta = gaussianDistribution(0.0, theta_sigma);

                double noisyOdomVx = odom.vx + noiseX;
                double noisyOdomVy = odom.vy + noiseY;
                double noisyOdomW = odom.w + noiseTheta;

                // Use a simple motion model with better integration for more accurate curves
                double theta_mid = p.theta + 0.5 * odom.w * dt; // Midpoint for better integration

                double cosT = std::cos(theta_mid);
                double sinT = std::sin(theta_mid);

                double dx = (noisyOdomVx * cosT - noisyOdomVy * sinT) * dt;
                double dy = (noisyOdomVx * sinT + noisyOdomVy * cosT) * dt;
                double dtheta = noisyOdomW * dt;

                pPred.x = p.x + dx;
                pPred.y = p.y + dy;
                pPred.theta = angleNormalize(p.theta + dtheta);

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
        std::vector<Particle> weightParticles(
            const LaserScan &scan, 
            const std::vector<Particle> &particles
        ) {

            double sigma = metersToFeet(0.067); // A1M8 lidar usually has a noise of 2-3 cm
            int N = particles.size();
            std::vector<double> logw(N, 0.0);
            std::vector<double> errs;
            std::vector<double> abs_errs;

            if (scan.ranges.size() != static_cast<size_t>(numBeams)) {
                std::cerr << "Error: Scan size (" << scan.ranges.size()
                          << ") does not match expected numBeams (" << numBeams << "). Aborting weighting." 
                          << std::endl;
                return particles;
            }

            for (int i = 0; i < N; i++) {
                const auto& p = particles[i];
                auto z_hat_data = lidar_scan(p);
                errs.clear();
                abs_errs.clear();

                for (int k = 0; k < numBeams; k++) {
                    double e = scan.ranges[k] - z_hat_data.ranges[k];
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
            // Only resample if low effective sample size
            double sumSquaredWeights = 0.0;
            for (const auto &p : particles) {
                sumSquaredWeights += p.weight * p.weight;
            }
            double effectiveSampleSize = 1 / sumSquaredWeights;
            if (effectiveSampleSize < 0.5 * particles.size()) {
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
            
            return particles;
        }

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

            // if (std::fabs(weight_sum - 1.0) > 1e-6) {
            //     std::cerr << "Warning: Total particle weight is not 1.0. It is " << weight_sum << "." << std::endl;
            // }

            double x = 0.0;
            double y = 0.0;
            double sin = 0.0; 
            double cos = 0.0;
            for (const auto &p : particles) {
                double w = p.weight;
                x += p.x * w;
                y += p.y * w;
                sin += std::sin(p.theta) * w;
                cos += std::cos(p.theta) * w;
            }
            double theta = std::atan2(sin, cos);
            return {x, y, theta};
        }

        /**
         * Get the number of beams used in the LIDAR scan simulation
         * @return The number of beams
         */
        int getNumBeams() const {
            return numBeams;
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
        const double maxScanRange;
        const int32_t numBeams;
        const double particleDropFraction = 0.7; // Fraction of particles to drop
        const Map map_;
        
        double minAngle = 0.0;
        double angleIncrement = (2.0 * M_PI) / (numBeams - 1); // Default to full 360° coverage

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
        std::vector<double> findIntersection(
            double startX, 
            double startY, 
            double endX, 
            double endY,
            double wallX1, 
            double wallY1, 
            double wallX2,
            double wallY2
        ) {
            double dxRay = endX - startX;
            double dyRay = endY - startY;
            double dxWall = wallX2 - wallX1;
            double dyWall = wallY2 - wallY1;

            double denominator = dxRay * dyWall - dyRay * dxWall;
            if (std::fabs(denominator) < 1e-9) {
                return {}; // Parallel lines
            }
            double t = ((startX - wallX1) * dyWall - (startY - wallY1) * dxWall) / denominator;
            double u = -((startX - wallX1) * dyRay - (startY - wallY1) * dxRay) / denominator;
            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
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

