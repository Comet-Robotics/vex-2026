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
        
        struct Point {
            double x, y;
        };

        struct LineSegment {
            double x1, y1, x2, y2;
        };

        struct Map {
            std::vector<LineSegment> walls;

            Map() {
                walls = {
                    {-6.0, -6.0, 6.0, -6.0},       // bottom wall
                    {6.0, -6.0, 6.0, 6.0},     // right wall
                    {6.0, 6.0, -6.0, 6.0},     // top wall
                    {-6.0, 6.0, -6.0, -6.0}        // left wall
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
        * Initialize particles randomly within the map boundaries
        * @return A vector of initialized particles
        */
        std::vector<Particle> initializeParticles()
        {
            std::vector<Particle> particles;
            for (int i = 0; i < numParticles; i++) {
                Particle p;
                p.x = static_cast<double>(rand()) / RAND_MAX * 12.0 - 6.0;
                p.y = static_cast<double>(rand()) / RAND_MAX * 12.0 - 6.0;
                p.theta = static_cast<double>(rand()) / RAND_MAX * 2.0 * M_PI;
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
        LaserScan lidar_scan(Particle p, double noise) {
            // std::vector<LidarMeasurement> measurements;
            // measurements.reserve(NUM_LIDAR_BEAMS);
            // Point ray_start{rx, ry};

            // for (int i = 0; i < NUM_LIDAR_BEAMS; ++i) {
            //     double angle = robot_theta + (static_cast<double>(i) / NUM_LIDAR_BEAMS) * M_PI_2;
            //     double min_dist = LIDAR_MAX_RANGE;
            //     std::optional<Point> hit_point = std::nullopt;
            //     Point ray_end{
            //         rx + std::cos(angle) * LIDAR_MAX_RANGE,
            //         ry + std::sin(angle) * LIDAR_MAX_RANGE
            //     };

            //     for (const auto& edge : obstacle_edges) {
            //         Point p0{edge.x1, edge.y1};
            //         Point p1{edge.x2, edge.y2};
            //         auto pt = intersect_ray_segment(p0, p1, ray_start, ray_end);
            //         if (pt) {
            //             double d = std::hypot(pt->x - rx, pt->y - ry);
            //             if (d < min_dist) {
            //                 min_dist = d;
            //                 hit_point = pt;
            //             }
            //         }
            //     }

            //     double noise = noise_dist(rng) * MEAS_NOISE_LIDAR;
            //     min_dist = std::max(0.0, std::min(LIDAR_MAX_RANGE, min_dist + noise));
            //     measurements.push_back({
            //         angle,
            //         min_dist,
            //         hit_point ? hit_point->x : std::numeric_limits<double>::quiet_NaN(),
            //         hit_point ? hit_point->y : std::numeric_limits<double>::quiet_NaN()
            //     });
            // }
            // return measurements;

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
                double xy_jitter = gaussianDistribution(0, xy_sigma);
                double theta_jitter = gaussianDistribution(0, theta_sigma);

                double vxb = odom.vx + vx_noise;
                double vyb = odom.vy + vy_noise;
                double wb = odom.w + gamma_noise;

                // transform body-frame delta to world-frame using current heading
                // dx_body = vxb * dt ; dy_body = vyb * dt
                double dx_world = std::cos(p.theta) * (vxb * dt) - std::sin(p.theta) * (vyb * dt);
                double dy_world = std::sin(p.theta) * (vxb * dt) + std::cos(p.theta) * (vyb * dt);
                double dtheta = wb * dt;

                double x_new = p.x + dx_world + xy_jitter;
                double y_new = p.y + dy_world + xy_jitter;
                double theta_new = angleNormalize(p.theta + dtheta + theta_jitter);

                predictedParticles.push_back({x_new, y_new, theta_new});
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
            // double sigma = metersToFeet(0.02); // A1M8 lidar usually has a noise of 2-3 cm
            // int32_t numParticles = particles.size();
            // int32_t numBeams = scan.ranges.size();
            // std::vector<Particle> weightedParticles;
            // std::vector<double> errs;
            // for (const auto &p : particles) {
            //     Particle pWeighted = p;
            //     for(int32_t i = 0; i < numBeams; ++i) {
            //         double angleOfScan = scan.angle_min + i * scan.angle_increment;
            //         double rayDistance = simulateRay(p, angleOfScan);
            //         double measuredDistance = metersToFeet(scan.ranges[i]);
            //         pWeighted.weight *= gaussianWeight(rayDistance, sigma, measuredDistance);
            //     }
            //     weightedParticles.push_back(pWeighted);
            // }

            // for (auto &data : scan.ranges) {
            //     double err = data;
            //     err = 
            // }

            double sigma = metersToFeet(0.02); // A1M8 lidar usually has a noise of 2-3 cm
            int N = particles.size();
            std::vector<double> logw(N, 0.0);

            for (int i = 0; i < N; i++) {
                const auto& p = particles[i];
                auto z_hat_data = lidar_scan(p, sigma);
                std::vector<double> errs;
                std::vector<double> abs_errs;
                errs.reserve(numBeams);
                abs_errs.reserve(numBeams);

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
                    ll -= 5.0;
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
            std::vector<Particle> new_particles;
            
            new_particles.reserve(particles.size());
            std::vector<double> weights;
            weights.reserve(particles.size());
            for (const auto &p : particles) {
                weights.push_back(p.weight);
            }

            /*
            Create a discrete distribution based on particle weights
            This generates indices according to the weights, where 
            particles with higher weights are more likely to be chosen
            */
            std::discrete_distribution<int> dist(
                weights.begin(), weights.end()
            );

            for (int i = 0; i < particles.size(); ++i) {
                new_particles.push_back(particles[dist(gen)]);
            }

            return new_particles;
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

            double x = 0.0;
            double y = 0.0;
            double sin = 0.0; 
            double cos = 0.0;
            int32_t numParticles = particles.size();
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

        /**
        * Calculate yaw from a quaternion
        * @param x The x component of the quaternion
        * @param y The y component of the quaternion
        * @param z The z component of the quaternion
        * @param w The w component of the quaternion
        * @return The yaw angle
        */
        double yawFromQuaternion(double x, double y, double z, double w) 
        {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }

    private:
        const int32_t numParticles;
        const double maxScanRange;
        const int32_t numBeams;
        const double particleDropFraction = 0.3; // Fraction of particles to drop based on error
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
         * Simulate a ray cast from a particle at a given angle
         * @param particle The particle from which the ray is cast
         * @param angle The angle at which the ray is cast relative to the particle's orientation
         * @return The distance to the nearest obstacle detected by the ray
         */
        double simulateRay(const Particle &particle, const double angle)
        {
            double rayStartX = particle.x;
            double rayStartY = particle.y;
            double rayEndX = rayStartX + cos(particle.theta + angle) * maxScanRange;
            double rayEndY = rayStartY + sin(particle.theta + angle) * maxScanRange;
            
            std::vector<std::vector<double>> intersectionPoints;
            for (const auto &wall : map_.walls) {
                auto intersection = findIntersection(rayStartX, rayStartY, rayEndX, rayEndY,
                                                    wall.x1, wall.y1, wall.x2, wall.y2);
                if (!intersection.empty()) {
                    intersectionPoints.push_back(intersection);
                }
            }
            double rayDistance = std::numeric_limits<double>::infinity();
            for (const auto &point : intersectionPoints) {
                double xDist = point[0] - rayStartX;
                double yDist = point[1] - rayStartY;
                double distance = std::sqrt(pow(xDist, 2) + pow(yDist, 2));
                if (distance < rayDistance) {
                    rayDistance = distance;
                }
            }
            return rayDistance;
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
         * Normalize an angle to the range [-pi, pi]
         * @param a The angle to normalize
         * @return The normalized angle
         */
        double angleNormalize(double a) {
            while (a <= -M_PI) a += 2.0*M_PI;
            while (a >  M_PI) a -= 2.0*M_PI;
            return a;
        }
};

