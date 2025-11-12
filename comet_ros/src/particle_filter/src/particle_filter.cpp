#include <random>
#include <cmath>
#include <algorithm>

class ParticleFilter
{
    public:
        ParticleFilter() : numParticles(100), maxScanRange(metersToFeet(6.0)) {}
        
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

        std::vector<Particle> initializeParticles()
        {
            std::vector<Particle> particles;
            for (int i = 0; i < numParticles; i++) {
                Particle p;
                p.x = static_cast<double>(rand()) / RAND_MAX * 12.0;
                p.y = static_cast<double>(rand()) / RAND_MAX * 12.0;
                p.theta = static_cast<double>(rand()) / RAND_MAX * 2.0 * M_PI;
                p.weight = 1.0;
                particles.push_back(p);
            }
            return particles;
        }

        std::vector<Particle> predictParticles(const std::vector<Particle> &particles, 
                                                const Odometry &odom,
                                                const double dt,
                                                const double xyNoiseMax = 0.01,
                                                const double thetaNoiseMax = 0.005,
                                                const double sigmaV = 0.01,     // linear velocity sigma
                                                const double sigmaW = 0.01,     // angular velocity sigma
                                                const double sigmaGamma = 0.002)    // heading sigma
        {
            std::vector<Particle> predictedParticles;
            for (const auto &p : particles) {
                Particle pPred = p;
                double xyNoise = std::clamp(gaussianDistribution(0.0, sigmaV), -xyNoiseMax, xyNoiseMax);
                double thetaNoise = std::clamp(gaussianDistribution(0.0, sigmaW), -thetaNoiseMax, thetaNoiseMax);
                double gammaNoise = std::clamp(gaussianDistribution(0.0, sigmaGamma), -sigmaGamma, sigmaGamma);
                pPred.x += odom.vx * dt * std::cos(pPred.theta) + xyNoise;
                pPred.y += odom.vy * dt * std::sin(pPred.theta) + xyNoise;
                pPred.theta += odom.w * dt + thetaNoise + gammaNoise;
                predictedParticles.push_back(pPred);
            }
            return predictedParticles;
        }

        std::vector<Particle> weightParticles(const LaserScan &scan, 
                                            const std::vector<Particle> &particles)
        {
            double sigma = metersToFeet(0.02); // A1M8 lidar usually has a noise of 2-3 cm
            int32_t numParticles = particles.size();
            int32_t numBeams = scan.ranges.size();
            std::vector<Particle> weightedParticles;
            for (const auto &p : particles) {
                Particle pWeighted = p;
                for(int32_t i = 0; i < numBeams; ++i) {
                    double angleOfScan = scan.angle_min + i * scan.angle_increment;
                    double rayDistance = simulateRay(p, angleOfScan);
                    double measuredDistance = metersToFeet(scan.ranges[i]);
                    pWeighted.weight *= gaussianWeight(rayDistance, sigma, measuredDistance);
                }
                weightedParticles.push_back(pWeighted);
            }

            // normalize weights
            double sumWeights = 0.0;
            for (const auto &p : weightedParticles) {
                sumWeights += p.weight;
            }
            if (sumWeights > 0) {
                for (auto &p : weightedParticles) {
                    p.weight /= sumWeights;
                }
            }
            return weightedParticles;
        }

        std::vector<Particle> resampleParticles(const std::vector<Particle> &particles)
        {
            // using low-variance resampling
            std::vector<Particle> resampledParticles;
            int32_t numParticles = particles.size();

            std::vector<double> cumulativeWeights(numParticles);  
            for(int32_t i = 0; i < numParticles; ++i) {
                cumulativeWeights[i] = particles[i].weight + ((i > 0) ? cumulativeWeights[i - 1] : 0.0);
            }
            cumulativeWeights.back() = 1.0;
            
            double r = static_cast<double>(rand()) / RAND_MAX / numParticles;   
            double U_m = r;
            int32_t i = 0;
            for (int32_t m = 0; m < numParticles; ++m) {
                U_m = r + (static_cast<double>(m) / numParticles);
                while (U_m > cumulativeWeights[i]) {
                    i++;
                }
                Particle p = particles[i];
                p.weight = 1.0 / numParticles;
                resampledParticles.push_back(p);
           }
            return resampledParticles;
        }

        std::vector<double> estimatePose(const std::vector<Particle> &particles)
        {
            double x = 0.0;
            double y = 0.0;
            double sin = 0.0; 
            double cos = 0.0;
            double theta = 0.0;
            int32_t numParticles = particles.size();
            for (const auto &p : particles) {
                x += p.x * p.weight;
                y += p.y * p.weight;
                sin += std::sin(p.theta) * p.weight;
                cos += std::cos(p.theta) * p.weight;
            }
            theta = std::atan2(sin, cos);
            return {x, y, theta};
        }

        double yawFromQuaternion(double x, double y, double z, double w) {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }

    private:
        const int32_t numParticles;
        const double maxScanRange;
        const Map map_;

        std::mt19937 gen{std::random_device{}()};
        double gaussianDistribution(double mu, double sigma) {
            std::normal_distribution<double> dist(mu, sigma);
            return dist(gen);
        }

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

        double metersToFeet(double meters)
        {
            return meters * 3.28084;
        }

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
};

