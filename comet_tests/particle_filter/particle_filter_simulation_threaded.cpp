#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono> // Added for timing
#include <numeric>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <optional>
#include <thread>

// --- Constants ---
constexpr double ROBOT_RADIUS = 0.75;
constexpr double ROBOT_MAX_SPEED = 5.0;
constexpr double ROBOT_MAX_ANGULAR_SPEED = 3.14159;
constexpr double LIDAR_MAX_RANGE = 15.0;
constexpr int NUM_LIDAR_BEAMS = 30;
constexpr double MEAS_NOISE_LIDAR = 0.1;
constexpr int NUM_PARTICLES = 5000;
constexpr double PARTICLE_DROP_FRACTION = 0.5;

// Simulation settings
constexpr int SIMULATION_STEPS = 600; // 10 seconds of simulation

// --- Data Structures ---
struct Pose {
    double x, y, theta;
};
struct Edge {
    double x1, y1, x2, y2;
};
struct Point {
    double x, y;
};
struct LidarMeasurement {
    double angle;
    double dist;
    double hit_x;
    double hit_y;
};

// --- Global Random Number Generator ---
std::default_random_engine rng(std::random_device{}());
std::normal_distribution<double> noise_dist(0.0, 1.0); // Will scale as needed

// --- Geometry Functions ---
std::optional<Point> intersect_ray_segment(Point p0, Point p1, Point r0, Point r1) {
    double x1 = p0.x, y1 = p0.y;
    double x2 = p1.x, y2 = p1.y;
    double x3 = r0.x, y3 = r0.y;
    double x4 = r1.x, y4 = r1.y;

    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (std::abs(denom) < 1e-6) {
        return std::nullopt;
    }

    double t = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double u = -((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    if (t >= 0 && t <= 1 && u >= 0) {
        return Point{x1 + t * (x2 - x1), y1 + t * (y2 - y1)};
    }
    return std::nullopt;
}

bool in_free_space(double x, double y) {
    return (x >= -6.0 && x <= 6.0 && y >= -6.0 && y <= 6.0);
}

// --- LIDAR Simulation ---
std::vector<LidarMeasurement> lidar_scan(double rx, double ry, double robot_theta, const std::vector<Edge>& obstacle_edges) {
    std::vector<LidarMeasurement> measurements;
    measurements.reserve(NUM_LIDAR_BEAMS);
    Point ray_start{rx, ry};

    for (int i = 0; i < NUM_LIDAR_BEAMS; ++i) {
        double angle = robot_theta + (static_cast<double>(i) / NUM_LIDAR_BEAMS) * M_PI_2;
        double min_dist = LIDAR_MAX_RANGE;
        std::optional<Point> hit_point = std::nullopt;
        Point ray_end{
            rx + std::cos(angle) * LIDAR_MAX_RANGE,
            ry + std::sin(angle) * LIDAR_MAX_RANGE
        };

        for (const auto& edge : obstacle_edges) {
            Point p0{edge.x1, edge.y1};
            Point p1{edge.x2, edge.y2};
            auto pt = intersect_ray_segment(p0, p1, ray_start, ray_end);
            if (pt) {
                double d = std::hypot(pt->x - rx, pt->y - ry);
                if (d < min_dist) {
                    min_dist = d;
                    hit_point = pt;
                }
            }
        }

        double noise = noise_dist(rng) * MEAS_NOISE_LIDAR;
        min_dist = std::max(0.0, std::min(LIDAR_MAX_RANGE, min_dist + noise));
        measurements.push_back({
            angle,
            min_dist,
            hit_point ? hit_point->x : std::numeric_limits<double>::quiet_NaN(),
            hit_point ? hit_point->y : std::numeric_limits<double>::quiet_NaN()
        });
    }
    return measurements;
}

// --- Particle Filter Functions ---
std::vector<Pose> predict_particles(const std::vector<Pose>& particles, double speed, double angular_speed, double dt) {
    std::vector<Pose> new_particles;
    new_particles.reserve(NUM_PARTICLES);
    double xy_noise = 0.03;
    double theta_noise = 0.0075;
    double speed_noise = 0.1;
    double angular_noise = 0.05;

    for (const auto& p : particles) {
        double noise_x = noise_dist(rng) * xy_noise;
        double noise_y = noise_dist(rng) * xy_noise;
        double noise_theta = noise_dist(rng) * theta_noise;
        double noise_speed = noise_dist(rng) * speed_noise;
        double noise_angular = noise_dist(rng) * angular_noise;
        new_particles.push_back({
            p.x + std::cos(p.theta) * (speed + noise_speed) * dt + noise_x,
            p.y + std::sin(p.theta) * (speed + noise_speed) * dt + noise_y,
            p.theta + (angular_speed + noise_angular) * dt + noise_theta
        });
    }
    return new_particles;
}

std::vector<double> correct_particles(const std::vector<Pose>& particles, const std::vector<double>& z_meas, const std::vector<Edge>& fake_edges) {
    double sigma2 = MEAS_NOISE_LIDAR * MEAS_NOISE_LIDAR;
    int N = particles.size();
    std::vector<double> logw(N, 0.0);

    const unsigned numThreads = std::max(1u, std::thread::hardware_concurrency());
    const int chunk = (N + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;

    auto worker = [&](int start, int end) {
        for (int i = start; i < end && i < N; ++i) {
            const auto& p = particles[i];
            auto z_hat_data = lidar_scan(p.x, p.y, p.theta, fake_edges);
            std::vector<double> abs_errs;
            abs_errs.reserve(NUM_LIDAR_BEAMS);

            for (int k = 0; k < NUM_LIDAR_BEAMS; ++k) {
                double e = z_meas[k] - z_hat_data[k].dist;
                abs_errs.push_back(std::abs(e));
            }

            std::nth_element(
                abs_errs.begin(),
                abs_errs.begin() + static_cast<int>(NUM_LIDAR_BEAMS * (1.0 - PARTICLE_DROP_FRACTION)),
                abs_errs.end()
            );
            double threshold = abs_errs[static_cast<int>(NUM_LIDAR_BEAMS * (1.0 - PARTICLE_DROP_FRACTION))];

            double ll = 0.0;
            for (int k = 0; k < NUM_LIDAR_BEAMS; ++k) {
                if (abs_errs[k] < threshold) {
                    ll += -0.5 * (abs_errs[k] * abs_errs[k]) / sigma2;
                }
            }
            if (!in_free_space(p.x, p.y)) {
                ll -= 1e9;
            }
            logw[i] = ll;
        }
    };

    for (unsigned t = 0; t < numThreads; ++t) {
        int start = t * chunk;
        int end = start + chunk;
        threads.emplace_back(worker, start, end);
    }

    for (auto& thread : threads) {
        thread.join();
    }

    double m = *std::max_element(logw.begin(), logw.end());
    std::vector<double> weights(N);
    double sum_w = 0.0;
    
    for(int i = 0; i < N; ++i) {
        double w = std::exp(logw[i] - m);
        weights[i] = w;
        sum_w += w;
    }
    for(int i = 0; i < N; ++i) {
        weights[i] /= (sum_w + 1e-300);
    }
    return weights;
}

std::vector<Pose> resample_particles(const std::vector<Pose>& particles,
                                     const std::vector<double>& weights)
{
    const int N = particles.size();
    std::vector<Pose> new_particles = particles; // copy like python

    // ---------- compute Neff ----------
    double sum_sq = 0.0;
    for (double w : weights)
        sum_sq += w * w;

    double neff = 1.0 / sum_sq;

    // ---------- low variance resampling ----------
    if (neff < 0.5 * N)
    {
        new_particles.clear();
        new_particles.reserve(N);

        std::uniform_real_distribution<double> uni(0.0, 1.0 / N);
        double r = uni(rng);

        double c = weights[0];
        int i = 0;

        for (int m = 0; m < N; ++m)
        {
            double U = r + static_cast<double>(m) / N;

            while (U > c && i < N - 1)
            {
                ++i;
                c += weights[i];
            }

            new_particles.push_back(particles[i]);
        }
    }

    // ---------- particle injection ----------
    constexpr double PARTICLE_INJECT_FRACTION = 0.03;
    int num_inject = static_cast<int>(PARTICLE_INJECT_FRACTION * N);

    if (num_inject > 0)
    {
        // indices sorted by weight (ascending → worst first)
        std::vector<int> indices(N);
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(),
                  [&](int a, int b) { return weights[a] < weights[b]; });

        std::uniform_real_distribution<double> pos_dist(-6.0, 6.0);

        for (int k = 0; k < num_inject; ++k)
        {
            int idx = indices[k];

            double new_px, new_py;

            do {
                new_px = pos_dist(rng);
                new_py = pos_dist(rng);
            } while (!in_free_space(new_px, new_py));

            // keep orientation like python
            double theta = new_particles[idx].theta;

            new_particles[idx] = Pose{new_px, new_py, theta};
        }
    }

    return new_particles;
}


Pose estimate_position(const std::vector<Pose>& particles, const std::vector<double>& weights) {
    double x = 0.0, y = 0.0, theta_sum = 0.0;
    for (size_t i = 0; i < particles.size(); ++i) {
        x += particles[i].x * weights[i];
        y += particles[i].y * weights[i];
        theta_sum += particles[i].theta * weights[i];
    }
    return {x, y, theta_sum};
}

// --- Environment Setup ---
std::vector<Edge> get_fake_edges() {
    std::vector<std::vector<Point>> fake_obstacles = {
        {{-6, -6}, {6, -6}, {6, 6}, {-6, 6}} // outer walls
    };
    std::vector<Edge> edges;
    for (const auto& obstacle : fake_obstacles) {
        for (size_t j = 0; j < obstacle.size(); ++j) {
            Point p0 = obstacle[j];
            Point p1 = obstacle[(j + 1) % obstacle.size()];
            edges.push_back({p0.x, p0.y, p1.x, p1.y});
        }
    }
    return edges;
}

std::vector<Edge> get_real_edges() {
    std::vector<std::vector<Point>> real_obstacles = {
        {{-6.0, -6.0}, {6.0, -6.0}, {6.0, 6.0}, {-6.0, 6.0}} // outer walls
    };
    for (int i : {-1, 1}) {
        for (int j : {-1, 1}) {
            real_obstacles.push_back({
                {2.0 * i - 0.2, 4.0 * j - 0.2},
                {2.0 * i + 0.2, 4.0 * j - 0.2},
                {2.0 * i + 0.2, 4.0 * j + 0.2},
                {2.0 * i - 0.2, 4.0 * j + 0.2},
            });
        }
    }
    real_obstacles.push_back({
        {-0.2, -0.2}, {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}
    });

    std::vector<Edge> edges;
    for (const auto& obstacle : real_obstacles) {
        for (size_t j = 0; j < obstacle.size(); ++j) {
            Point p0 = obstacle[j];
            Point p1 = obstacle[(j + 1) % obstacle.size()];
            edges.push_back({p0.x, p0.y, p1.x, p1.y});
        }
    }
    return edges;
}

// --- Main Simulation Loop ---
int main() {
    // --- Initial Setup ---
    Pose robot_pose = {-4.0, -4.0, 0.0};
    Pose opp_robot_pose = {4.0, 4.0, M_PI};
    
    std::vector<Edge> fake_edges = get_fake_edges();
    std::vector<Edge> real_edges = get_real_edges();

    std::vector<Pose> particles;
    std::uniform_real_distribution<double> pos_dist(-1.0, 1.0);
    std::uniform_real_distribution<double> theta_dist(-0.1, 0.1);
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles.push_back({
            robot_pose.x + pos_dist(rng),
            robot_pose.y + pos_dist(rng),
            robot_pose.theta + theta_dist(rng)
        });
    }

    std::vector<double> weights(NUM_PARTICLES, 1.0 / NUM_PARTICLES);
    Pose estimated_pose = robot_pose;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "--- Starting Max Speed Benchmark ---" << std::endl;
    std::cout << "Step | Real Pose (x, y, t) | Est. Pose (x, y, t) | Error (x, y, t)" << std::endl;

    // *** Start timer ***
    auto sim_start_time = std::chrono::high_resolution_clock::now();

    // --- Simulation Loop ---
    // This loop will run as fast as possible, with no artificial delay.
    auto time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < SIMULATION_STEPS; ++i) {
        // 
        auto DT_clock = std::chrono::high_resolution_clock::now() - time;
        time = std::chrono::high_resolution_clock::now();

        double DT = std::chrono::duration<double>(DT_clock).count();

        if (DT <= 0) DT = 1e-6; // Prevent division by zero

        // 1. Define Control Inputs
        double speed = 0.0, angular_speed = 0.0;
        double opp_speed = 0.0, opp_angular_speed = 0.0;
        
        if (i < 100) { speed = ROBOT_MAX_SPEED / 4; }
        else if (i < 150) { angular_speed = ROBOT_MAX_ANGULAR_SPEED / 4; }
        else if (i < 250) { speed = ROBOT_MAX_SPEED / 4; }
        else if (i < 300) { speed = 0; }
        else if (i < 400) { speed = -ROBOT_MAX_SPEED / 4; }
        else { angular_speed = -ROBOT_MAX_ANGULAR_SPEED / 4; }
        
        if (i % 200 < 50) opp_speed = ROBOT_MAX_SPEED;
        else if (i % 200 == 50) opp_angular_speed = ROBOT_MAX_ANGULAR_SPEED / (DT * 50);

        // 2. Update Real Robot Poses (Ground Truth)
        // **All physics use the fixed DT**
        robot_pose.x += std::cos(robot_pose.theta) * speed * DT;
        robot_pose.y += std::sin(robot_pose.theta) * speed * DT;
        robot_pose.theta += angular_speed * DT;

        opp_robot_pose.x += std::cos(opp_robot_pose.theta) * opp_speed * DT;
        opp_robot_pose.y += std::sin(opp_robot_pose.theta) * opp_speed * DT;
        opp_robot_pose.theta += opp_angular_speed * DT;

        // 3. Get Real LIDAR Scan
        std::vector<Edge> real_edges_with_opponent = real_edges;
        for (int j = 0; j < 4; ++j) {
            double ang1 = opp_robot_pose.theta + j * M_PI / 2.0;
            double ang2 = opp_robot_pose.theta + ((j + 1) % 4) * M_PI / 2.0;
            real_edges_with_opponent.push_back({
                opp_robot_pose.x + 1.41 * ROBOT_RADIUS * std::cos(ang1),
                opp_robot_pose.y + 1.41 * ROBOT_RADIUS * std::sin(ang1),
                opp_robot_pose.x + 1.41 * ROBOT_RADIUS * std::cos(ang2),
                opp_robot_pose.y + 1.41 * ROBOT_RADIUS * std::sin(ang2)
            });
        }
        
        auto lidar_data = lidar_scan(robot_pose.x, robot_pose.y, robot_pose.theta, real_edges_with_opponent);
        std::vector<double> distances;
        for(const auto& meas : lidar_data) distances.push_back(meas.dist);

        // 4. Particle Filter: Predict
        // **All predictions use the fixed DT**
        particles = predict_particles(particles, speed, angular_speed, DT);

        // 5. Particle Filter: Correct
        weights = correct_particles(particles, distances, fake_edges);
        
        // 6. Particle Filter: Resample
        particles = resample_particles(particles, weights);

        // 7. Estimate Pose
        estimated_pose = estimate_position(particles, weights);

        // 8. Log Output (every 30 steps)
        if (i % 30 == 0) {
            double err_x = robot_pose.x - estimated_pose.x;
            double err_y = robot_pose.y - estimated_pose.y;
            double err_t = robot_pose.theta - estimated_pose.theta;
            
            std::cout << std::setw(4) << i << " | "
                      << "(" << std::setw(6) << robot_pose.x << ", " << std::setw(6) << robot_pose.y << ", " << std::setw(6) << robot_pose.theta << ") | "
                      << "(" << std::setw(6) << estimated_pose.x << ", " << std::setw(6) << estimated_pose.y << ", " << std::setw(6) << estimated_pose.theta << ") | "
                      << "(" << std::setw(6) << err_x << ", " << std::setw(6) << err_y << ", " << std::setw(6) << err_t << ")" << std::endl;
        }
    }
    
    // *** Stop timer and print report ***
    auto sim_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sim_duration = sim_end_time - sim_start_time;
    
    std::cout << "--- Benchmark Complete ---" << std::endl;
    std::cout << std::setprecision(4);
    std::cout << "Simulated " << SIMULATION_STEPS << " steps (" << sim_duration.count() << " seconds of game time)." << std::endl;
    std::cout << "Total computation took " << sim_duration.count() << " seconds." << std::endl;
    std::cout << "Average steps per second (Hz): " << (SIMULATION_STEPS / sim_duration.count()) << std::endl;

    return 0;
}