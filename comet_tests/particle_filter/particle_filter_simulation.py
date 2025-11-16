import math
import numpy as np
import time
import matplotlib.path as mpath

# --- Constants ---

# Robot parameters
ROBOT_RADIUS = 0.75  # feet
ROBOT_MAX_SPEED = 5.0
ROBOT_MAX_ANGULAR_SPEED = math.pi
LIDAR_MAX_RANGE = 15.0  # feet
NUM_LIDAR_BEAMS = 20
MEAS_NOISE_LIDAR = 0.1  # feet
START_POSE = (-4.0, -4.0, 0.0)  # x, y, theta
NUM_PARTICLES = 250
PARTICLE_DROP_FRACTION = 0.75

# Benchmark parameters
# DT is no longer fixed.
SIMULATION_STEPS = 600  # Run for 600 iterations

# --- Environment Setup ---

fake_obstacles = [
    [(-6, -6), (6, -6), (6, 6), (-6, 6)],  # outer walls
]
fake_edges = []
for obstacle in fake_obstacles:
    for j in range(len(obstacle)):
        p0 = obstacle[j]
        p1 = obstacle[(j + 1) % len(obstacle)]
        fake_edges.append((*p0, *p1))
fake_edges = np.array(fake_edges, dtype=np.float32)


real_obstacles = [
    [(-6.0, -6.0), (6.0, -6.0), (6.0, 6.0), (-6.0, 6.0)],  # outer walls
]

# goal posts
for i in [-1, 1]:
    for j in [-1, 1]:
        real_obstacles.append([
            (2 * i - 0.2, 4 * j - 0.2),
            (2 * i + 0.2, 4 * j - 0.2),
            (2 * i + 0.2, 4 * j + 0.2),
            (2 * i - 0.2, 4 * j + 0.2),
        ])

# middle goal post
real_obstacles.append([
    (-0.2, -0.2),
    (0.2, -0.2),
    (0.2, 0.2),
    (-0.2, 0.2),
])

real_edges = []
for obstacle in real_obstacles:
    for j in range(len(obstacle)):
        p0 = obstacle[j]
        p1 = obstacle[(j + 1) % len(obstacle)]
        real_edges.append((*p0, *p1))
real_edges = np.array(real_edges, dtype=np.float32)


# --- Core Logic Functions ---

def intersect_ray_segment(p0, p1, r0, r1):
    x1, y1 = p0
    x2, y2 = p1
    x3, y3 = r0
    x4, y4 = r1

    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if abs(denom) < 1e-6:
        return None

    t = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    u = -((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    if 0 <= t <= 1 and u >= 0:
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        return (ix, iy)
    return None

def in_free_space(x, y):
    """Check if a point is in free space (not inside any obstacle)."""
    border_path = mpath.Path(fake_obstacles[0])
    if not border_path.contains_point((x, y)):
        return False
    for obs in fake_obstacles[1:]:
        path = mpath.Path(obs)
        if path.contains_point((x, y)):
            return False
    return True

def lidar_scan(rx, ry, robot_theta, obstacle_edges=fake_edges):
    distances = []
    for i in range(NUM_LIDAR_BEAMS):
        angle = robot_theta + (i / NUM_LIDAR_BEAMS) * 2 * math.pi
        min_dist = LIDAR_MAX_RANGE
        hit_point = None
        end = (rx + math.cos(angle) * LIDAR_MAX_RANGE, ry + math.sin(angle) * LIDAR_MAX_RANGE)
        for edge in obstacle_edges:
            p0 = edge[:2]
            p1 = edge[2:]
            pt = intersect_ray_segment(p0, p1, (rx, ry), end)
            if pt is not None:
                    d = math.hypot(pt[0] - rx, pt[1] - ry)
                    if d < min_dist:
                        min_dist = d
                        hit_point = pt
        # add measurement noise
        noise = np.random.normal(0.0, MEAS_NOISE_LIDAR)
        min_dist = max(0.0, min(LIDAR_MAX_RANGE, min_dist + noise))
        
        distances.append((angle, min_dist, hit_point[0] if hit_point else None, hit_point[1] if hit_point else None))
    return distances

def predict_particles(particles, control_input, robot_theta, xy_noise=0.03, theta_noise=0.0075):
    speed, angular_speed, dt = control_input # dt is passed in

    new_particles = np.zeros_like(particles)
    noise_xy = np.random.normal(0, xy_noise, size=(len(particles), 2))
    noise_theta = np.random.normal(0, theta_noise, size=len(particles))
    new_particles[:, 0] = particles[:, 0] + np.cos(particles[:, 2]) * speed * dt + noise_xy[:, 0]
    new_particles[:, 1] = particles[:, 1] + np.sin(particles[:, 2]) * speed * dt + noise_xy[:, 1]
    new_particles[:, 2] = particles[:, 2] + angular_speed * dt + noise_theta
    return new_particles

def correct_particles(particles, lidar_distances):
    sigma2 = MEAS_NOISE_LIDAR ** 2
    N = len(particles)
    logw = np.zeros(N, dtype=np.float64)
    z_meas = lidar_distances.flatten()

    for i in range(N):
        px, py, pth = particles[i]
        z_hat_data = lidar_scan(px, py, pth)

        # calculate error
        z_hat = np.array([d[1] for d in z_hat_data])
        err = z_meas - z_hat
        
        # drop worst PARTICLE_DROP_FRACTION of errors to be robust to outliers
        threshold = np.percentile(np.abs(err), 100 * (1.0 - PARTICLE_DROP_FRACTION))
        err = err[np.abs(err) < threshold]

        ll = -0.5 * np.sum((err ** 2) / sigma2)
        if not in_free_space(px, py):
            ll -= 5.0
        logw[i] = ll

    m = np.max(logw)
    w = np.exp(logw - m)
    w /= np.sum(w) + 1e-300
    return w

def resample_particles(particles, weights):
    indices = np.random.choice(len(particles), size=len(particles), p=weights)
    new_particles = particles[indices]
    return new_particles

def estimate_position(particles, weights):
    # Weighted mean for x,y, and circular mean for theta
    x = np.sum(particles[:, 0] * weights)
    y = np.sum(particles[:, 1] * weights)
    theta = np.sum(particles[:, 2] * weights)
    return np.array([x, y, theta])

# --- Main Simulation ---

# Initial setup
robot_x, robot_y, robot_theta = START_POSE
opp_robot_x, opp_robot_y, opp_robot_theta = 4.0, 4.0, math.pi

particles = np.column_stack((
    np.random.uniform(START_POSE[0]-1, START_POSE[0]+1, NUM_PARTICLES),
    np.random.uniform(START_POSE[1]-1, START_POSE[1]+1, NUM_PARTICLES),
    np.random.uniform(START_POSE[2]-0.1, START_POSE[2]+0.1, NUM_PARTICLES)
))

weights = np.ones(NUM_PARTICLES) / NUM_PARTICLES
estimated_pose = START_POSE

print("--- Starting Max Speed Benchmark ---")
print("Step | Real Pose (x, y, t) | Est. Pose (x, y, t) | Error (x, y, t)")

# *** Start timer ***
start_time = time.perf_counter()
last_time = start_time

# --- Simulation Loop ---
# This loop will run as fast as possible, with no artificial delay.
for i in range(SIMULATION_STEPS):
    
    # --- Calculate dt ---
    # This is the *actual* wall-clock time elapsed since the last frame.
    current_time = time.perf_counter()
    dt = current_time - last_time

    last_time = current_time
    if dt <= 0:  # Prevent division by zero
        dt = 1e-6
    
    # 1. Define Control Inputs (pre-programmed)
    speed = 0.0
    angular_speed = 0.0
    opp_speed = 0.0
    opp_angular_speed = 0.0
    
    if i < 100:
        speed = ROBOT_MAX_SPEED
    elif i < 150:
        angular_speed = ROBOT_MAX_ANGULAR_SPEED
    elif i < 250:
        speed = ROBOT_MAX_SPEED
    elif i < 300:
        speed = 0.0
    elif i < 400:
        speed = -ROBOT_MAX_SPEED
    else:
        angular_speed = -ROBOT_MAX_ANGULAR_SPEED

    if i % 200 < 50:
        opp_speed = ROBOT_MAX_SPEED
    elif i % 200 == 50:
        opp_angular_speed = ROBOT_MAX_ANGULAR_SPEED / (dt * 50)


    # 2. Update Real Robot Poses (Ground Truth)
    # **All physics use the variable dt**
    robot_x += math.cos(robot_theta) * speed * dt
    robot_y += math.sin(robot_theta) * speed * dt
    robot_theta += angular_speed * dt

    opp_robot_x += math.cos(opp_robot_theta) * opp_speed * dt
    opp_robot_y += math.sin(opp_robot_theta) * opp_speed * dt
    opp_robot_theta += opp_angular_speed * dt

    # 3. Get Real LIDAR Scan (from real map + opponent)
    real_edges_with_opponent = real_edges.copy()

    # add opponent robot as obstacle
    for j in range(4):
        p0 = (
            opp_robot_x + 1.41*ROBOT_RADIUS * math.cos(opp_robot_theta + j * math.pi / 2),
            opp_robot_y + 1.41*ROBOT_RADIUS * math.sin(opp_robot_theta + j * math.pi / 2),
        )
        p1 = (
            opp_robot_x + 1.41*ROBOT_RADIUS * math.cos(opp_robot_theta + ((j + 1) % 4) * math.pi / 2),
            opp_robot_y + 1.41*ROBOT_RADIUS * math.sin(opp_robot_theta + ((j + 1) % 4) * math.pi / 2),
        )
        real_edges_with_opponent = np.vstack([real_edges_with_opponent, [*p0, *p1]])

    lidar_data = lidar_scan(robot_x, robot_y, robot_theta, obstacle_edges=real_edges_with_opponent)
    distances = np.array([[dist] for _, dist, _, _ in lidar_data], dtype=np.float32)

    # 4. Particle Filter Steps
    # **All predictions use the variable dt**
    particles = predict_particles(particles, (speed, angular_speed, dt), robot_theta)
    weights = correct_particles(particles, distances)
    particles = resample_particles(particles, weights)
    
    # 5. Estimate Pose
    estimated_pose = estimate_position(particles, weights)

    # 6. Log Output (every 30 steps)
    if i % 30 == 0:
        err_x = robot_x - estimated_pose[0]
        err_y = robot_y - estimated_pose[1]
        err_t = robot_theta - estimated_pose[2]
        print(f"{i:4d} | "
              f"({robot_x:.3f}, {robot_y:.3f}, {robot_theta:.3f}) | "
              f"({estimated_pose[0]:.3f}, {estimated_pose[1]:.3f}, {estimated_pose[2]:.3f}) | "
              f"({err_x:.3f}, {err_y:.3f}, {err_t:.3f})")

# *** Stop timer and print report ***
end_time = time.perf_counter()
sim_duration = end_time - start_time

print("--- Benchmark Complete ---")
# Following the C++ example, "game time" is reported as the total wall-clock duration.
print(f"Simulated {SIMULATION_STEPS} steps ({sim_duration:.4f} seconds of game time).")
print(f"Total computation took {sim_duration:.4f} seconds.")
print(f"Average steps per second (FPS): {SIMULATION_STEPS / sim_duration:.2f}")