import pygame
import math
import numpy as np
import time
import matplotlib.path as mpath
import random

pygame.init()

# Screen setup
WIDTH, HEIGHT = 800, 800
SCALE = 800.0 / 12.0  # 12-foot field
CENTER = (WIDTH // 2, HEIGHT // 2)

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY  = (100, 100, 100)

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

# Pygame setup
flags = pygame.DOUBLEBUF
try:
    screen = pygame.display.set_mode((WIDTH, HEIGHT), flags, vsync=1)
except TypeError:
    screen = pygame.display.set_mode((WIDTH, HEIGHT), flags)
pygame.display.set_caption("Driveable Robot Simulation")
clock = pygame.time.Clock()

fake_obstacles = [
    [(-6, -6), (6, -6), (6, 6), (-6, 6)],  # outer walls
    # [(-3, -2), (-2, -1), (-2, 1), (-3, 1)],
    # [(1, -3), (3, -3), (3, -2), (1, -2)],
    # [(-1, 2), (2, 2), (2, 3), (-1, 3)],
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


def world_to_screen(x, y):
    sx = int(CENTER[0] + x * SCALE)
    sy = int(CENTER[1] - y * SCALE)
    return sx, sy

def draw_robot(x, y, theta):
    sx, sy = world_to_screen(x, y)
    pygame.draw.circle(screen, GREEN, (sx, sy), int(ROBOT_RADIUS * SCALE), 2)
    hx = sx + int(math.cos(theta) * ROBOT_RADIUS * SCALE * 2)
    hy = sy - int(math.sin(theta) * ROBOT_RADIUS * SCALE * 2)
    pygame.draw.line(screen, RED, (sx, sy), (hx, hy), 2)

def draw_estimated_robot(x, y, theta):
    sx, sy = world_to_screen(x, y)
    pygame.draw.circle(screen, (255, 0, 255), (sx, sy), int(ROBOT_RADIUS * SCALE), 2)
    hx = sx + int(math.cos(theta) * ROBOT_RADIUS * SCALE * 2)
    hy = sy - int(math.sin(theta) * ROBOT_RADIUS * SCALE * 2)
    pygame.draw.line(screen, (255, 100, 255), (sx, sy), (hx, hy), 2)

def draw_environment(edges):
    for edge in edges:
        p0 = world_to_screen(edge[0], edge[1])
        p1 = world_to_screen(edge[2], edge[3])
        pygame.draw.line(screen, GRAY, p0, p1, 2)

def draw_lidar(x, y, lidar_data):
    rsx, rsy = world_to_screen(x, y)
    for angle, dist, hit_x, hit_y in lidar_data:
        if hit_x is not None and hit_y is not None:
            hx, hy = world_to_screen(hit_x, hit_y)
            pygame.draw.line(screen, (255, 200, 0), (rsx, rsy), (hx, hy), 1)
            pygame.draw.circle(screen, (0, 255, 255), (hx, hy), 2)

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

def draw_particles(particles):
    for (px, py, _) in particles:
        psx, psy = world_to_screen(px, py)
        pygame.draw.circle(screen, (0, 0, 255), (psx, psy), 2)

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
    speed, angular_speed, dt = control_input

    # xy_noise = xy_noise + xy_noise * abs(speed)
    # theta_noise = theta_noise + theta_noise * abs(angular_speed)

    new_particles = np.zeros_like(particles)
    noise_xy = np.random.normal(0, xy_noise, size=(len(particles), 2))
    noise_theta = np.random.normal(0, theta_noise, size=len(particles))

    noise_xy += noise_xy * abs(speed)
    noise_theta += noise_theta * abs(angular_speed)

    new_particles[:, 0] = particles[:, 0] + np.cos(particles[:, 2]) * speed * dt + noise_xy[:, 0]
    new_particles[:, 1] = particles[:, 1] + np.sin(particles[:, 2]) * speed * dt + noise_xy[:, 1]
    new_particles[:, 2] = particles[:, 2] + angular_speed * dt + noise_theta
    # new_particles[:, 2] = robot_theta + noise_theta
    return new_particles

def correct_particles(particles, lidar_distances):
    # previous_time = time.time()
    sigma2 = MEAS_NOISE_LIDAR ** 2
    N = len(particles)
    logw = np.zeros(N, dtype=np.float64)
    z_meas = lidar_distances.flatten()

    # print("First part time", time.time() - previous_time)
    # previous_time = time.time()

    for i in range(N):
        px, py, pth = particles[i]
        # previous_time = time.time()
        z_hat_data = lidar_scan(px, py, pth)
        # print("Particle {} scan time:".format(i), time.time() - previous_time)

        # calculate log likelihood

        # calculate error
        z_hat = np.array([d[1] for d in z_hat_data])
        err = z_meas - z_hat
        
        # drop worst PARTICLE_DROP_FRACTION of errors to be robust to outliers
        threshold = np.percentile(np.abs(err), 100 * (1.0 - PARTICLE_DROP_FRACTION))
        err = err[np.abs(err) < threshold]

        ll = -0.5 * np.sum((err ** 2) / sigma2)
        if not in_free_space(px, py):
            ll -= 1e9
        logw[i] = ll

    m = np.max(logw)
    w = np.exp(logw - m)
    w /= np.sum(w) + 1e-300

    # print("Weights stats: min {:.4e}, max {:.4e}, mean {:.4e}, sum {:.4e}".format(
    #     np.min(w), np.max(w), np.mean(w), np.sum(w)))
    # print("Highest weight particle pose: x {:.2f}, y {:.2f}, theta {:.2f}".format(
    #     particles[np.argmax(w), 0], particles[np.argmax(w), 1], particles[np.argmax(w), 2]))
    # print("Highest weight particle LIDAR scan:", lidar_scan(particles[np.argmax(w), 0], particles[np.argmax(w), 1], particles[np.argmax(w), 2]))
    return w

def low_variance_resample(particles, weights, theta):
    JITTER_STD = 0.02
    INJECTION_FRACTION = 0.05

    N = len(weights)
    positions = (np.arange(N) + np.random.rand()) / N
    cumulative = np.cumsum(weights)
    idx = np.zeros(N, dtype=int)
    i, j = 0, 0
    while i < N:
        if positions[i] < cumulative[j]:
            idx[i] = j
            i += 1
        else:
            j += 1
    new_particles = particles[idx].copy()

    # jitter small gaussian to allow exploration (rejuvenation)
    new_particles[:, 0:2] += np.random.normal(0, JITTER_STD, size=(N, 2))
    new_particles[:, 2] += np.random.normal(0, JITTER_STD * 2.0, size=N)

    # particle injection: replace a small fraction with random samples over free space
    inject_count = int(np.floor(INJECTION_FRACTION * N))
    if inject_count > 0:
        # sample uniformly in map bounds but ensure free space
        injected = 0
        attempts = 0
        while injected < inject_count and attempts < inject_count * 20:
            rx = np.random.uniform(-5, 5)
            ry = np.random.uniform(-5, 5)
            if in_free_space(rx, ry):
                new_particles[injected] = [rx, ry, theta + np.random.normal(0, JITTER_STD * 2.0)]
                injected += 1
            attempts += 1
    return new_particles

def resample_particles(particles, weights):
    # # only resample if low effective sample size
    # neff = 1.0 / np.sum(weights ** 2)
    # if neff < 0.5 * len(particles):
    #     print("Resampling particles, Neff =", neff)
    #     indices = np.random.choice(len(particles), size=len(particles), p=weights)
    #     new_particles = particles[indices]
    #     return new_particles
    # return particles
    # Compute effective sample size
    neff = 1.0 / np.sum(weights ** 2)
    N = len(particles)
    if neff < 0.5 * N:
        new_particles = []

        # 1. Random starting point r in [0, 1/N)
        r = random.uniform(0.0, 1.0 / N)

        # 2. Low-variance wheel resampling
        c = weights[0]
        i = 0
        for m in range(N):
            U = r + m / N
            while U > c and i < N - 1:
                i += 1
                c += weights[i]
            new_particles.append(particles[i])

        new_particles = np.array(new_particles)
        return new_particles

    return particles

    # return low_variance_resample(particles, weights, robot_theta)

    # newParticles = particles.copy()

    # j = 0
    # cumulative_weight = 0.0
    # average_weight = np.average(weights)
    # rand_weight = np.random.uniform(0, average_weight)
    # xSum, ySum = 0.0, 0.0
    # for i in range(NUM_PARTICLES):
    #     targetWeight = i * average_weight + rand_weight
    #     while cumulative_weight < targetWeight:
    #         if j > NUM_PARTICLES - 1:
    #             break
    #         cumulative_weight += weights[j]
    #         j += 1

    #     newParticles[i] = particles[j-1]
        
    #     xSum += particles[i, 0]
    #     ySum += particles[i, 1]

    # return newParticles

def estimate_position(particles, weights):
    # Weighted mean for x,y, and circular mean for theta
    x = np.sum(particles[:, 0] * weights)
    y = np.sum(particles[:, 1] * weights)
    theta = np.sum(particles[:, 2] * weights)
    return np.array([x, y, theta])

# Initial setup
robot_x, robot_y, robot_theta = START_POSE
opp_robot_x, opp_robot_y, opp_robot_theta = 4.0, 4.0, math.pi

speed = 0.0
angular_speed = 0.0

# particles = np.column_stack((
#     np.random.uniform(-6, 6, NUM_PARTICLES),
#     np.random.uniform(-6, 6, NUM_PARTICLES),
#     np.random.uniform(0, 2 * math.pi, NUM_PARTICLES)
# ))

particles = np.column_stack((
    np.random.uniform(START_POSE[0]-1, START_POSE[0]+1, NUM_PARTICLES),
    np.random.uniform(START_POSE[1]-1, START_POSE[1]+1, NUM_PARTICLES),
    np.random.uniform(START_POSE[2]-0.1, START_POSE[2]+0.1, NUM_PARTICLES)
))

running = True
iteration = 1
weights = None
while running:
    start_time = time.time()
    dt = clock.tick_busy_loop(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    speed = (ROBOT_MAX_SPEED if keys[pygame.K_UP] else
             -ROBOT_MAX_SPEED if keys[pygame.K_DOWN] else 0.0)
    angular_speed = (ROBOT_MAX_ANGULAR_SPEED if keys[pygame.K_LEFT] else
                     -ROBOT_MAX_ANGULAR_SPEED if keys[pygame.K_RIGHT] else 0.0)

    robot_x += math.cos(robot_theta) * speed * dt
    robot_y += math.sin(robot_theta) * speed * dt
    robot_theta += angular_speed * dt


    opp_speed = (ROBOT_MAX_SPEED if keys[pygame.K_w] else
             -ROBOT_MAX_SPEED if keys[pygame.K_s] else 0.0)
    opp_angular_speed = (ROBOT_MAX_ANGULAR_SPEED if keys[pygame.K_a] else
                     -ROBOT_MAX_ANGULAR_SPEED if keys[pygame.K_d] else 0.0)

    opp_robot_x += math.cos(opp_robot_theta) * opp_speed * dt
    opp_robot_y += math.sin(opp_robot_theta) * opp_speed * dt
    opp_robot_theta += opp_angular_speed * dt

    # previous_time = time.time()

    real_edges_with_opponent = real_edges.copy()

    # add opponent robot as obstacle
    for i in range(4):
        p0 = (
            opp_robot_x + 1.41*ROBOT_RADIUS * math.cos(opp_robot_theta + i * math.pi / 2),
            opp_robot_y + 1.41*ROBOT_RADIUS * math.sin(opp_robot_theta + i * math.pi / 2),
        )
        p1 = (
            opp_robot_x + 1.41*ROBOT_RADIUS * math.cos(opp_robot_theta + ((i + 1) % 4) * math.pi / 2),
            opp_robot_y + 1.41*ROBOT_RADIUS * math.sin(opp_robot_theta + ((i + 1) % 4) * math.pi / 2),
        )
        real_edges_with_opponent = np.vstack([real_edges_with_opponent, [*p0, *p1]])

    lidar_data = lidar_scan(robot_x, robot_y, robot_theta, obstacle_edges=real_edges_with_opponent)
    # print("LIDAR scan time:", time.time() - previous_time)
    # previous_time = time.time()

    # print("LIDAR data:", lidar_data)

    distances = np.array([[dist] for _, dist, _, _ in lidar_data], dtype=np.float32)

    # Particle filter steps
    particles = predict_particles(particles, (speed, angular_speed, dt), robot_theta)
    # print("Prediction step time:", time.time() - previous_time)
    # previous_time = time.time()

    weights = correct_particles(particles, distances)
    # print("Correction step time:", time.time() - previous_time)
    # previous_time = time.time()

    particles = resample_particles(particles, weights)
    # print("Resampling step time:", time.time() - previous_time)

    estimated_pose = estimate_position(particles, weights)
    # print("Estimated pose: x {:.2f}, y {:.2f}, theta {:.2f}".format(estimated_pose[0], estimated_pose[1], estimated_pose[2]))

    # Draw everything
    screen.fill(BLACK)
    draw_robot(robot_x, robot_y, robot_theta)
    draw_robot(opp_robot_x, opp_robot_y, opp_robot_theta)
    draw_estimated_robot(estimated_pose[0], estimated_pose[1], estimated_pose[2])
    draw_environment(real_edges_with_opponent)
    draw_lidar(robot_x, robot_y, lidar_data)
    draw_particles(particles)
    pygame.display.set_caption(f"Driveable Robot Simulation — {clock.get_fps()} FPS")
    # print text on display showing error between estimated pose and true pose
    error_x = (estimated_pose[0] - robot_x) * 12  # convert to inches
    error_y = (estimated_pose[1] - robot_y) * 12  # convert to inches
    error_theta = estimated_pose[2] - robot_theta
    font = pygame.font.SysFont(None, 24)
    error_text = font.render(f"Error: x {error_x:.2f} in, y {error_y:.2f} in, θ {math.degrees(error_theta):.2f}°", True, WHITE)
    screen.blit(error_text, (10, 10))
    # print("FPS:", 1 / (time.time() - start_time))
    pygame.display.flip()

    iteration += 1

pygame.quit()
