import pygame
import numpy as np
import math

WIDTH, HEIGHT = 1100, 750
BG = (25, 25, 25)

PATH_COLOR = (80, 200, 255)
POINT_COLOR = (240, 240, 240)
HEADING_COLOR = (255, 180, 80)
ROBOT_COLOR = (255, 80, 80)

POINT_RADIUS = 8
SPEED = 150


# -------------------------------------------------
# Hermite math
# -------------------------------------------------
def hermite(p0, p1, t0, t1, s):
    h00 = 2*s**3 - 3*s**2 + 1
    h10 = s**3 - 2*s**2 + s
    h01 = -2*s**3 + 3*s**2
    h11 = s**3 - s**2
    return h00*p0 + h10*t0 + h01*p1 + h11*t1


def hermite_d(p0, p1, t0, t1, s):
    h00 = 6*s**2 - 6*s
    h10 = 3*s**2 - 4*s + 1
    h01 = -6*s**2 + 6*s
    h11 = 3*s**2 - 2*s
    return h00*p0 + h10*t0 + h01*p1 + h11*t1

def hermite_dd(p0, p1, t0, t1, s):
    h00 = 12*s - 6
    h10 = 6*s - 4
    h01 = -12*s + 6
    h11 = 6*s - 2
    return h00*p0 + h10*t0 + h01*p1 + h11*t1

def curvature(p0, p1, t0, t1, s):
    d = hermite_d(p0, p1, t0, t1, s)
    dd = hermite_dd(p0, p1, t0, t1, s)

    num = d[0]*dd[1] - d[1]*dd[0]
    denom = (d[0]**2 + d[1]**2)**1.5

    if denom < 1e-6:
        return 0

    return num / denom

# -------------------------------------------------
# Waypoint
# -------------------------------------------------
class Waypoint:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.theta = 0.0
        self.scale = 200

    def tangent(self):
        return np.array([
            math.cos(self.theta),
            math.sin(self.theta)
        ]) * self.scale


# -------------------------------------------------
# Draw robot
# -------------------------------------------------
def draw_robot(screen, pos, theta):
    w, h = 40, 25

    corners = np.array([
        [-w/2, -h/2],
        [ w/2, -h/2],
        [ w/2,  h/2],
        [-w/2,  h/2],
    ])

    R = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta),  math.cos(theta)]
    ])

    pts = (corners @ R.T) + pos
    pygame.draw.polygon(screen, ROBOT_COLOR, pts)


def near(mouse, p):
    return np.linalg.norm(mouse - p) < 12


# -------------------------------------------------
# Build full path
# -------------------------------------------------
def build_segments(points, samples=60):
    curve = []

    for i in range(len(points)-1):
        a = points[i]
        b = points[i+1]

        for j in range(samples):
            s = j / samples
            curve.append(
                hermite(a.pos, b.pos, a.tangent(), b.tangent(), s)
            )

    curve.append(points[-1].pos)
    return np.array(curve)


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    points = [
        Waypoint((150, 600)),
        Waypoint((600, 350)),
        Waypoint((950, 150))
    ]

    dragging = None
    robot_dist = 0.0

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        mouse = np.array(pygame.mouse.get_pos(), dtype=float)

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            # Add point
            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 3:
                points.append(Waypoint(mouse))

            # drag
            if e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                for i, p in enumerate(points):
                    if near(mouse, p.pos):
                        dragging = i

            if e.type == pygame.MOUSEBUTTONUP:
                dragging = None

            # delete last
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_BACKSPACE and len(points) > 2:
                    points.pop()

        # dragging
        if dragging is not None:
            points[dragging].pos[:] = mouse

        keys = pygame.key.get_pressed()

        # rotate selected point with arrow keys
        if dragging is not None:
            if keys[pygame.K_LEFT]:
                points[dragging].theta -= 2*dt
            if keys[pygame.K_RIGHT]:
                points[dragging].theta += 2*dt
            if keys[pygame.K_UP]:
                points[dragging].scale += 200*dt
            if keys[pygame.K_DOWN]:
                points[dragging].scale -= 200*dt
                points[dragging].scale = max(20, points[dragging].scale)

        # Check curvature of each segment
        for i in range(len(points)-1):
            a = points[i]
            b = points[i+1]
            max_curv = 0.0
            for j in range(1, 60):
                s = j / 60
                curv = abs(curvature(a.pos, b.pos, a.tangent(), b.tangent(), s))
                if curv > max_curv:
                    max_curv = curv
            print(f"Segment {i} to {i+1} max curvature: {max_curv:.4f}")

        curve = build_segments(points)

        # build arc length table
        dists = [0]
        for i in range(1, len(curve)):
            dists.append(dists[-1] + np.linalg.norm(curve[i] - curve[i-1]))

        dists = np.array(dists)
        total_length = dists[-1]

        robot_dist += SPEED * dt
        robot_dist %= total_length

        i = np.searchsorted(dists, robot_dist) - 1
        i = np.clip(i, 0, len(curve)-2)

        seg_len = dists[i+1] - dists[i]
        alpha = (robot_dist - dists[i]) / seg_len

        pos = (1-alpha)*curve[i] + alpha*curve[i+1]

        tangent = curve[i+1] - curve[i]
        theta = math.atan2(tangent[1], tangent[0])

        # draw
        screen.fill(BG)

        pygame.draw.lines(screen, PATH_COLOR, False, curve, 3)

        for p in points:
            pygame.draw.circle(screen, POINT_COLOR, p.pos.astype(int), POINT_RADIUS)
            pygame.draw.line(
                screen,
                HEADING_COLOR,
                p.pos,
                p.pos + p.tangent(),
                2
            )

        draw_robot(screen, pos, theta)

        fps = clock.get_fps()
        pygame.display.set_caption(f"Hermite Test - FPS: {fps:.2f}")

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
