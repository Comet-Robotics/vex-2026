import pygame
import numpy as np
import math

# -------------------------------------------------
# CONFIG
# -------------------------------------------------

WIDTH, HEIGHT = 1200, 800
BG = (25, 25, 25)

ROBOT_COLOR = (255, 80, 80)
POSE_COLOR = (230, 230, 230)
HEADING_COLOR = (255, 180, 80)

SPEED = 250.0
K_MAX = 0.03  # curvature safety limit (tune to taste)


# -------------------------------------------------
# Quintic spline math
# -------------------------------------------------

def quintic_coeffs(p0, v0, a0, p1, v1, a1):
    M = np.array([
        [1,0,0,0,0,0],
        [0,1,0,0,0,0],
        [0,0,2,0,0,0],
        [1,1,1,1,1,1],
        [0,1,2,3,4,5],
        [0,0,2,6,12,20]
    ], dtype=float)

    b = np.array([p0, v0, a0, p1, v1, a1], dtype=float)

    return np.linalg.solve(M, b)


def poly(c, s):
    return sum(c[i] * s**i for i in range(6))


def poly_d(c, s):
    return sum(i*c[i]*s**(i-1) for i in range(1,6))


def poly_dd(c, s):
    return sum(i*(i-1)*c[i]*s**(i-2) for i in range(2,6))


class QuinticSegment:
    def __init__(self, p0, theta0, p1, theta1):
        dist = np.linalg.norm(p1 - p0)
        scale = dist * 0.7

        v0 = np.array([math.cos(theta0), math.sin(theta0)]) * scale
        v1 = np.array([math.cos(theta1), math.sin(theta1)]) * scale

        a0 = np.zeros(2)
        a1 = np.zeros(2)

        self.cx = quintic_coeffs(p0[0], v0[0], 0, p1[0], v1[0], 0)
        self.cy = quintic_coeffs(p0[1], v0[1], 0, p1[1], v1[1], 0)

    def sample(self, s):
        x = poly(self.cx, s)
        y = poly(self.cy, s)

        dx = poly_d(self.cx, s)
        dy = poly_d(self.cy, s)

        ddx = poly_dd(self.cx, s)
        ddy = poly_dd(self.cy, s)

        theta = math.atan2(dy, dx)

        k = abs(dx*ddy - dy*ddx) / (dx*dx + dy*dy)**1.5

        return np.array([x,y]), theta, k


# -------------------------------------------------
# Waypoint
# -------------------------------------------------

class Pose:
    def __init__(self, pos, theta):
        self.pos = np.array(pos, dtype=float)
        self.theta = theta

    def heading_tip(self):
        L = 60
        return self.pos + np.array([
            math.cos(self.theta)*L,
            math.sin(self.theta)*L
        ])


# -------------------------------------------------
# Build full path
# -------------------------------------------------

def build_path(poses, samples=80):
    pts = []
    d1s = []
    curv = []

    for i in range(len(poses)-1):
        seg = QuinticSegment(
            poses[i].pos, poses[i].theta,
            poses[i+1].pos, poses[i+1].theta
        )

        for j in range(samples):
            s = j/(samples-1)
            p, theta, k = seg.sample(s)

            dx = poly_d(seg.cx, s)
            dy = poly_d(seg.cy, s)

            pts.append(p)
            d1s.append((dx, dy))
            curv.append(k)

    pts = np.array(pts)
    d1s = np.array(d1s)
    curv = np.array(curv)

    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    dist = np.insert(np.cumsum(d), 0, 0)

    return pts, d1s, curv, dist



# -------------------------------------------------
# Drawing helpers
# -------------------------------------------------

def draw_robot(screen, pos, theta):
    w, h = 40, 25

    corners = np.array([
        [-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]
    ])

    R = np.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])

    pts = corners @ R.T + pos
    pygame.draw.polygon(screen, ROBOT_COLOR, pts)


def curvature_color(k):
    t = min(k / K_MAX, 1.0)
    return (255*t, 255*(1-t), 80)


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    poses = [
        Pose((200,600), -0.5),
        Pose((600,500), 0.3),
        Pose((1000,200), 2.5)
    ]

    playing = True
    show_curvature = True

    dragging = None
    robot_dist = 0

    while True:
        dt = clock.tick(60)/1000

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return

            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    playing = not playing
                if e.key == pygame.K_c:
                    show_curvature = not show_curvature
                if e.key == pygame.K_a:
                    poses.append(Pose(pygame.mouse.get_pos(), 0))
                if e.key == pygame.K_d and len(poses) > 2:
                    m = np.array(pygame.mouse.get_pos())
                    i = np.argmin([np.linalg.norm(p.pos-m) for p in poses])
                    poses.pop(i)

            if e.type == pygame.MOUSEBUTTONDOWN:
                m = np.array(pygame.mouse.get_pos())

                for p in poses:
                    if np.linalg.norm(p.pos-m) < 12:
                        dragging = ("pos", p)
                    elif np.linalg.norm(p.heading_tip()-m) < 12:
                        dragging = ("theta", p)

            if e.type == pygame.MOUSEBUTTONUP:
                dragging = None

        if dragging:
            m = np.array(pygame.mouse.get_pos())
            mode, pose = dragging
            if mode == "pos":
                pose.pos = m
            else:
                d = m - pose.pos
                pose.theta = math.atan2(d[1], d[0])

        pts, d1s, curv, dists = build_path(poses)

        if playing:
            robot_dist += SPEED*dt
            robot_dist %= dists[-1]

        idx = np.searchsorted(dists, robot_dist)
        idx = min(idx, len(pts)-2)

        t = (robot_dist - dists[idx])/(dists[idx+1]-dists[idx]+1e-9)
        pos = pts[idx]*(1-t) + pts[idx+1]*t

        dx = d1s[idx][0]*(1-t) + d1s[idx+1][0]*t
        dy = d1s[idx][1]*(1-t) + d1s[idx+1][1]*t
        theta = math.atan2(dy, dx)


        # draw
        screen.fill(BG)

        for i in range(len(pts)-1):
            color = curvature_color(curv[i]) if show_curvature else (80,200,255)
            pygame.draw.line(screen, color, pts[i], pts[i+1], 3)

        for p in poses:
            pygame.draw.circle(screen, POSE_COLOR, p.pos, 8)
            pygame.draw.line(screen, HEADING_COLOR, p.pos, p.heading_tip(), 2)

        draw_robot(screen, pos, theta)

        pygame.display.flip()


if __name__ == "__main__":
    main()
