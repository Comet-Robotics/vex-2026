import pygame
import numpy as np
import math

WIDTH, HEIGHT = 1000, 700
BG = (25, 25, 25)

PATH_COLOR = (120, 255, 200)
CONTROL_COLOR = (255, 180, 80)
POINT_COLOR = (240, 240, 240)
ROBOT_COLOR = (255, 80, 80)

POINT_RADIUS = 8
SPEED = 120.0


# --------------------------------------------------
# Bezier math
# --------------------------------------------------
def bezier(p0, c0, c1, p1, t):
    """Cubic Bezier position"""
    u = 1 - t
    return (
        u**3 * p0 +
        3*u**2*t * c0 +
        3*u*t**2 * c1 +
        t**3 * p1
    )


def bezier_derivative(p0, c0, c1, p1, t):
    """Tangent"""
    u = 1 - t
    return (
        3*u**2*(c0 - p0) +
        6*u*t*(c1 - c0) +
        3*t**2*(p1 - c1)
    )


def build_curve(p0, c0, c1, p1, samples=300):
    pts = []
    for i in range(samples + 1):
        t = i / samples
        pts.append(bezier(p0, c0, c1, p1, t))
    return np.array(pts)


# --------------------------------------------------
# Drawing helpers
# --------------------------------------------------
def draw_point(screen, p, color):
    pygame.draw.circle(screen, color, p.astype(int), POINT_RADIUS)


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


# --------------------------------------------------
# Main
# --------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    # Control points
    p0 = np.array([150., 550.])
    c0 = np.array([350., 450.])
    c1 = np.array([650., 250.])
    p1 = np.array([850., 150.])

    dragging = None
    t = 0.0

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        mouse = np.array(pygame.mouse.get_pos(), dtype=float)

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False

            if e.type == pygame.MOUSEBUTTONDOWN:
                if near(mouse, p0): dragging = "p0"
                elif near(mouse, c0): dragging = "c0"
                elif near(mouse, c1): dragging = "c1"
                elif near(mouse, p1): dragging = "p1"

            if e.type == pygame.MOUSEBUTTONUP:
                dragging = None

        # Drag behavior
        if dragging:
            locals()[dragging][:] = mouse

        curve = build_curve(p0, c0, c1, p1)

        # robot motion
        t += (SPEED * dt) / 900
        if t > 1:
            t = 0

        pos = bezier(p0, c0, c1, p1, t)
        d = bezier_derivative(p0, c0, c1, p1, t)
        theta = math.atan2(d[1], d[0])

        # draw
        screen.fill(BG)

        pygame.draw.lines(screen, PATH_COLOR, False, curve, 3)

        # control polygon
        pygame.draw.lines(screen, CONTROL_COLOR, False, [p0, c0, c1, p1], 2)

        draw_point(screen, p0, POINT_COLOR)
        draw_point(screen, c0, CONTROL_COLOR)
        draw_point(screen, c1, CONTROL_COLOR)
        draw_point(screen, p1, POINT_COLOR)

        draw_robot(screen, pos, theta)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
