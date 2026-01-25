#pragma once

struct Twist2D {
    double vx = 0.0;
    double vy = 0.0;
    double w = 0.0;
    
    Twist2D() = default;
    Twist2D(double vx, double vy, double w) : vx(vx), vy(vy), w(w) {}
};