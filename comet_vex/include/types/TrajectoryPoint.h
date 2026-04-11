#pragma once

#include "Pose2D.h"

struct TrajectoryPoint {
    Pose2D pose;
    Pose2D velocity;
    double t;
};