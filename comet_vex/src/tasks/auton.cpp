#include "subsystems.h"
#include "tasks/auton.h"

using namespace pros;

enum class AutonMode
{
    TEST
};

/*
 *  VS is 2v2 auton
 *  SKILLS is self explanitory
 *  TEST is testing any autons or tuning
 */
inline constexpr AutonMode MODE = AutonMode::TEST;

void autonomousTest()
{
    while (true)
    {
        drivebase->setModuleSpeeds(0.5, 0, 0);
        drivebase->update();

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void autonomousTest2()
{
    drivebase->setTrajectory("paths/testPath.traj");
    while (!drivebase->atEnd())
    {
        drivebase->update();
    }
}

void runPath(const Trajectory &path, std::function<void(const std::string &)> onEvent = nullptr)
{
    if (path.getPoints().empty())
        return;

    TrajectoryFollower follower(constants::autonomous::TIME_TOLERANCE);
    follower.reset(path);

    while (!follower.isFinished())
    {
        Pose2D currentPose = drivebase->getPose();
        ChassisSpeeds speeds = follower.update(currentPose, onEvent);

        drivebase->basicSetModuleSpeeds(speeds.vx, speeds.vy, speeds.omega);
        drivebase->update();

        printf("Current Pose: X: %.2f, Y: %.2f, Heading: %.2f\n", currentPose.x, currentPose.y, AngleUtils::toDegrees(currentPose.heading));

        pros::delay(constants::TELEOP_POLL_TIME);
    }

    drivebase->goToPose(path.getPoints().back().pose);
}

void wait(double seconds)
{
    uint32_t start = pros::millis();
    while (pros::millis() - start < seconds * 1000.0)
    {
        drivebase->setModuleSpeeds(0, 0, 0, true);
        drivebase->update();
        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void autonomousTest3()
{
    Trajectory trajectory("/usd/paths/Auto.traj");
    std::vector<Trajectory> paths = trajectory.getSplits();

    drivebase->setPose(trajectory.getStart().pose);

    // debug: print paths
    for (size_t i = 0; i < paths.size(); ++i)
    {
        printf("Path %zu:\n", i);
        // print front and back of each path to verify splits look correct
        if (!paths[i].getPoints().empty())
        {
            const auto &start = paths[i].getPoints().front();
            const auto &end = paths[i].getPoints().back();
            printf("  StartTime: %.2f sec, StartPose: (x=%.2f, y=%.2f, heading=%.2f deg)\n", start.t, start.pose.x, start.pose.y, AngleUtils::toDegrees(start.pose.heading));
            printf("  EndTime: %.2f sec, EndPose: (x=%.2f, y=%.2f, heading=%.2f deg)\n", end.t, end.pose.x, end.pose.y, AngleUtils::toDegrees(end.pose.heading));
        }
    }

    if (paths.empty())
        return;

    for (size_t i = 0; i < paths.size(); ++i)
    {
        runPath(paths[i], [](const std::string &eventName)
                { printf("Event triggered: %s\n", eventName.c_str()); });
        wait(2);
    }

    // lock wheels after finishing to prevent pushing
    while (true)
    {
        drivebase->xWheels();
        drivebase->update();
        pros::delay(constants::TELEOP_POLL_TIME); // Added delay to prevent CPU lockup
    }
}

void pidToPoseTest()
{
    drivebase->resetPose();
    drivebase->goToPose({0, 48, AngleUtils::toRadians(45)});
    drivebase->goToPose({36, 12, AngleUtils::toRadians(90)});
    drivebase->goToPose({36, 0, AngleUtils::toRadians(90)});
    drivebase->goToPose({36, 24, AngleUtils::toRadians(90)});
    drivebase->goToPose({0, 0, AngleUtils::toRadians(0)});
}

void autonomous()
{
    // drivebase->setAutonomous(true);

    switch (MODE)
    {
    case AutonMode::TEST:
        pros::lcd::print(0, "Running autonomous test");
        autonomousTest3();
        break;
    default:
        break;
    }
};

void autonomous_initialize() {

};