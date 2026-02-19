#pragma once

#include "constants.h"
#include <cmath>
#include "utils/Math.h"

using namespace constants::drivebase;
class Drivebase : public lemlib::Chassis
{
public:
    Drivebase() : lemlib::Chassis(DRIVETRAIN, LATERAL_CONTROLLER, ANGULAR_CONTROLLER, SENSORS) {}

    /**
     * Calibrate the chassis, optionally using the IMU. This should be called at the beginning of autonomous and opcontrol to ensure accurate sensor readings. If useIMU is true, the IMU will be calibrated and the function will wait until calibration is complete before returning.
     * @param useIMU Whether to calibrate the IMU as part of chassis calibration
     */
    void calibrateChassis(bool useIMU)
    {
        this->calibrate(useIMU);
        while (IMU.is_calibrating())
        {
            pros::delay(10);
        }
    }

    /**
     * Get the IMU used for the drivebase. This is useful for getting the current heading of the robot, which is used for odometry and other functions. It can also be used for other purposes, such as balancing a robot on a platform.
     * @return The IMU used for the drivebase
     */
    pros::IMU getIMU()
    {
        return IMU;
    }

    /**
     * Get the current heading of the robot in degrees. This is useful for odometry and other functions that require the current heading of the robot. The heading is returned in the range of 0 to 360 degrees, where 0 degrees is facing up, 90 degrees is facing right, 180 degrees is facing down, and 270 degrees is facing left.
     */
    double getAngle()
    {
        return IMU.get_heading();
    }

    /**
     * A simple function to control the drivebase with a drive and turn value, where drive is the forward/backward movement and turn is the left/right movement. This is useful for controlling the robot with a controller, where the drive value is typically the left stick y-axis and the turn value is typically the right stick x-axis. The values are expected to be in the range of -127 to 127, which is the standard range for VEX motor control.
     * @param drive The forward/backward movement value, in the range of -127 to 127
     * @param turn The left/right movement value, in the range of -127 to 127
     */
    void errorDrive(float drive, float turn)
    {
        drive /= 127.0;
        turn /= 127.0;

        turn /= ((drive < 0.5) ? 1.5 : 1.2);

        // int driveSign = ((drive >= 0)? 1 : -1);
        // int turnSign  = ((turn >= 0)?  1 : -1);

        // drive = driveSign * pow(drive, 2);
        // turn = turnSign * pow(turn, 2);

        LEFT_MOTORS.move_voltage((drive + turn) * 12000);
        RIGHT_MOTORS.move_voltage((drive - turn) * 12000);
    }

    /**
     * A function that applies a signed power curve to the drive and turn values, which allows for finer control at low speeds while still allowing for full power at high speeds. The drive and turn values are first normalized to the range of -1 to 1, then the signed power function is applied, then the values are normalized again for a maximum total magnitude of 1, then scaled up to send voltage to the motors. The driveExponent and turnExponent constants can be adjusted to change the shape of the curve, with higher values resulting in a steeper curve and more sensitivity at low speeds.
     * @param drive The forward/backward movement value, in the range of -127 to 127
     * @param turn The left/right movement value, in the range of -127 to 127
     */
    void signedDrive(float drive, float turn)
    {
        drive /= 127.0;
        turn /= 127.0;

        drive = signedPower(drive, driveExponent);
        turn = signedPower(turn, turnExponent);

        // normalize for maximum magnitude of 1
        float maxMagnitude = std::max(std::abs(drive + turn), std::abs(drive - turn));
        if (maxMagnitude > 1)
        {
            drive /= maxMagnitude;
            turn /= maxMagnitude;
        }

        LEFT_MOTORS.move_voltage((drive + turn) * 12000);
        RIGHT_MOTORS.move_voltage((drive - turn) * 12000);
    }

    /**
     * A function that turns the robot to a point and then moves to that point. This is useful for autonomous routines where the robot needs to move to a specific location on the field. The function takes in the x and y coordinates of the point, as well as optional parameters for the turn and move functions, such as maximum speed and timeout. The function will first turn to face the point, and then move to the point. If async is true, the function will return immediately after starting the turn, and the move will start once the turn is complete. If async is false, the function will block until both the turn and move are complete.
     * @param x The x coordinate of the point to move to
     * @param y The y coordinate of the point to move to
     * @param timeout The maximum time to allow for each movement, in milliseconds. Default is 5000 (5 seconds)
     * @param turnParams Optional parameters for the turnToPoint function
     * @param moveParams Optional parameters for the moveToPoint function
     * @param async Whether to run the movements asynchronously. If true, the function will return immediately after starting the turn, and the move will start once the turn is complete. If false, the function will block until both movements are complete. Default is true.
     */
    void turnThenMoveToPoint(double x, double y, int timeout = DEFAULT_TIMEOUT, lemlib::TurnToPointParams turnParams = {}, lemlib::MoveToPointParams moveParams = {}, bool async = true)
    {
        turnToPoint(x, y, timeout, turnParams, async);
        moveToPoint(x, y, timeout, moveParams, async);
    }

    /**
     * A simpler version of turnThenMoveToPoint that uses default parameters for the turn and move functions, and runs asynchronously. This is useful for quickly moving to a point without needing to specify any parameters. The function will turn to face the point and then move to the point, using the default parameters for each movement.
     * @param x The x coordinate of the point to move to
     * @param y The y coordinate of the point to move to
     * @param async Whether to run the movements asynchronously. If true, the function will return immediately after starting the turn, and the move will start once the turn is complete. If false, the function will block until both movements are complete. Default is true.
     */
    void turnThenMoveToPoint(double x, double y, bool async)
    {
        turnToPoint(x, y, DEFAULT_TIMEOUT, {}, async);
        moveToPoint(x, y, DEFAULT_TIMEOUT, {}, async);
    }

    /**
     * A version of setPose that takes in the heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise. The function will convert the COMET angle to lemlib angle format, where 0 degrees is facing up and positive angles are clockwise, before calling the setPose function.
     * @param x The x coordinate of the pose
     * @param y The y coordinate of the pose
     * @param heading The heading of the pose in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise
     */
    void setPoseComet(double x, double y, double heading)
    {
        lemlib::Chassis::setPose(x, y, cometToLemlibAngle(heading));
    }

    /**
     * A version of turnThenMoveToPoint that takes in the heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise. The function will convert the COMET angle to lemlib angle format, where 0 degrees is facing up and positive angles are clockwise, before calling the turnToHeading function.
     * @param heading The heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise
     * @param timeout The maximum time to allow for each movement, in milliseconds. Default is 5000 (5 seconds)
     * @param turnParams Optional parameters for the turnToPoint function
     * @param async Whether to run the movements asynchronously. If true, the function will return immediately after starting the turn, allowing other code to run while the robot is turning. If false, the function will block until the turn is complete. Default is true.
     */
    void turnToHeadingComet(double heading, int timeout = DEFAULT_TIMEOUT, lemlib::TurnToHeadingParams turnParams = {}, bool async = true)
    {
        heading = cometToLemlibAngle(heading);
        turnToHeading(heading, timeout, turnParams, async);
    }

    /**
     * A version of swingToHeading that takes in the heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise. The function will convert the COMET angle to lemlib angle format, where 0 degrees is facing up and positive angles are clockwise, before calling the swingToHeading function.
     * @param heading The heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise
     * @param side The side that is locked during the turn. LEFT will swing towards the left (counter-clockwise) and RIGHT will swing towards the right (clockwise). AUTO will swing in the direction with the shortest distance to the target heading.
     * @param timeout The maximum time to allow for each movement, in milliseconds. Default is 5000 (5 seconds)
     * @param turnParams Optional parameters for the swingToHeading function
     * @param async Whether to run the movement asynchronously. If true, the function will return immediately after starting the turn, allowing other code to run while the robot is turning. If false, the function will block until the turn is complete. Default is true.
     */
    void swingToHeadingComet(double heading, lemlib::DriveSide side, int timeout = DEFAULT_TIMEOUT, lemlib::SwingToHeadingParams turnParams = {}, bool async = true)
    {
        heading = cometToLemlibAngle(heading);
        swingToHeading(heading, side, timeout, turnParams, async);
    }

    /**
     * A version of moveToPose that takes in the heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise. The function will convert the COMET angle to lemlib angle format, where 0 degrees is facing up and positive angles are clockwise, before calling the moveToPose function.
     * @param x The x coordinate of the point to move to
     * @param y The y coordinate of the point to move to
     * @param heading The heading of the point in COMET angle format, where 0 degrees is facing right and positive angles are counter-clockwise
     * @param timeout The maximum time to allow for the movement, in milliseconds. Default is 5000 (5 seconds)
     * @param moveParams Optional parameters for the moveToPose function
     * @param async Whether to run the movement asynchronously. If true, the function will return immediately after starting the movement, allowing other code to run while the robot is moving. If false, the function will block until the movement is complete. Default is true.
     */
    void moveToPoseComet(double x, double y, double heading, int timeout = DEFAULT_TIMEOUT, lemlib::MoveToPoseParams moveParams = {}, bool async = true)
    {
        heading = cometToLemlibAngle(heading);
        moveToPose(x, y, heading, timeout, moveParams, async);
    }

    /**
     * Wait until the robot is stationary, meaning that it has reached its target and is no longer in motion. This can be used after a movement function to ensure that the robot has finished moving before proceeding with the next action. The function checks if the robot is in motion using the isInMotion() function, and if it is, it waits for a short interval before checking again. The default interval is 20 milliseconds, but it can be adjusted as needed.
     * @param interval The time to wait between checks for whether the robot is in motion, in milliseconds. Default is 20 ms.
     */
    void waitUntilStationary(uint32_t interval = 20)
    {
        while (isInMotion())
        {
            pros::delay(interval);
        }
    }

    /**
     * A helper function that applies an exponent to a value while preserving the sign of the value. This is useful for applying a non-linear curve to controller inputs, where you want to have finer control at lower speeds while still allowing for full power at higher inputs. The function takes in a power value and an exponent, and returns the power value raised to the exponent, with the original sign of the power value preserved.
     *
     * Base must be in the range [-1, 1]. Exponent must be greater than 0.
     * @param base The value to apply the exponent to
     * @param exponent The exponent to apply to the base value
     */
    double signedPower(double base, double exponent)
    {
        int sign = (base >= 0) ? 1 : -1;
        base = std::pow(std::abs(base), exponent);
        base *= sign;
        return base;
    }

private:
    double driveExponent = 1.5;
    double turnExponent = 1.5;
};