#pragma once

#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include "utils/Pose2D.h"
#include "utils/Math.h"

/**
 * @class PoseEKF
 * @brief Extended Kalman Filter for robot pose estimation
 *
 * This class implements an Extended Kalman Filter to estimate the robot's pose (x, y, theta) based on control inputs (drive and turn)
 * and measurements from a particle filter. The state vector includes the robot's position, orientation, linear velocity, and angular
 * velocity. The EKF consists of a prediction step that updates the state based on the motion model and a correction step that incorporates
 * measurements to refine the pose estimate.
 */
class PoseEKF
{
public:
    /**
     * Constructor initializes the state vector and covariance matrices
     * State vector x: [x, y, theta, v, omega]
     * P: initial covariance (uncertainty in the state estimate)
     * Q: process noise covariance (uncertainty in the motion model)
     * R: measurement noise covariance (uncertainty in the sensor measurements)
     */
    PoseEKF()
    {
        x.setZero();

        P.setIdentity();
        P *= 0.01; // initial uncertainty

        Q.setZero();
        Q.diagonal() << 0.002, // x noise
            0.002,             // y noise
            0.001,             // theta noise
            0.5,               // v noise
            0.5,               // omega noise

            R.setZero();
        R.diagonal() << 0.05, // PF x noise
            0.05,             // PF y noise
            0.02;             // PF theta noise
    }

    /**
     * Predict the new pose based on the current state and control inputs (drive and turn)
     * @param drive The linear velocity (forward/backward)
     * @param turn The angular velocity (turning rate)
     * @param dt The time step for the prediction
     */
    void predict(double drive, double turn, double dt)
    {
        double theta = x(2);

        // predict new state
        x(0) += drive * std::cos(theta) * dt; // x
        x(1) += drive * std::sin(theta) * dt; // y
        x(2) += turn * dt;                    // theta
        x(3) = drive;                         // v
        x(4) = turn;                          // omega

        x(2) = normalizeAngle(x(2)); // normalize theta

        // Jacobian to linearize motion model
        Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();
        F(0, 2) = -drive * std::sin(theta) * dt; // dx/dtheta
        F(0, 3) = std::cos(theta) * dt;          // dx/dv
        F(1, 2) = drive * std::cos(theta) * dt;  // dy/dtheta
        F(1, 3) = std::sin(theta) * dt;          // dy/dv
        F(2, 4) = dt;                            // dtheta/domega

        P = F * P * F.transpose() + Q;
    }

    /**
     * Correct the pose estimate using a measurement from the particle filter
     * @param pf_scan Pose2D measurement from the particle filter (x, y in feet, theta in degrees)
     */
    void correct(const Pose2D &pf_scan)
    {
        Eigen::Vector3d z;                        // measurement vector from particle filter
        z << pf_scan.x, pf_scan.y, pf_scan.theta; // x, y, theta from particle filter

        Eigen::Matrix<double, 3, 5> H;
        H.setZero();
        H(0, 0) = 1; // dz/dx
        H(1, 1) = 1; // dz/dy
        H(2, 2) = 1; // dz/dtheta

        Eigen::Vector3d y = z - x.head<3>(); // measurement residual
        y(2) = normalizeAngle(y(2));         // normalize angle residual

        Eigen::Matrix3d S = H * P * H.transpose() + R;                   // measurement covariance
        Eigen::Matrix<double, 5, 3> K = P * H.transpose() * S.inverse(); // Kalman gain

        x += K * y; // update state estimate

        Eigen::Matrix<double, 5, 5> I = Eigen::Matrix<double, 5, 5>::Identity();
        P = (I - K * H) * P; // update covariance

        x(2) = normalizeAngle(x(2)); // normalize theta
    }

    /**
     * Get the current pose estimate as a Pose2D struct
     * @return Pose2D containing x, y, and theta (in radians)
     * Note: theta is normalized to the range [-pi, pi]
     */
    Pose2D getPose() const
    {
        return {x(0), x(1), x(2)};
    }

private:
    Eigen::VectorXd x; // 5x1 state vector: [x, y, theta, v, omega]
    Eigen::MatrixXd P; // 5x5 covariance
    Eigen::MatrixXd Q; // 5x5 process noise
    Eigen::MatrixXd R; // 3x3 measurement noise
};