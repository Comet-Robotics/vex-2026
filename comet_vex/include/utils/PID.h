#pragma once

#include <algorithm>

/**
 * @class PID
 * @brief A simple PID controller implementation with anti-windup and optional derivative filtering
 */
class PID {
public:
    /**
     * Constructor to initialize PID gains, time step, and internal state
      * @param kp Proportional gain
      * @param ki Integral gain
      * @param kd Derivative gain
      * @param dt Time step for the controller (seconds)
     */
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0, double dt = 0.01)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          integrator_(0.0), prev_meas_(0.0), prev_deriv_(0.0),
          out_min_(-1e9), out_max_(1e9),
          int_min_(-1e9), int_max_(1e9),
          tau_(0.0) {}

    /**
     * Calculate the PID output based on the setpoint and current measurement
      * @param setpoint The desired target value
      * @param measurement The current measured value
      * @param dt_override Optional time step override (if negative, uses internal dt)
      * @return The PID controller output, clamped to output limits
     */
    double update(double setpoint, double measurement, double dt_override = -1.0) {
        double dt = (dt_override > 0.0) ? dt_override : dt_;
        if (dt <= 0.0) return 0.0;

        double error = setpoint - measurement;

        // Proportional term
        double P = kp_ * error;

        // Integral term with anti-windup via clamping
        integrator_ += error * dt;
        integrator_ = std::clamp(integrator_, int_min_, int_max_);
        double I = ki_ * integrator_;

        // Derivative term: derivative on measurement to reduce setpoint "kick"
        double raw_deriv = -(measurement - prev_meas_) / dt; // negative because d(error)/dt = -d(meas)/dt when setpoint is constant
        if (tau_ > 0.0) {
            // First-order low-pass filter: alpha = tau / (tau + dt)
            double alpha = tau_ / (tau_ + dt);
            prev_deriv_ = alpha * prev_deriv_ + (1.0 - alpha) * raw_deriv;
            raw_deriv = prev_deriv_;
        }
        double D = kd_ * raw_deriv;

        prev_meas_ = measurement;

        double output = P + I + D;
        output = std::clamp(output, out_min_, out_max_);

        return output;
    }

    /**
     * Reset the PID controller's internal state (integrator and previous measurement/derivative)
      * @param integrator Optional initial value for the integrator (default is 0.0)
     */
    void reset(double integrator = 0.0) {
        integrator_ = integrator;
        prev_meas_ = 0.0;
        prev_deriv_ = 0.0;
    }

    // Setters
    void setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void setDt(double dt) { if (dt > 0.0) dt_ = dt; }
    void setOutputLimits(double min_out, double max_out) {
        out_min_ = std::min(min_out, max_out);
        out_max_ = std::max(min_out, max_out);
    }
    void setIntegralLimits(double min_int, double max_int) {
        int_min_ = std::min(min_int, max_int);
        int_max_ = std::max(min_int, max_int);
        // clamp current integrator to new bounds
        integrator_ = std::clamp(integrator_, int_min_, int_max_);
    }
    // Set derivative low-pass filter time constant tau (seconds). tau = 0 disables filtering.
    void setDerivativeFilterTau(double tau) { tau_ = std::max(0.0, tau); }

    // Getters
    double kp() const { return kp_; }
    double ki() const { return ki_; }
    double kd() const { return kd_; }
    double dt() const { return dt_; }

private:
    double kp_, ki_, kd_;
    double dt_;

    double integrator_;
    double prev_meas_;
    double prev_deriv_;

    double out_min_, out_max_;
    double int_min_, int_max_;

    double tau_; // derivative filter time constant
};