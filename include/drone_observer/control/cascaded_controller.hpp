//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_CASCADED_CONTROLLER_HPP
#define DRONE_OBSERVER_CASCADED_CONTROLLER_HPP
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <utility>
#include "drone_observer/control/trajectories.hpp"

namespace drone_observer::control {
    struct CascadedGains {
        // Outer loop: Position (P, I, D)
        Eigen::Vector3d Kp_pos{2.0, 2.0, 4.0};
        Eigen::Vector3d Kd_pos{1.5, 1.5, 2.5};
        Eigen::Vector3d Ki_pos{0.05, 0.05, 0.1};

        // Inner loop 1: Attitude angles (P)
        Eigen::Vector3d Kp_att{6.0, 6.0, 4.0};

        // Inner loop 2: Angular rates (P, I, D) - Calibrated for I = [0.005, 0.005, 0.009]
        Eigen::Vector3d Kp_rate{0.25, 0.25, 0.15};
        Eigen::Vector3d Kd_rate{0.01, 0.01, 0.005};
        Eigen::Vector3d Ki_rate{0.001, 0.001, 0.001};
    };

    struct QuadrotorParams {
        double mass{1.0}; // kg
        double gravity{9.81}; // m/s^2
        double arm_length{0.225}; // m
        double km{0.016}; // N*m / N
        Eigen::Vector3d inertia{0.005, 0.005, 0.009};
    };

    class CascadedController {
    public:
        explicit CascadedController(CascadedGains gains = CascadedGains{}, QuadrotorParams params = QuadrotorParams{})
            : gains_(std::move(gains)), params_(std::move(params)) {
        }

        void reset() {
            pos_integral_.setZero();
            rate_integral_.setZero();
            prev_rate_error_.setZero();
            filtered_derivative_.setZero();
            is_first_run_ = true;
        }

        Eigen::Vector4d update(const TrajectoryPoint &target,
                               const Eigen::Matrix<double, 13, 1> &state,
                               double dt,
                               bool return_rotor_forces = false) {
            if (dt <= 0.0) return Eigen::Vector4d::Zero();

            // 1. Extract Estimated States from 13D Quaternion Vector
            const Eigen::Vector3d pos_est = state.segment<3>(0);
            const Eigen::Vector3d vel_est = state.segment<3>(3);

            // Quaternion [qw, qx, qy, qz] at indices 6..9
            Eigen::Quaterniond q_est(state[6], state[7], state[8], state[9]);
            q_est.normalize(); // Ensure valid unit quaternion

            // Convert Quaternion -> Euler angles [roll, pitch, yaw]
            const double roll_est = std::atan2(2.0 * (q_est.w() * q_est.x() + q_est.y() * q_est.z()),
                                               1.0 - 2.0 * (q_est.x() * q_est.x() + q_est.y() * q_est.y()));
            const double pitch_est = std::asin(std::clamp(2.0 * (q_est.w() * q_est.y() - q_est.z() * q_est.x()), -1.0,
                                                          1.0));
            const double yaw_est = std::atan2(2.0 * (q_est.w() * q_est.z() + q_est.x() * q_est.y()),
                                              1.0 - 2.0 * (q_est.y() * q_est.y() + q_est.z() * q_est.z()));
            const Eigen::Vector3d att_est(roll_est, pitch_est, yaw_est);

            // Angular rates [p, q, r] at indices 10..12
            const Eigen::Vector3d rate_est = state.segment<3>(10);

            // 2. Outer Loop: Position & Velocity -> Desired Accelerations
            const Eigen::Vector3d pos_err = target.position - pos_est;
            const Eigen::Vector3d vel_err = target.velocity - vel_est;

            pos_integral_ += pos_err * dt;
            pos_integral_ = pos_integral_.cwiseMin(Eigen::Vector3d(1.0, 1.0, 1.0))
                    .cwiseMax(Eigen::Vector3d(-1.0, -1.0, -1.0));

            const Eigen::Vector3d a_cmd = target.acceleration
                                          + gains_.Kp_pos.cwiseProduct(pos_err)
                                          + gains_.Kd_pos.cwiseProduct(vel_err)
                                          + gains_.Ki_pos.cwiseProduct(pos_integral_);

            // 3. Virtual Force & Total Thrust Computation
            const Eigen::Vector3d F_des = params_.mass * (a_cmd + Eigen::Vector3d(0.0, 0.0, params_.gravity));
            const double T_total = F_des.norm();

            // 4. Desired Euler Angles (Tilt Mapping with Correct Sign for Pitch)
            const double psi_d = target.yaw;
            const double sin_psi = std::sin(psi_d);
            const double cos_psi = std::cos(psi_d);

            const double ax = a_cmd.x();
            const double ay = a_cmd.y();
            const double g = params_.gravity;

            const double phi_d = std::clamp((ax * sin_psi - ay * cos_psi) / g, -M_PI / 4.0, M_PI / 4.0);
            const double theta_d = std::clamp(-(ax * cos_psi + ay * sin_psi) / g, -M_PI / 4.0, M_PI / 4.0);
            const Eigen::Vector3d att_des(phi_d, theta_d, psi_d);

            // 5. Inner Loop 1: Attitude Error -> Desired Angular Rates
            Eigen::Vector3d att_err = att_des - att_est;
            att_err.z() = wrapAngle(att_err.z());

            const Eigen::Vector3d rate_des = gains_.Kp_att.cwiseProduct(att_err)
                                             + Eigen::Vector3d(0.0, 0.0, target.yaw_rate);

            // 6. Inner Loop 2: Rate Error -> Desired Body Torques
            const Eigen::Vector3d rate_err = rate_des - rate_est;

            rate_integral_ += rate_err * dt;
            rate_integral_ = rate_integral_.cwiseMin(Eigen::Vector3d(0.5, 0.5, 0.5))
                    .cwiseMax(Eigen::Vector3d(-0.5, -0.5, -0.5));

            // Derivative calculation with first-run guard & Low-Pass Filter (alpha_lpf = 0.2)
            Eigen::Vector3d raw_derivative = Eigen::Vector3d::Zero();
            if (!is_first_run_) {
                raw_derivative = (rate_err - prev_rate_error_) / dt;
            } else {
                is_first_run_ = false;
            }
            prev_rate_error_ = rate_err;

            constexpr double alpha_lpf = 0.2;
            filtered_derivative_ = alpha_lpf * raw_derivative + (1.0 - alpha_lpf) * filtered_derivative_;

            // Desired body torques
            const Eigen::Vector3d tau = gains_.Kp_rate.cwiseProduct(rate_err)
                                        + gains_.Kd_rate.cwiseProduct(filtered_derivative_)
                                        + gains_.Ki_rate.cwiseProduct(rate_integral_);

            // 7. Output Control Wrench
            const Eigen::Vector4d wrench(T_total, tau.x(), tau.y(), tau.z());

            if (!return_rotor_forces) {
                return wrench;
            }

            return allocateMotorForces(wrench);
        }

        [[nodiscard]] Eigen::Vector4d allocateMotorForces(const Eigen::Vector4d &wrench) const {
            const double d = params_.arm_length / std::numbers::sqrt2;
            const double km = params_.km;

            Eigen::Matrix4d A;
            A << 1.0, 1.0, 1.0, 1.0,
                    -d, -d, d, d,
                    -d, d, d, -d,
                    km, -km, km, -km;

            Eigen::Vector4d forces = A.colPivHouseholderQr().solve(wrench);
            return forces.cwiseMax(0.0);
        }

    private:
        static double wrapAngle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        CascadedGains gains_;
        QuadrotorParams params_;

        Eigen::Vector3d pos_integral_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d rate_integral_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d prev_rate_error_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d filtered_derivative_{Eigen::Vector3d::Zero()};
        bool is_first_run_{true};
    };
}
#endif //DRONE_OBSERVER_CASCADED_CONTROLLER_HPP
