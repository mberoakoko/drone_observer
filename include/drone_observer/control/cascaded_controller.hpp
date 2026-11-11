//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_CASCADED_CONTROLLER_HPP
#define DRONE_OBSERVER_CASCADED_CONTROLLER_HPP

        // Inner loop: Attitude angles (P)
        Eigen::Vector3d Kp_att{6.0, 6.0, 4.0};

        // Inner loop: Angular rates (P, I, D)
        Eigen::Vector3d Kp_rate{8.0, 8.0, 5.0};
        Eigen::Vector3d Kd_rate{0.5, 0.5, 0.3};
        Eigen::Vector3d Ki_rate{0.01, 0.01, 0.01};
    };

    struct QuadrotorParams {
        double mass{1.0}; // kg
        double gravity{9.81}; // m/s^2
        double arm_length{0.225}; // m (distance from center to motor)
        double km{0.016}; // Torque-to-thrust ratio coefficient (N*m / N)
        Eigen::Vector3d inertia{0.005, 0.005, 0.009}; // Principal moments of inertia [Ixx, Iyy, Izz]
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
        }

        /**
         * @brief Computes 4D control output from a target trajectory point and estimated 12D state.
         * @param target Desired trajectory point (position, velocity, acceleration, yaw, yaw_rate)
         * @param state Estimated 12D state vector [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
         * @param dt Sampling time step in seconds
         * @param return_rotor_forces If true, returns individual motor thrusts [f1, f2, f3, f4];
         *                            if false, returns total thrust and body torques [T, tau_x, tau_y, tau_z].
         */
        Eigen::Vector4d update(const TrajectoryPoint &target,
                               const Eigen::Matrix<double, 12, 1> &state,
                               double dt,
                               bool return_rotor_forces = false) {
            if (dt <= 0.0) return Eigen::Vector4d::Zero();

            // -------------------------------------------------------------------------
            // 1. Extract Estimated States
            // -------------------------------------------------------------------------
            const Eigen::Vector3d pos_est = state.segment<3>(0);
            const Eigen::Vector3d vel_est = state.segment<3>(3);
            const Eigen::Vector3d att_est = state.segment<3>(6); // [roll, pitch, yaw]
            const Eigen::Vector3d rate_est = state.segment<3>(9); // [p, q, r]

            // -------------------------------------------------------------------------
            // 2. Outer Loop: Position & Velocity -> Desired Accelerations
            // -------------------------------------------------------------------------
            const Eigen::Vector3d pos_err = target.position - pos_est;
            const Eigen::Vector3d vel_err = target.velocity - vel_est;

            // Position integral with anti-windup clamping
            pos_integral_ += pos_err * dt;
            pos_integral_ = pos_integral_.cwiseMin(Eigen::Vector3d(2.0, 2.0, 2.0))
                    .cwiseMax(Eigen::Vector3d(-2.0, -2.0, -2.0));

            // Command acceleration = Feedforward Accel + PID feedback
            const Eigen::Vector3d a_cmd = target.acceleration
                                          + gains_.Kp_pos.cwiseProduct(pos_err)
                                          + gains_.Kd_pos.cwiseProduct(vel_err)
                                          + gains_.Ki_pos.cwiseProduct(pos_integral_);

            // -------------------------------------------------------------------------
            // Virtual Force & Total Thrust Computation
            // -------------------------------------------------------------------------
            // Virtual total force vector required in world frame
            const Eigen::Vector3d F_des = params_.mass * (a_cmd + Eigen::Vector3d(0.0, 0.0, params_.gravity));

            // Total thrust required along body Z axis
            const double T_total = F_des.norm();

            // -------------------------------------------------------------------------
            // Desired Euler Angle Extraction (Differential Flatness / Tilt Mapping)
            // -------------------------------------------------------------------------
            const double psi_d = target.yaw;
            const double sin_psi = std::sin(psi_d);
            const double cos_psi = std::cos(psi_d);

            // Desired Roll (phi_d) and Pitch (theta_d) derived from virtual accelerations
            const double ax = a_cmd.x();
            const double ay = a_cmd.y();
            const double g = params_.gravity;

            const double phi_d = std::clamp((ax * sin_psi - ay * cos_psi) / g, -M_PI / 4.0, M_PI / 4.0);
            const double theta_d = std::clamp((ax * cos_psi + ay * sin_psi) / g, -M_PI / 4.0, M_PI / 4.0);

            const Eigen::Vector3d att_des(phi_d, theta_d, psi_d);

            // -------------------------------------------------------------------------
            // Inner Loop 1: Attitude Error -> Desired Angular Rates
            // -------------------------------------------------------------------------
            Eigen::Vector3d att_err = att_des - att_est;
            att_err.z() = wrapAngle(att_err.z()); // Normalize yaw error to [-pi, pi]

            const Eigen::Vector3d rate_des = gains_.Kp_att.cwiseProduct(att_err)
                                             + Eigen::Vector3d(0.0, 0.0, target.yaw_rate);

            // -------------------------------------------------------------------------
            // Inner Loop 2: Rate Error -> Desired Body Torques
            // -------------------------------------------------------------------------
            const Eigen::Vector3d rate_err = rate_des - rate_est;

            rate_integral_ += rate_err * dt;
            rate_integral_ = rate_integral_.cwiseMin(Eigen::Vector3d(1.0, 1.0, 1.0))
                    .cwiseMax(Eigen::Vector3d(-1.0, -1.0, -1.0));

            const Eigen::Vector3d rate_derivative = (rate_err - prev_rate_error_) / dt;
            prev_rate_error_ = rate_err;

            // Angular accelerations requested
            const Eigen::Vector3d alpha_cmd = gains_.Kp_rate.cwiseProduct(rate_err)
                                              + gains_.Kd_rate.cwiseProduct(rate_derivative)
                                              + gains_.Ki_rate.cwiseProduct(rate_integral_);

            // Body torques tau = I * alpha
            const Eigen::Vector3d tau = params_.inertia.cwiseProduct(alpha_cmd);

            // -------------------------------------------------------------------------
            // Output Wrench or Control Allocation
            // -------------------------------------------------------------------------
            Eigen::Vector4d wrench(T_total, tau.x(), tau.y(), tau.z());

            if (!return_rotor_forces) {
                return wrench;
            }

            return allocateMotorForces(wrench);
        }

    private:
        static double wrapAngle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        [[nodiscard]] Eigen::Vector4d allocateMotorForces(const Eigen::Vector4d &wrench) const {
            const double T = wrench[0];
            const double tau_x = wrench[1];
            const double tau_y = wrench[2];
            const double tau_z = wrench[3];

            const double d = params_.arm_length / std::numbers::sqrt2; // X-configuration offset
            const double km = params_.km;

            // Allocation matrix mapping [f1, f2, f3, f4]^T -> [T, tau_x, tau_y, tau_z]^T
            Eigen::Matrix4d A;
            A << 1.0, 1.0, 1.0, 1.0,
                    -d, -d, d, d,
                    -d, d, d, -d,
                    km, -km, km, -km;

            Eigen::Vector4d forces = A.colPivHouseholderQr().solve(wrench);
            return forces.cwiseMax(0.0); // Non-negative thrust limit
        }

        CascadedGains gains_;
        QuadrotorParams params_;

        Eigen::Vector3d pos_integral_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d rate_integral_{Eigen::Vector3d::Zero()};
        Eigen::Vector3d prev_rate_error_{Eigen::Vector3d::Zero()};
    };
}
#endif //DRONE_OBSERVER_CASCADED_CONTROLLER_HPP
