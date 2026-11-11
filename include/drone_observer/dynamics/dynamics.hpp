//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_DYNAMICS_HPP
#define DRONE_OBSERVER_DYNAMICS_HPP

#include <Eigen/Dense>
#include <ceres/jet.h>

namespace dynamics {
    struct Drone6DegFreedom {
    public:
        // Physical Parameters (Default 1.5 kg quadrotor properties)
        double mass = 1.5; // kg
        Eigen::Matrix3d J = (Eigen::Matrix3d() << 0.02, 0.0, 0.0,
                             0.0, 0.02, 0.0,
                             0.0, 0.0, 0.04).finished(); // Inertia matrix (kg*m^2)
        Eigen::Vector3d gravity{0.0, 0.0, -9.81}; // m/s^2
        double drag_coefficient = 0.1; // Linear drag coeff (N*s/m)

        // Default constructor
        Drone6DegFreedom() = default;

        // Custom physical parameters constructor
        Drone6DegFreedom(double m, const Eigen::Matrix3d &inertia, const Eigen::Vector3d &g, double Cd = 0.1)
            : mass(m), J(inertia), gravity(g), drag_coefficient(Cd) {
        }

        /**
         * Core Dynamic Equations: x_dot = f(x, u)
         * Templated for T = double or T = ceres::Jet<double, N>.<br/>
         * x[0..2]  : Position (x, y, z)<br/>
         * x[0..2]  : Position (x, y, z)<br/>
         * x[3..5]  : Velocity (vx, vy, vz)<br/>
         * x[6..9]  : Quaternion orientation (w, x, y, z)<br/>
         * x[10..12]: Angular velocity in body frame (omega_x, omega_y, omega_z)<br/>

         * @tparam T
         * @param x
         * @param u
         * @param x_dot
         */
        template<typename T>
        void operator()(const T x[13], const T u[4], T x_dot[13]) const {
            // --- 1. Unpack State Vector (13 States) ---
            // x[0..2]  : Position (x, y, z)
            // x[3..5]  : Velocity (vx, vy, vz)
            // x[6..9]  : Quaternion orientation (w, x, y, z)
            // x[10..12]: Angular velocity in body frame (omega_x, omega_y, omega_z)

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos(&x[0]);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> vel(&x[3]);

            // Eigen Quaternion constructor takes (w, x, y, z)
            Eigen::Quaternion<T> q(x[6], x[7], x[8], x[9]);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> omega(&x[10]);

            // --- 2. Unpack Control Inputs (4 Controls) ---
            // u[0]   : Total collective thrust along body Z-axis (N)
            // u[1..3]: Body torques (tau_x, tau_y, tau_z) in N*m
            T total_thrust = u[0];
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> torques(&u[1]);

            // --- 3. Position Kinematics: d(pos)/dt = vel ---
            Eigen::Map<Eigen::Matrix<T, 3, 1> > pos_dot(&x_dot[0]);
            pos_dot = vel;

            // --- 4. Translational Dynamics: d(vel)/dt = g + (1/m)*R(q)*Thrust - Drag ---
            Eigen::Map<Eigen::Matrix<T, 3, 1> > vel_dot(&x_dot[3]);
            Eigen::Matrix<T, 3, 1> body_thrust(T(0), T(0), total_thrust);

            // Transform thrust from body frame to world frame via quaternion rotation: R(q) * body_thrust
            Eigen::Matrix<T, 3, 1> world_thrust = q * body_thrust;
            Eigen::Matrix<T, 3, 1> drag_force = T(drag_coefficient) * vel;

            vel_dot = gravity.cast<T>() + (world_thrust - drag_force) / T(mass);

            // --- 5. Quaternion Kinematics: d(q)/dt = 0.5 * q * [0, omega] ---
            Eigen::Quaternion<T> omega_quat(T(0), omega.x(), omega.y(), omega.z());
            Eigen::Quaternion<T> q_dot;
            q_dot.coeffs() = T(0.5) * (q * omega_quat).coeffs();

            x_dot[6] = q_dot.w();
            x_dot[7] = q_dot.x();
            x_dot[8] = q_dot.y();
            x_dot[9] = q_dot.z();

            // --- 6. Rotational Dynamics: d(omega)/dt = J^-1 * (torques - omega x (J * omega)) ---
            Eigen::Map<Eigen::Matrix<T, 3, 1> > omega_dot(&x_dot[10]);

            Eigen::Matrix3d J_inv = J.inverse();
            Eigen::Matrix<T, 3, 1> J_omega = J.cast<T>() * omega;
            Eigen::Matrix<T, 3, 1> gyroscopic_moments = omega.cross(J_omega);

            omega_dot = J_inv.cast<T>() * (torques - gyroscopic_moments);
        }
    };
}
#endif //DRONE_OBSERVER_DYNAMICS_HPP
