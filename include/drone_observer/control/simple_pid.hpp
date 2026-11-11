//
// Created by mbero-akoko on 11/11/26.
//
#ifndef DRONE_OBSERVER_SIMPLE_PID_HPP
#define DRONE_OBSERVER_SIMPLE_PID_HPP

#include <Eigen/Dense>
#include <algorithm>

namespace drone_observer::controller {
    class AltitudePID {
    public:
        double Kp = 12.0;
        double Kd = 4.0;
        double Ki = 0.5;

        double mass = 1.5;
        double gravity = 9.81;

        double integral_error = 0.0;

        AltitudePID() = default;
        AltitudePID(double kp, double kd, double ki, double m = 1.5)
            : Kp(kp), Kd(kd), Ki(ki), mass(m) {}

        // Computes control vector u = [Thrust, 0, 0, 0]
        Eigen::Matrix<double, 4, 1> update(double z_target, double z_curr, double vz_curr, double dt) {
            double error_z = z_target - z_curr;
            integral_error += error_z * dt;

            // Anti-windup clamping
            integral_error = std::clamp(integral_error, -2.0, 2.0);

            const double a_des = Kp * error_z + Kd * (-vz_curr) + Ki * integral_error;
            const double thrust = mass * (gravity + a_des);

            Eigen::Matrix<double, 4, 1> u = Eigen::Matrix<double, 4, 1>::Zero();
            u[0] = std::max(0.0, thrust); // Total thrust along Z-axis

            return u;
        }

        void reset() {
            integral_error = 0.0;
        }
    };
}

#endif //DRONE_OBSERVER_SIMPLE_PID_HPP
