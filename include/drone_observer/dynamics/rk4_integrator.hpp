//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_RK4_INTEGRATOR_HPP
#define DRONE_OBSERVER_RK4_INTEGRATOR_HPP

#include <ceres/jet.h>

#include "../dynamics/dynamics.hpp"


namespace dynamics {
    template<typename T>
    void rk4_step(const T x[13], const T u[4], double dt, T x_next[13]) {
        Drone6DegFreedom dynamics;
        T k1[13], k2[13], k3[13], k4[13];
        T x_temp[13];

        dynamics(x, u, k1);

        for (int i = 0; i < 13; ++i) x_temp[i] = x[i] + T(0.5 * dt) * k1[i];
        dynamics(x_temp, u, k2);

        for (int i = 0; i < 13; ++i) x_temp[i] = x[i] + T(0.5 * dt) * k2[i];
        dynamics(x_temp, u, k3);

        for (int i = 0; i < 13; ++i) x_temp[i] = x[i] + T(dt) * k3[i];
        dynamics(x_temp, u, k4);

        for (int i = 0; i < 13; ++i) {
            x_next[i] = x[i] + T(dt / 6.0) * (k1[i] + T(2.0) * k2[i] + T(2.0) * k3[i] + k4[i]);
        }
    }

    // Compute predicted state x_next AND Jacobian F_x using Ceres Jets
    inline void predict_with_auto_diff(const Eigen::Matrix<double, 13, 1>& x_nominal,
                                       const Eigen::Matrix<double, 4, 1>& u_control,
                                       double dt,
                                       Eigen::Matrix<double, 13, 1>& x_next_out,
                                       Eigen::Matrix<double, 13, 13>& F_x_out)
    {
        using JetT = ceres::Jet<double, 13>; // 13 independent variables

        JetT x_jets[13];
        JetT u_jets[4];
        JetT x_next_jets[13];

        // 1. Initialize Jets: value = nominal state, derivative part = Identity matrix
        for (int i = 0; i < 13; ++i) {
            x_jets[i].a = x_nominal[i]; // Scalar value
            x_jets[i].v.setZero();      // Derivative vector
            x_jets[i].v[i] = 1.0;       // Seed derivative wrt state i
        }

        for (int i = 0; i < 4; ++i) {
            u_jets[i].a = u_control[i];
            u_jets[i].v.setZero();
        }

        // 2. Pass Jets through RK4 integration
        rk4_step<JetT>(x_jets, u_jets, dt, x_next_jets);

        // 3. Extract predicted state & 13x13 Jacobian F_x
        for (int i = 0; i < 13; ++i) {
            x_next_out[i] = x_next_jets[i].a; // Nominal result
            F_x_out.row(i) = x_next_jets[i].v; // Auto-diff row (d x_next[i] / d x)
        }
    }
}
#endif //DRONE_OBSERVER_RK4_INTEGRATOR_HPP
