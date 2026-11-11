//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_CERES_AUTO_DIFF_HPP
#define DRONE_OBSERVER_CERES_AUTO_DIFF_HPP

#include <Eigen/Dense>
#include <ceres/jet.h>
#include "../dynamics/dynamics.hpp"

namespace drone_observer::estimation {
    // C++23 Value-semantic return types for zero-overhead structured bindings
    struct PredictionResult {
        Eigen::Matrix<double, 13, 1> x_next;
        Eigen::Matrix<double, 13, 13> F_x;
    };

    struct MeasurementResult {
        Eigen::Matrix<double, 3, 1> z_pred;
        Eigen::Matrix<double, 3, 13> H;
    };

    class CeresAutoDiff {
    public:
        // -----------------------------------------------------------------------------
        // Templated RK4 Integrator using ceres::Jet or double
        // -----------------------------------------------------------------------------
        template<typename T>
        static void RK4Step(const dynamics::Drone6DegFreedom &dynamics,
                            const T x[13],
                            const T u[4],
                            double dt,
                            T x_next[13]) {
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

        // -----------------------------------------------------------------------------
        // Auto-Diff State Prediction Jacobian (F_x)
        // -----------------------------------------------------------------------------
        static auto predict(const dynamics::Drone6DegFreedom &dynamics,
                            const Eigen::Matrix<double, 13, 1> &x_nominal,
                            const Eigen::Matrix<double, 4, 1> &u_control,
                            double dt) -> PredictionResult {
            using JetT = ceres::Jet<double, 13>; // 13 independent state variables

            JetT x_jets[13];
            JetT u_jets[4];
            JetT x_next_jets[13];

            // Seed state Jets: value = nominal state, derivative identity = 1.0 at index i
            for (int i = 0; i < 13; ++i) {
                x_jets[i].a = x_nominal[i];
                x_jets[i].v.setZero();
                x_jets[i].v[i] = 1.0;
            }

            for (int i = 0; i < 4; ++i) {
                u_jets[i].a = u_control[i];
                u_jets[i].v.setZero();
            }

            // Forward propagation through RK4 using dual numbers
            RK4Step<JetT>(dynamics, x_jets, u_jets, dt, x_next_jets);

            // Populate C++23 return struct
            PredictionResult result;
            for (int i = 0; i < 13; ++i) {
                result.x_next[i] = x_next_jets[i].a;
                result.F_x.row(i) = x_next_jets[i].v;
            }

            return result;
        }

        // -----------------------------------------------------------------------------
        // Auto-Diff Measurement Jacobian (H) - e.g. 3D Position GPS Update
        // -----------------------------------------------------------------------------
        static auto observePosition(const Eigen::Matrix<double, 13, 1> &x_nominal) -> MeasurementResult {
            using JetT = ceres::Jet<double, 13>;

            JetT x_jets[13];
            for (int i = 0; i < 13; ++i) {
                x_jets[i].a = x_nominal[i];
                x_jets[i].v.setZero();
                x_jets[i].v[i] = 1.0;
            }

            // Observation function h(x) = [px, py, pz]
            JetT z_jets[3] = {x_jets[0], x_jets[1], x_jets[2]};

            MeasurementResult result;
            for (int i = 0; i < 3; ++i) {
                result.z_pred[i] = z_jets[i].a;
                result.H.row(i) = z_jets[i].v;
            }

            return result;
        }
    };
    // namespace example_usage {
    //     void EKFObserver::predict(const Eigen::Matrix<double, 4, 1>& u_control, double dt) {
    //         // Clean C++23 structured binding
    //         auto [x_next, F_x] = CeresAutoDiff::predict(dynamics_, state_, u_control, dt);
    //
    //         // Update internal EKF state & covariance
    //         state_ = x_next;
    //         P_ = F_x * P_ * F_x.transpose() + Q_;
    //     }
    //
    //     void EKFObserver::updatePositionGPS(const Eigen::Vector3d& z_measured) {
    //         // Clean C++23 measurement Jacobians
    //         auto [z_pred, H] = CeresAutoDiff::observePosition(state_);
    //
    //         Eigen::Matrix3d S = H * P_ * H.transpose() + R_gps_;
    //         Eigen::Matrix<double, 13, 3> K = P_ * H.transpose() * S.inverse();
    //
    //         state_ = state_ + K * (z_measured - z_pred);
    //         P_ = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P_;
    //     }
    // }
}

#endif //DRONE_OBSERVER_CERES_AUTO_DIFF_HPP
