//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_EKF_OBSERVER_HPP
#define DRONE_OBSERVER_EKF_OBSERVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "drone_observer/dynamics/dynamics.hpp"
#include "drone_observer/estimation/ceres_auto_diff.hpp"

namespace drone_observer::estimation {
    class EKFObserver {
        dynamics::Drone6DegFreedom dynamics_;
        Eigen::Matrix<double, 13, 1> x_;
        Eigen::Matrix<double, 13, 13> P_;
        Eigen::Matrix<double, 13, 13> Q_;

    public:
        EKFObserver(const dynamics::Drone6DegFreedom &dyn, const Eigen::Matrix<double, 13, 1> &x_init)
            : dynamics_(dyn), x_(x_init) {
            P_ = Eigen::Matrix<double, 13, 13>::Identity() * 0.01;
            Q_ = Eigen::Matrix<double, 13, 13>::Zero();
            Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4;
            Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
        }

        void predict(const Eigen::Matrix<double, 4, 1> &u, double dt) {
            auto [x_pred, F_x] = CeresAutoDiff::predict(dynamics_, x_, u, dt);
            x_ = x_pred;
            P_ = F_x * P_ * F_x.transpose() + Q_;
            Eigen::Map<Eigen::Quaterniond>(&x_[6]).normalize();
        }

        void updatePositionGPS(const Eigen::Vector3d &z_measured, const Eigen::Matrix3d &R_gps) {
            auto [z_pred, H] = CeresAutoDiff::observePosition(x_);
            Eigen::Matrix3d S = H * P_ * H.transpose() + R_gps;
            Eigen::Matrix<double, 13, 3> K = P_ * H.transpose() * S.inverse();

            x_ += K * (z_measured - z_pred);
            P_ = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P_;
            Eigen::Map<Eigen::Quaterniond>(&x_[6]).normalize();
        }

        [[nodiscard]] const Eigen::Matrix<double, 13, 1> &state() const { return x_; }
        [[nodiscard]] const Eigen::Matrix<double, 13, 13> &covariance() const { return P_; }
    };
}

#endif //DRONE_OBSERVER_EKF_OBSERVER_HPP
