//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_CKF_OBSERVER_HPP
#define DRONE_OBSERVER_CKF_OBSERVER_HPP


#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <cmath>
#include "drone_observer/dynamics/dynamics.hpp"
#include "drone_observer/estimation/ceres_auto_diff.hpp"

namespace drone_observer::estimation {
    class CubatureKalmanFilter {
        static constexpr int N = 13; // State dimension
        static constexpr int NUM_PTS = 2 * N; // 2n = 26 cubature points

        dynamics::Drone6DegFreedom dynamics_;
        double weight_ = 1.0 / static_cast<double>(NUM_PTS); // Equal weighting w_i = 1/(2n)

        Eigen::Matrix<double, N, NUM_PTS> xi_; // Unit cubature points matrix (N x 2N)

    public:
        explicit CubatureKalmanFilter(dynamics::Drone6DegFreedom dyn = dynamics::Drone6DegFreedom())
            : dynamics_(std::move(dyn)) {
            // Pre-compute 2n spherical-radial cubature points: xi_i = +/- sqrt(n) * e_i
            xi_.setZero();
            const double factor = std::sqrt(static_cast<double>(N));

            for (int i = 0; i < N; ++i) {
                xi_(i, i) = factor; // +sqrt(n) * e_i
                xi_(i, i + N) = -factor; // -sqrt(n) * e_i
            }
        }

        // -------------------------------------------------------------------------
        // Generate 2N Cubature Points: X_i = m + L * xi_i
        // -------------------------------------------------------------------------
        void generateCubaturePoints(const Eigen::Matrix<double, N, 1> &m,
                                    const Eigen::Matrix<double, N, N> &P,
                                    Eigen::Matrix<double, N, NUM_PTS> &X) const {
            // Lower triangular matrix L from Cholesky decomposition: P = L * L^T
            const Eigen::LLT<Eigen::Matrix<double, N, N> > llt(P);
            Eigen::Matrix<double, N, N> L = llt.matrixL();

            for (int i = 0; i < NUM_PTS; ++i) {
                X.col(i) = m + L * xi_.col(i);
            }
        }


        /**
         * Reconstruct Predicted Mean: m_pred = sum(w_i * X_star_i)
         * @param X_star propagated_state
         * @return predicted_mean
         */
        [[nodiscard]]
        Eigen::Matrix<double, N, 1> reconstruct_predicted_mean(Eigen::Matrix<double, N, NUM_PTS> X_star) const {
            Eigen::Matrix<double, N, 1> m_pred = Eigen::Matrix<double, N, 1>::Zero();
            for (int i = 0; i < NUM_PTS; ++i) {
                m_pred += weight_ * X_star.col(i);
            }
            return m_pred;
        }

        Eigen::Matrix<double, N, N> reconstruct_predicted_covariance(const Eigen::Matrix<double, N, N> &Q,
                                                                     Eigen::Matrix<double, N, NUM_PTS> X_star,
                                                                     Eigen::Matrix<double, N, 1> m_pred) const {
            Eigen::Matrix<double, N, N> P_pred = Q;
            for (int i = 0; i < NUM_PTS; ++i) {
                Eigen::Matrix<double, N, 1> diff = X_star.col(i) - m_pred;
                P_pred += weight_ * (diff * diff.transpose());
            }
            return P_pred;
        }

        Eigen::Matrix<double, N, NUM_PTS> propagate_cubature_point(const Eigen::Matrix<double, 4, 1> &u, double dt,
                                                                   Eigen::Matrix<double, N, NUM_PTS> X) const {
            Eigen::Matrix<double, N, NUM_PTS> X_star;

            // 1. Propagate each cubature point through non-linear dynamic RK4 model
            for (int i = 0; i < NUM_PTS; ++i) {
                Eigen::Matrix<double, N, 1> x_next;
                CeresAutoDiff::RK4Step<double>(dynamics_, X.col(i).data(), u.data(), dt, x_next.data());
                X_star.col(i) = x_next;
            }
            return X_star;
        }

        // -------------------------------------------------------------------------
        // CKF Time Prediction Step
        // -------------------------------------------------------------------------
        void predict(Eigen::Matrix<double, N, 1> &m,
                     Eigen::Matrix<double, N, N> &P,
                     const Eigen::Matrix<double, 4, 1> &u,
                     const Eigen::Matrix<double, N, N> &Q,
                     double dt) const {
            Eigen::Matrix<double, N, NUM_PTS> X;
            generateCubaturePoints(m, P, X);

            Eigen::Matrix<double, N, NUM_PTS> X_star = propagate_cubature_point(u, dt, X);
            const Eigen::Matrix<double, N, 1> m_pred = reconstruct_predicted_mean(X_star);

            // 3. Reconstruct Predicted Covariance: P_pred = sum(w_i * (X_star_i - m_pred)(...)^T) + Q
            const Eigen::Matrix<double, N, N> P_pred = reconstruct_predicted_covariance(Q, X_star, m_pred);

            m = m_pred;
            P = P_pred;
        }

        // -------------------------------------------------------------------------
        // CKF Measurement Update Step (e.g. 3D GPS Position)
        // -------------------------------------------------------------------------
        void updatePositionGPS(Eigen::Matrix<double, N, 1> &m,
                               Eigen::Matrix<double, N, N> &P,
                               const Eigen::Vector3d &z_measured,
                               const Eigen::Matrix3d &R_gps) const {
            Eigen::Matrix<double, N, NUM_PTS> X;
            generateCubaturePoints(m, P, X);

            // Transform cubature points through observation function h(x) = pos(3)
            Eigen::Matrix<double, 3, NUM_PTS> Z_pts;
            for (int i = 0; i < NUM_PTS; ++i) {
                Z_pts.col(i) = X.block<3, 1>(0, i);
            }

            // Predicted measurement mean
            Eigen::Vector3d z_pred = Eigen::Vector3d::Zero();
            for (int i = 0; i < NUM_PTS; ++i) {
                z_pred += weight_ * Z_pts.col(i);
            }

            // Innovation covariance S and Cross-covariance P_xz
            Eigen::Matrix3d S = R_gps;
            Eigen::Matrix<double, N, 3> P_xz = Eigen::Matrix<double, N, 3>::Zero();

            for (int i = 0; i < NUM_PTS; ++i) {
                Eigen::Vector3d z_diff = Z_pts.col(i) - z_pred;
                Eigen::Matrix<double, N, 1> x_diff = X.col(i) - m;

                S += weight_ * (z_diff * z_diff.transpose());
                P_xz += weight_ * (x_diff * z_diff.transpose());
            }

            // Kalman Gain
            Eigen::Matrix<double, N, 3> K = P_xz * S.inverse();

            // Updated State Mean & Covariance
            m = m + K * (z_measured - z_pred);
            P = P - K * S * K.transpose();
        }
    };
}


#endif //DRONE_OBSERVER_CKF_OBSERVER_HPP
