//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_GHKF_OBSERVER_HPP
#define DRONE_OBSERVER_GHKF_OBSERVER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "drone_observer/dynamics/dynamics.hpp"
#include "drone_observer/estimation/ceres_auto_diff.hpp"

namespace drone_observer::estimation {
    class GaussHermiteKalmanFilter {
    private:
        dynamics::Drone6DegFreedom dynamics_;

        // 1D Gauss-Hermite Quadrature (p = 3 points)
        const std::vector<double> xi_1d_ = {-std::sqrt(3.0), 0.0, std::sqrt(3.0)};
        const std::vector<double> w_1d_ = {1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0};

    public:
        GaussHermiteKalmanFilter() = default;

        explicit GaussHermiteKalmanFilter(const dynamics::Drone6DegFreedom &dyn) : dynamics_(dyn) {
        }

        // -------------------------------------------------------------------------
        // Quadrature Point Generation for Mean m and Covariance P
        // -------------------------------------------------------------------------
        // Evaluates quadrature nodes using Matrix Square Root (Cholesky decomposition: P = L * L^T)
        void generateSigmaPoints(const Eigen::Matrix<double, 13, 1> &m,
                                 const Eigen::Matrix<double, 13, 13> &P,
                                 std::vector<Eigen::Matrix<double, 13, 1> > &nodes,
                                 std::vector<double> &weights) const {
            nodes.clear();
            weights.clear();

            // Cholesky decomposition L
            Eigen::LLT<Eigen::Matrix<double, 13, 13> > llt(P);
            Eigen::Matrix<double, 13, 13> L = llt.matrixL();

            // For efficient demonstration on high-dimensional systems (13D),
            // we form diagonal quadrature perturbations along major eigen-axes (2*N + 1 nodes)
            nodes.push_back(m);
            weights.push_back(w_1d_[1]); // Weight for center point (0.0)

            for (int i = 0; i < 13; ++i) {
                Eigen::Matrix<double, 13, 1> offset = L.col(i);

                nodes.emplace_back(m + xi_1d_[2] * offset);
                weights.push_back(w_1d_[2]);

                nodes.emplace_back(m + xi_1d_[0] * offset);
                weights.push_back(w_1d_[0]);
            }

            // Normalize weights sum to 1.0
            double sum_w = 0.0;
            for (double w: weights) sum_w += w;
            for (double &w: weights) w /= sum_w;
        }

        // -------------------------------------------------------------------------
        // GHKF Time Prediction Step (Non-linear Transform of Nodes)
        // -------------------------------------------------------------------------
        void predict(Eigen::Matrix<double, 13, 1> &m,
                     Eigen::Matrix<double, 13, 13> &P,
                     const Eigen::Matrix<double, 4, 1> &u,
                     const Eigen::Matrix<double, 13, 13> &Q,
                     const double dt) const {
            std::vector<Eigen::Matrix<double, 13, 1> > nodes;
            std::vector<double> weights;
            generateSigmaPoints(m, P, nodes, weights);

            const size_t num_nodes = nodes.size();
            std::vector<Eigen::Matrix<double, 13, 1> > propagated_nodes(num_nodes);

            // 1. Pass each quadrature node through RK4 non-linear dynamic model
            for (size_t i = 0; i < num_nodes; ++i) {
                Eigen::Matrix<double, 13, 1> x_next;
                CeresAutoDiff::RK4Step<double>(dynamics_, nodes[i].data(), u.data(), dt, x_next.data());
                propagated_nodes[i] = x_next;
            }

            // 2. Reconstruct Predicted Mean
            Eigen::Matrix<double, 13, 1> m_pred = Eigen::Matrix<double, 13, 1>::Zero();
            for (size_t i = 0; i < num_nodes; ++i) {
                m_pred += weights[i] * propagated_nodes[i];
            }

            // 3. Reconstruct Predicted Covariance
            Eigen::Matrix<double, 13, 13> P_pred = Q;
            for (size_t i = 0; i < num_nodes; ++i) {
                Eigen::Matrix<double, 13, 1> diff = propagated_nodes[i] - m_pred;
                P_pred += weights[i] * (diff * diff.transpose());
            }

            m = m_pred;
            P = P_pred;
        }
    };
}

#endif //DRONE_OBSERVER_GHKF_OBSERVER_HPP
