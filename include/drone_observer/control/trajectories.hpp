//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_TRAJECTORIES_HPP
#define DRONE_OBSERVER_TRAJECTORIES_HPP

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

namespace drone_observer::control {
    struct TrajectoryPoint {
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
        double yaw{0.0};
        double yaw_rate{0.0};
    };

    class ITrajectory {
    public:
        virtual ~ITrajectory() = default;

        [[nodiscard]] virtual TrajectoryPoint evaluate(double t) const = 0;
    };

    /**
     * 3D Figure-8 / Lemniscate Trajectory
     */
    class Figure8Trajectory final : public ITrajectory {
    public:
        Figure8Trajectory(double scale_x, double scale_y, double scale_z, double freq, double z_offset = 2.0)
            : ax_(scale_x), ay_(scale_y), az_(scale_z), w_(freq), z0_(z_offset) {
        }

        [[nodiscard]] TrajectoryPoint evaluate(const double t) const override {
            TrajectoryPoint pt;
            const double wt = w_ * t;
            const double sin_wt = std::sin(wt);
            const double cos_wt = std::cos(wt);
            const double sin_2wt = std::sin(2.0 * wt);
            const double cos_2wt = std::cos(2.0 * wt);

            // Position
            pt.position << ax_ * sin_wt,
                    ay_ * sin_wt * cos_wt,
                    z0_ + az_ * sin_2wt;

            // Velocity (1st Derivative)
            pt.velocity << ax_ * w_ * cos_wt,
                    ay_ * w_ * cos_2wt,
                    2.0 * az_ * w_ * cos_2wt;

            // Acceleration (2nd Derivative)
            pt.acceleration << -ax_ * w_ * w_ * sin_wt,
                    -2.0 * ay_ * w_ * w_ * sin_2wt,
                    -4.0 * az_ * w_ * w_ * sin_2wt;

            // Yaw aligned with tangent velocity vector in XY plane
            pt.yaw = std::atan2(pt.velocity.y(), pt.velocity.x());
            return pt;
        }

    private:
        double ax_, ay_, az_, w_, z0_;
    };

    /**
     * Conical Helix Trajectory
     */
    class ConicalHelixTrajectory final : public ITrajectory {
    public:
        ConicalHelixTrajectory(double r_start, double r_rate, double z_rate, double freq, double z_offset = 1.0)
            : r0_(r_start), dr_(r_rate), dz_(z_rate), w_(freq), z0_(z_offset) {
        }

        [[nodiscard]] TrajectoryPoint evaluate(double t) const override {
            TrajectoryPoint pt;
            const double r = r0_ + dr_ * t;
            const double wt = w_ * t;
            const double sin_wt = std::sin(wt);
            const double cos_wt = std::cos(wt);

            // Position
            pt.position << r * cos_wt,
                    r * sin_wt,
                    z0_ + dz_ * t;

            // Velocity
            pt.velocity << dr_ * cos_wt - r * w_ * sin_wt,
                    dr_ * sin_wt + r * w_ * cos_wt,
                    dz_;

            // Acceleration
            pt.acceleration << -2.0 * dr_ * w_ * sin_wt - r * w_ * w_ * cos_wt,
                    2.0 * dr_ * w_ * cos_wt - r * w_ * w_ * sin_wt,
                    0.0;

            pt.yaw = std::atan2(pt.velocity.y(), pt.velocity.x());
            return pt;
        }

    private:
        double r0_, dr_, dz_, w_, z0_;
    };

    /**
     * Multi-Sine Chirp Trajectory
     */
    class MultiSineTrajectory final : public ITrajectory {
    public:
        MultiSineTrajectory(Eigen::Vector3d amps, Eigen::Vector3d freqs, double z_offset = 2.0)
            : A_(amps), w_(freqs), z0_(z_offset) {
        }

        [[nodiscard]] TrajectoryPoint evaluate(double t) const override {
            TrajectoryPoint pt;

            pt.position << A_.x() * std::sin(w_.x() * t),
                    A_.y() * std::cos(w_.y() * t),
                    z0_ + A_.z() * std::sin(w_.z() * t);

            pt.velocity << A_.x() * w_.x() * std::cos(w_.x() * t),
                    -A_.y() * w_.y() * std::sin(w_.y() * t),
                    A_.z() * w_.z() * std::cos(w_.z() * t);

            pt.acceleration << -A_.x() * w_.x() * w_.x() * std::sin(w_.x() * t),
                    -A_.y() * w_.y() * w_.y() * std::cos(w_.y() * t),
                    -A_.z() * w_.z() * w_.z() * std::sin(w_.z() * t);

            pt.yaw = std::atan2(pt.velocity.y(), pt.velocity.x());
            return pt;
        }

    private:
        Eigen::Vector3d A_, w_;
        double z0_;
    };
}

#endif //DRONE_OBSERVER_TRAJECTORIES_HPP
