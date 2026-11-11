//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_SENSOR_HPP
#define DRONE_OBSERVER_SENSOR_HPP

#include <Eigen/Dense>
#include <random>
#include <memory>

namespace drone_observer::sensor{
    /**
     * Abstract class for the sensors
     */
    class ISensor {
    public:
        virtual ~ISensor() = default;

        // Sample a noisy measurement given the true drone state (x = 13D state vector)
        virtual Eigen::VectorXd sample(const Eigen::Matrix<double, 13, 1>& x_true) = 0;
    };

    /**
     * Gps sensor class that adds some gaussian random noise into out system.
     */
    class GPSSensor : public ISensor {
    private:
        double std_dev_;
        std::default_random_engine generator_;
        std::normal_distribution<double> noise_dist_;

    public:
        explicit GPSSensor(double std_dev = 0.05, unsigned int seed = 42)
            : std_dev_(std_dev), generator_(seed), noise_dist_(0.0, std_dev) {}

        Eigen::VectorXd sample(const Eigen::Matrix<double, 13, 1>& x_true) override {
            Eigen::Vector3d z;
            z[0] = x_true[0] + noise_dist_(generator_); // px + noise
            z[1] = x_true[1] + noise_dist_(generator_); // py + noise
            z[2] = x_true[2] + noise_dist_(generator_); // pz + noise
            return z;
        }

        [[nodiscard]] double stdDev() const { return std_dev_; }
        [[nodiscard]] Eigen::Matrix3d covariance() const {
            return Eigen::Matrix3d::Identity() * (std_dev_ * std_dev_);
        }
    };


    class UWBRangeSensor : public ISensor {
    private:
        Eigen::Vector3d anchor_pos_;
        std::default_random_engine generator_;
        std::normal_distribution<double> noise_dist_;

    public:
        explicit UWBRangeSensor(const Eigen::Vector3d& anchor = Eigen::Vector3d::Zero(),
                               double std_dev = 0.02,
                               unsigned int seed = 42)
            : std_dev_(std_dev), anchor_pos_(anchor), generator_(seed), noise_dist_(0.0, std_dev) {}

        Eigen::VectorXd sample(const Eigen::Matrix<double, 13, 1>& x_true) override {
            Eigen::Vector3d pos = x_true.segment<3>(0);
            double true_range = (pos - anchor_pos_).norm();

            Eigen::Vector<double, 1> z;
            z[0] = true_range + noise_dist_(generator_);
            return z;
        }
    };

}

#endif //DRONE_OBSERVER_SENSOR_HPP
