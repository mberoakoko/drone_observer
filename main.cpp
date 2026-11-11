#include <iostream>
#include <iomanip>
#include <vector>
#include <random>

#include "drone_observer/dynamics/dynamics.hpp"
#include "drone_observer/control/simple_pid.hpp"
#include "drone_observer/estimation/ceres_auto_diff.hpp"
#include "visualization/visualizer.hpp"

using namespace drone_observer;

int main() {
    std::cout << "==========================================================\n";
    std::cout << "   DRONE OBSERVER: 3-Phase Altitude EKF Tracking Simulation\n";
    std::cout << "==========================================================\n\n";

    // --- 1. Simulation Parameters ---
    const double dt = 0.01;            // 100 Hz simulation loop
    const double total_time = 30.0;     // 9 seconds total mission duration
    const int num_steps = static_cast<int>(total_time / dt);

    // --- 2. Initialize Ground-Truth Drone Dynamics & Controller ---
    dynamics::Drone6DegFreedom dynamics;
    controller::AltitudePID controller(12.0, 4.0, 0.5, dynamics.mass);

    // Ground truth state x = [pos(3), vel(3), q(4), omega(3)]
    Eigen::Matrix<double, 13, 1> x_true = Eigen::Matrix<double, 13, 1>::Zero();
    x_true[0] = 0.0; x_true[1] = 0.0; x_true[2] = 1.0; // Start at Z = 1.0 m
    x_true[6] = 1.0;                                  // Unit quaternion (w=1, x=0, y=0, z=0)

    // --- 3. Initialize EKF State & Covariances ---
    Eigen::Matrix<double, 13, 1> x_hat = x_true;      // Filter estimate initialized at ground truth
    Eigen::Matrix<double, 13, 13> P = Eigen::Matrix<double, 13, 13>::Identity() * 0.01;

    // Process Noise Covariance (Q)
    Eigen::Matrix<double, 13, 13> Q = Eigen::Matrix<double, 13, 13>::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4; // Pos uncertainty
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3; // Vel uncertainty

    // Measurement Noise Covariance (R) for GPS (3D Position)
    const double gps_noise_std = 0.05; // 5 cm std deviation
    Eigen::Matrix3d R_gps = Eigen::Matrix3d::Identity() * (gps_noise_std * gps_noise_std);

    // Gaussian Noise Generator
    std::default_random_engine generator;
    std::normal_distribution<double> noise_dist(0.0, gps_noise_std);

    DroneVisualizer vis;
    vis.init(1024, 768);

    // Header formatting
    std::cout << std::setw(8) << "Time (s)"
              << std::setw(12) << "Phase"
              << std::setw(14) << "Target Z (m)"
              << std::setw(14) << "True Z (m)"
              << std::setw(14) << "EKF Z (m)"
              << std::setw(14) << "Z Error (m)" << "\n";
    std::cout << std::string(76, '-') << "\n";

    // --- 4. Main Simulation Loop ---
    for (int step = 0; step < num_steps; ++step) {
        double t = step * dt;

        // --- Step 4A: Mission Target Trajectory Logic ---
        double z_target = 1.0;
        std::string phase_name = "Hover";

        if (t >= 3.0 && t < 6.0) {
            z_target = 3.0; // Ascend target
            phase_name = "Ascend";
        } else if (t >= 6.0) {
            z_target = 1.0; // Descend target
            phase_name = "Descend";
        }

        // --- Step 4B: Compute Control Input u ---
        // Inputs: target Z, current estimated Z, estimated Vz
        Eigen::Matrix<double, 4, 1> u = controller.update(z_target, x_hat[2], x_hat[5], dt);

        // --- Step 4C: Step Forward True Physics (RK4) ---
        Eigen::Matrix<double, 13, 1> x_next_true;

        // x_true.data() and u.data() pass the raw double* pointers directly
        estimation::CeresAutoDiff::RK4Step<double>(dynamics,
                                                   x_true.data(),
                                                   u.data(),
                                                   dt,
                                                   x_next_true.data());

        x_true = x_next_true;

        // Normalize true quaternion to prevent numerical drift
        Eigen::Map<Eigen::Quaterniond> q_true(&x_true[6]);
        q_true.normalize();

        // --- Step 4D: EKF Prediction Step via Ceres Auto-Diff ---
        auto [x_pred, F_x] = estimation::CeresAutoDiff::predict(dynamics, x_hat, u, dt);
        x_hat = x_pred;
        P = F_x * P * F_x.transpose() + Q;

        // Normalize estimated quaternion
        Eigen::Map<Eigen::Quaterniond> q_hat(&x_hat[6]);
        q_hat.normalize();

        // --- Step 4E: EKF Measurement Update Step (GPS @ 100Hz) ---
        // Generate noisy GPS position measurement
        Eigen::Vector3d z_measured;
        z_measured[0] = x_true[0] + noise_dist(generator);
        z_measured[1] = x_true[1] + noise_dist(generator);
        z_measured[2] = x_true[2] + noise_dist(generator);

        // Ceres Auto-Diff observation Jacobian H
        auto [z_pred, H] = estimation::CeresAutoDiff::observePosition(x_hat);

        // Kalman Gain calculation
        Eigen::Matrix3d S = H * P * H.transpose() + R_gps;
        Eigen::Matrix<double, 13, 3> K = P * H.transpose() * S.inverse();

        // Update state estimate and error covariance
        x_hat = x_hat + K * (z_measured - z_pred);
        P = (Eigen::Matrix<double, 13, 13>::Identity() - K * H) * P;

        // --- Step 4F: Print Status Every 0.5s ---
        if (step % 50 == 0) {
            double error_z = std::abs(x_true[2] - x_hat[2]);
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(8) << t
                      << std::setw(12) << phase_name
                      << std::setw(14) << z_target
                      << std::setw(14) << x_true[2]
                      << std::setw(14) << x_hat[2]
                      << std::setw(14) << error_z << "\n";
        }

        // --- Step 4F: Update 3D Pangolin Renderer ---
        Eigen::Vector3d pos_true = x_true.segment<3>(0);
        Eigen::Vector3d pos_ekf = x_hat.segment<3>(0);
        Eigen::Vector3d target_pt(0.0, 0.0, z_target);

        vis.update(pos_true, q_true, pos_ekf, target_pt);

        // Sleep briefly to simulate real-time visualization pacing (100 Hz = 10ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << std::string(76, '-') << "\n";
    std::cout << "Mission simulation complete! EKF successfully tracked elevation transitions.\n";
    return 0;
}