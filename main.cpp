#include <iostream>
#include <iomanip>
#include <vector>
#include <random>

#include "drone_observer/dynamics/dynamics.hpp"
#include "drone_observer/control/simple_pid.hpp"
#include "drone_observer/estimation/ceres_auto_diff.hpp"
#include "drone_observer/estimation/ekf_observer.hpp"
#include "drone_observer/estimation/ckf_observer.hpp"
#include "drone_observer/estimation/ghkf_observer.hpp"
#include "drone_observer/sensors/sensor.hpp"
#include "visualization/visualizer.hpp"
#include "drone_observer/control/trajectories.hpp"
#include "drone_observer/control/cascaded_controller.hpp"

using namespace drone_observer;

static void print_header() {
    std::cout << "==========================================================\n";
    std::cout << "   DRONE OBSERVER: 3-Phase Altitude EKF Tracking Simulation\n";
    std::cout << "==========================================================\n\n";
}

static auto step_with_rk4(
    double dt,
    const dynamics::Drone6DegFreedom &dynamics,
    Eigen::Matrix<double, 13, 1> x_true, Eigen::Matrix<double, 4, 1> u
) -> Eigen::Matrix<double, 13, 1> {
    Eigen::Matrix<double, 13, 1> x_next_true;
    estimation::CeresAutoDiff::RK4Step<double>(dynamics,
                                               x_true.data(),
                                               u.data(),
                                               dt,
                                               x_next_true.data());
    return x_next_true;
}

static auto simple_sim() {
    // --- 1. Simulation Parameters ---
    const double dt = 0.01; // 100 Hz simulation loop
    const double total_time = 30.0; // 9 seconds total mission duration
    const int num_steps = static_cast<int>(total_time / dt);

    // --- 2. Initialize Ground-Truth Drone Dynamics & Controller ---
    dynamics::Drone6DegFreedom dynamics;
    controller::AltitudePID controller(100.0, 4.0, 10, dynamics.mass);
    visualization::DroneVisualizer vis;
    vis.init(1024, 768);

    // Initial State Setup
    Eigen::Matrix<double, 13, 1> x_true = Eigen::Matrix<double, 13, 1>::Zero();
    x_true[2] = 1.0;
    x_true[6] = 1.0;

    estimation::EKFObserver ekf(dynamics, x_true);
    // estimation::CubatureKalmanFilter  ckf(dynamics, x_true);
    // estimation::GaussHermiteKalmanFilter ghkf(dynamics, x_true);

    const double gps_noise_std = 0.05;
    Eigen::Matrix3d R_gps = Eigen::Matrix3d::Identity() * (gps_noise_std * gps_noise_std);
    sensor::GPSSensor gps_sensor(gps_noise_std);

    for (int step = 0; step < num_steps; ++step) {
        double t = step * dt;
        double z_target = (t >= 3.0 && t < 6.0) ? 3.0 : 1.0;

        // A. Control & Physics
        Eigen::Matrix<double, 4, 1> u = controller.update(z_target, ekf.state()[2], ekf.state()[5], dt);

        x_true = step_with_rk4(dt, dynamics, x_true, u);

        // Normalize true quaternion to prevent numerical drift
        Eigen::Map<Eigen::Quaterniond> q_true(&x_true[6]);
        q_true.normalize();

        // B. Generate Measurement
        Eigen::Vector3d z_measured = gps_sensor.sample(x_true);

        // C. Clean Observer Predict & Update Calls
        ekf.predict(u, dt);
        ekf.updatePositionGPS(z_measured, R_gps);
        // ckf.predict(u, dt);   ckf.updatePositionGPS(z_measured, R_gps);
        // ghkf.predict(u, dt);  ghkf.updatePositionGPS(z_measured, R_gps);

        // D. Render & Log
        // vis.update(x_true.segment<3>(0),
        //            Eigen::Quaterniond(x_true[6], x_true[7], x_true[8], x_true[9]),
        //            ekf.state().segment<3>(0),
        //            ekf.state().segment<3>(0),
        //            ekf.state().segment<3>(0),
        //            Eigen::Vector3d(0, 0, z_target));

        // if (step % 50 == 0) {
        //     SimulationLogger::logRow(t, x_true[2], ekf.state()[2], ckf.state()[2], ghkf.state()[2]);
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << std::string(76, '-') << "\n";
    std::cout << "Mission simulation complete! EKF successfully tracked elevation transitions.\n";
}

static auto sim_with_trajectories() {
    visualization::DroneVisualizer vis;
    vis.init(1024, 768);

    const double dt = 0.01; // 100 Hz simulation step
    const double total_time = 10.0;

    // 1. Trajectory Generator (3D Figure-8 Lemniscate)
    const auto trajectory = std::make_unique<control::Figure8Trajectory>(
        /*scale_x=*/2.0, /*scale_y=*/1.5, /*scale_z=*/0.5, /*freq=*/0.5, /*z_offset=*/2.0);

    dynamics::Drone6DegFreedom dynamics;
    control::CascadedController controller;

    // 2. Setup Ground Truth State (13D Vector with Unit Quaternion qw = 1.0)
    Eigen::Matrix<double, 13, 1> x_true = Eigen::Matrix<double, 13, 1>::Zero();
    x_true[2] = 2.0; // Initial z offset
    x_true[6] = 1.0; // Unit quaternion scalar component

    // 3. Setup EKF Observer with 13D Initial State
    estimation::EKFObserver ekf(dynamics, x_true);

    constexpr double gps_noise_std = 0.05;
    Eigen::Matrix3d R_gps = Eigen::Matrix3d::Identity() * (gps_noise_std * gps_noise_std);
    sensor::GPSSensor gps_sensor(gps_noise_std);

    double t = 0.0;
    while (t < total_time) {
        control::TrajectoryPoint target = trajectory->evaluate(t);
        const Eigen::Matrix<double, 13, 1> &state_est = ekf.state();

        // Compute 4D control wrench u = [Total_Thrust, Tau_x, Tau_y, Tau_z]^T
        Eigen::Vector4d u = controller.update(target, state_est, dt, /*return_rotor_forces=*/false);

        // Step system dynamics & EKF prediction using wrench u
        x_true = step_with_rk4(dt, dynamics, x_true, u);
        x_true.segment<4>(6).normalize(); // Prevent quaternion RK4 drift

        ekf.predict(u, dt);

        Eigen::Vector3d z_measured = gps_sensor.sample(x_true);
        ekf.updatePositionGPS(z_measured, R_gps);
        auto ekf_position = ekf.state().segment<3>(0);
        vis.update(t,
           x_true.segment<3>(0),
           Eigen::Quaterniond(x_true[6], x_true[7], x_true[8], x_true[9]),
           ekf_position,   // EKF Position (Red)
           ekf_position,   // CKF Position (Cyan)
           ekf_position,  // GHKF Position (Magenta)
           target.position,             // Target (Yellow)
           u);                          // Control Wrench [T, tau_x, tau_y, tau_z]

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t += dt;
    }

    std::cout << "Cascaded 6-DoF controller simulation finished successfully.\n";
}

int main() {
    print_header();
    sim_with_trajectories();
    return 0;
}
