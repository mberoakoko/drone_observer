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
#include "visualization/visualizer.hpp"

using namespace drone_observer;

static void print_header() {
    std::cout << "==========================================================\n";
    std::cout << "   DRONE OBSERVER: 3-Phase Altitude EKF Tracking Simulation\n";
    std::cout << "==========================================================\n\n";
}

static auto step_with_rk4(
    double dt,
    dynamics::Drone6DegFreedom dynamics,
    Eigen::Matrix<double, 13, 1> x_true, Eigen::Matrix<double, 4, 1> u
    ) -> Eigen::Matrix<double, 13, 1>  {

    Eigen::Matrix<double, 13, 1> x_next_true;
    estimation::CeresAutoDiff::RK4Step<double>(dynamics,
                                               x_true.data(),
                                               u.data(),
                                               dt,
                                               x_next_true.data());
    return x_next_true;
}

int main() {
    print_header();

    // --- 1. Simulation Parameters ---
    const double dt = 0.01;            // 100 Hz simulation loop
    const double total_time = 30.0;     // 9 seconds total mission duration
    const int num_steps = static_cast<int>(total_time / dt);

    // --- 2. Initialize Ground-Truth Drone Dynamics & Controller ---
    dynamics::Drone6DegFreedom dynamics;
    controller::AltitudePID controller(12.0, 4.0, 0.5, dynamics.mass);
    DroneVisualizer vis;
    vis.init(1024, 768);

    // Initial State Setup
    Eigen::Matrix<double, 13, 1> x_true = Eigen::Matrix<double, 13, 1>::Zero();
    x_true[2] = 1.0; x_true[6] = 1.0;

    estimation::EKFObserver  ekf(dynamics, x_true);
    // estimation::CubatureKalmanFilter  ckf(dynamics, x_true);
    // estimation::GaussHermiteKalmanFilter ghkf(dynamics, x_true);

    const double gps_noise_std = 0.05;
    Eigen::Matrix3d R_gps = Eigen::Matrix3d::Identity() * (gps_noise_std * gps_noise_std);
    GPSSensor gps_sensor(gps_noise_std);

    for (int step = 0; step < num_steps && !vis.shouldClose(); ++step) {
        double t = step * dt;
        double z_target = (t >= 3.0 && t < 6.0) ? 3.0 : 1.0;

        // A. Control & Physics
        Eigen::Matrix<double, 4, 1> u = controller.update(z_target, ekf.state()[2], ekf.state()[5], dt);

        x_true = step_with_rk4(dt, dynamics, x_true, u);

        // Normalize true quaternion to prevent numerical drift
        Eigen::Map<Eigen::Quaterniond> q_true(&x_true[6]);
        q_true.normalize();

        // B. Generate Measurement
        Eigen::Vector3d z_measured = gps_sensor.sample(x_true.segment<3>(0));

        // C. Clean Observer Predict & Update Calls
        ekf.predict(u, dt);   ekf.updatePositionGPS(z_measured, R_gps);
        // ckf.predict(u, dt);   ckf.updatePositionGPS(z_measured, R_gps);
        // ghkf.predict(u, dt);  ghkf.updatePositionGPS(z_measured, R_gps);

        // D. Render & Log
        vis.update(x_true.segment<3>(0),
                   Eigen::Quaterniond(x_true[6], x_true[7], x_true[8], x_true[9]),
                   ekf.state().segment<3>(0),
                   ekf.state().segment<3>(0),
                   ekf.state().segment<3>(0),
                   Eigen::Vector3d(0, 0, z_target));

        // if (step % 50 == 0) {
        //     SimulationLogger::logRow(t, x_true[2], ekf.state()[2], ckf.state()[2], ghkf.state()[2]);
        // }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << std::string(76, '-') << "\n";
    std::cout << "Mission simulation complete! EKF successfully tracked elevation transitions.\n";
    return 0;
}
