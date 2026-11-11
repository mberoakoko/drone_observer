//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_VISUALIZER_HPP
#define DRONE_OBSERVER_VISUALIZER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace drone_observer::visualization {
    class DroneVisualizer {
    public:
        explicit DroneVisualizer(size_t max_trace_points = 3000)
            : max_points_(max_trace_points) {
        }

        void init(int width = 1280, int height = 768) {
            pangolin::CreateWindowAndBind("Drone Observer & Telemetry Visualizer", width, height);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // ---------------------------------------------------------------------
            // 1. Setup 3D Render Viewport (Left 70% of screen)
            // ---------------------------------------------------------------------
            s_cam_ = pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(width * 0.7, height, 420, 420, (width * 0.7) / 2.0, height / 2.0, 0.1, 1000),
                pangolin::ModelViewLookAt(6.0, -6.0, 6.0, 0.0, 0.0, 1.5, pangolin::AxisZ)
            );

            handler_ = std::make_unique<pangolin::Handler3D>(s_cam_);

            d_cam_ = &pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 0.7, -static_cast<double>(width * 0.7) / height)
                    .SetHandler(handler_.get());

            // ---------------------------------------------------------------------
            // 2. Setup 2D Telemetry Logs & Plotters (Right 30% of screen)
            // ---------------------------------------------------------------------
            // Log 1: Position Errors (ex, ey, ez)
            log_pos_err_.SetLabels({"ex [m]", "ey [m]", "ez [m]"});
            plot_pos_err_ = std::make_unique<pangolin::Plotter>(&log_pos_err_, 0.0, 500.0, -2.0, 2.0, 50.0, 0.5);

            // Log 2: Control Inputs (Thrust T, Tau_x, Tau_y, Tau_z)
            log_control_.SetLabels({"Thrust [N]", "Tau_x [Nm]", "Tau_y [Nm]", "Tau_z [Nm]"});
            plot_control_ = std::make_unique<pangolin::Plotter>(&log_control_, 0.0, 500.0, -1.0, 15.0, 50.0, 2.0);

            // Stack plots on the right side
            pangolin::CreateDisplay()
                    .SetBounds(0.5, 1.0, 0.7, 1.0)
                    .AddDisplay(*plot_pos_err_);

            pangolin::CreateDisplay()
                    .SetBounds(0.0, 0.5, 0.7, 1.0)
                    .AddDisplay(*plot_control_);
        }

        /**
         * @brief Renders 3D environment, ground truth, observer estimates, and appends 2D telemetry plots.
         */
        void update(double t,
                    const Eigen::Vector3d &pos_true,
                    const Eigen::Quaterniond &q_true,
                    const Eigen::Vector3d &pos_ekf,
                    const Eigen::Vector3d &pos_ckf,
                    const Eigen::Vector3d &pos_ghkf,
                    const Eigen::Vector3d &pos_target,
                    const Eigen::Vector4d &u_control) {
            if (pangolin::ShouldQuit()) return;

            // 1. Update 3D Traces
            appendPoint(target_trace_, pos_target);
            appendPoint(truth_trace_, pos_true);
            appendPoint(ekf_trace_, pos_ekf);
            appendPoint(ckf_trace_, pos_ckf);
            appendPoint(ghkf_trace_, pos_ghkf);

            // 2. Log 2D Telemetry Data
            const Eigen::Vector3d pos_err = pos_target - pos_true;
            log_pos_err_.Log(pos_err.x(), pos_err.y(), pos_err.z());
            log_control_.Log(u_control[0], u_control[1], u_control[2], u_control[3]);

            // 3. Clear Screen and Render 3D Viewport
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam_->Activate(s_cam_);

            drawGrid();

            // 3D Line Strips
            drawTrace(target_trace_, 1.0f, 0.8f, 0.0f, 1.5f); // Yellow:  Reference Target Path
            drawTrace(truth_trace_, 0.0f, 1.0f, 0.0f, 2.5f); // Green:   Ground Truth Trajectory
            drawTrace(ekf_trace_, 1.0f, 0.0f, 0.0f, 2.0f); // Red:     EKF Observer Estimate
            drawTrace(ckf_trace_, 0.0f, 1.0f, 1.0f, 2.0f); // Cyan:    CKF Observer Estimate
            drawTrace(ghkf_trace_, 1.0f, 0.0f, 1.0f, 2.0f); // Magenta: GHKF Observer Estimate

            // Quadrotor Frame
            drawDroneFrame(pos_true, q_true);

            // 4. Render Pangolin 2D Plot Views
            pangolin::FinishFrame();
        }

    private:
        void appendPoint(std::vector<Eigen::Vector3d> &trace, const Eigen::Vector3d &pt) const {
            trace.push_back(pt);
            if (trace.size() > max_points_) {
                trace.erase(trace.begin());
            }
        }

        static void drawTrace(const std::vector<Eigen::Vector3d> &trace, float r, float g, float b, float width) {
            if (trace.size() < 2) return;

            glLineWidth(width);
            glColor3f(r, g, b);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: trace) {
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();
        }

        void drawGrid() const {
            glLineWidth(1.0f);
            glColor4f(0.3f, 0.3f, 0.3f, 0.4f);
            glBegin(GL_LINES);
            for (int i = -10; i <= 10; ++i) {
                glVertex3d(i, -10, 0);
                glVertex3d(i, 10, 0);
                glVertex3d(-10, i, 0);
                glVertex3d(10, i, 0);
            }
            glEnd();
        }

        void drawDroneFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q) const {
            glPushMatrix();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = pos;
            glMultMatrixd(T.data());

            glLineWidth(3.0f);
            glColor3f(0.8f, 0.8f, 0.8f);
            glBegin(GL_LINES);
            const double d = 0.225;
            glVertex3d(-d, -d, 0);
            glVertex3d(d, d, 0);
            glVertex3d(-d, d, 0);
            glVertex3d(d, -d, 0);
            glEnd();

            glPopMatrix();
        }

        size_t max_points_;
        std::vector<Eigen::Vector3d> target_trace_;
        std::vector<Eigen::Vector3d> truth_trace_;
        std::vector<Eigen::Vector3d> ekf_trace_;
        std::vector<Eigen::Vector3d> ckf_trace_;
        std::vector<Eigen::Vector3d> ghkf_trace_;

        pangolin::OpenGlRenderState s_cam_;
        std::unique_ptr<pangolin::Handler3D> handler_;
        pangolin::View *d_cam_{nullptr};

        // Pangolin Data Logging and 2D Plotters
        pangolin::DataLog log_pos_err_;
        pangolin::DataLog log_control_;
        std::unique_ptr<pangolin::Plotter> plot_pos_err_;
        std::unique_ptr<pangolin::Plotter> plot_control_;
    };
} // namespace drone_observer:
#endif //DRONE_OBSERVER_VISUALIZER_HPP
