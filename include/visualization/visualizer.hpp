//
// Created by mbero-akoko on 11/11/26.
//

#ifndef DRONE_OBSERVER_VISUALIZER_HPP
#define DRONE_OBSERVER_VISUALIZER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace drone_observer {
    class DroneVisualizer {
        const std::string window_name_ = "Drone Observer: Multi-Filter Comparison (EKF vs CKF vs GHKF)";

        std::vector<Eigen::Vector3d> true_path_;
        std::vector<Eigen::Vector3d> ekf_path_;
        std::vector<Eigen::Vector3d> ckf_path_;
        std::vector<Eigen::Vector3d> ghkf_path_;

        double arm_length_ = 0.25;

    public:
        DroneVisualizer() = default;

        void init(int width = 1024, int height = 768) {
            pangolin::CreateWindowAndBind(window_name_, width, height);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            pangolin::OpenGlRenderState &s_cam = getCameraState();
            pangolin::Handler3D *handler = new pangolin::Handler3D(s_cam);

            pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0 / 768.0)
                    .SetHandler(handler);
        }

        static pangolin::OpenGlRenderState &getCameraState() {
            static pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(3.5, 3.5, 3.5, 0.0, 0.0, 1.5, pangolin::AxisZ)
            );
            return s_cam;
        }

        void update(const Eigen::Vector3d &pos_true,
                    const Eigen::Quaterniond &q_true,
                    const Eigen::Vector3d &pos_ekf,
                    const Eigen::Vector3d &pos_ckf,
                    const Eigen::Vector3d &pos_ghkf,
                    const Eigen::Vector3d &z_target) {
            true_path_.push_back(pos_true);
            ekf_path_.push_back(pos_ekf);
            ckf_path_.push_back(pos_ckf);
            ghkf_path_.push_back(pos_ghkf);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.1f, 0.11f, 0.12f, 1.0f);

            pangolin::OpenGlRenderState &s_cam = getCameraState();
            pangolin::DisplayBase().Activate(s_cam);

            // 1. Draw Environment Grid & World Axes
            drawGrid();

            // 2. Draw Target Setpoint Ring
            drawTargetMarker(z_target);

            // 3. Draw All Trajectory Lines
            drawTrajectories();

            // 4. Draw True Drone Frame Mesh
            drawDroneFrame(pos_true, q_true);

            // 5. Draw Estimator Position Nodes
            glPointSize(8.0f);
            glBegin(GL_POINTS);

            // EKF = Red
            glColor3f(1.0f, 0.2f, 0.2f);
            glVertex3d(pos_ekf.x(), pos_ekf.y(), pos_ekf.z());

            // CKF = Cyan
            glColor3f(0.0f, 0.9f, 1.0f);
            glVertex3d(pos_ckf.x(), pos_ckf.y(), pos_ckf.z());

            // GHKF = Yellow
            glColor3f(1.0f, 0.85f, 0.1f);
            glVertex3d(pos_ghkf.x(), pos_ghkf.y(), pos_ghkf.z());

            glEnd();

            pangolin::FinishFrame();
        }

        bool shouldClose() const {
            return pangolin::ShouldQuit();
        }

    private:
        void drawGrid() {
            glColor3f(0.25f, 0.25f, 0.25f);
            glLineWidth(1.0f);
            glBegin(GL_LINES);
            for (int i = -5; i <= 5; ++i) {
                glVertex3f(i, -5, 0);
                glVertex3f(i, 5, 0);
                glVertex3f(-5, i, 0);
                glVertex3f(5, i, 0);
            }
            glEnd();

            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0.5f, 0, 0);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0.5f, 0);
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0, 0.5f);
            glEnd();
        }

        void drawTargetMarker(const Eigen::Vector3d &target) {
            glColor4f(1.0f, 0.84f, 0.0f, 0.4f);
            glLineWidth(1.5f);
            glBegin(GL_LINE_LOOP);
            const int segs = 16;
            for (int i = 0; i < segs; ++i) {
                double theta = 2.0 * M_PI * i / segs;
                glVertex3d(target.x() + 0.3 * cos(theta), target.y() + 0.3 * sin(theta), target.z());
            }
            glEnd();
        }

        void drawTrajectories() {
            // True Path (Green)
            glLineWidth(2.5f);
            glColor3f(0.0f, 1.0f, 0.4f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: true_path_)
                glVertex3d(pt.x(), pt.y(), pt.z());
            glEnd();

            // EKF Path (Red)
            glLineWidth(1.5f);
            glColor3f(1.0f, 0.2f, 0.2f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: ekf_path_)
                glVertex3d(pt.x(), pt.y(), pt.z());
            glEnd();

            // CKF Path (Cyan)
            glLineWidth(1.5f);
            glColor3f(0.0f, 0.9f, 1.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: ckf_path_)
                glVertex3d(pt.x(), pt.y(), pt.z());
            glEnd();

            // GHKF Path (Yellow)
            glLineWidth(1.5f);
            glColor3f(1.0f, 0.85f, 0.1f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: ghkf_path_)
                glVertex3d(pt.x(), pt.y(), pt.z());
            glEnd();
        }

        void drawDroneFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q) {
            glPushMatrix();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = pos;
            glMultMatrixd(T.data());

            glLineWidth(3.0f);
            glColor3f(0.9f, 0.9f, 0.9f);
            glBegin(GL_LINES);
            glVertex3d(arm_length_ * 0.707, arm_length_ * 0.707, 0);
            glVertex3d(-arm_length_ * 0.707, -arm_length_ * 0.707, 0);
            glVertex3d(arm_length_ * 0.707, -arm_length_ * 0.707, 0);
            glVertex3d(-arm_length_ * 0.707, arm_length_ * 0.707, 0);
            glEnd();

            glColor3f(0.2f, 0.6f, 1.0f);
            const double r_rotor = 0.08;
            double offsets[4][2] = {
                {arm_length_ * 0.707, arm_length_ * 0.707},
                {-arm_length_ * 0.707, -arm_length_ * 0.707},
                {arm_length_ * 0.707, -arm_length_ * 0.707},
                {-arm_length_ * 0.707, arm_length_ * 0.707}
            };

            for (int k = 0; k < 4; ++k) {
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < 12; ++i) {
                    double th = 2.0 * M_PI * i / 12;
                    glVertex3d(offsets[k][0] + r_rotor * cos(th), offsets[k][1] + r_rotor * sin(th), 0.01);
                }
                glEnd();
            }
            glPopMatrix();
        }
    };
} // namespace drone_observer
#endif //DRONE_OBSERVER_VISUALIZER_HPP
