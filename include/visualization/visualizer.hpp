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
    private:
        const std::string window_name_ = "Drone Observer: 3D Tracking Visualizer";
        std::vector<Eigen::Vector3d> true_path_;
        std::vector<Eigen::Vector3d> ekf_path_;

        double arm_length_ = 0.25; // 25 cm arm radius

    public:
        DroneVisualizer() = default;

        void init(int width = 1024, int height = 768) const {
            pangolin::CreateWindowAndBind(window_name_, width, height);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            // Define Projection and Initial Viewpoint Camera
            pangolin::OpenGlRenderState &s_cam = getCameraState();

            // Add Interactive Viewport
            auto *handler = new pangolin::Handler3D(s_cam);
            pangolin::View &d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0 / 768.0)
                    .SetHandler(handler);
        }

        static pangolin::OpenGlRenderState &getCameraState() {
            static pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
                pangolin::ModelViewLookAt(3.0, 3.0, 3.0, 0.0, 0.0, 1.5, pangolin::AxisZ)
            );
            return s_cam;
        }

        void update(const Eigen::Vector3d &pos_true,
                    const Eigen::Quaterniond &q_true,
                    const Eigen::Vector3d &pos_ekf,
                    const Eigen::Vector3d &z_target) {
            true_path_.push_back(pos_true);
            ekf_path_.push_back(pos_ekf);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(0.1f, 0.11f, 0.12f, 1.0f); // Dark background

            pangolin::OpenGlRenderState &s_cam = getCameraState();
            d_cam_activate(s_cam);

            // 1. Draw Ground Plane & Grid
            drawGrid();

            // 2. Draw Target Altitude Marker (Yellow Wireframe Ring/Box)
            drawTargetMarker(z_target);

            // 3. Draw Trajectory Lines (Green = True Ground Truth, Red = EKF Estimate)
            drawTrajectories();

            // 4. Draw Drone Frame at True Pose
            drawDroneFrame(pos_true, q_true);

            // 5. Draw EKF Estimate Position as a Magenta Point
            glPointSize(8.0f);
            glBegin(GL_POINTS);
            glColor3f(1.0f, 0.0f, 1.0f); // Magenta
            glVertex3d(pos_ekf.x(), pos_ekf.y(), pos_ekf.z());
            glEnd();

            pangolin::FinishFrame();
        }

        bool shouldClose() const {
            return pangolin::ShouldQuit();
        }

    private:
        void d_cam_activate(pangolin::OpenGlRenderState &s_cam) {
            pangolin::DisplayBase().Activate(s_cam);
        }

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

            // Coordinate Axes at Origin
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0.5f, 0, 0); // X-Red
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0.5f, 0); // Y-Green
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0, 0.5f); // Z-Blue
            glEnd();
        }

        void drawTargetMarker(const Eigen::Vector3d &target) {
            glColor4f(1.0f, 0.84f, 0.0f, 0.4f); // Transparent Gold
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
            glLineWidth(2.0f);
            glColor3f(0.0f, 1.0f, 0.4f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: true_path_) {
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();

            // EKF Estimated Path (Red dashed look)
            glLineWidth(2.0f);
            glColor3f(1.0f, 0.2f, 0.2f);
            glBegin(GL_LINE_STRIP);
            for (const auto &pt: ekf_path_) {
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();
        }

        void drawDroneFrame(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q) {
            glPushMatrix();

            // Apply Position Translation and Quaternion Rotation to OpenGL matrix
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = pos;
            glMultMatrixd(T.data());

            // Quadrotor Cross Arms (Cyan/White)
            glLineWidth(3.0f);
            glColor3f(0.9f, 0.9f, 0.9f);
            glBegin(GL_LINES);
            // Arm 1: (+x, +y) to (-x, -y)
            glVertex3d(arm_length_ * 0.707, arm_length_ * 0.707, 0);
            glVertex3d(-arm_length_ * 0.707, -arm_length_ * 0.707, 0);
            // Arm 2: (+x, -y) to (-x, +y)
            glVertex3d(arm_length_ * 0.707, -arm_length_ * 0.707, 0);
            glVertex3d(-arm_length_ * 0.707, arm_length_ * 0.707, 0);
            glEnd();

            // Draw 4 Rotor Rings
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
