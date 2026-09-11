#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("extendedKalmanFilter_node"), is_initialized_(false)
    {
        ekf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose_ekf", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, std::bind(&ExtendedKalmanFilterNode::odomCallback, this, std::placeholders::_1));

        // Initialer Zustand: [x, y, theta]
        x_ = Eigen::Vector3d(0.5, -1.5, 0.0);

        // Initiale Kovarianzmatrix P
        P_ = Eigen::Matrix3d::Identity() * 0.1;

      // Prozessrauschen Q (Modellvertrauen erhöhen)
        Q_ = Eigen::Matrix3d::Zero();
        Q_(0, 0) = 0.005; // x
        Q_(1, 1) = 0.005; // y
        Q_(2, 2) = 0.002; // theta

        // Messrauschen R (Sensorunsicherheit vergrößern, damit die Landmarke nicht übersteuert)
        R_ = Eigen::Matrix2d::Zero();
        R_(0, 0) = 0.5;  // Varianz Distanz
        R_(1, 1) = 0.5;  // Varianz Peilwinkel

        // Landmarke (Zentrum 0, 0)
        landmark_ = Eigen::Vector2d(0.0, 0.0);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Mathematischer EKF (RViz-Crash-Safe) gestartet.");
    }

private:
    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    Eigen::Vector2d landmark_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Time last_time_;
    bool is_initialized_;

    static double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto t_start = std::chrono::high_resolution_clock::now();


        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5) return;

        double odom_x = msg->pose.pose.position.x;
        double odom_y = msg->pose.pose.position.y;
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        double odom_yaw = 2.0 * std::atan2(q_z, q_w);

        if (!is_initialized_) {
            x_[0] = odom_x;
            x_[1] = odom_y;
            x_[2] = odom_yaw;
            is_initialized_ = true;
            return;
        }

        double v = msg->twist.twist.linear.x;
        double w = msg->twist.twist.angular.z;

        // ==========================================
        // 1. PRÄDIKTION
        // ==========================================
        double theta = x_[2];
        x_[0] += v * std::cos(theta) * dt;
        x_[1] += v * std::sin(theta) * dt;
        x_[2] += w * dt;
        x_[2] = normalizeAngle(x_[2]);

        // Jacobi-Matrix G
        Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
        G(0, 2) = -v * std::sin(theta) * dt;
        G(1, 2) =  v * std::cos(theta) * dt;

        // Kovarianzprädiktion
        P_ = G * P_ * G.transpose() + Q_;

        // ==========================================
        // 2. KORREKTUR (Landmarkenmessung)
        // ==========================================
        double dx_meas = landmark_[0] - odom_x;
        double dy_meas = landmark_[1] - odom_y;
        double z_r = std::sqrt(dx_meas * dx_meas + dy_meas * dy_meas);
        double z_phi = normalizeAngle(std::atan2(dy_meas, dx_meas) - odom_yaw);
        Eigen::Vector2d z(z_r, z_phi);

        double dx = landmark_[0] - x_[0];
        double dy = landmark_[1] - x_[1];
        double q = dx * dx + dy * dy;
        double r_pred = std::sqrt(q);
        double phi_pred = normalizeAngle(std::atan2(dy, dx) - x_[2]);
        Eigen::Vector2d h(r_pred, phi_pred);

        if (r_pred > 0.05) {
            Eigen::Matrix<double, 2, 3> H;
            H(0, 0) = -dx / r_pred;
            H(0, 1) = -dy / r_pred;
            H(0, 2) = 0.0;

            H(1, 0) = dy / q;
            H(1, 1) = -dx / q;
            H(1, 2) = -1.0;

            Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
            Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

            Eigen::Vector2d y = z - h;
            y[1] = normalizeAngle(y[1]);

            x_ = x_ + K * y;
            x_[2] = normalizeAngle(x_[2]);

            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            P_ = (I - K * H) * P_;
        }

        // ==========================================
        // 3. PUBLISH MIT RViz-SICHERER KOVARIANZ
        // ==========================================
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "odom";
        pose_msg.header.stamp = now;

        pose_msg.pose.pose.position.x = x_[0];
        pose_msg.pose.pose.position.y = x_[1];
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.z = std::sin(x_[2] / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(x_[2] / 2.0);

        // 6x6 ROS 2 Kovarianzmatrix absichern
        pose_msg.pose.covariance.fill(0.0);

        // x, y Position
        pose_msg.pose.covariance[0]  = std::max(1e-4, P_(0, 0)); // Var(x)
        pose_msg.pose.covariance[1]  = P_(0, 1);                 // Cov(x, y)
        pose_msg.pose.covariance[5]  = P_(0, 2);                 // Cov(x, yaw)

        pose_msg.pose.covariance[6]  = P_(1, 0);                 // Cov(y, x)
        pose_msg.pose.covariance[7]  = std::max(1e-4, P_(1, 1)); // Var(y)
        pose_msg.pose.covariance[11] = P_(1, 2);                 // Cov(y, yaw)

        // z-Achse (verhindert Division durch Null / NaN in RViz)
        pose_msg.pose.covariance[14] = 1e-4;                     // Var(z)

        // Roll, Pitch (müssen positiv sein)
        pose_msg.pose.covariance[21] = 1e-4;                     // Var(roll)
        pose_msg.pose.covariance[28] = 1e-4;                     // Var(pitch)

        // Yaw
        pose_msg.pose.covariance[30] = P_(2, 0);                 // Cov(yaw, x)
        pose_msg.pose.covariance[31] = P_(2, 1);                 // Cov(yaw, y)
        pose_msg.pose.covariance[35] = std::max(1e-4, P_(2, 2)); // Var(yaw)

        ekf_pose_pub_->publish(pose_msg);

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

        static int step_counter = 0;
        static double total_time = 0.0;
        total_time += elapsed_us;
        step_counter++;

        if (step_counter % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "[PF] Mittlere Laufzeit: %.2f µs", total_time / 50.0);
            total_time = 0.0;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}