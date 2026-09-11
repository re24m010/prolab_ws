#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalmanFilter_node"), current_v_(0.0), current_w_(0.0), is_initialized_(false)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose_kf", 10);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KalmanFilterNode::motionCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, std::bind(&KalmanFilterNode::measurementCallback, this, std::placeholders::_1));

        x_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Matrix3d::Identity() * 0.1;

        Q_ = Eigen::Matrix3d::Zero();
        Q_(0, 0) = 0.02;
        Q_(1, 1) = 0.02;
        Q_(2, 2) = 0.05;

        R_ = Eigen::Matrix3d::Zero();
        R_(0, 0) = 0.1;
        R_(1, 1) = 0.1;
        R_(2, 2) = 0.2;

        last_time_ = this->now();
    }

private:
    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;

    double current_v_;
    double current_w_;
    rclcpp::Time last_time_;
    bool is_initialized_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    static double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void motionCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_v_ = msg->linear.x;
        current_w_ = msg->angular.z;
    }

    void measurementCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Am Anfang der Berechnungen in measurementCallback:
        auto t_start = std::chrono::high_resolution_clock::now();


        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5) return;

        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        double yaw_measured = 2.0 * std::atan2(q_z, q_w);

        Eigen::Vector3d z(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            yaw_measured
        );

        if (!is_initialized_) {
            x_ = z;
            is_initialized_ = true;
            return;
        }

        // 1. Prädiktion
        double theta_old = x_[2];
        x_[0] += current_v_ * std::cos(theta_old) * dt;
        x_[1] += current_v_ * std::sin(theta_old) * dt;
        x_[2] += current_w_ * dt;
        x_[2] = normalizeAngle(x_[2]);

        P_ = P_ + Q_;

        // 2. Korrektur
        Eigen::Vector3d y = z - x_;
        y[2] = normalizeAngle(y[2]);

        Eigen::Matrix3d S = P_ + R_;
        Eigen::Matrix3d K = P_ * S.inverse();

        x_ = x_ + K * y;
        x_[2] = normalizeAngle(x_[2]);

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K) * P_;

        // 3. Veröffentlichung (RViz-Crash-Safe)
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "odom";

        pose_msg.pose.pose.position.x = x_[0];
        pose_msg.pose.pose.position.y = x_[1];
        pose_msg.pose.pose.position.z = 0.0;

        double half_theta = x_[2] / 2.0;
        pose_msg.pose.pose.orientation.z = std::sin(half_theta);
        pose_msg.pose.pose.orientation.w = std::cos(half_theta);

        pose_msg.pose.covariance.fill(0.0);
        pose_msg.pose.covariance[0]  = std::max(1e-4, P_(0, 0));
        pose_msg.pose.covariance[1]  = P_(0, 1);
        pose_msg.pose.covariance[5]  = P_(0, 2);

        pose_msg.pose.covariance[6]  = P_(1, 0);
        pose_msg.pose.covariance[7]  = std::max(1e-4, P_(1, 1));
        pose_msg.pose.covariance[11] = P_(1, 2);

        pose_msg.pose.covariance[14] = 1e-4; // z
        pose_msg.pose.covariance[21] = 1e-4; // roll
        pose_msg.pose.covariance[28] = 1e-4; // pitch

        pose_msg.pose.covariance[30] = P_(2, 0);
        pose_msg.pose.covariance[31] = P_(2, 1);
        pose_msg.pose.covariance[35] = std::max(1e-4, P_(2, 2));

        pose_pub_->publish(pose_msg);

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

        // Gedrosseltes Logging (z. B. alle 50 Schritte), um das Terminal nicht zu überfluten:
        static int step_counter = 0;
        static double total_time = 0.0;
        total_time += elapsed_us;
        step_counter++;

        if (step_counter % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Mittlere Ausführungszeit (letzte 50 Schritte): %.2f µs", total_time / 50.0);
            total_time = 0.0;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}