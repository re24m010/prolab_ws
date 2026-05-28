#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class ExtendedKalmanFilterNode : public rclcpp::Node
{
public:
    ExtendedKalmanFilterNode() : Node("extendedKalmanFilter_node"), current_v_(0.0), current_w_(0.0)
    {
        // Publisher für die EKF Schätzung
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose_ekf", 10);

        // Subscriptions
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ExtendedKalmanFilterNode::motionCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ExtendedKalmanFilterNode::measurementCallback, this, std::placeholders::_1));

        // Systemzustand [x, y, theta]
        x_ = {0.0, 0.0, 0.0};

        // Zustandskovarianz P
        P_ = {0.1, 0.0, 0.0,
              0.0, 0.1, 0.0,
              0.0, 0.0, 0.1};

        // Prozessrauschen Q für EKF (leicht andere Werte als beim KF zur Unterscheidung im Experiment)
        Q_ = {0.015, 0.0,   0.0,
              0.0,   0.015, 0.0,
              0.0,   0.0,   0.04};

        // Messrauschen R
        R_ = {0.08, 0.0,  0.0,
              0.0,  0.08, 0.0,
              0.0,  0.0,  0.15};

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "C++ Extended Kalman Filter Node (extendedKalmanFilter_node) gestartet.");
    }

private:
    void motionCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_v_ = msg->linear.x;
        current_w_ = msg->angular.z;
    }

    void measurementCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0) return;

        // --- 1. PRÄDIKTION (Nicht-lineares Modell) ---
        double theta_old = x_[2];
        
        // Update des Zustands mit echten nicht-linearen Bewegungsgleichungen
        x_[0] += current_v_ * std::cos(theta_old) * dt;
        x_[1] += current_v_ * std::sin(theta_old) * dt;
        x_[2] += current_w_ * dt;

        // Normalisierung des Winkels auf [-PI, PI]
        x_[2] = std::atan2(std::sin(x_[2]), std::cos(x_[2]));

        // --- DYNAMISCHE JACOBI-MATRIX F_k BERECHNEN ---
        // F_k ist die partielle Ableitung der Bewegungsgleichungen nach x, y, theta
        std::vector<double> F = {
            1.0, 0.0, -current_v_ * std::sin(theta_old) * dt,
            0.0, 1.0,  current_v_ * std::cos(theta_old) * dt,
            0.0, 0.0,  1.0
        };

        // P = F * P * F^T + Q
        // Schritt A: FP = F * P
        std::vector<double> FP(9, 0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    FP[r*3 + c] += F[r*3 + k] * P_[k*3 + c];
                }
            }
        }
        // Schritt B: P = FP * F^T + Q
        std::vector<double> P_next(9, 0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    P_next[r*3 + c] += FP[r*3 + k] * F[c*3 + k]; // Transponiert durch c*3 + k
                }
                P_next[r*3 + c] += Q_[r*3 + c];
            }
        }
        P_ = P_next;

        // --- 2. KORREKTUR (Update Step) ---
        std::vector<double> z = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            x_[2] 
        };

        // Innovation y
        std::vector<double> y = { z[0] - x_[0], z[1] - x_[1], z[2] - x_[2] };
        y[2] = std::atan2(std::sin(y[2]), std::cos(y[2])); // Winkel-Innovation normieren

        // Da wir Positionen direkt messen, ist H wieder linear (Identitätsmatrix)
        // S = P + R
        std::vector<double> S(9);
        for (size_t i = 0; i < 9; ++i) {
            S[i] = P_[i] + R_[i];
        }

        // Invertierung der 3x3 Matrix S
        double det = S[0]*(S[4]*S[8] - S[5]*S[7]) - S[1]*(S[3]*S[8] - S[5]*S[6]) + S[2]*(S[3]*S[7] - S[4]*S[6]);
        if (std::abs(det) < 1e-6) return;

        double inv_det = 1.0 / det;
        std::vector<double> S_inv(9);
        S_inv[0] = (S[4]*S[8] - S[5]*S[7]) * inv_det;
        S_inv[1] = (S[2]*S[8] - S[1]*S[8]) * inv_det;
        S_inv[2] = (S[1]*S[5] - S[2]*S[4]) * inv_det;
        S_inv[3] = (S[5]*S[6] - S[3]*S[8]) * inv_det;
        S_inv[4] = (S[0]*S[8] - S[2]*S[6]) * inv_det;
        S_inv[5] = (S[2]*S[3] - S[0]*S[5]) * inv_det;
        S_inv[6] = (S[3]*S[7] - S[4]*S[6]) * inv_det;
        S_inv[7] = (S[1]*S[6] - S[0]*S[7]) * inv_det;
        S_inv[8] = (S[0]*S[4] - S[1]*S[3]) * inv_det;

        // Kalman-Gain: K = P * S_inv
        std::vector<double> K(9, 0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    K[r*3 + c] += P_[r*3 + k] * S_inv[k*3 + c];
                }
            }
        }

        // Zustand korrigieren: x = x + K * y
        x_[0] += K[0]*y[0] + K[1]*y[1] + K[2]*y[2];
        x_[1] += K[3]*y[0] + K[4]*y[1] + K[5]*y[2];
        x_[2] += K[6]*y[0] + K[7]*y[1] + K[8]*y[2];
        x_[2] = std::atan2(std::sin(x_[2]), std::cos(x_[2]));

        // Kovarianz korrigieren: P = (I - K)*P
        std::vector<double> I_minus_K = {
            1.0 - K[0], -K[1],      -K[2],
            -K[3],      1.0 - K[4], -K[5],
            -K[6],      -K[7],      1.0 - K[8]
        };
        
        std::vector<double> P_new(9, 0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    P_new[r*3 + c] += I_minus_K[r*3 + k] * P_[k*3 + c];
                }
            }
        }
        P_ = P_new;

        publishPose();
    }

    void publishPose()
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        msg.pose.position.x = x_[0];
        msg.pose.position.y = x_[1];
        msg.pose.position.z = 0.0;

        double half_theta = x_[2] / 2.0;
        msg.pose.orientation.z = std::sin(half_theta);
        msg.pose.orientation.w = std::cos(half_theta);

        pose_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<double> x_;
    std::vector<double> P_;
    std::vector<double> Q_;
    std::vector<double> R_;

    double current_v_;
    double current_w_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtendedKalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
