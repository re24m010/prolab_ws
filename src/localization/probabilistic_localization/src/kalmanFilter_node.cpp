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

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalmanFilter_node"), current_v_(0.0), current_w_(0.0)
    {
        // Publisher für die geschätzte Pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose_kf", 10);

        // Subscriber für Geschwindigkeitsbefehle und Odometrie
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KalmanFilterNode::motionCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&KalmanFilterNode::measurementCallback, this, std::placeholders::_1));

        // Initialisierung des Systemzustands [x, y, theta]
        x_ = {0.0, 0.0, 0.0};

        // Zustandskovarianz P (Anfangsungewissheit)
        P_ = {0.1, 0.0, 0.0,
              0.0, 0.1, 0.0,
              0.0, 0.0, 0.1};

        // Prozessrauschen Q (Modell-Unsicherheit) -> AUFGABE: Im Experiment variieren!
        Q_ = {0.02, 0.0,  0.0,
              0.0,  0.02, 0.0,
              0.0,  0.0,  0.05};

        // Messrauschen R (Sensor-Unsicherheit) -> AUFGABE: Im Experiment variieren!
        R_ = {0.1, 0.0, 0.0,
              0.0, 0.1, 0.0,
              0.0, 0.0, 0.2};

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "C++ Kalman Filter Node (kalmanFilter_node) erfolgreich gestartet.");
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

        // --- 1. PRÄDIKTION (Prediction Step) ---
        double theta_old = x_[2];
        
        // Zustand vorhersagen (A-Priori)
        x_[0] += current_v_ * std::cos(theta_old) * dt;
        x_[1] += current_v_ * std::sin(theta_old) * dt;
        x_[2] += current_w_ * dt;

        // Da es ein lineares KF ist, nehmen wir die Jacobi/Übergangsmatrix F als Identitätsmatrix an.
        // P = F * P * F^T + Q -> Da F = I, gilt einfach: P = P + Q
        for (size_t i = 0; i < 9; ++i) {
            P_[i] += Q_[i];
        }

        // --- 2. KORREKTUR (Update Step) ---
        // Messvektor z aus der verrauschten Odometrie holen
        std::vector<double> z = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            x_[2] // Zur Stabilisierung der Orientierung im linearen Filter
        };

        // Innovation (Messabweichung) y = z - H*x (H ist Identitätsmatrix)
        std::vector<double> y = { z[0] - x_[0], z[1] - x_[1], z[2] - x_[2] };

        // Innovationskovarianz S = H * P * H^T + R -> Da H = I, gilt S = P + R
        std::vector<double> S(9);
        for (size_t i = 0; i < 9; ++i) {
            S[i] = P_[i] + R_[i];
        }

        // Invertierung der 3x3 Matrix S (Determinanten-Methode)
        double det = S[0]*(S[4]*S[8] - S[5]*S[7]) - S[1]*(S[3]*S[8] - S[5]*S[6]) + S[2]*(S[3]*S[7] - S[4]*S[6]);
        if (std::abs(det) < 1e-6) return; // Schutz vor Division durch Null

        double inv_det = 1.0 / det;
        std::vector<double> S_inv(9);
        S_inv[0] = (S[4]*S[8] - S[5]*S[7]) * inv_det;
        S_inv[1] = (S[2]*S[8] - S[1]*S[8]) * inv_det; // Vereinfachte Kofaktormatrix
        S_inv[2] = (S[1]*S[5] - S[2]*S[4]) * inv_det;
        S_inv[3] = (S[5]*S[6] - S[3]*S[8]) * inv_det;
        S_inv[4] = (S[0]*S[8] - S[2]*S[6]) * inv_det;
        S_inv[5] = (S[2]*S[3] - S[0]*S[5]) * inv_det;
        S_inv[6] = (S[3]*S[7] - S[4]*S[6]) * inv_det;
        S_inv[7] = (S[1]*S[6] - S[0]*S[7]) * inv_det;
        S_inv[8] = (S[0]*S[4] - S[1]*S[3]) * inv_det;

        // Kalman-Gain berechnen: K = P * S_inv (Da H = I)
        std::vector<double> K(9, 0.0);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    K[r*3 + c] += P_[r*3 + k] * S_inv[k*3 + c];
                }
            }
        }

        // Zustand korrigieren (A-Posteriori): x = x + K * y
        x_[0] += K[0]*y[0] + K[1]*y[1] + K[2]*y[2];
        x_[1] += K[3]*y[0] + K[4]*y[1] + K[5]*y[2];
        x_[2] += K[6]*y[0] + K[7]*y[1] + K[8]*y[2];

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

        // Schätzung an RViz senden
        publishPose();
    }

    void publishPose()
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom"; // Laut Aufgabenstellung gleicher Bezugsrahmen

        msg.pose.position.x = x_[0];
        msg.pose.position.y = x_[1];
        msg.pose.position.z = 0.0;

        // Euler-Winkel in Quaternion umrechnen
        double half_theta = x_[2] / 2.0;
        msg.pose.orientation.z = std::sin(half_theta);
        msg.pose.orientation.w = std::cos(half_theta);

        pose_pub_->publish(msg);
    }

    // ROS 2 Kommunikations-Objekte
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Filtervariablen (Zustand und Matrizen als abgeflachte 1D-Vektoren)
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
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
