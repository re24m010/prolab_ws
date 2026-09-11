#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class NoiseSimulatorNode : public rclcpp::Node
{
public:
    NoiseSimulatorNode() : Node("noise_simulator_node")
    {
        // Subscriber für die perfekte Odometrie aus Gazebo
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NoiseSimulatorNode::odomCallback, this, std::placeholders::_1));

        // Publisher für die künstlich verrauschte Odometrie
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_noisy", 10);

        // Zufallsgenerator initialisieren
        std::random_device rd;
        gen_.seed(rd());

        // Definition des Sensorrauschens (R) für das Experiment
        // AUFGABE: Diese Standardabweichungen (Sigmas) für die Experimente variieren!
        linear_noise_sigma_ = 0.15;  // 15 cm Rauschen auf Position X und Y
        angular_noise_sigma_ = 0.20; // ~11 Grad Rauschen auf die Orientierung (Yaw)

        RCLCPP_INFO(this->get_logger(), "Noise Simulator Node erfolgreich gestartet. Topic: /odom_noisy");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Normalverteilte Rauschgeneratoren
        std::normal_distribution<double> dist_linear(0.0, linear_noise_sigma_);
        std::normal_distribution<double> dist_angular(0.0, angular_noise_sigma_);

        // Kopiere die originale Nachricht
        auto noisy_msg = *msg;

        // 1. Rauschen auf Position beaufschlagen
        noisy_msg.pose.pose.position.x += dist_linear(gen_);
        noisy_msg.pose.pose.position.y += dist_linear(gen_);

        // 2. Rauschen auf Orientierung (Yaw) beaufschlagen
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        double current_yaw = 2.0 * std::atan2(q_z, q_w);

        // Rauschen addieren und winkel-normalisieren
        double noisy_yaw = current_yaw + dist_angular(gen_);
        noisy_yaw = std::atan2(std::sin(noisy_yaw), std::cos(noisy_yaw));

        // Zurück in Quaternion für die ROS 2 Nachricht umrechnen
        noisy_msg.pose.pose.orientation.z = std::sin(noisy_yaw / 2.0);
        noisy_msg.pose.pose.orientation.w = std::cos(noisy_yaw / 2.0);

        // Verrauschte Nachricht publishen
        odom_pub_->publish(noisy_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::mt19937 gen_;
    double linear_noise_sigma_;
    double angular_noise_sigma_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoiseSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}