#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilterNode : public rclcpp::Node
{
public:
    ParticleFilterNode() : Node("particleFilter_node"), current_v_(0.0), current_w_(0.0), num_particles_(150)
    {
        // Publisher für die PF Schätzung
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose_pf", 10);

        // Subscriptions
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ParticleFilterNode::motionCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ParticleFilterNode::measurementCallback, this, std::placeholders::_1));

        // Random Number Generator initialisieren
        std::random_device rd;
        gen_.seed(rd());

        // Partikelschwarm initialisieren (alle starten nahe [0,0,0] mit gleichem Gewicht)
        std::normal_distribution<double> d_x(0.0, 0.05);
        std::normal_distribution<double> d_y(0.0, 0.05);
        std::normal_distribution<double> d_th(0.0, 0.05);

        particles_.resize(num_particles_);
        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].x = d_x(gen_);
            particles_[i].y = d_y(gen_);
            particles_[i].theta = d_th(gen_);
            particles_[i].weight = 1.0 / num_particles_;
        }

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "C++ Partikelfilter Node (particleFilter_node) gestartet.");
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

        // Rausch-Generatoren für die Partikelbewegung (Process Noise Q Simulation)
        // AUFGABE: Werte im Experiment variieren!
        std::normal_distribution<double> noise_v(0.0, 0.02);
        std::normal_distribution<double> noise_w(0.0, 0.05);

        // --- 1. PRÄDIKTION (Jeden Partikel einzeln bewegen + Rauschen addieren) ---
        for (int i = 0; i < num_particles_; ++i) {
            double v_noisy = current_v_ + noise_v(gen_);
            double w_noisy = current_w_ + noise_w(gen_);

            particles_[i].x += v_noisy * std::cos(particles_[i].theta) * dt;
            particles_[i].y += v_noisy * std::sin(particles_[i].theta) * dt;
            particles_[i].theta += w_noisy * dt;
            particles_[i].theta = std::atan2(std::sin(particles_[i].theta), std::cos(particles_[i].theta));
        }

        // --- 2. MESS-UPDATE (Gewichtung basierend auf Sensorwerten) ---
        double z_x = msg->pose.pose.position.x;
        double z_y = msg->pose.pose.position.y;
        
        // Sensorrauschen-Annahme R
        // AUFGABE: Werte im Experiment variieren!
        double sigma_sensor = 0.1; 
        double weight_sum = 0.0;

        for (int i = 0; i < num_particles_; ++i) {
            // Euklidischer Abstand zwischen Partikel-Hypothese und echter Messung
            double dist_x = z_x - particles_[i].x;
            double dist_y = z_y - particles_[i].y;
            double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

            // Gaußsche Wahrscheinlichkeitsdichte als Gewicht (je näher am Sensor, desto höher das Gewicht)
            particles_[i].weight = std::exp(-(distance * distance) / (2.0 * sigma_sensor * sigma_sensor));
            weight_sum += particles_[i].weight;
        }

        // Gewichte normalisieren (Summe muss 1.0 ergeben)
        if (weight_sum > 0.0) {
            for (int i = 0; i < num_particles_; ++i) {
                particles_[i].weight /= weight_sum;
            }
        } else {
            // Falls alle Partikel zu weit weg waren, Gewichte gleichmäßig verteilen
            for (int i = 0; i < num_particles_; ++i) {
                particles_[i].weight = 1.0 / num_particles_;
            }
        }

        // --- 3. RESAMPLING (Systematische Auswahl starker Partikel) ---
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);
        
        std::uniform_real_distribution<double> uni_dist(0.0, 1.0 / num_particles_);
        double r = uni_dist(gen_);
        double c = particles_[0].weight;
        int idx = 0;

        for (int i = 0; i < num_particles_; ++i) {
            double u = r + i * (1.0 / num_particles_);
            while (u > c && idx < num_particles_ - 1) {
                idx++;
                c += particles_[idx].weight;
            }
            new_particles.push_back(particles_[idx]);
            new_particles.back().weight = 1.0 / num_particles_; // Gewicht zurücksetzen
        }
        particles_ = new_particles;

        // --- 4. ZUSTANDSSCHÄTZUNG (Mittelwert aller Partikel berechnen) ---
        double mean_x = 0.0;
        double mean_y = 0.0;
        double mean_sin = 0.0;
        double mean_cos = 0.0;

        for (int i = 0; i < num_particles_; ++i) {
            mean_x += particles_[i].x;
            mean_y += particles_[i].y;
            mean_sin += std::sin(particles_[i].theta);
            mean_cos += std::cos(particles_[i].theta);
        }

        double est_x = mean_x / num_particles_;
        double est_y = mean_y / num_particles_;
        double est_th = std::atan2(mean_sin, mean_cos);

        publishPose(est_x, est_y, est_th);
    }

    void publishPose(double x, double y, double th)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = 0.0;

        double half_theta = th / 2.0;
        msg.pose.orientation.z = std::sin(half_theta);
        msg.pose.orientation.w = std::cos(half_theta);

        pose_pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::vector<Particle> particles_;
    int num_particles_;
    double current_v_;
    double current_w_;
    rclcpp::Time last_time_;
    std::mt19937 gen_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}
