#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

struct Particle {
    double x;
    double y;
    double yaw;
    double weight;
};

class ParticleFilterNode : public rclcpp::Node
{
public:
    ParticleFilterNode() : Node("particleFilter_node"), num_particles_(100), is_initialized_(false)
    {
        pf_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose_pf", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_noisy", 10, std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1));

        // Landmarke im Ursprung (0, 0)
        landmark_x_ = 0.0;
        landmark_y_ = 0.0;

        // Messrauschen-Standardabweichungen
        sigma_range_ = 0.2;
        sigma_bearing_ = 0.1;

        // Zufallsgenerator
        std::random_device rd;
        gen_ = std::default_random_engine(rd());

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Mathematischer PF (Option A + PoseWithCovarianceStamped) gestartet.");
    }

private:
    int num_particles_;
    std::vector<Particle> particles_;
    bool is_initialized_;

    double landmark_x_;
    double landmark_y_;
    double sigma_range_;
    double sigma_bearing_;

    std::default_random_engine gen_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pf_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Time last_time_;

    static double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double gaussianProb(double diff, double sigma)
    {
        return std::exp(-0.5 * std::pow(diff / sigma, 2.0)) / (std::sqrt(2.0 * M_PI) * sigma);
    }

    void initializeParticles(double x0, double y0, double yaw0)
    {
        std::normal_distribution<double> dist_pos(0.0, 0.05);
        std::normal_distribution<double> dist_yaw(0.0, 0.02);

        particles_.clear();
        particles_.resize(num_particles_);
        for (int i = 0; i < num_particles_; ++i) {
            particles_[i].x = x0 + dist_pos(gen_);
            particles_[i].y = y0 + dist_pos(gen_);
            particles_[i].yaw = normalizeAngle(yaw0 + dist_yaw(gen_));
            particles_[i].weight = 1.0 / num_particles_;
        }
        is_initialized_ = true;
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
            initializeParticles(odom_x, odom_y, odom_yaw);
            return;
        }

        double v = msg->twist.twist.linear.x;
        double w = msg->twist.twist.angular.z;

        // ==========================================
        // 1. PRÄDIKTION (Propagierung mit Prozessrauschen)
        // ==========================================
        std::normal_distribution<double> noise_v(0.0, 0.03);
        std::normal_distribution<double> noise_w(0.0, 0.02);

        for (auto& p : particles_) {
            double v_p = v + noise_v(gen_);
            double w_p = w + noise_w(gen_);

            p.x += v_p * std::cos(p.yaw) * dt;
            p.y += v_p * std::sin(p.yaw) * dt;
            p.yaw = normalizeAngle(p.yaw + w_p * dt);
        }

        // ==========================================
        // 2. KORREKTUR (Messmodell & Gewichtung)
        // ==========================================
        // Landmarken-Messung aus verrauschter Odometrie generieren
        double dx_meas = landmark_x_ - odom_x;
        double dy_meas = landmark_y_ - odom_y;
        double z_range = std::sqrt(dx_meas * dx_meas + dy_meas * dy_meas);
        double z_bearing = normalizeAngle(std::atan2(dy_meas, dx_meas) - odom_yaw);

        double total_weight = 0.0;
        for (auto& p : particles_) {
            double dx_p = landmark_x_ - p.x;
            double dy_p = landmark_y_ - p.y;
            double pred_range = std::sqrt(dx_p * dx_p + dy_p * dy_p);
            double pred_bearing = normalizeAngle(std::atan2(dy_p, dx_p) - p.yaw);

            double diff_range = z_range - pred_range;
            double diff_bearing = normalizeAngle(z_bearing - pred_bearing);

            // Likelihood aus Gauß-Dichten
            double p_r = gaussianProb(diff_range, sigma_range_);
            double p_b = gaussianProb(diff_bearing, sigma_bearing_);
            p.weight = p_r * p_b + 1e-9; // Numerische Stabilität
            total_weight += p.weight;
        }

        // Gewichte normalisieren
        for (auto& p : particles_) {
            p.weight /= total_weight;
        }

        // ==========================================
        // 3. SYSTEMATISCHES RESAMPLING (Low-Variance)
        // ==========================================
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);

        std::uniform_real_distribution<double> dist_u(0.0, 1.0 / num_particles_);
        double r = dist_u(gen_);
        double c = particles_[0].weight;
        int idx = 0;

        for (int m = 0; m < num_particles_; ++m) {
            double u = r + (double)m / num_particles_;
            while (u > c && idx < num_particles_ - 1) {
                idx++;
                c += particles_[idx].weight;
            }
            new_particles.push_back(particles_[idx]);
            new_particles.back().weight = 1.0 / num_particles_;
        }
        particles_ = std::move(new_particles);

        // ==========================================
        // 4. ZUSTAND & KOVARIANZ BERECHNEN
        // ==========================================
        double mean_x = 0.0, mean_y = 0.0;
        double sum_sin = 0.0, sum_cos = 0.0;

        for (const auto& p : particles_) {
            mean_x += p.x;
            mean_y += p.y;
            sum_sin += std::sin(p.yaw);
            sum_cos += std::cos(p.yaw);
        }
        mean_x /= num_particles_;
        mean_y /= num_particles_;
        double mean_yaw = std::atan2(sum_sin, sum_cos);

        // Empirische Kovarianz der Partikelwolke
        double cov_xx = 0.0, cov_yy = 0.0, cov_xy = 0.0, cov_yaw = 0.0;
        for (const auto& p : particles_) {
            double dx = p.x - mean_x;
            double dy = p.y - mean_y;
            double dyaw = normalizeAngle(p.yaw - mean_yaw);

            cov_xx += dx * dx;
            cov_yy += dy * dy;
            cov_xy += dx * dy;
            cov_yaw += dyaw * dyaw;
        }
        cov_xx /= num_particles_;
        cov_yy /= num_particles_;
        cov_xy /= num_particles_;
        cov_yaw /= num_particles_;

        // ==========================================
        // 5. PUBLISH MIT KOVARIANZ (RViz-Crash-Safe)
        // ==========================================
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "odom";

        pose_msg.pose.pose.position.x = mean_x;
        pose_msg.pose.pose.position.y = mean_y;
        pose_msg.pose.pose.position.z = 0.0;

        double half_yaw = mean_yaw / 2.0;
        pose_msg.pose.pose.orientation.z = std::sin(half_yaw);
        pose_msg.pose.pose.orientation.w = std::cos(half_yaw);

        pose_msg.pose.covariance.fill(0.0);
        pose_msg.pose.covariance[0]  = std::max(1e-4, cov_xx);
        pose_msg.pose.covariance[1]  = cov_xy;
        pose_msg.pose.covariance[6]  = cov_xy;
        pose_msg.pose.covariance[7]  = std::max(1e-4, cov_yy);

        // Stabilisierung gegen RViz-Division durch 0
        pose_msg.pose.covariance[14] = 1e-4; // z
        pose_msg.pose.covariance[21] = 1e-4; // roll
        pose_msg.pose.covariance[28] = 1e-4; // pitch
        pose_msg.pose.covariance[35] = std::max(1e-4, cov_yaw);

        pf_pose_pub_->publish(pose_msg);

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
    rclcpp::spin(std::make_shared<ParticleFilterNode>());
    rclcpp::shutdown();
    return 0;
}