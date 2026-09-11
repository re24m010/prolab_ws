#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

struct Waypoint {
    double x;
    double y;
};

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node"), current_wp_idx_(0), start_received_(false), robot_yaw_(0.0)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryNode::odomCallback, this, std::placeholders::_1));

        waypoints_ = {
            {0.5, -0.4},
            {0.6, 0.7},
            {-0.5, 0.4},
            {-0.6, -0.6}
        };

        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryNode::controlLoop, this));
        RCLCPP_INFO(this->get_logger(), "Trajectory-Node mit statischem Soll-Pfad gestartet!");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        robot_yaw_ = 2.0 * std::atan2(q_z, q_w);

        // Ursprüngliche Startposition nur beim allerersten Aufruf fixieren
        if (!start_received_) {
            init_x_ = robot_x_;
            init_y_ = robot_y_;
            start_received_ = true;
            publishPlannedPath();
        }
    }

    void publishPlannedPath()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = this->now();

        // 1. Feste Startposition (bleibt ortsfest im Raum stehen)
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = path_msg.header;
        start_pose.pose.position.x = init_x_;
        start_pose.pose.position.y = init_y_;
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(start_pose);

        // 2. Feste Wegpunkte
        for (const auto& wp : waypoints_) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.header;
            p.pose.position.x = wp.x;
            p.pose.position.y = wp.y;
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;
            path_msg.poses.push_back(p);
        }

        path_pub_->publish(path_msg);
    }

    void controlLoop()
    {
        if (!start_received_) return;

        // Statischen Pfad erneut publizieren (für RViz)
        publishPlannedPath();

        geometry_msgs::msg::Twist cmd_msg;

        if (current_wp_idx_ >= waypoints_.size()) {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            cmd_pub_->publish(cmd_msg);
            RCLCPP_INFO_ONCE(this->get_logger(), "Route beendet! Roboter steht.");
            return;
        }

        Waypoint target = waypoints_[current_wp_idx_];
        
        double dx = target.x - robot_x_;
        double dy = target.y - robot_y_;
        double distance = std::sqrt(dx * dx + dy * dy);

        double target_yaw = std::atan2(dy, dx);
        double alpha = target_yaw - robot_yaw_;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));

        if (distance > 0.22) {
            cmd_msg.angular.z = 1.4 * alpha;
            double speed_factor = std::cos(alpha);
            if (speed_factor < 0.0) speed_factor = 0.0; 
            cmd_msg.linear.x = 0.12 * speed_factor; 
        } else {
            RCLCPP_INFO(this->get_logger(), "Wegpunkt %d/%ld erreicht!", (int)current_wp_idx_ + 1, waypoints_.size());
            current_wp_idx_++;
        }

        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_wp_idx_;
    bool start_received_;
    double init_x_, init_y_;
    double robot_x_, robot_y_, robot_yaw_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}