#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

enum class RobotState {
    DRIVE_FORWARD,
    TURN_90_DEG,
    STOP
};

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node"), state_(RobotState::DRIVE_FORWARD), loop_count_(0)
    {
        // Publisher für die Fahrbefehle des Roboters
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer für die Statemachine (läuft mit 10 Hz / alle 100ms)
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryNode::controlLoop, this));

        state_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Automatischer Trajectory Node gestartet. Roboter fährt jetzt ein Quadrat.");
    }

private:
    void controlLoop()
    {
        auto now = this->now();
        double elapsed_time = (now - state_start_time_).seconds();
        geometry_msgs::msg::Twist cmd_msg;

        switch (state_)
        {
            case RobotState::DRIVE_FORWARD:
                // Fahre für 4 Sekunden geradeaus mit 0.15 m/s
                if (elapsed_time < 4.0) {
                    cmd_msg.linear.x = 0.15;
                    cmd_msg.angular.z = 0.0;
                } else {
                    // Wechsel zum Drehen
                    state_ = RobotState::TURN_90_DEG;
                    state_start_time_ = now;
                    RCLCPP_INFO(this->get_logger(), "Ecke erreicht. Drehe um 90 Grad...");
                }
                break;

            case RobotState::TURN_90_DEG:
                // Drehe für ca. 2.7 Sekunden mit 0.5 rad/s (~90 Grad bei flachen Reifenschlupf)
                if (elapsed_time < 2.7) {
                    cmd_msg.linear.x = 0.0;
                    cmd_msg.angular.z = 0.5;
                } else {
                    loop_count_++;
                    // Ein Quadrat hat 4 Ecken. Nach 4 Runden hält der Roboter an.
                    if (loop_count_ >= 4) {
                        state_ = RobotState::STOP;
                        RCLCPP_INFO(this->get_logger(), "Quadrat-Trajektorie erfolgreich beendet. Stoppe.");
                    } else {
                        state_ = RobotState::DRIVE_FORWARD;
                    }
                    state_start_time_ = now;
                }
                break;

            case RobotState::STOP:
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                break;
        }

        cmd_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    RobotState state_;
    rclcpp::Time state_start_time_;
    int loop_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
