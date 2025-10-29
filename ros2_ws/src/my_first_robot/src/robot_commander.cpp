#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class RobotCommander : public rclcpp::Node
{
public:
    RobotCommander()
    : Node("robot_commander")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&RobotCommander::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = {"joint0", "joint1"};
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = {1.0, 0.5};
        point.time_from_start = rclcpp::Duration(2s);
        message.points.push_back(point);
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCommander>());
    rclcpp::shutdown();
    return 0;
}