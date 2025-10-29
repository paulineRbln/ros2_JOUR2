/**
 * Exercise 5: Plan a Cartesian motion to follow a circle
 * Move the end-effector along a circular path using Franka Panda robot
 * Based on: https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html
 */

#include <memory>
#include <thread>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <functional>

static const std::string PLANNING_GROUP = "fr3_arm";
static const std::string LOGNAME = "exercise_05_cartesian_circle";
static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");


// Helper: Visualize waypoints as spheres
void visualizeWaypoints(const EigenSTL::vector_Isometry3d& waypoints, moveit_visual_tools::MoveItVisualTools& visual_tools) {
    for (const auto& waypoint_pose : waypoints) {
        visual_tools.publishSphere(waypoint_pose, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    }
    visual_tools.trigger();
}

// Helper: Plan and execute a Cartesian path through waypoints
bool planAndExecuteCartesian(
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const EigenSTL::vector_Isometry3d& eigen_waypoints,
    const moveit::core::JointModelGroup* joint_model_group_ptr,
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const std::string& description,
    moveit::core::RobotStatePtr current_state,
    moveit::core::RobotModelConstPtr robot_model_ptr){

    // TODO : Compute Cartesian path

    visual_tools.prompt("Press 'Next' in RViz to EXECUTE the circular motion");
    if (trajectory_states.size() > 0) {
        // Convert trajectory states to RobotTrajectory for execution
        RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Executing circular motion with %zu states...", trajectory_states.size());

        // TODO : Create RobotTrajectory

        // TODO : Add waypoints to RobotTrajectory

        // TODO : Execute the trajectory
        bool success = moveit_cpp_ptr->execute(PLANNING_GROUP, robot_trajectory, true);
        if (success) {
            RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "\u2713 %s Execution Succeeded!", description.c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "\u2717 %s Execution Failed!", description.c_str());
        }
        visual_tools.trigger();
        return success;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "\u2717 No trajectory to execute!");
        return false;
    }
}

int main(int argc, char * argv[])

{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto const logger = rclcpp::get_logger(LOGNAME);
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);
    
    // We spin up a SingleThreadedExecutor for MoveItCpp
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Wait for ROS to be ready
    RCLCPP_INFO(logger, "Starting Exercise 5: Cartesian Circle Motion with Franka FR3...");

    // Initialize MoveItCpp
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

    // Create PlanningComponent for the Franka arm
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
        PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    
    // Initialize MoveIt Visual Tools
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "fr3_link0", "moveit_visual_tools", 
                                                        moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();    
    RCLCPP_INFO(logger, "Visual tools initialized. Use RViz buttons to proceed.");
    
    // Get current state and pose as center of the circle
    // TODO : Get current end-effector pose 

    // TODO : Extract position and orientation for circle center

    // TODO : Define circle parameters

    // TODO : Generate waypoints along the circle
   
    // Visualize the circle path
    RCLCPP_INFO(logger, "Visualizing circular path with %d waypoints...", num_points);
    visualizeWaypoints(eigen_waypoints, visual_tools);
    // Wait for user to start planning
    visual_tools.prompt("Press 'Next' in RViz to start PLANNING circular motion");
    // Execute circular motion by moving through waypoints
    RCLCPP_INFO(logger, "Planning circular motion through waypoints...");

    planAndExecuteCartesian( visual_tools, eigen_waypoints, joint_model_group_ptr, moveit_cpp_ptr, "Circular Path", current_state, robot_model_ptr);
    RCLCPP_INFO(logger, "\n=== Exercise 5 completed! ===");
    // Keep node alive to see results
    visual_tools.prompt("Press 'Next' to exit");

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
