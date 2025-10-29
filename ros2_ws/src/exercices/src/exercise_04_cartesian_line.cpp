/**
 * Exercise 4: Plan a Cartesian motion in a straight line
 * Move the end-effector along a linear path using Franka Panda robot
 * Based on: https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html
 */

#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const std::string PLANNING_GROUP = "fr3_arm";
static const std::string LOGNAME = "exercise_04_cartesian_line";
static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");

// Helper: Plan and execute a motion to a target pose
bool planAndExecute(
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    const moveit::core::JointModelGroup* joint_model_group_ptr,
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const std::string& description)
{
    // TODO : Plan the motion
    if(plan_solution) {
        visual_tools.publishTrajectoryLine(plan_solution.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
        visual_tools.prompt("Press 'Next' in RViz to EXECUTE the trajectory");
        // TODO : Execute the planned motion
        if (success) {
            RCLCPP_INFO(logger,  "\u2713 %s execution succeeded!", description.c_str());
        } else {
            RCLCPP_WARN(logger, "\u2717 %s execution failed!", description.c_str());
        }
        return success;
    } else {
        RCLCPP_ERROR(logger, "\u2717 %s planning failed!", description.c_str());
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
    RCLCPP_INFO(logger, "Starting Exercise 4: Cartesian Line Motion with Franka FR3...");

    // Initialize MoveItCpp
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

    // Create PlanningComponent for the Franka arm
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
        PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto robot_start_state = planning_components->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    
    // Initialize MoveIt Visual Tools
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "fr3_link0", "moveit_visual_tools", 
                                                        moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Visual tools initialized. Use RViz buttons to proceed.");
    
    // Wait for user to press Next to start planning
    RCLCPP_INFO(logger, "\n=== Ready to plan motion ===");
    visual_tools.prompt("Press 'Next' in RViz to PLAN the trajectory");
    
    // Plan 1: Move to a position
    RCLCPP_INFO(logger, "\n=== Planning motion to position ===");

    // TODO : Define the target pose for the target pose
    planAndExecute(planning_components, visual_tools, target_pose, joint_model_group_ptr, moveit_cpp_ptr, "Move to start pose");
    
    // Keep node alive to see the results
    visual_tools.prompt("Press 'Next' to exit");
    
    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    
    return 0;
}
