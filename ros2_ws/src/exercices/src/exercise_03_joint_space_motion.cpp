

/**
 * Exercise 3: Plan a motion in joint space
 * Move the end-effector between a predefined joint states using Franka Panda robot
 * Based on: https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html
 */


#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "fr3_arm";
static const std::string LOGNAME = "exercise_03_joint_space_motion";
static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");

// Helper: Plan and execute a motion to a joint target
bool planAndExecute(
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components,
    moveit_visual_tools::MoveItVisualTools& visual_tools,
    const std::vector<double>& joint_group_positions,
    const moveit::core::JointModelGroup* joint_model_group_ptr,
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const std::string& description)
{
    // TODO: Plan the motion
    
    if (plan_solution1)
    {
        visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
        visual_tools.prompt("Press 'Next' in RViz to EXECUTE the motion");
        RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Executing motion...");

        // TODO : Execute the planned motion 
        bool blocking = true;
        moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr->execute(plan_solution1.trajectory, blocking, CONTROLLERS); 
        
        if (success)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "\u2713 %s execution succeeded!", description.c_str());
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(LOGNAME), "%s execution completed with warnings or failed", description.c_str());
        }
        return success;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "\u2717 %s planning failed!", description.c_str());
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
    RCLCPP_INFO(logger, "Starting Exercise 3: Joint Space Motion with Franka FR3...");
    
    // Initialize MoveItCpp
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
    moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
    
    // Initialize MoveIt Visual Tools
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "fr3_link0", "moveit_visual_tools", 
                                                            moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();
    RCLCPP_INFO(logger, "Visual tools initialized. Use RViz buttons to proceed.");

    // Create PlanningComponent for the Franka arm
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
        PLANNING_GROUP, moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    
    // TODO : Define the target joint positions 
    visual_tools.prompt("Press 'Next' in RViz to PLAN the trajectory");
    planAndExecute(planning_components, visual_tools, joint_group_positions_1, joint_model_group_ptr, moveit_cpp_ptr, "Joint position");
    RCLCPP_INFO(logger, "\n=== Exercise 3 completed! ===");

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
