/**
 * Exercise 8: Pick and place with two obstacles
 * Move to pre-grasp → grasp → retreat → place → retreat
 * Attach object to gripper during grasp using apply_planning_scene
 * Based on: https://moveit.picknik.ai/main/doc/examples/moveit_cpp/moveitcpp_tutorial.html
 */


#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
 #include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const std::string PLANNING_GROUP = "fr3_arm";
static const std::string LOGNAME = "exercise_08_pick_and_place";
static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");

// Helper: Add obstacles and graspable object to the scene
void addObstaclesAndObject(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    moveit_msgs::msg::CollisionObject& graspable_object,
    geometry_msgs::msg::Pose& cylinder_pose,
    shape_msgs::msg::SolidPrimitive& cylinder)
{
    // TODO : Create a first  CollisionObject message, fill it and add it to the planning scene
    
    // TODO : Create a second CollisionObject message, fill it and add it to the planning scene
    
    // TODO : Create a graspable CollisionObject message, fill it and add it to the planning scene
    
}

// Helper: Attach object to gripper
void attachObject(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const moveit_msgs::msg::CollisionObject& graspable_object)
{
   // TODO : Create an AttachedCollisionObject message, fill it and apply it to the planning scene
}

// Helper: Detach object from gripper and re-add to scene
void detachAndPlaceObject(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    moveit_msgs::msg::CollisionObject& graspable_object,
    const geometry_msgs::msg::Pose& place_pose,
    const shape_msgs::msg::SolidPrimitive& cylinder)
{
   // TODO : Create an AttachedCollisionObject message to detach the object, fill it and apply it to the planning scene
}

// Helper: Plan and execute a motion to a pose
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
            RCLCPP_INFO(RCLCPP_GET_LOGGER(LOGNAME),  "\u2713 %s execution succeeded!", description.c_str());
        } else {
            RCLCPP_WARN(RCLCPP_GET_LOGGER(LOGNAME), "\u2717 %s execution failed!", description.c_str());
        }
        return success;
    } else {
        RCLCPP_ERROR(RCLCPP_GET_LOGGER(LOGNAME), "\u2717 %s planning failed!", description.c_str());
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
    RCLCPP_INFO(logger, "Starting Exercise 8: Pick and Place with Franka FR3...");

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

    // Add obstacles and object
    moveit_msgs::msg::CollisionObject graspable_object;
    geometry_msgs::msg::Pose cylinder_pose;
    shape_msgs::msg::SolidPrimitive cylinder;
    visual_tools.prompt("Press 'Next' in RViz to ADD obstacles and graspable object");
    RCLCPP_INFO(logger, "Adding obstacles to the scene...");

    addObstaclesAndObject(moveit_cpp_ptr, graspable_object, cylinder_pose, cylinder);


    // Step 1: Move to pre-grasp position
    visual_tools.prompt("Press 'Next' to move to PRE-GRASP position");
    RCLCPP_INFO(logger, "Planning motion to pre-grasp...");

    // TODO : Define pre-grasp target pose above the cylinder object and plan and execute motion
    
    planAndExecute(planning_components, visual_tools, pregrasp_pose, joint_model_group_ptr, moveit_cpp_ptr, "Pre-grasp motion");


    // Step 2: Move to grasp position
    visual_tools.prompt("Press 'Next' to GRASP object");
    RCLCPP_INFO(logger, "Planning motion to grasp...");

    // TODO : Define grasp target pose at the cylinder object and plan and execute motion

    if (planAndExecute(planning_components, visual_tools, grasp_pose, joint_model_group_ptr, moveit_cpp_ptr, "Grasp motion")) {
        attachObject(moveit_cpp_ptr, graspable_object);
        RCLCPP_INFO(logger, "✓ Object attached to gripper");
    }


    // Step 3: Retreat after grasp
    visual_tools.prompt("Press 'Next' to RETREAT with object");
    RCLCPP_INFO(logger, "Planning retreat motion...");
    
    // TODO : Define retreat target pose above the grasp pose and plan and execute motion

    planAndExecute(planning_components, visual_tools, retreat_pose, joint_model_group_ptr, moveit_cpp_ptr, "Retreat motion");


    // Step 4: Move to place position
    visual_tools.prompt("Press 'Next' to move to PLACE position");
    RCLCPP_INFO(logger, "Planning motion to place location...");
    
    // TODO : Define place target pose and plan and execute motion
    if (planAndExecute(planning_components, visual_tools, place_pose, joint_model_group_ptr, moveit_cpp_ptr, "Place motion")) {
        detachAndPlaceObject(moveit_cpp_ptr, graspable_object, place_pose.pose, cylinder);
        RCLCPP_INFO(logger, "✓ Object detached and placed");
    }

    RCLCPP_INFO(logger, "\n=== Exercise 8 completed! ===");
    
    visual_tools.prompt("Press 'Next' to exit");

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
