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
    moveit_msgs::msg::CollisionObject &graspable_object,
    const std::string &frame_id,
    const std::string &id,
    geometry_msgs::msg::Pose &cylinder_pose,
    shape_msgs::msg::SolidPrimitive &cylinder)
{

    // TODO: Create a CollisionObject message, fill it and add it to the planning scene
    graspable_object.header.frame_id = frame_id;
    graspable_object.id = id;
    graspable_object.primitives.push_back(cylinder);
    graspable_object.primitive_poses.push_back(cylinder_pose);
    graspable_object.operation = graspable_object.ADD;
    RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Add an object into the world");
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(graspable_object);
}

void attachObject(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const moveit_msgs::msg::CollisionObject &graspable_object)
{
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "fr3_hand_tcp";
    attached_object.object = graspable_object;
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = {"fr3_link7", "fr3_hand_tcp", "fr3_link6", "fr3_rightfinger", "fr3_leftfinger"};
    {
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
        scene->processAttachedCollisionObjectMsg(attached_object);
    }
    moveit_cpp_ptr->getPlanningSceneMonitor()->triggerSceneUpdateEvent(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
}

// Helper: Detach object from gripper and re-add to scene
void detachAndPlaceObject(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    moveit_msgs::msg::CollisionObject &graspable_object,
    const geometry_msgs::msg::Pose &place_pose,
    const shape_msgs::msg::SolidPrimitive &cylinder)
{
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = "graspable_cylinder";
    detach_object.link_name = "fr3_hand_tcp";
    detach_object.object.operation = detach_object.object.REMOVE;
    {
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
        scene->processAttachedCollisionObjectMsg(detach_object);
    }
    graspable_object.primitive_poses[0] = place_pose;
    graspable_object.primitive_poses[0].position.z -= cylinder.dimensions[0] / 2.0 + 0.01;
    {
        planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
        scene->processCollisionObjectMsg(graspable_object);
    }
    moveit_cpp_ptr->getPlanningSceneMonitor()->triggerSceneUpdateEvent(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
}

// Helper: Plan and execute a motion to a pose
bool planAndExecute(
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components,
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    const geometry_msgs::msg::PoseStamped &goal_pose,
    const moveit::core::JointModelGroup *joint_model_group_ptr,
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const std::string &description)
{

    // TODO : Plan the motion
    planning_components->setStartStateToCurrentState();

    planning_components->setGoal(goal_pose, "fr3_hand_tcp");

    auto plan_solution1 = planning_components->plan();

    if (plan_solution1)
    {
        visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
        visual_tools.prompt("Press 'Next' in RViz to EXECUTE the trajectory");

        auto success = planning_components->execute();
        RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
        // visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

        if (success)
        {
            RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "\u2713 %s execution succeeded!", description.c_str());
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger(LOGNAME), "\u2717 %s execution failed!", description.c_str());
        }
        return success;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGNAME), "\u2717 %s planning failed!", description.c_str());
        return false;
    }
}

int main(int argc, char *argv[])
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
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

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
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = 0.35;
    cylinder_pose.position.y = 0;
    cylinder_pose.position.z = 0.075;

    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = cylinder.CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[cylinder.CYLINDER_HEIGHT] = 0.15;
    cylinder.dimensions[cylinder.CYLINDER_RADIUS] = 0.03;

    addObstaclesAndObject(moveit_cpp_ptr, graspable_object, "fr3_link0", "cyl1", cylinder_pose, cylinder);

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.15;
    primitive.dimensions[primitive.BOX_Y] = 0.15;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    geometry_msgs::msg::Pose box1;
    box1.orientation.w = 1.0;
    box1.position.x = 0.45;
    box1.position.y = -0.25;
    box1.position.z = 0.15;

    addObstaclesAndObject(moveit_cpp_ptr, graspable_object, "fr3_link0", "box1",  box1, primitive);
    box1.position.y = 0.25;
    addObstaclesAndObject(moveit_cpp_ptr, graspable_object, "fr3_link0", "box2", box1, primitive);

    // Step 1: Move to pre-grasp position
    visual_tools.prompt("Press 'Next' to move to PRE-GRASP position");
    RCLCPP_INFO(logger, "Planning motion to pre-grasp...");

    // TODO : Define pre-grasp target pose above the cylinder object and plan and execute motion
    // TODO : Define the target pose for the target pose
    geometry_msgs::msg::PoseStamped pregrasp_pose;
    pregrasp_pose.header.frame_id = "fr3_link0";
    pregrasp_pose.pose.orientation.x = 1.0;
    pregrasp_pose.pose.orientation.y = 0.0;
    pregrasp_pose.pose.orientation.z = 0.0;
    pregrasp_pose.pose.orientation.w = 0.0;
    pregrasp_pose.pose.position.x = 0.35;
    pregrasp_pose.pose.position.y = 0.0;
    pregrasp_pose.pose.position.z = 0.30;

    planAndExecute(planning_components, visual_tools, pregrasp_pose, joint_model_group_ptr, moveit_cpp_ptr, "Pre-grasp motion");

    // Step 2: Move to grasp position
    visual_tools.prompt("Press 'Next' to GRASP object");
    RCLCPP_INFO(logger, "Planning motion to grasp...");

    // TODO : Define grasp target pose at the cylinder object and plan and execute motion
    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = "fr3_link0";
    grasp_pose.pose.orientation.x = 1.0;
    grasp_pose.pose.orientation.y = 0.0;
    grasp_pose.pose.orientation.z = 0.0;
    grasp_pose.pose.orientation.w = 0.0;
    grasp_pose.pose.position.x = 0.35;
    grasp_pose.pose.position.y = 0.0;
    grasp_pose.pose.position.z = 0.220;
    if (planAndExecute(planning_components, visual_tools, grasp_pose, joint_model_group_ptr, moveit_cpp_ptr, "Grasp motion"))
    {
        attachObject(moveit_cpp_ptr, graspable_object);
        RCLCPP_INFO(logger, "✓ Object attached to gripper");
    }

    // Step 3: Retreat after grasp
    visual_tools.prompt("Press 'Next' to RETREAT with object");
    RCLCPP_INFO(logger, "Planning retreat motion...");

    // TODO : Define retreat target pose above the grasp pose and plan and execute motion
// TODO : Define grasp target pose at the cylinder object and plan and execute motion
    geometry_msgs::msg::PoseStamped retreat_pose;
    retreat_pose.header.frame_id = "fr3_link0";
    retreat_pose.pose.orientation.x = 1.0;
    retreat_pose.pose.orientation.y = 0.0;
    retreat_pose.pose.orientation.z = 0.0;
    retreat_pose.pose.orientation.w = 0.0;
    retreat_pose.pose.position.x = 0.35;
    retreat_pose.pose.position.y = -0.35;
    retreat_pose.pose.position.z = 0.05;
    
    planAndExecute(planning_components, visual_tools, retreat_pose, joint_model_group_ptr, moveit_cpp_ptr, "Retreat motion");

    // Step 4: Move to place position
    visual_tools.prompt("Press 'Next' to move to PLACE position");
    RCLCPP_INFO(logger, "Planning motion to place location...");

    // TODO : Define place target pose and plan and execute motion
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = "fr3_link0";
    place_pose.pose.orientation.x = 1.0;
    place_pose.pose.orientation.y = 0.0;
    place_pose.pose.orientation.z = 0.0;
    place_pose.pose.orientation.w = 0.0;
    place_pose.pose.position.x = 0.35;
    place_pose.pose.position.y = -0.45;
    place_pose.pose.position.z = 0.20;
 
    if (planAndExecute(planning_components, visual_tools, place_pose, joint_model_group_ptr, moveit_cpp_ptr, "Place motion"))
    {
        detachAndPlaceObject(moveit_cpp_ptr, graspable_object, place_pose.pose, cylinder);
        RCLCPP_INFO(logger, "✓ Object detached and placed");
    }

    RCLCPP_INFO(logger, "\n=== Exercise 8 completed! ===");

    visual_tools.prompt("Press 'Next' to exit");

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
