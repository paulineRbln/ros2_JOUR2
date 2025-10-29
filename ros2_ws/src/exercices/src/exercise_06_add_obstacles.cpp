/**
 * Exercise 6: Add two obstacles in the planning scene and avoid them using Franka Panda robot
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
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

static const std::string PLANNING_GROUP = "fr3_arm";
static const std::string LOGNAME = "exercise_06_add_obstacles";
static const std::vector<std::string> CONTROLLERS(1, "fr3_arm_controller");

// Helper: Add a box obstacle to the planning scene
void addBoxObstacle(
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr,
    const std::string &frame_id,
    const std::string &id,
    const shape_msgs::msg::SolidPrimitive &box,
    const geometry_msgs::msg::Pose &box_pose)
{
    // TODO: Create a CollisionObject message, fill it and add it to the planning scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = id;
    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    RCLCPP_INFO(rclcpp::get_logger(LOGNAME), "Add an object into the world");
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
  scene->processCollisionObjectMsg(collision_object);
}

// Helper: Plan and execute a motion to a joint target
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

    planning_components->setGoal(goal_pose, "fr3_link8");

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
    RCLCPP_INFO(logger, "Starting Exercise 6: Add Obstacles with Franka FR3...");

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

    // Prompt user before adding obstacles
    visual_tools.prompt("Press 'Next' in RViz to ADD obstacles to the scene");
    RCLCPP_INFO(logger, "Adding two box obstacles to the scene...");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Obstacle 1
    // TODO: Define box1 shape and pose, then add it to the planning scene with addBoxObstacle function
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
   
    addBoxObstacle(moveit_cpp_ptr, "fr3_link0", "box1", primitive, box1);
    // TODO : Define box2 shape and pose, then add it to the planning scene with addBoxObstacle function
    // Obstacle 2
    box1.position.y = 0.25;
    addBoxObstacle(moveit_cpp_ptr, "fr3_link0", "box2", primitive, box1);

    RCLCPP_INFO(logger, "Two collision objects applied to move_group planning scene");
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Prompt user to plan
    visual_tools.prompt("Press 'Next' in RViz to PLAN motion avoiding obstacles");
    RCLCPP_INFO(logger, "Planning motion to avoid obstacles...");

    // TODO : Define the target pose for the target pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "fr3_link0";
    target_pose.pose.orientation.x = 1.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
    target_pose.pose.position.x = 0.35;
    target_pose.pose.position.y = -0.45;
    target_pose.pose.position.z = 0.05;

    planAndExecute(planning_components, visual_tools, target_pose, joint_model_group_ptr, moveit_cpp_ptr, "Plan avoiding obstacles");

    RCLCPP_INFO(logger, "\n=== Exercise 6 completed! ===");
    // Keep node alive to see results
    visual_tools.prompt("Press 'Next' to exit");

    // Shutdown
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
