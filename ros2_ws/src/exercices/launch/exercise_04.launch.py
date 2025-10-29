#!/usr/bin/env python3

"""
Launch file for Exercise 4: Cartesian Line Motion with Franka FR3 Robot
This launch file sets up MoveIt with fake hardware for the Franka robot
and runs exercise 4 using the MoveItCpp API.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    
    # Declare arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='dont-care',
        description='Robot IP (not needed for fake hardware)'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware for simulation'
    )
    
    fake_sensor_commands_arg = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Fake sensor commands'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    
    # Load robot description
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=true',
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands, ' ros2_control:=true'])
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}
    
    # Load robot description semantic
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}
    
    # Load kinematics
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    
    # Load OMPL planning config
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)
    
    # Load controllers config
    moveit_controllers = load_yaml('franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    
    # Trajectory execution parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # MoveItCpp specific parameters
    moveit_cpp_yaml = {
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/attached_collision_object',
            'publish_planning_scene_topic': '/publish_planning_scene',
            'monitored_planning_scene_topic': '/monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        },
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'ompl': ompl_planning_pipeline_config['move_group'],
        'plan_request_params': {
            'planning_attempts': 10,
            'planning_pipeline': 'ompl',
            'planner_id': '',
            'max_velocity_scaling_factor': 1.0,
            'max_acceleration_scaling_factor': 1.0,
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': moveit_controllers,
    }

    # Get custom RViz config from exercices_manipulation package
    rviz_config_file = os.path.join(
        get_package_share_directory('exercices_manipulation'),
        'config',
        'moveit.rviz'
    )
    
    # Include the Franka MoveIt launch file
    franka_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_fr3_moveit_config'),
                'launch',
                'moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'use_rviz': 'false',
            'rviz_config': rviz_config_file,
        }.items()
    )

    # Launch a separate RViz node with the provided config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Exercise 4 node with all required MoveItCpp parameters
    exercise_04_node = Node(
        package='exercices_manipulation',
        executable='exercise_04_cartesian_line',
        name='exercise_04_cartesian_line',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            moveit_cpp_yaml,
            trajectory_execution,
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        franka_moveit_launch,
        exercise_04_node,
        rviz_node,
    ])
