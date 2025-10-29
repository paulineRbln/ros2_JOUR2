# MoveIt2 Manipulation Exercises

## Visualizing and Launching the 2R Robot (Exercises 1 & 2)

### Exercise 1: Visualize the 2R Robot in RViz


To visualize your 2R robot model in RViz:

```bash
source install/setup.bash
ros2 launch my_first_robot 01_2r_robot.launch.launch.py
```

This will display your 2R robot in RViz using its URDF. You can inspect the robot's links and joints, and verify the model is correct.

---


### Exercise 2: Launch the 2R Robot in MoveIt

To launch your 2R robot with MoveIt and be able to move it in RViz:

```bash
source install/setup.bash
ros2 launch my_first_robot_moveit_config demo.launch.py
```

This will:
- Start RViz with the MoveIt Motion Planning plugin
- Load your robot model
- Allow you to plan and execute motions using the MoveIt interface

You can interactively move the robot using the MoveIt "Planning" tab in RViz.

---

This package contains C++ scripts for MoveIt2 manipulation exercises using ROS2 Humble.

## Prerequisites

Make sure you have:
- ROS2 Humble installed
- MoveIt2 installed
- Your robot's MoveIt configuration package built and sourced

## Building the Exercises

1. Navigate to your workspace root:
```bash
cd /home/lucas/ros2_intro/3_manip
```

2. Build the exercises package:
```bash
colcon build --packages-select exercices_manipulation
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Running the Exercises

### Two Ways to Run Exercises

Each exercise should be run using launch files

## Resources

- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [MoveIt2 API Documentation](https://moveit.picknik.ai/humble/api/html/index.html)
- [ROS2 Control Documentation](https://control.ros.org/humble/)
