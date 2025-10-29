cmake_minimum_required(VERSION 3.8)
project(exercices_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Set dependencies for all executables
set(DEPENDENCIES
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  shape_msgs
  moveit_visual_tools
)


# Exercise 3: Joint space motion
# add_executable(exercise_03_joint_space_motion src/exercise_03_joint_space_motion.cpp)
# ament_target_dependencies(exercise_03_joint_space_motion ${DEPENDENCIES})

# # Exercise 4: Cartesian line
# add_executable(exercise_04_cartesian_line src/exercise_04_cartesian_line.cpp)
# ament_target_dependencies(exercise_04_cartesian_line ${DEPENDENCIES})

# # Exercise 5: Cartesian circle
# add_executable(exercise_05_cartesian_circle src/exercise_05_cartesian_circle.cpp)
# ament_target_dependencies(exercise_05_cartesian_circle ${DEPENDENCIES})

# # Exercise 6: Add obstacles
# add_executable(exercise_06_add_obstacles src/exercise_06_add_obstacles.cpp)
# ament_target_dependencies(exercise_06_add_obstacles ${DEPENDENCIES})

# # Exercise 7: Create graspable object
# add_executable(exercise_07_create_graspable_object src/exercise_07_create_graspable_object.cpp)
# ament_target_dependencies(exercise_07_create_graspable_object ${DEPENDENCIES})

# # Exercise 8: Pick and place with two obstacles
# add_executable(exercise_08_pick_and_place src/exercise_08_pick_and_place.cpp)
# ament_target_dependencies(exercise_08_pick_and_place ${DEPENDENCIES})

# # Exercise 9: Constrained motion planning
# add_executable(exercise_09_constrained_motion src/exercise_09_constrained_motion.cpp)
# ament_target_dependencies(exercise_09_constrained_motion ${DEPENDENCIES})

# Install executables
# install(TARGETS
#   exercise_03_joint_space_motion
#   exercise_04_cartesian_line
#   exercise_05_cartesian_circle
#   exercise_06_add_obstacles
#   exercise_07_create_graspable_object
#   exercise_08_pick_and_place
#   exercise_09_constrained_motion
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
