cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include Boost headers globally
include_directories(${Boost_INCLUDE_DIRS})

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  vacuum_object
  friction_compensation
  jointposition_impedance_control
  white_light
  damping
  impedance_force
  data_collect
  follow_path
  teleop_master
  teleop_slave
  teleop_slave_joint
)

# Add ROS 2 examples with rclcpp and sensor_msgs dependencies
set(ROS2_EXAMPLES
  teleop_pub
  teleop_pub_habtic
  teleop_sub
  teleop_sub_habtic
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

# Loop to add ROS 2 examples
foreach(ros2_example ${ROS2_EXAMPLES})
  add_executable(${ros2_example} ${ros2_example}.cpp)
  target_link_libraries(${ros2_example} Franka::Franka examples_common Eigen3::Eigen3)
  ament_target_dependencies(${ros2_example} rclcpp sensor_msgs)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

target_link_libraries(teleop_master ${Boost_LIBRARIES})
target_link_libraries(teleop_slave ${Boost_LIBRARIES})

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES} ${ROS2_EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(PROGRAMS
  /home/jihai/Jihai/franka_ws/src/libfranka/examples/realsense.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)