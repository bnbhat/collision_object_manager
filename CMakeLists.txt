cmake_minimum_required(VERSION 3.8)
project(collision_object_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(arc_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

add_executable(${PROJECT_NAME}_main src/main.cpp
                              src/collision_object_manager.cpp
                              src/human_skeleton.cpp
                              src/ros_timer.cpp
                              src/collision_object.cpp
                              src/moveit_pose_interface.cpp)

target_include_directories(${PROJECT_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${Eigen3_INCLUDE_DIRS}")
  
target_compile_features(${PROJECT_NAME}_main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ${PROJECT_NAME}_main
  "rclcpp"
  "moveit_ros_planning_interface"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "moveit_visual_tools"
  "arc_interfaces"
  "Eigen3"
)

add_executable(mock_gripper_node src/mock_gripper.cpp)

target_include_directories(mock_gripper_node  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_compile_features(mock_gripper_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mock_gripper_node
  "rclcpp"
  "moveit_ros_planning_interface"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "moveit_visual_tools"
  "arc_interfaces"
)

add_executable(collision_object_test test/collision_object_test.cpp
                              src/collision_object_manager.cpp
                              src/human_skeleton.cpp
                              src/ros_timer.cpp
                              src/collision_object.cpp)

target_include_directories(collision_object_test  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(collision_object_test  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collision_object_test 
  "rclcpp"
  "moveit_ros_planning_interface"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "moveit_visual_tools"
  "arc_interfaces"
)

install(TARGETS ${PROJECT_NAME}_main collision_object_test mock_gripper_node
  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Add this after your main executable
if(BUILD_TESTING)
  # Include Google Test for ROS 2
  find_package(ament_cmake_gtest REQUIRED)

  # Add your test file
  ament_add_gtest(${PROJECT_NAME}_test test/test_human_skeleton.cpp
                  src/human_skeleton.cpp)  # Include the source file you want to test

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    "${Eigen3_INCLUDE_DIRS}")

  target_compile_features(${PROJECT_NAME}_test PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(
    ${PROJECT_NAME}_test
    "rclcpp"
    "moveit_ros_planning_interface"
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "moveit_visual_tools"
    "arc_interfaces"
    "Eigen3"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(timer_test test/timer_test.cpp src/ros_timer.cpp)
  target_include_directories(timer_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(timer_test
    "rclcpp"
  )
  target_link_libraries(timer_test gtest gtest_main)
endif()

ament_package()
