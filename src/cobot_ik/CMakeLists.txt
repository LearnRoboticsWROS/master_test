cmake_minimum_required(VERSION 3.8)
project(cobot_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)


# find_package(gazebo_ros_link_attacher REQUIRED)
find_package(linkattacher_msgs REQUIRED)  
find_package(ros2_linkattacher REQUIRED)  
find_package(gazebo_ros REQUIRED)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(test_ik src/test_ik.cpp)
ament_target_dependencies(test_ik rclcpp moveit_ros_planning_interface)

add_executable(pick_place src/pick_place.cpp)
ament_target_dependencies(pick_place rclcpp moveit_ros_planning_interface)

add_executable(pick_place_fake_gripping src/pick_place_fake_gripping.cpp)
ament_target_dependencies(pick_place_fake_gripping rclcpp moveit_ros_planning_interface gazebo_msgs)

add_executable(pick_place_link_attacher src/pick_place_link_attacher.cpp)
ament_target_dependencies(pick_place_link_attacher rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher)

add_executable(attach src/attach.cpp)
ament_target_dependencies(attach rclcpp moveit_ros_planning_interface gazebo_msgs linkattacher_msgs ros2_linkattacher)

install(TARGETS 
  test_ik
  pick_place
  pick_place_fake_gripping
  pick_place_link_attacher
  attach
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
