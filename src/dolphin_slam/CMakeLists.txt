cmake_minimum_required(VERSION 3.8)
project(dolphin_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Descriptors.msg"
  "msg/PlaceCellActivity.msg"
  "msg/ExperienceEvent.msg"
  "msg/LocalViewMatch.msg"
  "msg/RobotOdometry.msg"
  "srv/ImageRequest.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/ResetSLAM.srv"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes
install(PROGRAMS
  nodes/image_processing_node.py
  nodes/local_view_node.py
  nodes/place_cell_node.py
  nodes/experience_map_node.py
  nodes/robot_state_node.py
  nodes/dataset_player_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/run_dolphin_slam.sh
  scripts/analyze_dataset.py
  scripts/visualize_results.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_image_processing test/test_image_processing.py)
  ament_add_pytest_test(test_place_cell_network test/test_place_cell_network.py)
  ament_add_pytest_test(test_experience_map test/test_experience_map.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
