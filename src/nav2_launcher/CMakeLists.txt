cmake_minimum_required(VERSION 3.5)
project(nav2_launcher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(boxbot_wayline_planner SHARED src/wayline_planner.cpp)
target_compile_features(boxbot_wayline_planner PRIVATE cxx_std_17)
target_link_libraries(boxbot_wayline_planner nav2_navfn_planner::nav2_navfn_planner pluginlib::pluginlib)
pluginlib_export_plugin_description_file(nav2_core wayline_planner_plugin.xml)

add_library(boxbot_capture_wayline_bt_node SHARED src/capture_wayline.cpp)
target_compile_features(boxbot_capture_wayline_bt_node PRIVATE cxx_std_17)
target_compile_definitions(boxbot_capture_wayline_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(boxbot_capture_wayline_bt_node
  behaviortree_cpp::behaviortree_cpp nav2_util::nav2_util_core)
install(TARGETS boxbot_wayline_planner boxbot_capture_wayline_bt_node DESTINATION lib)
add_executable(boxbot_waypoint_follower src/waypoint_follower_main.cpp src/waypoint_result.cpp)
target_link_libraries(boxbot_waypoint_follower nav2_waypoint_follower::waypoint_follower_core)
target_link_options(boxbot_waypoint_follower PRIVATE "LINKER:--export-dynamic")
install(TARGETS boxbot_waypoint_follower DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(waypoint_result test/test_waypoint_result.cpp src/waypoint_result.cpp
    TIMEOUT 10 ENV ROS_DOMAIN_ID=193)
  target_link_libraries(waypoint_result nav2_waypoint_follower::waypoint_follower_core)
  target_link_options(waypoint_result PRIVATE "LINKER:--export-dynamic")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(waypoint_preemption test/test_waypoint_preemption.py TIMEOUT 45)
  ament_add_pytest_test(component_loading test/test_component_loading.py TIMEOUT 30)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
