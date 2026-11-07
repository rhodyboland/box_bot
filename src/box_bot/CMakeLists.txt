cmake_minimum_required(VERSION 3.5)
project(box_bot)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gnss_migration test/test_gnss_migration.py)
  ament_add_pytest_test(hub test/test_hub.py)
  ament_add_pytest_test(console_logging test/test_console_logging.py)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#config
install(
  DIRECTORY config launch behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/esp32_hub.py
    scripts/gnss_navsat.py
    scripts/gps_status_logger.py
    scripts/imu_yaw_offset.py
    scripts/imu_tilt_cloud_filter.py
    scripts/motion_chain_logger.py
    scripts/rtk2go_ntrip_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/gnss_helpers.py scripts/hub_protocol.py DESTINATION lib/${PROJECT_NAME})

ament_package()
