cmake_minimum_required(VERSION 3.8)
project(terrain_mapping)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build and link the bundled InEKF directly so package relocation does not
# depend on stale paths embedded in pre-generated inekfConfig.cmake.
set(INEKF_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/invariant-ekf")
if(NOT EXISTS "${INEKF_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Missing third-party dependency: ${INEKF_SOURCE_DIR}")
endif()

add_subdirectory(${INEKF_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/inekf EXCLUDE_FROM_ALL)
include_directories(${INEKF_SOURCE_DIR}/include)

add_executable(tf_mapping_relay src/tf_mapping_relay.cpp)
ament_target_dependencies(tf_mapping_relay
  rclcpp
  tf2
  tf2_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

target_link_libraries(tf_mapping_relay
  inekf
)

install(TARGETS tf_mapping_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
