cmake_minimum_required(VERSION 3.8)
project(chess_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # For Clang/clangd, avoid adding broad -Wall/-Wextra/-Wpedantic here
  # to reduce C++17/clang-specific warnings in editor diagnostics.
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Build the library and executables
add_library(chess_robot_core src/robot_controller.cpp)
add_executable(robot_controller src/main.cpp)
add_library(game_state_debug src/game_state.cpp)
add_library(engine_client src/engine_client.cpp)

ament_target_dependencies(chess_robot_core 
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
)

ament_target_dependencies(robot_controller
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)

target_link_libraries(robot_controller chess_robot_core)

target_include_directories(game_state_debug PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/chess_robot
  ${rclcpp_INCLUDE_DIRS}
)

target_include_directories(engine_client PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/chess_robot
  ${rclcpp_INCLUDE_DIRS}
)

# Fix for rclcpp namespace error
target_include_directories(chess_robot_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/chess_robot
  ${rclcpp_INCLUDE_DIRS}
)

target_include_directories(robot_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/chess_robot
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(robot_controller engine_client game_state_debug)

# Export and install headers so downstream code can include them reliably.
ament_export_include_directories(include)

# Install public headers.
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the executable
install(TARGETS
  chess_robot_core
  robot_controller
  game_state_debug
  engine_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts with execute permissions
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files if any
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)
# Install Python executables (imrec and imrec_node)
install(PROGRAMS
  scripts/imrec/imrec_node.py
  scripts/imrec/imrec.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
