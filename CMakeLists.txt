#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_gait_ctrl)
set(L3XZ_GAIT_CTRL_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(l3xz REQUIRED)
find_package(l3xz_kinematic_engine REQUIRED)
find_package(ros2_heartbeat REQUIRED)
find_package(ros2_loop_rate_monitor REQUIRED)
#######################################################################################
include_directories(
        ${L3XZ_GAIT_CTRL_TARGET} include
)
#######################################################################################
add_executable(
        ${L3XZ_GAIT_CTRL_TARGET}

        src/gait/state/Init_Coxa.cpp
        src/gait/state/Init_Femur.cpp
        src/gait/state/Init_Tibia.cpp
        src/gait/state/PositionTrajectory.cpp
        src/gait/state/Turning.cpp
        src/gait/state/Walking.cpp
        src/gait/GaitController.cpp
        src/gait/GaitControllerInput.cpp
        src/gait/GaitControllerOutput.cpp

        src/Node.cpp

        src/main.cpp
)
#######################################################################################
target_compile_features(${L3XZ_GAIT_CTRL_TARGET} PRIVATE cxx_std_17)
#######################################################################################
ament_target_dependencies(${L3XZ_GAIT_CTRL_TARGET} rclcpp std_msgs nav_msgs geometry_msgs l3xz l3xz_kinematic_engine ros2_heartbeat ros2_loop_rate_monitor)
#######################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#######################################################################################
install(
        TARGETS ${L3XZ_GAIT_CTRL_TARGET}
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
