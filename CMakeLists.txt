#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_ctrl)
set(L3XZ_CTRL_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(orocos_kdl REQUIRED)
#######################################################################################
include_directories(${L3XZ_CTRL_TARGET}
  include
  ${orocos_kdl_INCLUDE_DIRS}
)
#######################################################################################
add_executable(${L3XZ_CTRL_TARGET}
  src/gait/state/Standing.cpp
  src/gait/state/StandUp.cpp
  src/gait/state/Turning.cpp
  src/gait/state/Walking.cpp
  src/gait/GaitController.cpp
  src/gait/GaitControllerInput.cpp
  src/gait/GaitControllerOutput.cpp

  src/kinematic/Engine.cpp
  src/kinematic/FK_Output.cpp
  src/kinematic/IK_Output.cpp

  src/ControlNode.cpp

  src/main.cpp
)
#######################################################################################
target_compile_features(${L3XZ_CTRL_TARGET} PRIVATE cxx_std_17)
#######################################################################################
target_link_libraries(${L3XZ_CTRL_TARGET} orocos-kdl)
#######################################################################################
ament_target_dependencies(${L3XZ_CTRL_TARGET} rclcpp geometry_msgs)
#######################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegAngle.msg"
)
#######################################################################################
rosidl_target_interfaces(${L3XZ_CTRL_TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")
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
install(TARGETS
  ${L3XZ_CTRL_TARGET}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################