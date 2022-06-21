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
#######################################################################################
add_executable(${L3XZ_CTRL_TARGET}
  src/main.cpp
)
#######################################################################################
ament_target_dependencies(${L3XZ_CTRL_TARGET} rclcpp)
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
  DESTINATION lib/${PROJECT_NAME})
#######################################################################################
ament_package()
#######################################################################################
