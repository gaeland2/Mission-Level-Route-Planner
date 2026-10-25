cmake_minimum_required(VERSION 3.8)
project(mission_level_route_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


## ADDED -------------------------------------------------Build dependencies here IMPORTANTTTT->>>
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

#include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_libraries(${OCTOMAP_LIBRARIES})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#FOR MISSION LEVEL ROUITE PLANNER NODE
add_executable(mission_level_route_planner_node src/mission_level_route_planner_node.cpp)
ament_target_dependencies(mission_level_route_planner_node rclcpp)


# FOR OCTOMAP CLIENT
add_executable(octomap_client src/OctomapClient.cpp)
ament_target_dependencies(octomap_client rclcpp octomap_msgs octomap)
target_include_directories(octomap_client PRIVATE ${OCTOMAP_INCLUDE_DIRS} include)
target_link_libraries(octomap_client ${OCTOMAP_LIBRARIES})


target_include_directories(mission_level_route_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mission_level_route_planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

## --------------------------------------------------------


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


## LAUNCH FILES
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
  )
install(TARGETS mission_level_route_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS octomap_client
  DESTINATION lib/${PROJECT_NAME})
##0-


ament_package()
