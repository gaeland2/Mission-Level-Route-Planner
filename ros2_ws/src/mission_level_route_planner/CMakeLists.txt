cmake_minimum_required(VERSION 3.8)
project(mission_level_route_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

# Add rclcpp, geometry_msgs, nav_msgs, octomap_msgs, octomap, mavros_msgs here once the ROS layer exists.

# Core planner library: no ROS dependencies, so it is easy to unit test.
add_library(route_planner_core
  src/grid.cpp
  src/dijkstra.cpp
  src/context.cpp)
target_include_directories(route_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(planner_demo src/planner_demo.cpp)
target_link_libraries(planner_demo route_planner_core)

find_package(rclcpp REQUIRED)

add_executable(mission_route_planner_node src/mission_level_route_planner_node.cpp)
target_link_libraries(mission_route_planner_node route_planner_core)
ament_target_dependencies(mission_route_planner_node
  rclcpp octomap_msgs octomap nav_msgs)
target_link_libraries(mission_route_planner_node ${OCTOMAP_LIBRARIES})

install(TARGETS mission_route_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS route_planner_core
  EXPORT      export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS planner_demo DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dijkstra test/test_dijkstra.cpp)
  target_link_libraries(test_dijkstra route_planner_core)

  ament_add_gtest(test_altitude_limits test/test_altitude_limits.cpp)
  target_link_libraries(test_altitude_limits route_planner_core)

  ament_add_gtest(test_safety_margin test/test_safety_margin.cpp)
  target_link_libraries(test_safety_margin route_planner_core)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
