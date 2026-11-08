cmake_minimum_required(VERSION 3.8)
project(larp_route_follower)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(dronehacks_msgs REQUIRED)
find_path(PCL_CONVERSIONS_INCLUDE pcl_conversions/pcl_conversions.h
  PATHS /opt/ros/$ENV{ROS_DISTRO}/include/pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(PCL_COMMON REQUIRED IMPORTED_TARGET pcl_common pcl_common-1.12)
pkg_search_module(PCL_IO REQUIRED IMPORTED_TARGET pcl_io pcl_io-1.12)
pkg_search_module(PCL_FILTERS REQUIRED IMPORTED_TARGET pcl_filters pcl_filters-1.12)

# Reuse larp_navigation's map, route search, validator and stop code from its
# source tree, read-only. Nothing in larp_navigation is modified or rebuilt.
set(LARP_NAVIGATION_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../larp_navigation
  CACHE PATH "larp_navigation source tree providing ROG map and navigation headers")
set(SUPER_ROOT ${LARP_NAVIGATION_SOURCE_DIR}/third_party/SUPER)
if(NOT EXISTS ${SUPER_ROOT}/rog_map/include/rog_map/rog_map.h)
  message(FATAL_ERROR "larp_navigation sources not found at ${LARP_NAVIGATION_SOURCE_DIR}")
endif()

file(GLOB_RECURSE FOLLOWER_ROG_SRC CONFIGURE_DEPENDS
  ${SUPER_ROOT}/rog_map/src/rog_map/*.cpp
  ${SUPER_ROOT}/rog_map/include/*.cpp)
add_library(follower_rog_map STATIC ${FOLLOWER_ROG_SRC})
target_include_directories(follower_rog_map PUBLIC ${SUPER_ROOT}/rog_map/include ${PCL_CONVERSIONS_INCLUDE})
target_compile_definitions(follower_rog_map PUBLIC USE_ROS2 ORIGIN_AT_CORNER FMT_HEADER_ONLY
  ROOT_DIR="${CMAKE_CURRENT_BINARY_DIR}/")
target_link_libraries(follower_rog_map Eigen3::Eigen PkgConfig::PCL_COMMON PkgConfig::PCL_IO PkgConfig::PCL_FILTERS yaml-cpp)
ament_target_dependencies(follower_rog_map rclcpp sensor_msgs visualization_msgs pcl_msgs message_filters)

# Only SUPER's A* route search and polynomial containers: no corridor
# generation, MINCO or trajectory optimizers are compiled.
add_library(follower_route_core STATIC
  ${SUPER_ROOT}/super_planner/src/super_core/astar.cpp
  ${SUPER_ROOT}/super_planner/src/utils/trajectory.cpp
  ${SUPER_ROOT}/super_planner/src/utils/piece.cpp
  ${SUPER_ROOT}/super_planner/src/utils/root_finder.cpp)
target_compile_definitions(follower_route_core PUBLIC BACKWARD_HAS_BACKTRACE_SYMBOL=1)
target_include_directories(follower_route_core PUBLIC
  ${SUPER_ROOT}/super_planner/include
  ${LARP_NAVIGATION_SOURCE_DIR}/src/navigation/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(follower_route_core follower_rog_map)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/log)

add_executable(route_follower_node src/route_follower_node.cpp)
target_link_libraries(route_follower_node follower_route_core)
ament_target_dependencies(route_follower_node rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs dronehacks_msgs)

install(TARGETS route_follower_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Host-side tmux launchers (run from the deployment folder or with DEPLOY_DIR set).
set(FOLLOWER_SCRIPTS scripts/launch_follower_system_tmux.sh scripts/launch_follower_full_system_tmux.sh
  scripts/launch_follower_mission_tmux.sh)
install(PROGRAMS ${FOLLOWER_SCRIPTS} DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS ${FOLLOWER_SCRIPTS} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(hermite_test test/hermite_test.cpp)
  target_link_libraries(hermite_test follower_route_core)
  add_test(NAME hermite COMMAND hermite_test)
  add_executable(follower_planner_test test/follower_planner_test.cpp)
  target_link_libraries(follower_planner_test follower_route_core)
  add_test(NAME follower_planner COMMAND follower_planner_test
    ${LARP_NAVIGATION_SOURCE_DIR}/config/navigation.yaml)
  add_test(NAME follower_planner_flight_config COMMAND follower_planner_test
    ${LARP_NAVIGATION_SOURCE_DIR}/config/navigation_fine.yaml fine)
  add_executable(closed_loop_test test/closed_loop_test.cpp)
  target_link_libraries(closed_loop_test follower_route_core)
  add_test(NAME closed_loop COMMAND closed_loop_test ${LARP_NAVIGATION_SOURCE_DIR}/config/navigation.yaml)
  add_test(NAME closed_loop_flight_config COMMAND closed_loop_test ${LARP_NAVIGATION_SOURCE_DIR}/config/navigation_fine.yaml)
  set_tests_properties(closed_loop closed_loop_flight_config PROPERTIES TIMEOUT 300)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME launch_wrapper COMMAND ${Python3_EXECUTABLE} -m pytest -q
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_wrapper.py)
  add_test(NAME launch_scripts COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/check_scripts.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts ${CMAKE_CURRENT_SOURCE_DIR}/../../jetson/offboard/launch_system_tmux.sh)
endif()

ament_package()
