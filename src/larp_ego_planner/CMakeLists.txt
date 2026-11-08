cmake_minimum_required(VERSION 3.22)
project(larp_ego_planner LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(dronehacks_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ShadowTrajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs)
set(EGO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ego-planner-swarm/src/planner)
add_library(ego_core STATIC
  ${EGO_ROOT}/plan_env/src/grid_map.cpp
  ${EGO_ROOT}/plan_env/src/raycast.cpp
  ${EGO_ROOT}/plan_env/src/obj_predictor.cpp
  ${EGO_ROOT}/path_searching/src/dyn_a_star.cpp
  ${EGO_ROOT}/bspline_opt/src/uniform_bspline.cpp
  ${EGO_ROOT}/bspline_opt/src/bspline_optimizer.cpp
  ${EGO_ROOT}/bspline_opt/src/gradient_descent_optimizer.cpp
  ${EGO_ROOT}/traj_utils/src/polynomial_traj.cpp
  ${EGO_ROOT}/traj_utils/src/planning_visualization.cpp
  ${EGO_ROOT}/plan_manage/src/planner_manager.cpp)
set_target_properties(ego_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
foreach(component plan_env path_searching bspline_opt traj_utils plan_manage)
  target_include_directories(ego_core PUBLIC ${EGO_ROOT}/${component}/include)
endforeach()
target_include_directories(ego_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ego_core Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(ego_core rclcpp nav_msgs std_msgs sensor_msgs geometry_msgs
  visualization_msgs cv_bridge message_filters pcl_conversions)
add_library(ego_optimizer SHARED src/optimizer.cpp)
target_include_directories(ego_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ego_optimizer PRIVATE ego_core PUBLIC Eigen3::Eigen)
install(TARGETS ego_optimizer EXPORT ego_optimizer_targets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(ego_optimizer_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
add_executable(ego_shadow_node src/ego_shadow_node.cpp)
target_include_directories(ego_shadow_node PRIVATE include)
target_link_libraries(ego_shadow_node ego_core)
ament_target_dependencies(ego_shadow_node dronehacks_msgs)
rosidl_get_typesupport_target(typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(ego_shadow_node ${typesupport})
install(TARGETS ego_shadow_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES third_party/ego-planner-swarm/LICENSE third_party/COPYING.LESSER third_party/NOTICES.md
  third_party/upstream-lock.json DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(snapshot_grid_test test/snapshot_grid_test.cpp)
  target_include_directories(snapshot_grid_test PRIVATE include)
  target_link_libraries(snapshot_grid_test Eigen3::Eigen)
  ament_target_dependencies(snapshot_grid_test dronehacks_msgs sensor_msgs)
  add_test(NAME snapshot_grid COMMAND snapshot_grid_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME ego_shadow_pipeline COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/shadow_pipeline.py
    --node $<TARGET_FILE:ego_shadow_node>
    --config ${CMAKE_CURRENT_SOURCE_DIR}/config/ego.yaml)
  set_tests_properties(ego_shadow_pipeline PROPERTIES TIMEOUT 90
    ENVIRONMENT "ROS_DOMAIN_ID=188;ROS_LOCALHOST_ONLY=1")
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
