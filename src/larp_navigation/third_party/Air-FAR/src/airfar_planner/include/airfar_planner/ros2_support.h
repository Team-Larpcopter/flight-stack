#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <type_traits>

namespace airfar_ros {
inline rclcpp::Node::SharedPtr node;
template<class T, class D>
void param(const rclcpp::Node::SharedPtr& nh, std::string name, T& value, const D& fallback) {
  while (!name.empty() && name.front() == '/') name.erase(name.begin());
  const std::string prefix = "airfar_planner/";
  if (name.rfind(prefix, 0) == 0) name.erase(0, prefix.size());
  std::replace(name.begin(), name.end(), '/', '.');
  using RosType = std::conditional_t<std::is_floating_point_v<T>, double,
    std::conditional_t<std::is_integral_v<T> && !std::is_same_v<T,bool>, int64_t, T>>;
  if (!nh->has_parameter(name)) nh->declare_parameter<RosType>(name, static_cast<RosType>(fallback));
  value = static_cast<T>(nh->get_parameter(name).get_value<RosType>());
}
}
#define ROS_INFO(...) RCLCPP_INFO(airfar_ros::node->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(airfar_ros::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(airfar_ros::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(airfar_ros::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_COND(c, ...) do { if(c) ROS_INFO(__VA_ARGS__); } while(0)
#define ROS_WARN_COND(c, ...) do { if(c) ROS_WARN(__VA_ARGS__); } while(0)
#define ROS_ERROR_COND(c, ...) do { if(c) ROS_ERROR(__VA_ARGS__); } while(0)
#define ROS_WARN_ONCE(...) RCLCPP_WARN_ONCE(airfar_ros::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_THROTTLE(s, ...) RCLCPP_WARN_THROTTLE(airfar_ros::node->get_logger(), *airfar_ros::node->get_clock(), static_cast<int64_t>((s)*1000), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(s, ...) RCLCPP_ERROR_THROTTLE(airfar_ros::node->get_logger(), *airfar_ros::node->get_clock(), static_cast<int64_t>((s)*1000), __VA_ARGS__)
