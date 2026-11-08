#pragma once
#include <Eigen/Core>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace larp_ego_planner {
// No ROS I/O or controller. The caller owns the map, timing and certification.
struct Settings {
  double velocity, acceleration, resolution;
  double control_point_distance{.1}, horizon{.6};
};
struct Spline { Eigen::MatrixXd controls; double interval; };
class Optimizer {
 public:
  Optimizer(Settings settings, std::function<int(Eigen::Vector3d)> occupied);
  ~Optimizer();
  std::optional<Spline> plan(const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
    const std::vector<Eigen::Vector3d>& guide);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
