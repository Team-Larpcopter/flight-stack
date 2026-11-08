#pragma once
#include <Eigen/Core>
#include <dronehacks_msgs/msg/map_snapshot.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace larp_ego_planner {
// ROG snapshots use globally aligned voxel centers. Missing cells and all space
// outside the snapshot are blocked. Inflate occupied AND unknown cells in 3-D.
// This conservative grid supports shadow optimization, not flight certification.
class SnapshotGrid {
 public:
  using Point = Eigen::Vector3d;
  explicit SnapshotGrid(const dronehacks_msgs::msg::MapSnapshot& msg, double radius)
      : resolution_(msg.resolution), lower_(msg.lower.x, msg.lower.y, msg.lower.z),
        upper_(msg.upper.x, msg.upper.y, msg.upper.z), radius_(radius) {
    if (msg.header.frame_id != "map" || !msg.localization_epoch || !msg.revision ||
        !std::isfinite(resolution_) || resolution_ < .02 || resolution_ > .5 ||
        !std::isfinite(radius_) || radius_ <= 0 || radius_ > 1. ||
        !lower_.allFinite() || !upper_.allFinite() ||
        !(upper_.array() > lower_.array()).all() ||
        lower_.cwiseAbs().maxCoeff() > 10000 || upper_.cwiseAbs().maxCoeff() > 10000)
      throw std::invalid_argument("invalid snapshot geometry");
    origin_ = (lower_ / resolution_).array().floor().matrix() * resolution_;
    const Point dimensions = ((upper_ - origin_) / resolution_).array().ceil();
    if (dimensions.maxCoeff() > 1000 || dimensions.prod() > 4000000)
      throw std::invalid_argument("snapshot exceeds bounded grid size");
    size_ = dimensions.cast<int>();
    cells_.assign(size_.prod(), 1);
    readCloud(msg.free, 0);
    readCloud(msg.occupied, 1);  // Occupied wins over free.
    const int steps = static_cast<int>(std::ceil(radius_ / resolution_));
    // Separable box dilation, including the unknown exterior. Its clearance is
    // at least the configured sphere radius; narrow passages may be rejected.
    for (int axis = 0; axis < 3; ++axis) {
      auto expanded = cells_;
      for (int x = 0; x < size_.x(); ++x)
        for (int y = 0; y < size_.y(); ++y)
          for (int z = 0; z < size_.z(); ++z) {
            Eigen::Vector3i i(x, y, z);
            if (cells_[address(i)]) continue;
            for (int delta = -steps; delta <= steps; ++delta) {
              auto j = i; j[axis] += delta;
              if (!inside(j) || cells_[address(j)]) { expanded[address(i)] = 1; break; }
            }
          }
      cells_.swap(expanded);
    }
  }
  double resolution() const { return resolution_; }
  int occupied(const Point& p) const {
    if (!p.allFinite() || (p.array() < (lower_.array() + radius_)).any() ||
        (p.array() > (upper_.array() - radius_)).any()) return 1;
    const Eigen::Vector3i i = ((p - origin_) / resolution_).array().floor().cast<int>();
    return !inside(i) || cells_[address(i)];
  }
 private:
  bool inside(const Eigen::Vector3i& i) const {
    return (i.array() >= 0).all() && (i.array() < size_.array()).all();
  }
  size_t address(const Eigen::Vector3i& i) const {
    return (size_t(i.x()) * size_.y() + i.y()) * size_.z() + i.z();
  }
  void readCloud(const sensor_msgs::msg::PointCloud2& cloud, char value) {
    if (!cloud.width && cloud.data.empty()) return;
    if (cloud.header.frame_id != "map" || cloud.is_bigendian ||
        cloud.height != 1 || !cloud.point_step ||
        uint64_t(cloud.width) * cloud.point_step != cloud.row_step ||
        cloud.data.size() != cloud.row_step)
      throw std::invalid_argument("invalid snapshot cloud layout");
    for (const auto* name : {"x", "y", "z"}) {
      const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [name](const auto& f) { return f.name == name; });
      if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          field->count != 1 || uint64_t(field->offset) + 4 > cloud.point_step)
        throw std::invalid_argument("snapshot requires FLOAT32 xyz");
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const Point p(*x, *y, *z);
      if (!p.allFinite()) throw std::invalid_argument("nonfinite snapshot point");
      if ((p.array() < lower_.array()).any() || (p.array() > upper_.array()).any()) continue;
      const Eigen::Vector3i i = ((p - origin_) / resolution_).array().floor().cast<int>();
      if (inside(i)) cells_[address(i)] = value;
    }
  }
  double resolution_;
  Point lower_, upper_, origin_;
  double radius_;
  Eigen::Vector3i size_;
  std::vector<char> cells_;
};
}  // namespace larp_ego_planner
