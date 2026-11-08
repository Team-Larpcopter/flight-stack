#include <larp_ego_planner/snapshot_grid.hpp>
#include <limits>
#include <iostream>

using larp_ego_planner::SnapshotGrid;
using Point = Eigen::Vector3d;
void require(bool value) { if (!value) throw std::runtime_error("snapshot regression failed"); }
sensor_msgs::msg::PointCloud2 cloud(const std::vector<Point>& points) {
  sensor_msgs::msg::PointCloud2 msg; msg.header.frame_id = "map";
  sensor_msgs::PointCloud2Modifier mod(msg); mod.setPointCloud2FieldsByString(1, "xyz"); mod.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
  for (const auto& p : points) { *x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z; }
  return msg;
}
int main() {
  dronehacks_msgs::msg::MapSnapshot msg;
  msg.header.frame_id = "map"; msg.localization_epoch = 1; msg.revision = 1; msg.resolution = .1;
  msg.lower.x = -1.03; msg.lower.y = -1.; msg.lower.z = -1.;
  msg.upper.x = 1.; msg.upper.y = 1.; msg.upper.z = 1.;
  std::vector<Point> free;
  for (int x = -10; x < 10; ++x) for (int y = -10; y < 10; ++y) for (int z = -10; z < 10; ++z)
    free.push_back(Point(x + .5, y + .5, z + .5) * .1);
  msg.free = cloud(free); msg.occupied = cloud({});
  SnapshotGrid empty(msg, .1);
  require(!empty.occupied({-.25, .05, .05}));
  require(empty.occupied({1.1, 0, 0}));
  require(empty.occupied({.95, 0, 0}));
  require(empty.occupied({std::numeric_limits<double>::quiet_NaN(), 0, 0}));
  msg.occupied = cloud({{.05, .05, .05}});
  SnapshotGrid blocked(msg, .1);
  require(blocked.occupied({.05, .05, .05}));  // occupied overrides free
  require(blocked.occupied({.05, .05, .15}));  // vertical inflation
  require(blocked.occupied({-.05, .05, .05}));
  require(!blocked.occupied({-.25, .05, .05}));
  msg.free = cloud({});
  SnapshotGrid unknown(msg, .1);
  require(unknown.occupied({-.25, .05, .05}));
  auto rejects = [&](auto malformed) {
    bool rejected = false;
    try { SnapshotGrid bad(malformed, .1); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
  };
  auto bad = msg; bad.resolution = 0; rejects(bad);
  bad = msg; bad.header.frame_id = "global"; rejects(bad);
  bad = msg; bad.upper.x = 1000; rejects(bad);
  bad = msg; bad.free = cloud({{0, 0, 0}}); bad.free.fields[0].datatype = 8; rejects(bad);
  bad = msg; bad.free = cloud({{0, 0, 0}}); bad.free.data.pop_back(); rejects(bad);
  std::cout << "snapshot geometry, unknown space, inflation and malformed input checks passed\n";
}
