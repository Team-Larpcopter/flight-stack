#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <optional>

namespace navigation {
struct TimedPose {
  int64_t stamp_ns;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

class PoseBuffer {
 public:
  explicit PoseBuffer(int64_t history_ns = 2'000'000'000,
                      int64_t max_gap_ns = 50'000'000)
      : history_(history_ns), max_gap_(max_gap_ns) {
    if (history_ns <= 0 || max_gap_ns <= 0 || history_ns < max_gap_ns)
      throw std::invalid_argument("invalid pose buffer limits");
  }
  void clear() { poses_.clear(); }
  bool push(TimedPose pose) {
    if (pose.stamp_ns < 0 || !pose.position.allFinite() ||
        !pose.orientation.coeffs().allFinite() ||
        std::abs(pose.orientation.norm()-1) > 1e-3 ||
        (!poses_.empty() && pose.stamp_ns <= poses_.back().stamp_ns)) return false;
    pose.orientation.normalize();
    poses_.push_back(pose);
    while (poses_.size() > 2 && pose.stamp_ns-poses_[1].stamp_ns > history_) poses_.pop_front();
    return true;
  }
  std::optional<TimedPose> at(int64_t stamp_ns) const {
    if (poses_.empty() || stamp_ns < poses_.front().stamp_ns || stamp_ns > poses_.back().stamp_ns)
      return {};
    const auto upper=std::lower_bound(poses_.begin(),poses_.end(),stamp_ns,
      [](const TimedPose& p,int64_t stamp) { return p.stamp_ns < stamp; });
    if (upper->stamp_ns == stamp_ns) return *upper;
    const auto lower=std::prev(upper);
    const int64_t interval=upper->stamp_ns-lower->stamp_ns;
    if (interval > max_gap_) return {};
    const double f=double(stamp_ns-lower->stamp_ns)/double(interval);
    return TimedPose{stamp_ns,lower->position+f*(upper->position-lower->position),
      lower->orientation.slerp(f,upper->orientation)};
  }
 private:
  int64_t history_, max_gap_;
  std::deque<TimedPose> poses_;
};
}
