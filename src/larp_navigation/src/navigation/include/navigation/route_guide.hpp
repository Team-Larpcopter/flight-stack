#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>

namespace navigation {
using Point = Eigen::Vector3d;
using Polyline = std::vector<Point, Eigen::aligned_allocator<Point>>;

struct Route {
  std::string frame;
  uint64_t goal_id{0}, map_revision{0}, route_id{0};
  double capture_time{0};
  Polyline points;
};

// This consumes Air-FAR geometry; it does not choose exploration targets.
// The validity predicate checks each connector/segment against the current map.
class RouteGuide {
 public:
  using SegmentValid = std::function<bool(const Point&, const Point&)>;
  explicit RouteGuide(double max_age = 1.0, double max_connector = 0.5,
                      double association_window = 2.0, double endpoint_margin = .15)
      : max_age_(max_age), max_connector_(max_connector), window_(association_window), endpoint_margin_(endpoint_margin) {
    if (!(max_age > 0 && max_connector > 0 && association_window > 0) ||
        !std::isfinite(max_age + max_connector + association_window + endpoint_margin) || endpoint_margin<=0)
      throw std::invalid_argument("invalid route limits");
  }

  bool accept(Route route, const std::string& frame, uint64_t goal_id,
              uint64_t current_map_revision, double now) {
    if(goal_id!=sequence_goal_) {sequence_goal_=goal_id;highest_route_=highest_revision_=0;}
    // A rejected update invalidates the previous route; never silently fly it.
    if (route.frame != frame || route.goal_id != goal_id || route.points.size() < 2 ||
        route.map_revision > current_map_revision || !std::isfinite(now) ||
        !std::isfinite(route.capture_time) || now < route.capture_time ||
        now - route.capture_time > max_age_) { clear(); return false; }
    for (const auto& p : route.points) if (!p.allFinite()) { clear(); return false; }
    if (!route.route_id || route.route_id <= highest_route_ || route.map_revision < highest_revision_) {
      clear(); return false;
    }
    Polyline points;
    for (const auto& p : route.points)
      if (points.empty() || (p - points.back()).norm() > 1e-6) points.push_back(p);
    if (points.size() < 2) { clear(); return false; }
    route.points = std::move(points);
    highest_route_=route.route_id;highest_revision_=route.map_revision;
    route_ = std::move(route);
    lengths_.assign(route_.points.size(), 0.0);
    for (size_t i = 1; i < lengths_.size(); ++i)
      lengths_[i] = lengths_[i-1] + (route_.points[i] - route_.points[i-1]).norm();
    progress_ = 0;
    valid_ = true;
    return true;
  }

  void clear() { valid_ = false; route_ = {}; lengths_.clear(); progress_ = 0; }

  Polyline nearby(const Point& start, double horizon, double now,
                  uint64_t goal_id, const SegmentValid& valid_segment) {
    Polyline result;
    if (!valid_ || goal_id != route_.goal_id || !start.allFinite() ||
        !std::isfinite(now) || now < route_.capture_time ||
        now - route_.capture_time > max_age_ || !std::isfinite(horizon) || horizon <= 0)
      return result;
    double best_distance = std::numeric_limits<double>::infinity(), best_s = progress_;
    Point projection = start;
    size_t segment = 0;
    for (size_t i = 0; i + 1 < route_.points.size(); ++i) {
      if (lengths_[i+1] < progress_ || lengths_[i] > progress_ + window_) continue;
      const Point d = route_.points[i+1] - route_.points[i];
      const double length = lengths_[i+1] - lengths_[i];
      const double low = std::max(0.0, (progress_-lengths_[i])/length);
      const double high = std::min(1.0, (progress_+window_-lengths_[i])/length);
      const double t = std::clamp((start-route_.points[i]).dot(d)/d.squaredNorm(), low, high);
      const Point candidate = route_.points[i] + t*d;
      const double distance = (candidate-start).norm();
      if (distance < best_distance - 1e-9) {
        best_distance = distance; best_s = lengths_[i]+t*length;
        projection = candidate; segment = i;
      }
    }
    if (best_distance > max_connector_ || !valid_segment(start, projection)) return {};
    result.push_back(start);
    double remaining = horizon;
    auto append = [&](const Point& target) {
      const Point d = target-result.back();
      const double length = d.norm();
      if (length < 1e-6) return true;
      const Point endpoint = result.back() + d*(std::min(length,remaining)/length);
      if (!valid_segment(result.back(), endpoint)) {
        // A long Air-FAR edge may leave observed space. Retain its certified
        // free prefix instead of requiring the next graph vertex to be visible.
        // Segment validity means the ENTIRE segment is valid, so this predicate
        // is monotone along a fixed ray; no obstacle can be jumped over.
        const Point origin=result.back(), delta=endpoint-origin;
        double lo=0,hi=1;
        for(int i=0;i<16;++i) {
          const double mid=(lo+hi)*.5;
          if(valid_segment(origin,origin+mid*delta)) lo=mid; else hi=mid;
        }
        // Avoid asking the optimizer to stop exactly on a discrete free/unknown
        // boundary; small polynomial roundoff must not cross that boundary.
        const double distance=lo*delta.norm()-endpoint_margin_;
        if(distance>.01) result.push_back(origin+distance*delta.normalized());
        return false;
      }
      result.push_back(endpoint); remaining -= std::min(length,remaining);
      return remaining > 1e-6;
    };
    if (!append(projection)) return result.size()>=2 ? result : Polyline{};
    for (size_t i = segment+1; i < route_.points.size() && remaining > 1e-6; ++i)
      if (!append(route_.points[i])) break;
    if (result.size() < 2) return {};
    progress_ = best_s;
    return result;
  }
 private:
  double max_age_, max_connector_, window_, endpoint_margin_, progress_{0};
  bool valid_{false};
  uint64_t sequence_goal_{0},highest_route_{0},highest_revision_{0};
  Route route_;
  std::vector<double> lengths_;
};
}
