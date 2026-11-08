#pragma once
// Closed-form trajectory construction for the route follower. No optimizer:
// every curve is a chain of quintic Hermite pieces, continuous in position,
// velocity and acceleration, that ends at rest. Certification is separate.
#include <data_structure/base/trajectory.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace larp_route_follower {
using Vec3 = Eigen::Vector3d;
using Waypoints = std::vector<Vec3, Eigen::aligned_allocator<Vec3>>;

// Coefficients in SUPER's order: column 0 multiplies t^5, column 5 is constant.
inline Eigen::MatrixXd quinticHermite(const Vec3& p0, const Vec3& v0, const Vec3& a0,
                                      const Vec3& p1, const Vec3& v1, const Vec3& a1, double t) {
  const Vec3 d = p1 - p0 - t * v0 - .5 * t * t * a0, dv = v1 - v0 - t * a0, da = a1 - a0;
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3, 6);
  c.col(5) = p0;
  c.col(4) = v0;
  c.col(3) = .5 * a0;
  c.col(2) = 10 * d / std::pow(t, 3) - 4 * dv / (t * t) + .5 * da / t;
  c.col(1) = -15 * d / std::pow(t, 4) + 7 * dv / std::pow(t, 3) - da / (t * t);
  c.col(0) = 6 * d / std::pow(t, 5) - 3 * dv / std::pow(t, 4) + .5 * da / std::pow(t, 3);
  return c;
}

inline Eigen::MatrixXd holdPiece(const Vec3& p) {
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3, 6);
  c.col(5) = p;
  return c;
}

// Rest-to-rest quintic peaks: velocity 1.875 L/T, acceleration 5.774 L/T^2.
inline double segmentDuration(double length, double cruise, double accel) {
  return std::max({length / cruise, std::sqrt(5.774 * length / accel), .2});
}

struct FollowOptions {
  double cruise{.06};       // nominal speed along the route [m/s]
  double accel{.04};        // nominal acceleration used for timing [m/s^2]
  double time_scale{1.};    // >1 slows every segment uniformly
  bool stop_at_corners{false};  // true: straight segments only, no bowing
};

// Position trajectory from (p, v0, a0) at waypoints.front() through every
// waypoint, ending at rest at waypoints.back(). Interior knot velocity follows
// the corner bisector, scaled by cos(turn angle): sharp corners slow to zero.
inline geometry_utils::Trajectory followWaypoints(const Waypoints& w, const Vec3& v0, const Vec3& a0,
                                                  double start, const FollowOptions& o) {
  geometry_utils::Trajectory out;
  out.start_WT = start;
  if (w.size() < 2 || !(o.cruise > 0) || !(o.accel > 0) || !(o.time_scale > 0)) return out;
  std::vector<Vec3, Eigen::aligned_allocator<Vec3>> vel(w.size(), Vec3::Zero()), acc(w.size(), Vec3::Zero());
  vel.front() = v0;
  acc.front() = a0;
  if (!o.stop_at_corners) {
    for (size_t k = 1; k + 1 < w.size(); ++k) {
      const Vec3 in = w[k] - w[k - 1], next = w[k + 1] - w[k];
      if (in.norm() < 1e-6 || next.norm() < 1e-6) continue;
      const Vec3 u1 = in.normalized(), u2 = next.normalized(), bisector = u1 + u2;
      const double straightness = std::max(0., u1.dot(u2));
      if (bisector.norm() < 1e-6) continue;
      vel[k] = o.cruise * straightness * bisector.normalized();
    }
  }
  for (size_t k = 0; k + 1 < w.size(); ++k) {
    const double length = (w[k + 1] - w[k]).norm();
    // Peak speed of a quintic is 1.875x its mean when both knots are at rest
    // and approaches the mean when both carry cruise speed: keep the peak at cruise.
    const double carried = std::clamp(std::min(vel[k].norm(), vel[k + 1].norm()) / o.cruise, 0., 1.);
    const double peak_factor = 1.875 - .875 * carried;
    double t = o.time_scale * segmentDuration(peak_factor * length, o.cruise, o.accel);
    // A moving first knot must also have time to absorb its own velocity.
    if (k == 0) t = std::max(t, o.time_scale * 2. * v0.norm() / o.accel);
    out.emplace_back(t, quinticHermite(w[k], vel[k], acc[k], w[k + 1], vel[k + 1], acc[k + 1], t));
  }
  return out;
}

inline double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Scalar quintic in row 0 (rows 1-2 zero), the yaw convention of the bridge.
inline Eigen::MatrixXd yawPiece(double y0, double yv0, double ya0, double y1, double t) {
  Eigen::MatrixXd c = Eigen::MatrixXd::Zero(3, 6);
  const Vec3 p0(y0, 0, 0), v0(yv0, 0, 0), a0(ya0, 0, 0), p1(y1, 0, 0);
  c.row(0) = quinticHermite(p0, v0, a0, p1, Vec3::Zero(), Vec3::Zero(), t).row(0);
  return c;
}

inline double yawDuration(double delta, double rate, double accel) {
  const double d = std::abs(delta);
  return std::max({1.875 * d / rate, std::sqrt(5.774 * d / accel), .2});
}
}  // namespace larp_route_follower
