#pragma once
// Closed-form trajectory construction for the route follower. No optimizer:
// every curve is a chain of quintic Hermite pieces, continuous in position,
// velocity and acceleration, that ends at rest. Certification is separate.
#include <data_structure/base/trajectory.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
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

// Same circular-fillet geometry as flight_trajectory/trajectory.py::_build.
// Fixed <=8 samples/corner and <=64 total knots; no duration-sized table or
// iterative optimizer. Quintic interpolation below makes acceleration continuous.
inline Waypoints roundedWaypoints(const Waypoints& w, double radius=.08) {
  if(w.size()<3)return w;
  Waypoints out{w.front()};
  for(size_t i=1;i+1<w.size();++i) {
    const Vec3 in=w[i]-w[i-1],next=w[i+1]-w[i];
    if(in.norm()<1e-6 || next.norm()<1e-6)return {};
    const Vec3 u=in.normalized(),v=next.normalized();
    const double dot=std::clamp(u.dot(v),-1.,1.),turn=std::acos(dot);
    if(turn<1e-4){out.push_back(w[i]);continue;}
    if(turn>3.1)return {}; // corner-stop candidate handles reversals
    const double trim=std::min({radius*std::tan(turn/2),.45*in.norm(),.45*next.norm()});
    const double r=trim/std::tan(turn/2);
    const Vec3 normal=(v-dot*u).normalized(),entry=w[i]-trim*u,centre=entry+r*normal;
    const int count=std::clamp(int(std::ceil(turn/(M_PI/8))),1,8);
    for(int k=0;k<=count;++k) {
      const double phi=turn*k/count;
      const Vec3 p=centre-r*std::cos(phi)*normal+r*std::sin(phi)*u;
      if((p-out.back()).norm()>1e-6)out.push_back(p);
    }
    if(out.size()>63)return {};
  }
  if((w.back()-out.back()).norm()>1e-6)out.push_back(w.back());
  return out;
}

// Position trajectory from (p, v0, a0) at waypoints.front() through every
// waypoint, ending at rest at waypoints.back(). Interior knot velocity follows
// the corner bisector, scaled by cos(turn angle): sharp corners slow to zero.
inline geometry_utils::Trajectory followWaypoints(const Waypoints& input, const Vec3& v0, const Vec3& a0,
                                                  double start, const FollowOptions& o) {
  geometry_utils::Trajectory out;
  out.start_WT = start;
  const Waypoints w=o.stop_at_corners?input:roundedWaypoints(input);
  if (w.size()>64)return out;
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
      const double turn=std::acos(std::clamp(u1.dot(u2),-1.,1.));
      const double curvature=turn/std::max(.001,.5*(in.norm()+next.norm()));
      const double speed=std::min({o.cruise, std::sqrt(.5*o.accel/std::max(curvature,1e-6)),
                                  .12/std::max(curvature,1e-6)});
      vel[k] = speed/o.time_scale * straightness * bisector.normalized();
    }
  }
  // Linear forward/backward speed passes, as in flight_trajectory._profile.
  // Keep the measured initial derivatives unchanged for a C2 handoff.
  for(size_t k=1;k+1<w.size();++k) {
    const double cap=std::sqrt(vel[k-1].squaredNorm()+o.accel*(w[k]-w[k-1]).norm());
    if(vel[k].norm()>cap)vel[k]*=cap/vel[k].norm();
  }
  for(size_t k=w.size()-1;k>1;--k) {
    const double cap=std::sqrt(vel[k].squaredNorm()+o.accel*(w[k]-w[k-1]).norm());
    if(vel[k-1].norm()>cap)vel[k-1]*=cap/vel[k-1].norm();
  }
  for (size_t k = 0; k + 1 < w.size(); ++k) {
    const double length = (w[k + 1] - w[k]).norm();
    // Peak speed of a quintic is 1.875x its mean when both knots are at rest
    // and approaches the mean when both carry cruise speed: keep the peak at cruise.
    const double carried = std::clamp(std::min(vel[k].norm(), vel[k + 1].norm()) / o.cruise, 0., 1.);
    const double peak_factor = 1.875 - .875 * carried;
    const double mean=.5*(vel[k].norm()+vel[k+1].norm());
    const double rest_time=o.time_scale * segmentDuration(peak_factor * length, o.cruise, o.accel);
    double t=std::max(.05,length/std::max(mean,length/rest_time));
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
