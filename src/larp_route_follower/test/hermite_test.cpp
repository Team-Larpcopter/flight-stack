#include <larp_route_follower/hermite.hpp>
#include <iostream>
#include <stdexcept>

using larp_route_follower::Vec3;
static void require(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}
static double peakSpeed(const geometry_utils::Trajectory& t) {
  double peak = 0;
  for (double s = 0; s <= t.getTotalDuration(); s += .01) peak = std::max(peak, t.getVel(s).norm());
  return peak;
}

int main() {
  // Boundary conditions of a single quintic are exact.
  const Vec3 p0(0, 0, 1), v0(.1, 0, 0), a0(0, .05, 0), p1(1, .5, 1.2), v1(0, .2, 0), a1(0, 0, 0);
  geometry_utils::Trajectory one;
  one.emplace_back(2., larp_route_follower::quinticHermite(p0, v0, a0, p1, v1, a1, 2.));
  require((one.getPos(0) - p0).norm() < 1e-9 && (one.getVel(0) - v0).norm() < 1e-9 && (one.getAcc(0) - a0).norm() < 1e-9,
          "quintic start conditions");
  require((one.getPos(2) - p1).norm() < 1e-9 && (one.getVel(2) - v1).norm() < 1e-9 && (one.getAcc(2) - a1).norm() < 1e-9,
          "quintic end conditions");

  // A cornered route: passes every waypoint, C2 at knots, ends at rest, nominal peak near cruise.
  larp_route_follower::Waypoints w{{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {2, 1, 1.5}};
  for (const bool corners : {false, true}) {
    larp_route_follower::FollowOptions o{.06, .04, 1., corners};
    const auto t = larp_route_follower::followWaypoints(w, Vec3::Zero(), Vec3::Zero(), 5., o);
    const auto knots=corners?w:larp_route_follower::roundedWaypoints(w);
    require(t.getPieceNum() == int(knots.size()-1) && t.start_WT == 5., "one piece per rounded segment");
    double at = 0;
    for (int i = 0; i < t.getPieceNum(); ++i) {
      require((t.getPos(at) - knots[i]).norm() < 1e-9, "trajectory misses a waypoint");
      at += t[i].getDuration();
      if (i + 1 < t.getPieceNum()) {
        const double d = t[i].getDuration();
        require((t[i].getPos(d) - t[i + 1].getPos(0)).norm() < 1e-9, "position discontinuity");
        require((t[i].getVel(d) - t[i + 1].getVel(0)).norm() < 1e-9, "velocity discontinuity");
        require((t[i].getAcc(d) - t[i + 1].getAcc(0)).norm() < 1e-9, "acceleration discontinuity");
      }
    }
    const double end = t.getTotalDuration();
    require((t.getPos(end) - w.back()).norm() < 1e-9, "does not end at the last waypoint");
    require(t.getVel(end).norm() < 1e-9 && t.getAcc(end).norm() < 1e-9, "does not end at rest");
    const double peak = peakSpeed(t);
    std::cout << (corners ? "stop-at-corners" : "smooth") << ": duration " << end << " s, peak " << peak << " m/s\n";
    require(peak < .06 * 1.25, "nominal peak speed far above cruise");
  }

  // A moving first knot keeps its velocity continuously.
  const auto moving = larp_route_follower::followWaypoints(w, Vec3(.05, 0, 0), Vec3::Zero(), 0., {});
  require((moving.getVel(0) - Vec3(.05, 0, 0)).norm() < 1e-9, "initial velocity not preserved");

  // Capsule-bounded construction: every curve stays within the reserved 3 cm.
  const auto bounded=larp_route_follower::followBoundedWaypoints(w,Vec3::Zero(),Vec3::Zero(),0.,{});
  require(bounded.size()==w.size()-1,"bounded construction failed");
  for(size_t i=0;i<bounded.size();++i) {
    const Vec3 edge=w[i+1]-w[i];
    for(int j=0;j<=100;++j) {
      const Vec3 p=bounded[i].getPos(bounded[i].getDuration()*j/100.);
      const double u=std::clamp((p-w[i]).dot(edge)/edge.squaredNorm(),0.,1.);
      require((p-w[i]-u*edge).norm()<=.03000001,"curve escaped reserved margin");
    }
    if(i+1<bounded.size()) {
      const double end=bounded[i].getDuration();
      require((bounded[i].getVel(end)-bounded[i+1].getVel(0)).norm()<1e-8,"bounded velocity jump");
      require((bounded[i].getAcc(end)-bounded[i+1].getAcc(0)).norm()<1e-8,"bounded acceleration jump");
    }
  }
  // Yaw piece: scalar in row 0, ends at rest at the target.
  geometry_utils::Trajectory yaw;
  const double ty = larp_route_follower::yawDuration(1.2, .28, .4);
  yaw.emplace_back(ty, larp_route_follower::yawPiece(.3, 0, 0, 1.5, ty));
  require(std::abs(yaw.getPos(ty).x() - 1.5) < 1e-9 && yaw.getVel(ty).norm() < 1e-9, "yaw end");
  require(std::abs(yaw.getPos(0).y()) < 1e-12 && std::abs(yaw.getPos(ty / 2).z()) < 1e-12, "yaw rows 1-2 must be zero");
  require(peakSpeed(yaw) <= .28 + 1e-6, "yaw rate exceeds its bound");
  require(std::abs(larp_route_follower::wrapAngle(3 * M_PI) - M_PI) < 1e-9, "wrap");
  std::cout << "hermite tests passed\n";
  return 0;
}
