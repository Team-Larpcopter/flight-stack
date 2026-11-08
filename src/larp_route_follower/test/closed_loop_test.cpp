// Closed loop over a whole route: the body tracks the committed curve exactly,
// plan() runs every 100 ms and ensureSafeCommand() every 50 ms (the node's
// cadence). The follower must reach the goal with no blank command, no failed
// cycle, no braking, and no jump in the commanded position between commits.
#include <navigation/observed_map.hpp>
#include <larp_route_follower/follower_route_planner.hpp>
#include <iostream>
#include <stdexcept>

static void require(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  const bool course = argc > 2;
  double now = 10;
  auto map = std::make_shared<navigation::ObservedMap>(argv[1], [&] { return now; }, course);
  const Eigen::Vector3d course_lo(-4.1624,-5.5626,0.),course_hi(1.9336,.5334,1.8);
  if(course)map->setHardBoundary(course_lo,course_hi,.4);
  super_utils::RobotState body;
  body.p = {0, 0, 1};
  body.v.setZero();
  body.a.setZero();
  body.j.setZero();
  body.q.setIdentity();
  body.yaw = 0;
  body.rcv = true;
  body.rcv_time = now;
  map->setBody(body);
  // Depth-like sensing: rays from the body, stopped by the pillar (an AABB) as
  // a camera would be. Re-run as the body moves so the map grows with the flight.
  const Eigen::Vector3d pillar_lo(.55, .55, 0), pillar_hi(1.05, 1.05, 2);
  auto sense = [&](const Eigen::Vector3d& origin) {
    if(course)return;
    rog_map::PointCloud cloud;
    for (int i = 0; i < 8000; ++i) {
      const double z = 1. - 2. * (i + .5) / 8000., az = i * 2.399963229728653, rad = std::sqrt(1 - z * z);
      const Eigen::Vector3d dir(rad * std::cos(az), rad * std::sin(az), z);
      double hit = 4.;
      double t0 = 0, t1 = hit;  // slab intersection with the pillar box
      bool blocked = true;
      for (int k = 0; k < 3 && blocked; ++k) {
        if (std::abs(dir[k]) < 1e-12) {
          blocked = origin[k] > pillar_lo[k] && origin[k] < pillar_hi[k];
          continue;
        }
        double a = (pillar_lo[k] - origin[k]) / dir[k], b = (pillar_hi[k] - origin[k]) / dir[k];
        if (a > b) std::swap(a, b);
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
        blocked = t0 <= t1;
      }
      if (blocked) hit = std::max(t0, .01);
      const Eigen::Vector3d q = origin + hit * dir;
      rog_map::PointType p;
      p.x = q.x();
      p.y = q.y();
      p.z = q.z();
      p.intensity = 1;
      cloud.push_back(p);
    }
    for (int i = 0; i < 4; ++i) map->integrate(cloud, {origin, Eigen::Quaterniond::Identity()});
    map->setBody(body);
  };
  if(course)map->loadCadPrior(argv[2],true);
  sense(body.p);
  map->declareLaunchCylinder(.8, .1, 1.6);
  auto clock = std::make_shared<navigation::PlannerClock>([&] { return now; }, [](const std::string&) {});
  larp_route_follower::FollowerRoutePlanner planner(argv[1], map, clock, {}, course);
  const Eigen::Vector3d goal=course ? Eigen::Vector3d(-2.82128,-.77724,1.) : Eigen::Vector3d(1.6,1.6,1.);
  require(planner.setGoal(1, goal, 0), "goal rejected");
  if(!course)require(planner.accept({"map", 1, 0, 1, now, {{0, 0, 1}, {1.6, 0, 1}, goal}}, 0), "route rejected");

  if(course) {
    planner.setSearchBounds(course_lo,course_hi);
    planner.setFlightEnvelope(course_lo+Eigen::Vector3d::Constant(.4),course_hi-Eigen::Vector3d::Constant(.4));
    planner.setMapRevision(1);
    const auto warmup=std::chrono::steady_clock::now();
    while(!planner.prepareRoute(body.p))
      require(std::chrono::steady_clock::now()-warmup<std::chrono::seconds(30), "course preplanning timed out");
    std::cout << "course cached in " << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-warmup).count()
              << " ms, vertices " << planner.gridRoute()->route().points.size() << std::endl;
  }
  int plans = 0, successes = 0, failures = 0, blank = 0, unsafe = 0;
  double max_jump = 0, max_speed = 0, max_plan_ms = 0;
  const double start = now;
  bool reached = false;
  for (int step = 0; step < 20 * (course ? 1200 : 400) && !reached; ++step) {
    now = start + step * .05;
    const auto committed = planner.position();
    if (!committed.empty()) {
      const double at = std::min(now - committed.start_WT, committed.getTotalDuration());
      body.p = committed.getPos(at);
      body.v = committed.getVel(at);
      body.a = committed.getAcc(at);
      body.yaw = planner.yaw().getPos(std::min(now - planner.yaw().start_WT, planner.yaw().getTotalDuration())).x();
      max_speed = std::max(max_speed, body.v.norm());
    }
    body.rcv_time = now;
    map->setBody(body);
    if(course && step%2000==0)std::cout << "course t=" << now-start << " position=" << body.p.transpose() << " speed=" << body.v.norm() << std::endl;
    if (step % 20 == 0) sense(body.p);
    if (step % 2 == 0) {
      const auto before = planner.position();
      const auto wall = std::chrono::steady_clock::now();
      const auto ret = planner.plan();
      max_plan_ms = std::max(max_plan_ms,
                             std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count());
      ++plans;
      if (ret == super_utils::SUCCESS) {
        ++successes;
        // Commit continuity: the new curve must agree with the old one now.
        if (!before.empty()) {
          const auto after = planner.position();
          const double t_old = std::min(now - before.start_WT, before.getTotalDuration());
          max_jump = std::max(max_jump, (after.getPos(now - after.start_WT) - before.getPos(t_old)).norm());
        }
      } else if (ret == super_utils::FAILED) {
        ++failures;
        std::cerr << "t=" << now - start << " failed: " << planner.lastFailure() << '\n';
      }
    }
    if (!planner.ensureSafeCommand()) ++unsafe;
    if (planner.position().empty()) ++blank;
    reached = (body.p - goal).norm() < .02 && body.v.norm() < .01;
  }
  std::cout << std::flush;
  std::cout << "closed loop: " << (now - start) << " s, plans " << plans << ", commits " << successes << ", failures "
            << failures << ", blank " << blank << ", unsafe " << unsafe << ", braking " << planner.recoveryCount()
            << ", max commit jump " << max_jump * 1000 << " mm, max speed " << max_speed << " m/s, max plan "
            << max_plan_ms << " ms, final distance " << (body.p - goal).norm() << " m" << std::endl;
  require(reached, "did not reach the goal");
  require((course || failures == 0) && blank == 0 && unsafe == 0 && planner.recoveryCount() == 0, "planner was not consistent");
  require(max_jump < 1e-3, "commanded position jumped between commits");
  std::cout << "closed loop test passed\n";
  return 0;
}
