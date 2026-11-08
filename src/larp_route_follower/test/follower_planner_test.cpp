// Follower backend on a real ObservedMap (same fixture style as
// larp_navigation/tests/route_planner_test.cpp): plan, certify, consistency,
// truncation at a depth-observed wall, and failure preserving the committed curve.
#include <navigation/observed_map.hpp>
#include <larp_route_follower/follower_route_planner.hpp>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

static void require(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  // "fine": the flight config (0.06 m/s, short look-ahead); corner/goal checks scale to it.
  const bool fine = argc > 2 && std::string(argv[2]) == "fine";
  double now = 10;
  auto map = std::make_shared<navigation::ObservedMap>(argv[1], [&] { return now; });
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
  // Synthetic omnidirectional rays carve a known-free sphere (unit fixture only).
  rog_map::PointCloud shell;
  for (int i = 0; i < 20000; ++i) {
    const double z = 1. - 2. * (i + .5) / 20000., az = i * 2.399963229728653, rad = std::sqrt(1 - z * z);
    rog_map::PointType p;
    p.x = 4 * rad * std::cos(az);
    p.y = 4 * rad * std::sin(az);
    p.z = 1 + 4 * z;
    p.intensity = 1;
    shell.push_back(p);
  }
  for (int i = 0; i < 10; ++i) map->integrate(shell, {body.p, body.q});
  map->declareLaunchCylinder(.8, .1, 1.6);
  require(map->isKnownFreeInflate(body.p), "test start volume was not observed free");
  auto clock = std::make_shared<navigation::PlannerClock>([&] { return now; },
                                                          [](const std::string& m) { std::cerr << m << '\n'; });
  larp_route_follower::FollowerSettings settings;
  settings.horizon = 3.;
  larp_route_follower::FollowerRoutePlanner planner(argv[1], map, clock, settings, false);

  // 1) A cornered route: the follower must reproduce it, certified, from the measured state.
  require(planner.setGoal(1, {1.5, 1.5, 1}, 0), "goal rejected");
  require(planner.accept({"map", 1, 0, 1, now, {{0, 0, 1}, {1.5, 0, 1}, {1.5, 1.5, 1}}}, 0), "route rejected");
  const auto t0 = std::chrono::steady_clock::now();
  const auto ret = planner.plan();
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "follower return " << ret << " in " << ms << " ms (" << planner.lastFailure() << ")\n";
  require(ret == super_utils::SUCCESS, "follower failed to plan along a free route");
  require(planner.ensureSafeCommand() && !planner.braking(), "committed curve not certified");
  const auto traj = planner.position(), yaw = planner.yaw();
  require((traj.getPos(0) - body.p).norm() < 1e-9, "trajectory does not start at the measured state");
  require(std::abs(traj.getTotalDuration() - yaw.getTotalDuration()) < 1e-3, "yaw/position duration mismatch");
  require(traj.getVel(traj.getTotalDuration()).norm() < 1e-6, "trajectory does not end at rest");
  const Eigen::Vector3d first_goal(1.5, 1.5, 1), first_end = traj.getPos(traj.getTotalDuration());
  std::cout << "first curve: " << traj.getTotalDuration() << " s, ends at " << first_end.transpose() << "\n";
  require((first_end - first_goal).norm() < (body.p - first_goal).norm() - .2, "plan made no progress to the goal");
  // Open space: with enough look-ahead the corner is shortcut straight to the goal.
  if (!fine) require((first_end - first_goal).norm() < 1e-6, "free-space plan did not reach the goal directly");

  // 2) Consistency: a live, certified curve is kept rather than re-derived each tick.
  now += .1;
  body.p = traj.getPos(.1);
  body.v = traj.getVel(.1);
  body.rcv_time = now;
  map->setBody(body);
  require(planner.plan() == super_utils::NO_NEED, "live curve was replaced inside the replan period");
  require(planner.position().start_WT == traj.start_WT, "committed curve changed without need");

  // 3) Stale odometry: planning fails and the committed curve is preserved untouched.
  now += .5;
  require(planner.plan() == super_utils::FAILED, "planned on stale odometry");
  require(planner.position().start_WT == traj.start_WT, "failed plan discarded the committed curve");

  // 4) A depth-observed wall across the route: the guide is truncated before it
  //    and the new curve stays certified short of the wall.
  now = 30;
  body.p = {0, 0, 1};
  body.v.setZero();
  body.rcv_time = now;
  map->setBody(body);
  planner.cancel();
  rog_map::PointCloud wall;
  for (int y = -15; y <= 15; ++y)
    for (int z = -10; z <= 10; ++z) {
      rog_map::PointType p;
      p.x = 1.05;
      p.y = y * .1 + .05;
      p.z = 1 + z * .1 + .05;
      p.intensity = 1;
      wall.push_back(p);
    }
  for (int i = 0; i < 10; ++i) map->integrate(wall, {body.p, body.q});
  require(planner.setGoal(2, {2.5, 0, 1}, 0), "second goal rejected");
  require(planner.accept({"map", 2, 0, 2, now, {{0, 0, 1}, {2.5, 0, 1}}}, 0), "second route rejected");
  const auto blocked = planner.plan();
  std::cout << "blocked-route return " << blocked << " (" << planner.lastFailure() << ")\n";
  require(blocked == super_utils::SUCCESS, "no certified prefix before the wall");
  const auto stop = planner.position();
  const auto end = stop.getPos(stop.getTotalDuration());
  std::cout << "stops at x=" << end.x() << " before the wall at x=1.05\n";
  require(end.x() < 1.05 - .3 && planner.ensureSafeCommand(), "curve crosses into the wall clearance");

  // 5) A pillar on the chord forces the follower to keep the route corner.
  now = 50;
  body.p = {0, 0, 1};
  body.v.setZero();
  body.rcv_time = now;
  map->setBody(body);
  planner.cancel();
  rog_map::PointCloud pillar;
  for (int x = -2; x <= 2; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = -10; z <= 10; ++z) {
        rog_map::PointType p;
        p.x = -.8 + x * .1 + .05;
        p.y = .8 + y * .1 + .05;
        p.z = 1 + z * .1 + .05;
        p.intensity = 1;
        pillar.push_back(p);
      }
  for (int i = 0; i < 10; ++i) map->integrate(pillar, {body.p, body.q});
  require(planner.setGoal(3, {-1.6, 1.6, 1}, 0), "pillar goal rejected");
  require(planner.accept({"map", 3, 0, 3, now, {{0, 0, 1}, {0, 1.6, 1}, {-1.6, 1.6, 1}}}, 0), "pillar route rejected");
  const auto around = planner.plan();
  std::cout << "pillar return " << around << " (" << planner.lastFailure() << ")\n";
  require(around == super_utils::SUCCESS, "no certified curve around the pillar");
  const auto bend = planner.position();
  bool corner = false;
  for (double s = 0; s <= bend.getTotalDuration(); s += .02)
    if ((bend.getPos(s) - Eigen::Vector3d(0, 1.6, 1)).norm() < .3) corner = true;
  require(planner.ensureSafeCommand(), "pillar curve not certified");
  // With a short look-ahead the curve ends before the corner but must follow the route leg.
  if (!fine) require(corner, "trajectory cut the corner through the pillar");
  else require(std::abs(bend.getPos(bend.getTotalDuration()).x()) < .05, "curve left the first route leg");

  // A withheld command must not advance a virtual curve while preplanning.
  planner.suspendExecution();
  require(planner.position().empty(), "suspension retained timed command");
  now += 3.; body.rcv_time=now; map->setBody(body);
  require(planner.prepareRoute(body.p), "suspension discarded cached geometry");
  require(planner.position().empty(), "preplanning created a timed command");
  require(planner.plan()==super_utils::SUCCESS, "resume failed");
  require((planner.position().getPos(0)-body.p).norm()<1e-9,
          "resume skipped forward along an unexecuted curve");

  // Stationary recovery is delayed, holds translation, and yields to a new route.
  body.p={0,0,1};body.v.setZero();body.a.setZero();body.yaw=0;body.rcv_time=now;map->setBody(body);
  larp_route_follower::FollowerSettings scan_settings;scan_settings.scan_when_blocked=true;
  larp_route_follower::FollowerRoutePlanner scanning(argv[1],map,clock,scan_settings,false);
  require(scanning.setGoal(1,{1.5,-1.5,1},0), "scan goal rejected");
  require(scanning.plan()==super_utils::FAILED && scanning.position().empty(), "scan started before blocked timeout");
  now+=3.1;body.rcv_time=now;map->setBody(body);
  require(scanning.plan()==super_utils::SUCCESS && scanning.ensureSafeCommand(), "stationary scan unavailable");
  const auto scan_position=scanning.position(),scan_yaw=scanning.yaw();
  require((scan_position.getPos(scan_position.getTotalDuration())-body.p).norm()<1e-9,
          "scan translated toward an unseen opening");
  now+=1.;body.yaw=scan_yaw.getPos(1.).x();body.rcv_time=now;map->setBody(body);
  require(scanning.accept({"map",1,0,1,now,{{0,0,1},{1.5,-1.5,1}}},0), "opening route rejected");
  require(scanning.plan()==super_utils::SUCCESS && scanning.ensureSafeCommand(), "scan did not resume route following");
  require((scanning.position().getPos(0)-body.p).norm()<1e-9, "scan resume jumped position");

  // A newly observed frontal wall must repair the cached master route, not
  // repeatedly return a short prefix ending at that same wall.
  body.yaw=0;body.rcv_time=now;map->setBody(body);
  auto repair_map=std::make_shared<navigation::ObservedMap>(argv[1],[&]{return now;});
  repair_map->setBody(body);
  for(int frame=0;frame<10;++frame)repair_map->integrate(shell,{body.p,body.q});
  larp_route_follower::FollowerRoutePlanner repair(argv[1],repair_map,clock);
  require(repair.setGoal(1,{2.5,0,1},0), "repair goal rejected");repair.setMapRevision(1);
  auto prepare=[&] {
    const auto began=std::chrono::steady_clock::now();
    while(!repair.prepareRoute(body.p))
      require(std::chrono::steady_clock::now()-began<std::chrono::seconds(5), "cached route repair timed out");
  };
  prepare();const auto searches=repair.gridRoute()->searches();
  rog_map::PointCloud frontal_wall;
  for(double y=-.5;y<=.5;y+=.05)for(double z=.2;z<=1.8;z+=.05) {
    rog_map::PointType point;point.x=1.;point.y=y;point.z=z;point.intensity=1;frontal_wall.push_back(point);
  }
  for(int frame=0;frame<3;++frame)repair_map->integrate(frontal_wall,{body.p,body.q});
  require(repair_map->isOccupied(navigation::Point(1.,0.,1.)), "three observed wall frames did not update map");
  repair.setMapRevision(2);prepare();
  require(repair.gridRoute()->searches()>searches, "blocked cached prefix did not trigger route repair");
  bool detour=false;
  for(const auto& point:repair.gridRoute()->route().points)detour|=std::abs(point.y())>.5;
  require(detour, "repaired path still crosses the new frontal wall");

  // 6) Preloaded route file round trip.
  const char* file = "/tmp/larp_route_follower_test_route.txt";
  if (FILE* f = std::fopen(file, "w")) {
    std::fputs("# x y z\n0 0 1\n1 0 1  # corner\n1 1 1\n", f);
    std::fclose(f);
  }
  const auto loaded = larp_route_follower::FollowerRoutePlanner::loadRoute(file);
  require(loaded.size() == 3 && (loaded[1] - Eigen::Vector3d(1, 0, 1)).norm() < 1e-12, "route file parse");
  std::remove(file);
  std::cout << "follower planner tests passed\n";
  return 0;
}
