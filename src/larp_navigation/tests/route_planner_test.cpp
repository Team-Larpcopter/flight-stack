#include <navigation/observed_map.hpp>
#include <navigation/route_planner.hpp>
#include <iostream>
#include <stdexcept>
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  double now=10;
  auto map=std::make_shared<navigation::ObservedMap>(argv[1],[&]{return now;});
  super_utils::RobotState body;body.p={0,0,1};body.v.setZero();body.a.setZero();body.j.setZero();
  body.q.setIdentity();body.yaw=0;body.rcv=true;body.rcv_time=now;map->setBody(body);
  // Synthetic omnidirectional rays are a unit fixture, never simulator inputs.
  // The map begins unknown and ROG must explicitly carve this test volume.
  rog_map::PointCloud shell;
  for(int i=0;i<20000;++i) {
    const double z=1.-2.*(i+.5)/20000.,az=i*2.399963229728653,rad=std::sqrt(1-z*z);
    rog_map::PointType p;p.x=4*rad*std::cos(az);p.y=4*rad*std::sin(az);p.z=1+4*z;p.intensity=1;shell.push_back(p);
  }
  for(int i=0;i<10;++i)map->integrate(shell,{body.p,body.q});
  // Synthetic rays cannot observe their own minimum-range blind volume.
  map->declareLaunchCylinder(.8,.1,1.6);
  require(map->isKnownFreeInflate(body.p),"test start volume was not observed free");
  auto clock=std::make_shared<navigation::PlannerClock>([&]{return now;},[](const auto& m){std::cerr<<m<<'\n';});
  navigation::RoutePlanner planner(argv[1],map,clock);
  require(planner.setGoal(1,{1,1,1},0),"goal rejected");
  require(planner.plan()==super_utils::FAILED,"planned without required global route route");
  navigation::Route route{"map",1,10,1,now,{{0,0,1},{1,0,1},{1,1,1}}};
  require(planner.accept(route,10),"route rejected");
  const auto ret=planner.plan();
  std::cout<<"SUPER return code "<<ret<<std::endl;
  require(ret==super_utils::SUCCESS,"SUPER failed to produce safe trajectory from route");
  require(!planner.position().empty() && planner.committedStillFree(),"committed curve not certified");
  bool corner=false;for(const auto& p:clock->latest_frontend)if((p-Eigen::Vector3d(1,0,1)).norm()<1e-6)corner=true;
  require(corner,"global route route corner did not reach SUPER geometric frontend");
  const auto traj=planner.position();
  require((traj.getPos(0)-body.p).norm()<1e-9,"planner moved measured initial state to a voxel center");
  require((traj.getPos(traj.getTotalDuration())-Eigen::Vector3d(1,1,1)).norm()<.1,"goal terminal position wrong");
  require(traj.getVel(traj.getTotalDuration()).norm()<.05,"trajectory does not stop");
  // A failed replacement must preserve the exact committed forward curve,
  // including its endpoint hold after the polynomial duration has elapsed.
  now=traj.start_WT+traj.getTotalDuration()+.1;
  body.p=traj.getPos(traj.getTotalDuration());body.v.setZero();body.rcv_time=now;map->setBody(body);
  planner.invalidateRoute();
  require(planner.plan()==super_utils::FAILED,"replacement unexpectedly succeeded without a route");
  require(planner.position().start_WT==traj.start_WT && planner.ensureSafeCommand(),
          "failed replacement discarded the certified terminal hold");
  require((planner.position().getPos(traj.getTotalDuration())-body.p).norm()<1e-9,
          "failed replacement changed the terminal hold position");
  now=10;body.p={0,0,1};body.rcv_time=now;map->setBody(body);
  // Regression: a goal just past an inflated wall must not replace the
  // externally supplied free-prefix endpoint merely because it is nearby.
  rog_map::PointCloud wall;
  for(int y=-15;y<=15;++y)for(int z=-10;z<=10;++z) {
    rog_map::PointType p;p.x=1.55;p.y=y*.1+.05;p.z=1+z*.1+.05;p.intensity=1;wall.push_back(p);
  }
  for(int i=0;i<10;++i)map->integrate(wall,{body.p,body.q});
  double boundary=.1;
  while(boundary<2 && map->isKnownFreeInflate(Eigen::Vector3d(boundary,0,1)))boundary+=.01;
  require(boundary<2,"wall fixture did not bound free space");
  require(!map->isLineFree(body.p,Eigen::Vector3d(boundary+.02,0,1),true,true),"line query skipped unsafe endpoint cell");
  navigation::RoutePlanner clipped(argv[1],map,clock);
  const Eigen::Vector3d blocked_goal(boundary+.02,0,1);
  require(clipped.setGoal(2,blocked_goal,0),"blocked goal not accepted for routing");
  navigation::Route blocked_route{"map",2,20,2,now,{{0,0,1},blocked_goal}};
  require(clipped.accept(blocked_route,20),"blocked-route envelope rejected");
  // Allow the production coordinator's bounded horizon reduction after a
  // rejected optimization; the final clearance assertions remain mandatory.
  bool clipped_committed=false;
  for(int attempt=0;attempt<8 && !clipped_committed;++attempt)
    clipped_committed=clipped.plan()==super_utils::SUCCESS;
  require(clipped_committed,"cannot stop on observed side of nearby blocked goal");
  const auto stopped=clipped.position();
  require(clipped.committedStillFree(),"blocked-goal trajectory is not certified");
  require(stopped.getPos(stopped.getTotalDuration()).x()<boundary-.03,"snapped safe prefix into nearby blocked goal");
  // Newly observed obstruction across the future committed route: certify a
  // stop now and never fall back to the old unsafe command after a failed replan.
  now+=.1;body.p=stopped.getPos(.1);body.v=stopped.getVel(.1);body.rcv_time=now;map->setBody(body);
  for(auto& p:wall)p.x=.75;
  for(int i=0;i<10;++i)map->integrate(wall,{body.p,body.q});
  require(!clipped.committedStillFree(),"new wall did not invalidate committed route");
  require(clipped.ensureSafeCommand() && clipped.braking(),"could not certify safe braking recovery");
  const auto brake=clipped.position();
  require(clipped.committedStillFree(),"recovery stop not certified");
  require(clipped.plan()==super_utils::NO_NEED,"replanned during braking motion");
  now=brake.start_WT+brake.getTotalDuration()+.3;
  body.p=brake.getPos(brake.getTotalDuration());body.v.setZero();body.rcv_time=now;map->setBody(body);
  clipped.invalidateRoute();
  require(clipped.plan()==super_utils::FAILED,"planned without a fresh post-stop route");
  require(clipped.braking() && clipped.position().start_WT==brake.start_WT,"failed replan resumed unsafe old trajectory");
  std::cout<<"global route polyline -> SUPER frontend -> certified stopping polynomial passed\n";
}
