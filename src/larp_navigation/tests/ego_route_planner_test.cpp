#include <navigation/observed_map.hpp>
#include <navigation/ego_route_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>

void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  rclcpp::init(argc,argv);
  double now=10.;
  auto map=std::make_shared<navigation::ObservedMap>(argv[1],[&]{return now;});
  super_utils::RobotState body;body.p={0,0,1};body.v.setZero();body.a.setZero();body.j.setZero();
  body.q.setIdentity();body.yaw=0;body.rcv=true;body.rcv_time=now;map->setBody(body);
  auto clock=std::make_shared<navigation::PlannerClock>([&]{return now;},[](const auto& m){std::cerr<<m<<'\n';});
  navigation::EgoRoutePlanner planner(argv[1],map,clock);
  planner.setMapRevision(1);
  require(planner.setGoal(1,{1,0,1},0),"goal rejected");
  require(planner.plan()==super_utils::FAILED && !planner.ensureSafeCommand(),"unknown map authorized motion");
  rog_map::PointCloud shell;
  for(int i=0;i<20000;++i) {
    const double z=1.-2.*(i+.5)/20000.,az=i*2.399963229728653,rad=std::sqrt(1-z*z);
    rog_map::PointType p;p.x=4*rad*std::cos(az);p.y=4*rad*std::sin(az);p.z=1+4*z;p.intensity=1;shell.push_back(p);
  }
  for(int i=0;i<10;++i)map->integrate(shell,{body.p,body.q});
  map->declareLaunchCylinder(.8,.1,1.6);
  // Reset the route search backoff after replacing unknown with observed free.
  planner.setGoal(2,{1,0,1},0);planner.setMapRevision(2);
  bool committed=false;
  for(int i=0;i<20 && !committed;++i)committed=planner.plan()==super_utils::SUCCESS;
  require(committed && planner.ensureSafeCommand(),"EGO did not commit certified motion");
  auto first=planner.position();
  require(first.size()>1 && first[0].getDegree()==3,"test did not exercise EGO cubic optimization");
  require((first.getPos(0)-body.p).norm()<1e-9,"initial position changed");
  require(first.getVel(0).norm()<1e-9 && first.getAcc(0).norm()<1e-9,"initial derivatives changed");
  require(first.getVel(first.getTotalDuration()).norm()<1e-9 && first.getAcc(first.getTotalDuration()).norm()<1e-9,"missing exact stopping endpoint");
  now+=.1;body.p=first.getPos(.1);body.v=first.getVel(.1);body.a=first.getAcc(.1);body.rcv_time=now;map->setBody(body);
  require(planner.plan()==super_utils::SUCCESS,"moving replan failed");
  const auto replanned=planner.position();
  for(double t:{0.,.05,.099}) {
    require((replanned.getPos(t)-first.getPos(.1+t)).norm()<1e-7,"handoff prefix position changed");
    require((replanned.getVel(t)-first.getVel(.1+t)).norm()<1e-7,"handoff prefix velocity changed");
    require((replanned.getAcc(t)-first.getAcc(.1+t)).norm()<1e-7,"handoff prefix acceleration changed");
  }
  // Perfect tracking fixture drives successive local segments to the mission
  // endpoint without restarting its terminal curve and hiding mission completion.
  bool arrived=false;
  for(int i=0;i<500 && !arrived;++i) {
    const auto previous=planner.position();now+=.1;
    const double at=std::min(now-previous.start_WT,previous.getTotalDuration());
    body.p=previous.getPos(at);body.v=previous.getVel(at);body.a=previous.getAcc(at);body.rcv_time=now;map->setBody(body);
    planner.plan();require(planner.ensureSafeCommand(),"following EGO lost its certificate");
    const auto current=planner.position();
    arrived=now>=current.start_WT+current.getTotalDuration() && (body.p-Eigen::Vector3d(1,0,1)).norm()<.01;
  }
  require(arrived,"local replanning never completed the final goal");
  const auto final_start=planner.position().start_WT;
  require(planner.plan()==super_utils::NO_NEED && planner.position().start_WT==final_start,"final command was restarted");
  // Match main's persistent endpoint hold when a replacement cannot be routed.
  require(planner.setGoal(3,{50,0,1},0),"unreachable replacement goal rejected");
  require(planner.plan()==super_utils::FAILED && planner.ensureSafeCommand(),"failed replacement lost the terminal certificate");
  require(planner.position().start_WT==final_start &&
    (planner.position().getPos(planner.position().getTotalDuration())-body.p).norm()<1e-9,
    "failed replacement changed the endpoint hold");
  // New geometry invalidates a future segment. Recovery must certify a stop,
  // and a failed subsequent route search must never restore the old command.
  require(planner.setGoal(4,{0,0,1},0),"return goal rejected");
  require(planner.plan()==super_utils::SUCCESS,"return trajectory failed");
  const auto returning=planner.position();now+=.1;
  body.p=returning.getPos(.1);body.v=returning.getVel(.1);body.a=returning.getAcc(.1);
  body.rcv_time=now;map->setBody(body);
  // Put the obstruction on the actual future endpoint. A fixed x=.2 wall can
  // become clear of the trajectory when production radius/inflation changes.
  const double wall_x=returning.getPos(returning.getTotalDuration()).x();
  rog_map::PointCloud wall;
  for(int y=-15;y<=15;++y)for(int z=-10;z<=10;++z) {
    rog_map::PointType p;p.x=wall_x;p.y=y*.1+.05;p.z=1+z*.1+.05;p.intensity=1;wall.push_back(p);
  }
  for(int i=0;i<10;++i)map->integrate(wall,{body.p,body.q});
  require(planner.ensureSafeCommand() && planner.braking(),"new wall did not trigger certified braking");
  const auto stop=planner.position();
  require(planner.plan()==super_utils::NO_NEED,"replanned during braking");
  now=stop.start_WT+stop.getTotalDuration()+.3;
  body.p=stop.getPos(stop.getTotalDuration());body.v.setZero();body.a.setZero();body.rcv_time=now;map->setBody(body);
  planner.setGoal(5,{wall_x,0,1},0);planner.setMapRevision(3);
  require(planner.plan()==super_utils::FAILED && planner.braking(),"blocked goal resumed old command");
  require(planner.position().start_WT==stop.start_WT,"failed replan replaced certified stop");
  // Cancel does not leave an executable trajectory; clock rollback latches shut.
  planner.cancel();require(!planner.active() && !planner.ensureSafeCommand() && planner.position().empty(),"cancel retained command");
  require(planner.setGoal(6,body.p+Eigen::Vector3d(0,.15,0),0),"nearby goal rejected");
  require(planner.plan()==super_utils::SUCCESS && planner.ensureSafeCommand(),"terminal capture failed");
  now-=1.;require(planner.plan()==super_utils::FAILED && !planner.ensureSafeCommand(),"clock rollback authorized command");
  require(!planner.setGoal(7,{.5,0,1},0),"clock fault did not latch");
  std::cout<<"EGO unknown rejection, exact endpoints, moving handoff, final arrival, cancellation and clock fault passed\n";
  rclcpp::shutdown();
}
