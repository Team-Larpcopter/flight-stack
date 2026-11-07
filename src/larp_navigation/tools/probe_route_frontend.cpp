// Offline CAD fixture only: no ROS, flight commands, simulator truth or route search.
#include <navigation/observed_map.hpp>
#include <navigation/route_planner.hpp>
#include <fstream>
#include <iostream>
int main(int argc,char** argv) {
 if(argc!=3){std::cerr<<"usage: probe_route_frontend MISSION_JSON ROUTE_XYZ\n";return 2;}
 const auto mission=YAML::LoadFile(argv[1]);
 const auto config=mission["navigation_config_file"].as<std::string>();
 auto map=std::make_shared<navigation::ObservedMap>(config,[]{return 1.;},true);
 auto vec=[](const YAML::Node& n){return Eigen::Vector3d(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());};
 map->setHardBoundary(vec(mission["hard_boundary"]["lower"]),vec(mission["hard_boundary"]["upper"]),.4);
 navigation::Polyline route;std::ifstream in(argv[2]);double x,y,z;while(in>>x>>y>>z)route.emplace_back(x,y,z);
 if(route.size()<2)return 2;
 super_utils::RobotState body;body.p=route.front();body.v.setZero();body.a.setZero();body.j.setZero();body.q.setIdentity();body.yaw=0;body.rcv=true;body.rcv_time=1;map->setBody(body);
 rog_map::PointCloud cloud;rog_map::PointType p;p.x=0;p.y=0;p.z=-.05;p.intensity=1;cloud.push_back(p);
 map->integrate(cloud,{body.p,body.q});map->loadCadPrior(mission["cad_prior_file"].as<std::string>());
 bool all_edges=true;
 for(size_t i=1;i<route.size();++i) {
   const bool free=map->isLineFree(route[i-1],route[i],true,true);all_edges=all_edges&&free;
   std::cout<<"ROG_EDGE "<<i<<" FREE "<<free<<'\n';
   if(!free) {
     const double length=(route[i]-route[i-1]).norm();
     for(double t=0;t<=length;t+=.01) {
       const auto q=route[i-1]+(route[i]-route[i-1])*(t/std::max(length,1e-9));
       if(!map->isKnownFreeInflate(q)){std::cout<<"BLOCKED_SAMPLE "<<q.transpose()<<'\n';break;}
     }
   }
 }
 auto clock=std::make_shared<navigation::PlannerClock>([]{return 1.;},[](const auto& s){std::cerr<<s<<'\n';});
 navigation::RoutePlanner planner(config,map,clock);
 planner.setFlightEnvelope(vec(mission["flight_envelope"]["lower"]),vec(mission["flight_envelope"]["upper"]));
 planner.setGoal(1,vec(mission["goal"]["position"]),0);
 planner.accept({"map",1,1,1,1,route},1);
 auto ret=planner.plan();
 for(int attempt=1;attempt<6 && ret!=super_utils::SUCCESS;++attempt) {
   std::cout<<"REPLAN_ATTEMPT "<<attempt+1<<'\n';ret=planner.plan();
 }
 std::cout<<"SUPER_RESULT "<<ret<<" CERTIFIED "<<planner.committedStillFree()<<'\n';
 for(const auto& q:clock->latest_frontend)std::cout<<"FRONTEND "<<q.transpose()<<'\n';
 return ret==super_utils::SUCCESS && planner.committedStillFree() && all_edges?0:1;
}
