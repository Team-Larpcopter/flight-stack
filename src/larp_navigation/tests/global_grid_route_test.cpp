#include <navigation/observed_map.hpp>
#include <navigation/global_grid_route.hpp>
#include <navigation/route_planner.hpp>
#include <iostream>
#include <stdexcept>
#include <thread>
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(int argc,char** argv) {
  if(argc!=2 && argc!=3 && argc!=4)return 2;
  double now=10.;
  auto map=std::make_shared<navigation::ObservedMap>(argv[1],[&]{return now;},true);
  auto clock=std::make_shared<navigation::PlannerClock>([&]{return now;},[](const auto& s){std::cerr<<s<<'\n';});
  navigation::Point lo(-2,-2,0),hi(2,2,2.5),start(-.8,0,1),goal(.8,0,1);
  if(argc>=3) {
    lo={-4.1624,-5.5626,0};hi={1.9336,.5334,argc==4?std::stod(argv[3]):2.95};start={0,0,1};goal={-1.90688,-.47244,1};
    map->setHardBoundary(lo,hi,.4);map->loadCadPrior(argv[2],true);
  }
  rog_map::PointCloud free,wall;
  if(argc==2) {
    map->setHardBoundary(lo,hi,.4);
    for(int x=-20;x<20;++x)for(int y=-20;y<20;++y)for(int z=0;z<25;++z) {
      rog_map::PointType p;p.x=(x+.5)*.1;p.y=(y+.5)*.1;p.z=(z+.5)*.1;p.intensity=1;
      free.push_back(p);
      if(x==0 && y>=-8 && y<8)wall.push_back(p);
    }
    map->replaceClassifications(wall,free);
  }
  auto edge=[&](const auto& a,const auto& b){return map->isLineFree(a,b,true,true);};
  navigation::GlobalGridRoute route(argv[1],map,clock,edge);route.setBounds(lo,hi);
  auto nearby=route.nearby(start,goal,1,1,1.);
  std::cout<<"search_ms="<<route.lastSearchMs()<<" reason="<<route.reason()<<" points="<<route.route().points.size()<<std::endl;
  for(const auto& p:route.route().points)std::cout<<"ROUTE "<<p.x()<<" "<<p.y()<<" "<<p.z()<<std::endl;
  require(nearby.size()>=2,"full-goal A* search failed");
  require((route.route().points.front()-start).norm()<1e-8,"start relocated");
  require((route.route().points.back()-goal).norm()<1e-8,"goal relocated or horizon-only route");
  for(size_t i=1;i<route.route().points.size();++i)require(edge(route.route().points[i-1],route.route().points[i]),"unsafe A* edge");
  if(argc==2) {
    double detour=0;for(auto p:route.route().points)detour=std::max(detour,std::abs(p.y()));
    require(detour>1.1,"did not route around the wall beyond local horizon");
  }
  now+=.1;require(route.nearby(start,goal,1,1,1.).size()>=2 && route.searches()==1,"unchanged route not cached");
  now+=.1;require(route.nearby(start,goal,1,2,1.).size()>=2 && route.searches()==1,"unchanged geometry revision forced a new search");
  path_search::Astar bounded(argv[1],clock,map);bounded.setSearchBounds(lo,hi);bounded.setEdgeValidator(edge);
  navigation::Polyline timed;
  require(bounded.pointToPointPathSearch(start,goal,path_search::ON_INF_MAP|path_search::UNKNOWN_AS_OCCUPIED|path_search::FULL_GOAL,-1.,timed,1e-12)==super_utils::TIME_OUT,"timeout used frozen simulation clock");
  if(argc==2) {
    // Closing both ends of the wall invalidates the cached route; no fallback.
    wall.clear();for(auto p:free)if(std::abs(p.x-.05)<.001)wall.push_back(p);
    map->replaceClassifications(wall,free);now+=.1;
    require(route.nearby(start,goal,1,3,1.).empty() && route.route().points.empty(),"map change reused blocked route");
    const auto attempts=route.searches();now+=.01;
    require(route.nearby(start,goal,1,3,1.).empty() && route.searches()==attempts,"failed search retried without backoff");
    wall.clear();map->replaceClassifications(wall,free);now+=.1;
    std::this_thread::sleep_for(std::chrono::milliseconds(510));
    require(route.nearby(start,goal,1,4,1.).size()>=2,"map reopening did not recover");
    require(route.nearby(start,{5,0,1},2,4,1.).empty(),"accepted out-of-map goal");
  }
  // Exercise the real trajectory frontend using this exact shared map.
  super_utils::RobotState body;body.p=start;body.q.setIdentity();body.yaw=0;body.v.setZero();body.a.setZero();body.j.setZero();body.rcv=true;body.rcv_time=now;map->setBody(body);
  navigation::RoutePlanner planner(argv[1],map,clock,true);planner.setSearchBounds(lo,hi);planner.setMapRevision(5);
  if(argc==4)planner.setFlightEnvelope(lo+navigation::Point::Constant(.4),hi-navigation::Point::Constant(.4));
  require(planner.setGoal(1,goal,0),"goal rejected");
  auto result=planner.plan();
  // Match the runtime coordinator: rejected curves remain uncommitted while
  // its existing bounded nearby-guide reduction retries at the next plan tick.
  for(int attempt=1;result!=super_utils::SUCCESS && attempt<5;++attempt) {
    require(planner.position().empty(),"failed optimization committed a curve");
    // Global search failures use a 500 ms steady-clock cooldown. Advancing the
    // simulated clock alone does not allow a real retry after a search timeout.
    if(planner.gridRoute()->route().points.empty())
      std::this_thread::sleep_for(std::chrono::milliseconds(510));
    now+=.11;body.rcv_time=now;map->setBody(body);
    result=planner.plan();
    std::cout<<"SUPER retry="<<attempt<<" result="<<int(result)<<std::endl;
  }
  std::cout<<"SUPER result="<<int(result)<<" cached points="<<planner.gridRoute()->route().points.size()<<std::endl;
  require(result==super_utils::SUCCESS && planner.committedStillFree(),"A* guide did not produce a certified SUPER trajectory");
  std::cout<<"PASS SUPER A*: full goal, exact endpoints, collision edges, cache, revision invalidation, deadline, trajectory\n";
}
