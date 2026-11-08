#include <larp_route_follower/master_route.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
using namespace larp_route_follower;
using Point=navigation::Point;
void check(bool x,const char* s){if(!x)throw std::runtime_error(s);}
int main(int argc,char**argv){
  const std::string file="/tmp/master-route-unit.xyz";
  {std::ofstream f(file);f<<"# gates 40\n";for(int i=0;i<=40;++i)f<<i*.1<<" 0 1\n";}
  bool wall=false,sealed=false;size_t calls=0;
  auto free=[&](const Point&a,const Point&b){++calls;
    for(int i=0;i<=20;++i){Point p=a+(b-a)*(i/20.);if(wall&&p.x()>.75&&p.x()<1.25&&(sealed||std::abs(p.y())<.35))return false;}return true;};
  MasterRoute m(free);m.load(file);m.setBounds({-.5,-1,.8},{4.2,1,1.2});check(m.activate({4,0,1}),"activate");
  auto p=m.nearby({0,0,1},1.5,1);check(p.size()>1&&m.searches()==0,"cached route needs no search");
  wall=true;bool repaired=false;double maxms=0;
  for(int k=0;k<5000;++k){p=m.nearby({0,0,1},1.5,2);maxms=std::max(maxms,m.lastMs());
    check(m.lastPops()<=256,"pop bound");check(m.lastChecks()<=64,"collision bound");check(m.nodeCount()<=40000,"storage bound");
    if(m.reason()=="local repair joined forward CAD route"){repaired=true;break;}}
  check(repaired,"repair failed");for(size_t i=1;i<m.path().size();++i)check(free(m.path()[i-1],m.path()[i]),"repair collides");
  Point body(0,0,1);size_t old=0;
  for(int i=0;i<2000&&(body-Point(4,0,1)).norm()>.03;++i){p=m.nearby(body,.5,2);if(p.size()<2)std::cerr<<"stalled at "<<body.transpose()<<" progress "<<m.progress()<<" reason "<<m.reason()<<"\n";check(p.size()>1,"following repair stalled");
    Point d=p[1]-body;body+=d.normalized()*std::min(.03,d.norm());check(m.progress()>=old,"progress reversed");old=m.progress();}
  check((body-Point(4,0,1)).norm()<.03,"did not reach goal");
  // A newly blocked map must never return an old cached edge as free.
  m.activate({4,0,1});sealed=true;
  for(int k=0;k<200;++k){auto q=m.nearby({0,0,1},1.5,3);
    for(size_t i=1;i<q.size();++i)check(free(q[i-1],q[i]),"stale collision cache");
    check(m.lastChecks()<=64 && m.lastPops()<=256,"blocked work bound");}
  check(m.reason()!="local repair joined forward CAD route","sealed wall bypassed");
  // Clear the live obstacle: normal progress resumes without a full-goal search.
  wall=false;auto reopened=m.nearby({0,0,1},1.5,4);check(reopened.size()>1,"clearing did not restore route");
  check(!m.activate({3,0,1}),"wrong goal accepted");
  if(argc>1){MasterRoute course([](auto,auto){return true;});course.load(argv[1]);check(course.activate({-2.82128,-.77724,1}),"course goal");
    Point b(0,0,1);for(int i=0;i<3000&&(b-Point(-2.82128,-.77724,1)).norm()>.02;++i){auto q=course.nearby(b,1.5,1);check(q.size()>1,"course progress stalled");Point d=q[1]-b;b+=d.normalized()*std::min(.03,d.norm());}
    check((b-Point(-2.82128,-.77724,1)).norm()<.02,"course goal not reached");check(course.searches()==0,"unchanged course searched");}
  std::cout<<"master route tests passed; max repair slice "<<maxms<<" ms, searches="<<m.searches()<<"\n";
}
