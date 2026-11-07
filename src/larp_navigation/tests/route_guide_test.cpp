#include <navigation/route_guide.hpp>
#include <iostream>
#include <stdexcept>

using namespace navigation;
void require(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
int main() {
  auto free = [](const Point&, const Point&) { return true; };
  Route route{"map", 7, 10, 1, 5.0, {{0,0,1},{1,0,1},{1,1,1},{3,1,1}}};
  RouteGuide guide;
  require(guide.accept(route,"map",7,10,5.0),"valid route rejected");
  auto path=guide.nearby({0,0,1},2.5,5.1,7,free);
  require(path.size()==4,"lost route corners");
  require((path[1]-Point(1,0,1)).norm()<1e-9,"shortcut across first corner");
  require((path[2]-Point(1,1,1)).norm()<1e-9,"shortcut across second corner");
  require((path.back()-Point(1.5,1,1)).norm()<1e-9,"horizon not measured along polyline");
  auto blocked=[](const Point& a,const Point& b) { return a.y()<0.5 && b.y()<0.5; };
  path=guide.nearby({0,0,1},3.0,5.1,7,blocked);
  require(path.size()==3 && path.back().y()<.35 && path.back().y()>.34,"did not retain a margin inside free prefix");
  RouteGuide connector;
  require(connector.accept(route,"map",7,10,5.0),"connector route rejected");
  const auto short_path=connector.nearby({0,-.2,1},.1,5.1,7,free);
  require(short_path.size()==2 && (short_path.back()-Point(0,-.1,1)).norm()<1e-9,"lost horizon inside route connector");
  require(guide.nearby({0,0,1},3,6.1,7,free).empty(),"accepted stale route");
  require(guide.nearby({0,0,1},3,5.1,8,free).empty(),"accepted route from old goal");
  require(guide.nearby({5,5,1},3,5.1,7,free).empty(),"connected far from route");
  require(!guide.accept(route,"other",7,10,5.1),"accepted wrong frame");
  require(guide.nearby({0,0,1},3,5.1,7,free).empty(),"retained route after invalidation");
  require(!guide.accept(route,"map",7,10,5.1),"replayed old route after invalidation");
  route.route_id=2;
  require(guide.accept(route,"map",7,10,5.1),"new route rejected after invalidation");
  guide.clear();
  require(!guide.accept(route,"map",7,10,5.1),"clear forgot route sequence high-water mark");
  route.points[1].x()=std::numeric_limits<double>::quiet_NaN();
  require(!guide.accept(route,"map",7,10,5.1),"accepted NaN geometry");
  std::cout << "route geometry, blocked segments, freshness, goal and frame checks passed\n";
}
