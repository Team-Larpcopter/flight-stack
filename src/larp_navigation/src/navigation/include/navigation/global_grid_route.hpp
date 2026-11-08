#pragma once
#include <navigation/route_guide.hpp>
#include <navigation/planner_clock.hpp>
#include <path_search/astar.h>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace navigation {
// Route lifecycle around SUPER's A*. Owns no map copy and no search algorithm.
class GlobalGridRoute {
 public:
  using EdgeFree=std::function<bool(const Point&,const Point&)>;
  GlobalGridRoute(const std::string& config,rog_map::ROGMap::Ptr map,
                  std::shared_ptr<PlannerClock> clock,EdgeFree free)
    :map_(std::move(map)),clock_(std::move(clock)),free_(std::move(free)),astar_(config,clock_,map_) {
    const auto cfg=YAML::LoadFile(config)["global_route"];
    budget_=cfg && cfg["search_budget_ms"]?cfg["search_budget_ms"].as<double>()*.001:.02;
    if(!std::isfinite(budget_) || budget_<=0 || budget_>.25)throw std::invalid_argument("global route search budget must be (0,250] ms");
    budget_=std::min(budget_,.02); // per-tick work slice, no total search deadline
    astar_.setVisualProcessEn(false);
    astar_.setEdgeValidator(free_);
    setBounds(map_->getLocalMapOrigin()-map_->getLocalMapSize()*.5,
              map_->getLocalMapOrigin()+map_->getLocalMapSize()*.5);
  }
  void setBounds(const Point& lo,const Point& hi){astar_.setSearchBounds(lo,hi);clear();}
  void clear(){astar_.cancelSearch();route_={};guide_.clear();reason_="no cached global route";retry_pending_=false;}
  const Route& route() const{return route_;}
  const std::string& reason()const{return reason_;}
  uint64_t searches()const{return searches_;}
  double lastSearchMs()const{return last_search_ms_;}
  Polyline nearby(const Point& start,const Point& goal,uint64_t goal_id,uint64_t revision,double horizon) {
    const double now=clock_->getSimTime();
    if(!revision || !goal_id || !start.allFinite() || !goal.allFinite()) {clear();reason_="invalid route identity/pose";return {};}
    if(route_.goal_id && route_.goal_id!=goal_id)clear();
    if(!map_->isKnownFreeInflate(start) || !free_(start,start)) {clear();reason_="measured start is not configuration-free";return {};}
    if(!map_->isKnownFreeInflate(goal) || !free_(goal,goal)) {clear();reason_="goal is not configuration-free";return {};}
    if(!route_.points.empty() && route_.map_revision!=revision) {
      if(!valid(route_.points)){clear();reason_="map change invalidated cached route";}
      else route_.map_revision=revision;
    }
    if(!route_.points.empty() && guide_.renew(now)) {
      auto path=guide_.nearby(start,horizon,now,goal_id,free_);
      if(path.size()>=2){route_.capture_time=now;reason_="cached SUPER A* route";return path;}
    }
    route_={};guide_.clear();
    const auto wall=std::chrono::steady_clock::now();
    // Depth revises the map each frame; do not let that bypass failed-search backoff.
    if(astar_.searchPending() && (retry_goal_!=goal_id || (goal-retry_goal_point_).norm()>1e-8 ||
                                (start-retry_start_).norm()>.2)) {
      astar_.cancelSearch();retry_pending_=false;
    }
    const bool continuing=astar_.searchPending();
    if(!continuing && retry_pending_ && retry_goal_==goal_id &&
       (start-retry_start_).norm()<.1 && wall-last_attempt_<std::chrono::milliseconds(500))return {};
    if(!continuing) {
      retry_pending_=true;retry_goal_=goal_id;retry_revision_=revision;
      retry_start_=start;retry_goal_point_=goal;last_attempt_=wall;++searches_;
    }
    Polyline path;
    const auto code=astar_.pointToPointPathSearch(retry_start_,goal,
      path_search::ON_INF_MAP|path_search::UNKNOWN_AS_OCCUPIED|path_search::DONT_USE_INF_NEIGHBOR|path_search::FULL_GOAL|path_search::RESUMABLE,
      -1.,path,budget_);
    last_search_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-wall).count();
    if(code==super_utils::NO_NEED){reason_="SUPER A* searching (progress retained)";return {};}
    if(code!=super_utils::REACH_GOAL) {astar_.cancelSearch();reason_="SUPER A* failed: "+super_utils::RET_CODE_STR[code];return {};}
    // Preserve the actual endpoints; never relocate an occupied start or goal.
    path.insert(path.begin(),start);path.push_back(goal);
    Polyline compact;
    for(const auto& p:path) {
      if(!compact.empty() && (p-compact.back()).norm()<1e-8)continue;
      if(compact.size()>=2) {
        const Point a=compact.back()-compact[compact.size()-2],b=p-compact.back();
        if(a.cross(b).norm()<1e-9 && a.dot(b)>0 && free_(compact[compact.size()-2],p))compact.pop_back();
      }
      compact.push_back(p);
    }
    if(compact.size()<2 || !valid(compact)){reason_="SUPER A* endpoint/edge validation failed";return {};}
    route_={"map",goal_id,revision,searches_,now,compact};
    if(!guide_.accept(route_,"map",goal_id,revision,now)){clear();reason_="internal route contract rejected";return {};}
    retry_pending_=false;reason_="new full-goal SUPER A* route";
    return guide_.nearby(start,horizon,now,goal_id,free_);
  }
 private:
  bool valid(const Polyline& path)const {
    for(size_t i=1;i<path.size();++i)if(!free_(path[i-1],path[i]))return false;
    return path.size()>=2;
  }
  rog_map::ROGMap::Ptr map_;std::shared_ptr<PlannerClock> clock_;EdgeFree free_;
  path_search::Astar astar_;RouteGuide guide_;Route route_;
  double budget_,last_search_ms_{0};uint64_t searches_{0},retry_goal_{0},retry_revision_{0};
  bool retry_pending_{false};Point retry_start_=Point::Zero(),retry_goal_point_=Point::Zero();
  std::chrono::steady_clock::time_point last_attempt_{};std::string reason_{"no cached global route"};
};
}
