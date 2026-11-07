#pragma once
#include <navigation/planner_clock.hpp>
#include <navigation/route_guide.hpp>
#include <navigation/trajectory_validator.hpp>
#include <navigation/certified_stop.hpp>
#include <navigation/physical_clearance.hpp>
#include <navigation/perception_heading.hpp>
#include <super_core/super_planner.h>
#include <yaml-cpp/yaml.h>

namespace navigation {
// Single-owner coordinator: route callbacks, map updates and planning run on
// the same executor. A separate controller must sample the committed trajectory.
class RoutePlanner {
  struct Limits {double v,a,budget,yaw_rate,yaw_acceleration,radius;};
  static Limits loadLimits(const std::string& file) {
    const auto root=YAML::LoadFile(file),s=root["navigation_safety"];
    Limits l{s["max_velocity"].as<double>(),s["max_acceleration"].as<double>(),s["validation_budget_ms"].as<double>(),
      s["max_yaw_rate"].as<double>(),s["max_yaw_acceleration"].as<double>(),root["super_planner"]["robot_r"].as<double>()};
    if(!std::isfinite(l.v+l.a+l.budget+l.yaw_rate+l.yaw_acceleration+l.radius) || l.radius<=0 || l.v<=0 || l.a<=0 || l.budget<=0 ||
       l.yaw_rate<=0 || l.yaw_rate>1 || l.yaw_acceleration<=0 || l.yaw_acceleration>2 ||
       root["traj_opt"]["boundary"]["max_vel"].as<double>()>=l.v ||
       root["traj_opt"]["boundary"]["max_acc"].as<double>()>=l.a)
      throw std::invalid_argument("optimizer targets require margin below hard dynamic limits");
    return l;
  }
 public:
  RoutePlanner(const std::string& config,rog_map::ROGMap::Ptr map,
               std::shared_ptr<PlannerClock> clock)
    :map_(std::move(map)),clock_(std::move(clock)),limits_(loadLimits(config)),planner_(config,clock_,map_,true),
     validator_([this](const Point& lo,const Point& hi){return boxFree(lo,hi);},limits_.v,limits_.a,limits_.budget),
     yaw_validator_([](const Point&,const Point&){return true;},limits_.yaw_rate,limits_.yaw_acceleration,limits_.budget) {
    if(YAML::LoadFile(config)["navigation_safety"]["prefer_observable_heading"].as<bool>(false))
      planner_.setYawHeadingSelector([this](const Point&,double heading){
        super_utils::RobotState current;planner_.getRobotState(current);
        if(!current.rcv)return heading;
        return perceptionHeading(current.p,heading,envelope_lower_-Point::Constant(limits_.radius),
          envelope_upper_+Point::Constant(limits_.radius),[this](const Point& q){return map_->isOccupied(q);},current.yaw);
      });
    planner_.setGuideProvider([this](const Point& start,double horizon,super_utils::vec_Vec3f& path){
      path=guide_.nearby(start,horizon*guide_horizon_scale_,clock_->getSimTime(),goal_id_,
        [this](const Point& a,const Point& b){return lineFree(a,b);});
      last_guide_valid_=path.size()>=2;
      return last_guide_valid_;
    },true);
    planner_.setCorridorFilter([this](auto& poly){
      if(!envelope_lower_.allFinite() || !envelope_upper_.allFinite())return;
      auto planes=poly.GetPlanes();const auto count=planes.rows();planes.conservativeResize(count+6,4);
      planes.bottomRows(6).setZero();
      for(int axis=0;axis<3;++axis) {
        planes(count+2*axis,axis)=1.;planes(count+2*axis,3)=-envelope_upper_[axis];
        planes(count+2*axis+1,axis)=-1.;planes(count+2*axis+1,3)=envelope_lower_[axis];
      }
      poly.SetPlanes(planes);
    });
    planner_.setTrajectoryValidator([this](const auto& p,const auto& y){
      if(y.empty() || std::abs(p.getTotalDuration()-y.getTotalDuration())>1e-3) return false;
      if(!yaw_validator_(y)){clock_->warn("Yaw trajectory rejected: "+yaw_validator_.failure());return false;}
      const bool valid=validator_(p);
      if(!valid)clock_->warn("Trajectory rejected: "+validator_.failure());
      return valid;
    });
  }
  bool setGoal(uint64_t id,const Point& goal,double yaw) {
    if(!id || id<=goal_id_ || !goal.allFinite() || !std::isfinite(yaw))return false;
    goal_id_=id;goal_=goal;yaw_=yaw;active_=true;new_goal_=true;guide_horizon_scale_=1.;guide_.clear();return true;
  }
  void setFlightEnvelope(const Point& lower,const Point& upper) {
    if(!lower.allFinite() || !upper.allFinite() || !(lower.array()<upper.array()).all())
      throw std::invalid_argument("invalid flight envelope");
    envelope_lower_=lower;envelope_upper_=upper;
  }
  void cancel(){active_=false;braking_=false;guide_.clear();}
  void invalidateRoute(){guide_.clear();}
  bool accept(Route route,uint64_t current_revision) {
    if(!active_)return false;
    const auto now=clock_->getSimTime();
    if(route.frame!="map" || route.goal_id!=goal_id_ || route.map_revision>current_revision ||
       !std::isfinite(route.capture_time) || route.capture_time>now || now-route.capture_time>1.) {
      guide_.clear();return false;
    }
    return guide_.accept(std::move(route),"map",goal_id_,current_revision,clock_->getSimTime());
  }
  super_utils::RET_CODE plan() {
    last_guide_valid_=false;
    if(!active_)return super_utils::FAILED;
    super_utils::RobotState state;planner_.getRobotState(state);
    if(!state.rcv || clock_->getSimTime()-state.rcv_time>.3)return super_utils::FAILED;
    if(braking_) {
      if(clock_->getSimTime()<stop_.position.start_WT+stop_.position.getTotalDuration()+.2 || state.v.norm()>.1)
        return super_utils::NO_NEED;
      // Never resume the old invalidated curve. Remain stopped until SUPER
      // atomically commits a newly certified plan from the measured rest state.
      const auto result=planner_.PlanFromRest(goal_,yaw_,true);
      if(result==super_utils::SUCCESS){braking_=false;new_goal_=false;}
      adaptGuideHorizon(result);
      return result;
    }
    const auto previous=planner_.getCommittedPositionTrajectory();
    const bool expired=previous.empty() || clock_->getSimTime()>=previous.start_WT+previous.getTotalDuration();
    if(expired && state.v.norm()>.1)return super_utils::FAILED;
    auto result=expired ? planner_.PlanFromRest(goal_,yaw_,new_goal_) : planner_.ReplanOnce(goal_,yaw_,new_goal_);
    if(result==super_utils::SUCCESS)new_goal_=false;
    adaptGuideHorizon(result);
    return result;
  }
  geometry_utils::Trajectory position(){return braking_?stop_.position:planner_.getCommittedPositionTrajectory();}
  geometry_utils::Trajectory yaw(){return braking_?stop_.yaw:planner_.getCommittedYawTrajectory();}
  bool committedStillFree(){
    const auto p=position();
    if(p.empty())return false;
    if(validator_(p))return true;
    clock_->warn("Committed trajectory rejected: "+validator_.failure());return false;
  }
  bool ensureSafeCommand() {
    if(committedStillFree())return true;
    auto stop=certifiedStop(position(),yaw(),clock_->getSimTime(),validator_);
    if(!stop || !yaw_validator_(stop->yaw))return false;
    stop_=std::move(*stop);braking_=true;++recovery_count_;
    clock_->warn("Certified braking recovery: stopping in current observed free space");return true;
  }
  bool braking()const{return braking_;}
  uint64_t recoveryCount()const{return recovery_count_;}
  bool active()const{return active_;}
  uint64_t goalId()const{return goal_id_;}
 private:
  bool insideEnvelope(const Point& p) const {
    return (p.array()>=envelope_lower_.array()).all() && (p.array()<=envelope_upper_.array()).all();
  }
  bool lineFree(const Point& a,const Point& b) const {
    return insideEnvelope(a) && insideEnvelope(b) && map_->isLineFree(a,b,true,true);
  }
  bool boxFree(const Point& lo,const Point& hi) const {
    if(!lo.allFinite() || !hi.allFinite() || !insideEnvelope(lo) || !insideEnvelope(hi))return false;
    Eigen::Vector3i a,b;map_->infMapPosToGlobalIndex(lo,a);map_->infMapPosToGlobalIndex(hi,b);
    const Eigen::Vector3i count=b-a+Eigen::Vector3i::Ones();
    if(count.minCoeff()<=0 || count.maxCoeff()>64 || int64_t(count.x())*count.y()*count.z()>4096)return false;
    // The inflation grid is a conservative route-search approximation, not a
    // second robot geometry. An inflated cell may still contain a valid centre
    // position. Certify the declared sphere against full occupied AND unknown
    // voxels instead; CIRI already reserves radius plus voxel circumradius.
    const double search_radius=limits_.radius+map_->getResolution()*.5;
    const Point expanded_lo=lo-Point::Constant(search_radius),expanded_hi=hi+Point::Constant(search_radius);
    if(!map_->insideLocalMap(expanded_lo) || !map_->insideLocalMap(expanded_hi))return false;
    super_utils::vec_Vec3f obstacles;
    for(const auto type:{super_utils::OCCUPIED,super_utils::UNKNOWN}) {
      map_->boxSearch(expanded_lo,expanded_hi,type,obstacles);
      for(const auto& p:obstacles)
        if(!sphereBoxClear(lo,hi,p,map_->getResolution(),limits_.radius))return false;
    }
    return true;
  }
  void adaptGuideHorizon(super_utils::RET_CODE result) {
    // Retry the same Air-FAR polyline with a shorter nearby section when the
    // optimizer cannot certify it. Never relax geometry or command a rejected
    // curve; an existing certified trajectory/stop remains authoritative.
    if(!last_guide_valid_)return;
    if(result==super_utils::SUCCESS)guide_horizon_scale_=std::min(1.,guide_horizon_scale_*1.1);
    else if(result!=super_utils::NO_NEED)guide_horizon_scale_=std::max(.125,guide_horizon_scale_*.5);
  }
  double guide_horizon_scale_{1.};bool last_guide_valid_{false};
  rog_map::ROGMap::Ptr map_;std::shared_ptr<PlannerClock> clock_;
  Limits limits_;
  super_planner::SuperPlanner planner_;RouteGuide guide_;TrajectoryValidator validator_,yaw_validator_;
  StopCommand stop_;bool braking_{false};uint64_t recovery_count_{0};
  Point goal_{Point::Zero()};double yaw_{0};uint64_t goal_id_{0};bool active_{false},new_goal_{false};
  Point envelope_lower_{Point::Constant(-std::numeric_limits<double>::infinity())};
  Point envelope_upper_{Point::Constant(std::numeric_limits<double>::infinity())};
};
}
