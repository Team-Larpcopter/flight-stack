#pragma once
#include <navigation/planner_contract.hpp>
#include <navigation/ego_spline.hpp>
#include <navigation/certified_stop.hpp>
#include <larp_ego_planner/optimizer.hpp>
#include <traj_opt/yaw_traj_opt.h>

namespace navigation {
// Single-owner EGO local optimizer over the same observed ROG map and global
// route as SUPER. Only independently certified, stopping polynomials commit.
class EgoRoutePlanner final : public PlannerContract {
 public:
  EgoRoutePlanner(const std::string& config,rog_map::ROGMap::Ptr map,std::shared_ptr<PlannerClock> clock)
    :map_(std::move(map)),clock_(std::move(clock)),limits_(PlannerLimits::load(config)),
     settings_(settings(config,map_->getResolution())),
     optimizer_(settings_,[this](Point p){return inside(p) && map_->isKnownFreeInflate(p)?0:1;}),
     route_(config,map_,clock_,[this](const Point& a,const Point& b){return lineFree(a,b);}),
     validator_([this](const Point& lo,const Point& hi){return boxFree(lo,hi);},limits_.v,limits_.a,limits_.budget),
     yaw_validator_([](const Point&,const Point&){return true;},limits_.yaw_rate,limits_.yaw_acceleration,limits_.budget),
     yaw_opt_(limits_.yaw_rate*.8,limits_.yaw_acceleration*.8) {
    const auto root=YAML::LoadFile(config);
    lead_=root["super_planner"]["replan_forward_dt"].as<double>();
    goal_yaw_enabled_=root["super_planner"]["goal_yaw_en"].as<bool>(false);
    if(!std::isfinite(lead_) || lead_<=0 || lead_>.2)throw std::invalid_argument("invalid EGO handoff lead");
  }
  void setMapRevision(uint64_t revision)override{revision_=revision;}
  void setSearchBounds(const Point& lo,const Point& hi)override{route_.setBounds(lo,hi);}
  const GlobalGridRoute* gridRoute()const override{return &route_;}
  void setFlightEnvelope(const Point& lo,const Point& hi)override {
    if(!lo.allFinite() || !hi.allFinite() || !(lo.array()<hi.array()).all())throw std::invalid_argument("invalid flight envelope");
    lower_=lo;upper_=hi;
  }
  bool setGoal(uint64_t id,const Point& goal,double yaw)override {
    if(clock_fault_ || !id || id<=goal_id_ || !goal.allFinite() || !std::isfinite(yaw))return false;
    goal_id_=id;goal_=goal;goal_yaw_=yaw;active_=true;route_.clear();horizon_scale_=1.;return true;
  }
  void cancel()override{active_=false;position_.clear();yaw_.clear();braking_=false;route_.clear();}
  bool active()const override{return active_;}
  uint64_t goalId()const override{return goal_id_;}
  uint64_t recoveryCount()const override{return recovery_count_;}
  bool braking()const{return braking_;}
  geometry_utils::Trajectory position()override{return position_;}
  geometry_utils::Trajectory yaw()override{return yaw_;}
  bool ensureSafeCommand()override {
    if(!validClock() || !active_)return false;
    if(validator_(position_) && yaw_validator_(yaw_))return true;
    auto stop=certifiedStop(position_,yaw_,clock_->getSimTime(),validator_);
    if(!stop || !yaw_validator_(stop->yaw))return false;
    position_=std::move(stop->position);yaw_=std::move(stop->yaw);braking_=true;++recovery_count_;
    clock_->warn("EGO certified braking recovery");return true;
  }
  super_utils::RET_CODE plan()override {
    using namespace super_utils;
    if(!validClock() || !active_)return FAILED;
    const double now=clock_->getSimTime();
    const auto wall=std::chrono::steady_clock::now();
    const auto state=map_->getRobotState();
    if(!state.rcv || !state.p.allFinite() || !state.v.allFinite() || !state.a.allFinite() ||
       !std::isfinite(state.yaw+state.rcv_time) || now-state.rcv_time>.3 || now-state.rcv_time<-.005)return FAILED;
    const double end=position_.empty()?now:position_.start_WT+position_.getTotalDuration();
    if(braking_ && (now<end+.2 || state.v.norm()>.1))return NO_NEED;
    const bool live=!position_.empty() && now<end && !braking_;
    if(!live && state.v.norm()>.1)return FAILED;
    // Preserve a completed final command so the bridge can report is_final.
    if(!position_.empty() && !braking_ &&
       (position_.getPos(position_.getTotalDuration())-goal_).norm()<1e-6 &&
       (state.p-goal_).norm()<.08 && state.v.norm()<.05)return NO_NEED;
    if(live && end-now<lead_+.02)return NO_NEED;
    const double handoff=live?now+lead_:now;
    Point p=state.p,v=state.v,a=state.a;
    Eigen::Vector4d initial_yaw(state.yaw,0,0,0);
    geometry_utils::Trajectory prefix,prefix_yaw;
    if(live) {
      const double at=handoff-position_.start_WT;
      p=position_.getPos(at);v=position_.getVel(at);a=position_.getAcc(at);
      initial_yaw<<yaw_.getPos(at).x(),yaw_.getVel(at).x(),yaw_.getAcc(at).x(),yaw_.getJer(at).x();
      if(!polynomialSlice(position_,now-position_.start_WT,at,prefix) ||
         !polynomialSlice(yaw_,now-yaw_.start_WT,at,prefix_yaw))return FAILED;
    }
    Polyline guide;
    if((p-goal_).norm()<1e-6 && lineFree(p,goal_))guide={p,goal_};
    else guide=route_.nearby(p,goal_,goal_id_,revision_,settings_.horizon*horizon_scale_);
    if(guide.size()<2)return FAILED;
    // Voxel routes contain tiny entry/exit zigzags even in open space. Remove
    // only corners whose replacement chord is configuration-free; otherwise
    // uneven min-snap seed intervals can produce a wildly overshooting seed.
    Polyline compact{guide.front()};
    for(size_t i=0;i+1<guide.size();) {
      size_t next=guide.size()-1;
      while(next>i+1 && !lineFree(guide[i],guide[next]))--next;
      compact.push_back(guide[next]);i=next;
    }
    guide=std::move(compact);
    geometry_utils::Trajectory candidate;
    const double distance=(guide.back()-p).norm();
    if(distance<.2) {
      // EGO deliberately rejects sub-20 cm requests. Finish with a bounded
      // Hermite capture, subject to the identical full-curve certificate.
      const double duration=std::max({1.,2.5*distance/settings_.velocity,
        3.*std::sqrt(distance/settings_.acceleration),3.*v.norm()/settings_.acceleration});
      candidate=terminalCapture(p,v,a,guide.back(),duration,handoff);
    } else {
      const auto spline=optimizer_.plan(p,v,a,{guide.begin(),guide.end()});
      if(!spline)return failed("optimizer rejected candidate");
      candidate=egoPolynomial(spline->controls,spline->interval,p,v,a,guide.back(),handoff);
    }
    geometry_utils::Trajectory candidate_yaw;
    const Eigen::Vector4d final_yaw(goal_yaw_,0,0,0);
    if(!yaw_opt_.optimize(initial_yaw,final_yaw,candidate,candidate_yaw,5,false,!goal_yaw_enabled_))
      return failed("yaw optimization failed");
    prefix.append(candidate);prefix.start_WT=now;
    prefix_yaw.append(candidate_yaw);prefix_yaw.start_WT=now;
    if(!validator_(prefix))return failed("position: "+validator_.failure());
    if(!yaw_validator_(prefix_yaw))return failed("yaw: "+yaw_validator_.failure());
    if(std::abs(prefix.getTotalDuration()-prefix_yaw.getTotalDuration())>1e-3)return failed("yaw duration mismatch");
    // A moving handoff must be ready before its preserved prefix expires.
    if(live && std::chrono::duration<double>(std::chrono::steady_clock::now()-wall).count()>=lead_)
      return failed("handoff deadline exceeded");
    position_=std::move(prefix);yaw_=std::move(prefix_yaw);braking_=false;
    horizon_scale_=std::min(1.,horizon_scale_*1.1);return SUCCESS;
  }
 private:
  static larp_ego_planner::Settings settings(const std::string& file,double resolution) {
    const auto root=YAML::LoadFile(file),cfg=root["ego_planner"],boundary=root["traj_opt"]["boundary"];
    const double v=boundary["max_vel"].as<double>(),a=boundary["max_acc"].as<double>();
    larp_ego_planner::Settings out{v,a,resolution,.1,std::min(.6,v*8.)};
    if(cfg) {
      out.control_point_distance=cfg["control_points_distance"].as<double>(out.control_point_distance);
      out.horizon=cfg["planning_horizon"].as<double>(out.horizon);
    }
    if(!std::isfinite(out.horizon+out.control_point_distance) || out.horizon<.2 || out.horizon>2 ||
       out.control_point_distance<.03 || out.control_point_distance>.4)
      throw std::invalid_argument("EGO horizon must be [0.2,2] m and control point distance [0.03,0.4] m");
    return out;
  }
  bool validClock() {
    const double now=clock_->getSimTime();
    if(!std::isfinite(now) || now<last_time_) {clock_fault_=true;active_=false;position_.clear();yaw_.clear();}
    last_time_=now;return !clock_fault_;
  }
  super_utils::RET_CODE failed(const std::string& reason) {
    horizon_scale_=std::max(.25,horizon_scale_*.75);
    clock_->warn("EGO candidate rejected: "+reason);return super_utils::FAILED;
  }
  bool inside(const Point& p)const{return p.allFinite() && (p.array()>=lower_.array()).all() && (p.array()<=upper_.array()).all();}
  bool lineFree(const Point& a,const Point& b)const{return inside(a) && inside(b) && map_->isLineFree(a,b,true,true);}
  bool boxFree(const Point& lo,const Point& hi)const{return certifiedBoxFree(map_,limits_.radius,lower_,upper_,lo,hi);}
  rog_map::ROGMap::Ptr map_;std::shared_ptr<PlannerClock> clock_;PlannerLimits limits_;
  larp_ego_planner::Settings settings_;larp_ego_planner::Optimizer optimizer_;GlobalGridRoute route_;
  TrajectoryValidator validator_,yaw_validator_;traj_opt::YawTrajOpt yaw_opt_;
  geometry_utils::Trajectory position_,yaw_;
  Point goal_{Point::Zero()},lower_{Point::Constant(-INFINITY)},upper_{Point::Constant(INFINITY)};
  double goal_yaw_{0},lead_{.1},horizon_scale_{1},last_time_{-INFINITY};
  uint64_t goal_id_{0},revision_{0},recovery_count_{0};
  bool active_{false},braking_{false},clock_fault_{false},goal_yaw_enabled_{false};
};
}
