#pragma once
#include <navigation/global_grid_route.hpp>
#include <navigation/physical_clearance.hpp>
#include <data_structure/base/trajectory.h>

namespace navigation {
class PlannerContract {
 public:
  virtual ~PlannerContract()=default;
  virtual void setMapRevision(uint64_t)=0;
  virtual void setSearchBounds(const Point&,const Point&)=0;
  virtual void setFlightEnvelope(const Point&,const Point&)=0;
  virtual const GlobalGridRoute* gridRoute()const=0;
  virtual bool setGoal(uint64_t,const Point&,double)=0;
  virtual void cancel()=0;
  virtual super_utils::RET_CODE plan()=0;
  virtual geometry_utils::Trajectory position()=0;
  virtual geometry_utils::Trajectory yaw()=0;
  virtual bool ensureSafeCommand()=0;
  virtual uint64_t recoveryCount()const=0;
  virtual bool active()const=0;
  virtual uint64_t goalId()const=0;
};
struct PlannerLimits {
  double v,a,budget,yaw_rate,yaw_acceleration,radius;
  static PlannerLimits load(const std::string& file) {
    const auto root=YAML::LoadFile(file),s=root["navigation_safety"];
    PlannerLimits l{s["max_velocity"].as<double>(),s["max_acceleration"].as<double>(),s["validation_budget_ms"].as<double>(),
      s["max_yaw_rate"].as<double>(),s["max_yaw_acceleration"].as<double>(),root["super_planner"]["robot_r"].as<double>()};
    const double opt_v=root["traj_opt"]["boundary"]["max_vel"].as<double>();
    const double opt_a=root["traj_opt"]["boundary"]["max_acc"].as<double>();
    if(!std::isfinite(l.v+l.a+l.budget+l.yaw_rate+l.yaw_acceleration+l.radius+opt_v+opt_a) ||
       l.radius<=0 || l.v<=0 || l.a<=0 || l.budget<=0 || opt_v<=0 || opt_a<=0 ||
       l.yaw_rate<=0 || l.yaw_rate>1 || l.yaw_acceleration<=0 || l.yaw_acceleration>2 || opt_v>=l.v || opt_a>=l.a)
      throw std::invalid_argument("optimizer targets require margin below hard dynamic limits");
    return l;
  }
};
// Shared whole-curve certificate geometry for both local planners. Unknown
// voxels and the map exterior are occupied; the body is the declared sphere.
inline bool certifiedBoxFree(const rog_map::ROGMap::Ptr& map,double radius,
    const Point& envelope_lower,const Point& envelope_upper,const Point& lo,const Point& hi) {
  if(!lo.allFinite() || !hi.allFinite() || !(lo.array()>=envelope_lower.array()).all() ||
     !(hi.array()<=envelope_upper.array()).all())return false;
  Eigen::Vector3i a,b;map->infMapPosToGlobalIndex(lo,a);map->infMapPosToGlobalIndex(hi,b);
  const Eigen::Vector3i count=b-a+Eigen::Vector3i::Ones();
  if(count.minCoeff()<=0 || count.maxCoeff()>64 || int64_t(count.x())*count.y()*count.z()>4096)return false;
  const double search_radius=radius+map->getResolution()*.5;
  const Point expanded_lo=lo-Point::Constant(search_radius),expanded_hi=hi+Point::Constant(search_radius);
  if(!map->insideLocalMap(expanded_lo) || !map->insideLocalMap(expanded_hi))return false;
  super_utils::vec_Vec3f obstacles;
  for(const auto type:{super_utils::OCCUPIED,super_utils::UNKNOWN}) {
    map->boxSearch(expanded_lo,expanded_hi,type,obstacles);
    for(const auto& p:obstacles)if(!sphereBoxClear(lo,hi,p,map->getResolution(),radius))return false;
  }
  return true;
}
}
