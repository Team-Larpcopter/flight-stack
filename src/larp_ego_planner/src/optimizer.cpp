#include <larp_ego_planner/optimizer.hpp>
#include <ego_planner/planner_manager.h>
#include <stdexcept>

namespace larp_ego_planner {
struct Optimizer::Impl {
  rclcpp::Node::SharedPtr node;
  ego_planner::EGOPlannerManager manager;
};
Optimizer::Optimizer(Settings s, std::function<int(Eigen::Vector3d)> occupied)
    : impl_(std::make_unique<Impl>()) {
  if (!std::isfinite(s.velocity+s.acceleration+s.resolution+s.control_point_distance+s.horizon) ||
      s.velocity<=0 || s.acceleration<=0 || s.resolution<=0 || s.control_point_distance<=0 || s.horizon<=0)
    throw std::invalid_argument("invalid EGO optimizer settings");
  rclcpp::NodeOptions options;
  options.use_global_arguments(false).enable_rosout(false).start_parameter_services(false)
    .start_parameter_event_publisher(false);
  options.parameter_overrides({
    {"manager/max_vel",s.velocity}, {"manager/max_acc",s.acceleration},
    {"manager/max_jerk",4.0}, {"manager/feasibility_tolerance",.05},
    {"manager/control_points_distance",s.control_point_distance},
    {"manager/planning_horizon",s.horizon}, {"manager/use_distinctive_trajs",false},
    {"manager/drone_id",0}, {"optimization/lambda_smooth",1.0},
    {"optimization/lambda_collision",.5}, {"optimization/lambda_feasibility",.1},
    {"optimization/lambda_fitness",1.0}, {"optimization/dist0",.2},
    {"optimization/swarm_clearance",.5}, {"optimization/max_vel",s.velocity},
    {"optimization/max_acc",s.acceleration}});
  impl_->node=std::make_shared<rclcpp::Node>("ego_optimizer",options);
  auto map=std::make_shared<GridMap>();
  map->setExternalMap(s.resolution,std::move(occupied));
  impl_->manager.initPlanModules(impl_->node,nullptr,map);
  impl_->manager.deliverTrajToOptimizer();
}
Optimizer::~Optimizer()=default;
std::optional<Spline> Optimizer::plan(const Eigen::Vector3d& p,const Eigen::Vector3d& v,
    const Eigen::Vector3d& a,const std::vector<Eigen::Vector3d>& guide) {
  if(guide.size()<2 || guide.size()>64 || !p.allFinite() || !v.allFinite() || !a.allFinite())return {};
  for(const auto& point:guide)if(!point.allFinite())return {};
  auto& manager=impl_->manager;
  PolynomialTraj* seed=nullptr;
  // Preserve route corners in EGO's polynomial initialization. A direct seed
  // is sufficient for a straight local guide.
  if(guide.size()>2) {
    std::vector<Eigen::Vector3d> waypoints(guide.begin()+1,guide.end());
    if(!manager.planGlobalTrajWaypoints(p,v,a,waypoints,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()))return {};
    seed=&manager.global_data_.global_traj_;
    if(!std::isfinite(seed->getTimeSum()) || seed->getTimeSum()>30.)return {};
  }
  if(!manager.reboundReplan(p,v,a,guide.back(),Eigen::Vector3d::Zero(),true,false,seed,true))return {};
  auto& trajectory=manager.local_data_.position_traj_;
  Spline out{trajectory.getControlPoint(),trajectory.getInterval()};
  if(out.controls.rows()!=3 || out.controls.cols()<6 || out.controls.cols()>67 ||
     !out.controls.allFinite() || !std::isfinite(out.interval) || out.interval<=0)return {};
  return out;
}
}
