#include <airfar_planner/airfar_planner.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <array>
#include <unordered_map>
#include <navigation/observed_map.hpp>

namespace {
std::shared_ptr<navigation::ObservedMap> collision_mirror;
using VisibilityKey=std::array<float,6>;
struct VisibilityHash {
 size_t operator()(const VisibilityKey& key) const {
   size_t h=0;for(float v:key)h^=std::hash<float>{}(v)+0x9e3779b9+(h<<6)+(h>>2);return h;
 }
};
std::unordered_map<VisibilityKey,bool,VisibilityHash> visibility_cache;
std::vector<uint8_t> cached_occupied,cached_free;
bool finite(const geometry_msgs::msg::Point& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
bool decode(const sensor_msgs::msg::PointCloud2& msg, const std_msgs::msg::Header& header,
            const geometry_msgs::msg::Point& lo, const geometry_msgs::msg::Point& hi,
            PointCloud& out) {
  if(msg.header.frame_id!=header.frame_id || msg.header.stamp!=header.stamp ||
     uint64_t(msg.width)*msg.height>2'000'000 || msg.is_bigendian ||
     msg.row_step!=uint64_t(msg.width)*msg.point_step ||
     msg.data.size()!=uint64_t(msg.row_step)*msg.height) return false;
  for(const auto* field : {"x","y","z"}) {
    const auto it=std::find_if(msg.fields.begin(),msg.fields.end(),[&](const auto& f){return f.name==field;});
    if(it==msg.fields.end() || it->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
       it->count!=1 || it->offset+4>msg.point_step) return false;
  }
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
    for(;x!=x.end();++x,++y,++z) {
      if(!std::isfinite(*x+*y+*z) || *x<lo.x || *x>hi.x || *y<lo.y || *y>hi.y || *z<lo.z || *z>hi.z) return false;
      PCLPoint p; p.x=*x;p.y=*y;p.z=*z;p.intensity=0;out.push_back(p);
    }
  } catch(const std::exception&) { return false; }
  return true;
}
}

bool DPMaster::SnapshotFresh() const {
  const auto age=nh->now().nanoseconds()-rclcpp::Time(snapshot_header_.stamp).nanoseconds();
  return map_revision_>0 && age>=-5'000'000 && age<1'000'000'000 &&
    std::chrono::steady_clock::now()-snapshot_received_<std::chrono::seconds(2);
}

void DPMaster::PublishRoute(const PointStack& points, const std::string& reason) {
  dronehacks_msgs::msg::RoutePolyline out;
  out.header=snapshot_header_;out.localization_epoch=epoch_;out.goal_id=goal_id_;
  out.map_revision=map_revision_;out.route_id=++route_id_;out.reason=reason;
  out.status=out.INVALID;
  if(!points.empty() && !SnapshotFresh())out.reason="Air-FAR route exceeded snapshot freshness deadline";
  if(goal_active_ && SnapshotFresh() && points.size()>=2) {
    for(const auto& p:points) {
      if(!std::isfinite(p.x+p.y+p.z)) {out.points.clear();break;}
      out.points.push_back(DPUtil::Point3DToGeoMsgPoint(p));
    }
    bool clear=out.points.size()>=2;
    if(clear && DPUtil::configuration_edge_free) {
      for(size_t i=1;i<points.size();++i)
        if(!DPUtil::configuration_edge_free(points[i-1],points[i])) {clear=false;break;}
      if(!clear)out.reason="Air-FAR route failed authoritative ROG visibility";
    }
    if(clear)out.status=out.VALID;else out.points.clear();
  }
  route_pub_->publish(out);
}

void DPMaster::GoalCallback(dronehacks_msgs::msg::NavigationGoal::ConstSharedPtr msg) {
  if(!snapshot_mode_) return;
  if(msg->header.frame_id!=master_params_.world_frame || !msg->localization_epoch ||
     (epoch_ && msg->localization_epoch!=epoch_) || !msg->goal_id || msg->goal_id<goal_id_ || (msg->goal_id==goal_id_ && !msg->cancel)) return;
  if(!msg->cancel && !finite(msg->pose.position)) return;
  goal_id_=msg->goal_id;mission_goal_=*msg;goal_active_=!msg->cancel;goal_pending_=goal_active_;
  PublishRoute({},msg->cancel ? "goal cancelled" : "goal changed");
  if(goal_pending_ && is_odom_init_) ApplyPendingGoal();
}

void DPMaster::ApplyPendingGoal() {
  if(!goal_pending_ || !goal_active_) return;
  const auto& p=mission_goal_.pose.position;
  Point3D goal(p.x,p.y,p.z);
  const int layer=map_handler_.GetLayerId(goal);
  if(layer<0 || size_t(layer)>=DPUtil::layerIdx2Height_.size()) {
    goal_active_=false;PublishRoute({},"goal outside configured vertical envelope");return;
  }
  // Retain the requested altitude; layers index graph connectivity only.
  graph_planner_.UpdateGoal(goal,layer,false);
  goal_pending_=false;is_goal_update_=true;
}

void DPMaster::SnapshotCallback(dronehacks_msgs::msg::MapSnapshot::ConstSharedPtr msg) {
  if(!snapshot_mode_) return;
  auto reject=[&](const char* why){snapshot_header_=std_msgs::msg::Header();PublishRoute({},why);};
  const auto age=nh->now().nanoseconds()-rclcpp::Time(msg->header.stamp).nanoseconds();
  if(msg->header.frame_id!=master_params_.world_frame || !msg->localization_epoch || !msg->revision ||
     age < -5'000'000 || age>1'000'000'000 || msg->localization_epoch<epoch_ ||
     (msg->localization_epoch==epoch_ && msg->revision<=map_revision_) ||
     !std::isfinite(msg->resolution) || msg->resolution<.02 || msg->resolution>1 ||
     !finite(msg->lower) || !finite(msg->upper) || msg->lower.x>=msg->upper.x ||
     msg->lower.y>=msg->upper.y || msg->lower.z>=msg->upper.z ||
     msg->upper.x-msg->lower.x>100 || msg->upper.y-msg->lower.y>100 || msg->upper.z-msg->lower.z>30) {
    reject("invalid snapshot metadata");return;
  }
  nav_msgs::msg::Odometry odom;
  odom.header=msg->body_pose.header;odom.child_frame_id="base_link";odom.pose.pose=msg->body_pose.pose;
  const auto& q=odom.pose.pose.orientation;
  const double qn=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(odom.header.frame_id!=msg->header.frame_id || odom.header.stamp!=msg->header.stamp ||
     odom.child_frame_id!="base_link" || !finite(odom.pose.pose.position) || !std::isfinite(qn) || std::abs(qn-1)>.001) {
    reject("invalid snapshot pose");return;
  }
  if(DPUtil::configuration_radius>0 && std::abs(msg->resolution-DPUtil::kLeafSize)>1e-6) {
    reject("configuration-space voxel size differs from source map");return;
  }
  PointCloud obs,free;
  if(!decode(msg->occupied,msg->header,msg->lower,msg->upper,obs) ||
     !decode(msg->free,msg->header,msg->lower,msg->upper,free)) {reject("invalid snapshot cloud");return;}
  if(DPUtil::configuration_radius>0) {
    if(!collision_mirror) {
      const auto config=nh->get_parameter("navigation_config_file").as_string();
      if(config.empty()){reject("configuration-space routing requires ROG configuration");return;}
      collision_mirror=std::make_shared<navigation::ObservedMap>(config,[]{return 0.;},true);
      const auto lower=nh->get_parameter("hard_boundary_lower").as_double_array();
      const auto upper=nh->get_parameter("hard_boundary_upper").as_double_array();
      if(!lower.empty() || !upper.empty()) {
        if(lower.size()!=3 || upper.size()!=3)throw std::invalid_argument("mirror boundary requires three axes");
        collision_mirror->setHardBoundary({lower[0],lower[1],lower[2]},{upper[0],upper[1],upper[2]},.4);
      }
      DPUtil::configuration_point_free=[](const Point3D& p) {
        return collision_mirror->isKnownFreeInflate(Eigen::Vector3d(p.x,p.y,p.z));
      };
      DPUtil::configuration_edge_free=[](const Point3D& a,const Point3D& b) {
        std::array<float,3> p{a.x,a.y,a.z},q{b.x,b.y,b.z};if(q<p)std::swap(p,q);
        VisibilityKey key{p[0],p[1],p[2],q[0],q[1],q[2]};
        // A moving measured start has unique endpoints on every snapshot.
        // Do not let these one-use connectors evict/fill the static graph cache.
        const auto is_start=[](const Point3D& p){return p.x==DPUtil::odom_pos.x && p.y==DPUtil::odom_pos.y && p.z==DPUtil::odom_pos.z;};
        const bool cacheable=!is_start(a) && !is_start(b);
        if(cacheable) {const auto found=visibility_cache.find(key);if(found!=visibility_cache.end())return found->second;}
        const bool clear=collision_mirror->isLineFree(Eigen::Vector3d(a.x,a.y,a.z),Eigen::Vector3d(b.x,b.y,b.z),true,true);
        if(cacheable && visibility_cache.size()<4'000'000)visibility_cache.emplace(key,clear);
        return clear;
      };
    }
    // Reuse visibility only for byte-identical classified geometry. Any change
    // (including free-to-unknown) invalidates all edges before updating the map.
    if(cached_occupied!=msg->occupied.data || cached_free!=msg->free.data) {
      visibility_cache.clear();cached_occupied=msg->occupied.data;cached_free=msg->free.data;
    }
    const auto mirror_start=std::chrono::steady_clock::now();
    collision_mirror->replaceClassifications(obs,free);
    RCLCPP_INFO_THROTTLE(nh->get_logger(),*nh->get_clock(),5000,"ROG snapshot mirror update %.1f ms",
      std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-mirror_start).count());
  }
  if(epoch_ && epoch_!=msg->localization_epoch) {
    ResetEnvironmentAndGraph();map_revision_=0;goal_id_=0;
    const auto& p=odom.pose.pose.position;map_handler_.SetMapOrigin(Point3D(p.x,p.y,p.z));
  }
  epoch_=msg->localization_epoch;map_revision_=msg->revision;snapshot_header_=msg->header;
  if(goal_active_ && mission_goal_.localization_epoch!=epoch_) {goal_active_=false;goal_pending_=false;}
  snapshot_received_=std::chrono::steady_clock::now();
  OdomCallBack(std::make_shared<nav_msgs::msg::Odometry>(odom));
  map_handler_.UpdateRobotPosition(robot_pos_,cur_layer_idxs_);
  *temp_obs_ptr_=std::move(obs);*temp_free_ptr_=std::move(free);
  // ROG classifications are authoritative. Do not infer free space from a
  // missing return, point intensity, a clear ray beyond depth, or simulator truth.
  map_handler_.ClearRegion(Point3D(msg->lower.x,msg->lower.y,msg->lower.z),Point3D(msg->upper.x,msg->upper.y,msg->upper.z));
  map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
  // A snapshot is authoritative classified geometry, not an incremental scan.
  // The goal may arrive after CAD initialization, or changing layers may expose
  // previously unmatched contours without any newly occupied voxel. Restricting
  // vertex creation to an occupancy delta loses those graph vertices forever.
  *DPUtil::new_obs_cloud_ = *temp_obs_ptr_;
  map_handler_.GetSurroundObsCloud(DPUtil::surround_obs_cloud_);
  map_handler_.GetSurroundFreeCloud(DPUtil::surround_free_cloud_);
  DPUtil::cur_dyobs_cloud_->clear();
  DPUtil::UpdateKdTrees(DPUtil::new_obs_cloud_,DPUtil::surround_free_cloud_,DPUtil::surround_obs_cloud_);
  is_cloud_init_=!DPUtil::surround_free_cloud_->empty() || !DPUtil::surround_obs_cloud_->empty();
  snapshot_dirty_=true;
  ApplyPendingGoal();
}
