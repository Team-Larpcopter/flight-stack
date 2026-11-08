// Route-follower variant of larp_navigation/src/navigation/depth_map_node.cpp.
// Copied verbatim except for the planner backend selection (marked below), so
// mapping, topics, parameters, readiness and certificates stay identical.
#include <navigation/observed_map.hpp>
#include <navigation/pose_buffer.hpp>
#include <larp_route_follower/follower_route_planner.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <dronehacks_msgs/msg/map_snapshot.hpp>
#include <dronehacks_msgs/msg/map_state.hpp>
#include <dronehacks_msgs/msg/navigation_goal.hpp>
#include <dronehacks_msgs/msg/route_polyline.hpp>
#include <dronehacks_msgs/msg/committed_trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include <cstring>
#include <optional>

class DepthMapNode : public rclcpp::Node {
 public:
  DepthMapNode():Node("depth_map") {
    config_=declare_parameter<std::string>("config_file","");
    const auto rig=YAML::LoadFile(declare_parameter<std::string>("rig_file",""));
    Eigen::Isometry3d bi=Eigen::Isometry3d::Identity(),ic=bi;
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) {
      bi.matrix()(r,c)=rig["T_body_imu"][r][c].as<double>();
      ic.matrix()(r,c)=rig["T_imu_left"][r][c].as<double>();
    }
    body_camera_=bi*ic;
    if(rig["T_body_depth"])for(int r=0;r<4;++r)for(int c=0;c<4;++c)
      body_camera_.matrix()(r,c)=rig["T_body_depth"][r][c].as<double>();
    depth_frame_=rig["depth_frame"]?rig["depth_frame"].as<std::string>():"";
    snapshot_include_free_=declare_parameter<bool>("snapshot_include_free",true);
    takeoff_height_=declare_parameter<double>("takeoff_height",1.0);
    route_ready_=create_publisher<std_msgs::msg::Bool>("/autonomy/route_ready",1);
    ready_=create_publisher<std_msgs::msg::Bool>("/autonomy/takeoff_ready",1);
    epoch_=declare_parameter<int64_t>("localization_epoch",1);
    const double depth_age=declare_parameter<double>("max_depth_age_s",.5);
    if(!std::isfinite(depth_age) || depth_age<=0 || depth_age>1.)throw std::invalid_argument("max_depth_age_s must be (0,1]");
    max_depth_age_ns_=int64_t(depth_age*1e9);
    stride_=declare_parameter<int>("pixel_stride",4);
    ignore_top_rows_=declare_parameter<int>("ignore_top_depth_rows",0);
    min_depth_=declare_parameter<double>("min_depth_m",.3);
    max_depth_=declare_parameter<double>("max_depth_m",6.0);
    if(ignore_top_rows_<0 || epoch_==0 || stride_<1 || stride_>16 || !(min_depth_>0 && max_depth_>min_depth_)) throw std::invalid_argument("invalid mapping limits");
    cad_prior_=declare_parameter<std::string>("cad_prior_file","");
    static_cad_only_=declare_parameter<bool>("static_cad_only",false);
    if(static_cad_only_ && cad_prior_.empty())throw std::invalid_argument("static CAD mode requires a CAD prior");
    if(static_cad_only_)RCLCPP_WARN(get_logger(),"STATIC CAD TEST: sensor occupancy updates disabled; VIO, freshness and immutable geofence remain active");
    map_=std::make_shared<navigation::ObservedMap>(config_,[this]{return now().seconds();},!cad_prior_.empty());
    const auto boundary_lower=declare_parameter<std::vector<double>>("hard_boundary_lower",std::vector<double>{});
    const auto boundary_upper=declare_parameter<std::vector<double>>("hard_boundary_upper",std::vector<double>{});
    const auto boundary_reference=declare_parameter<std::string>("hard_boundary_reference","");
    if(declare_parameter<bool>("require_hard_boundary",false) &&
       (boundary_lower.empty() || boundary_upper.empty()))
      throw std::invalid_argument("this deployment requires a surveyed hard boundary");
    if(!boundary_lower.empty() || !boundary_upper.empty()) {
      if(boundary_lower.size()!=3 || boundary_upper.size()!=3 || boundary_reference.empty())
        throw std::invalid_argument("hard boundary needs three axes and surveyed frame reference");
      boundary_lower_=boundary_lower;boundary_upper_=boundary_upper;
      map_->setHardBoundary({boundary_lower[0],boundary_lower[1],boundary_lower[2]},
        {boundary_upper[0],boundary_upper[1],boundary_upper[2]},.4);
      RCLCPP_INFO(get_logger(),"Immutable boundary active: %s",boundary_reference.c_str());
    }
    launch_radius_=declare_parameter<double>("declared_launch_radius_m",0.0);
    launch_min_z_=declare_parameter<double>("declared_launch_min_z_m",.1);
    launch_max_z_=declare_parameter<double>("declared_launch_max_z_m",1.5);
    const auto launch_reference=declare_parameter<std::string>("launch_clearance_reference","");
    if(launch_radius_>0 && launch_reference.empty())throw std::invalid_argument("launch clearance requires explicit reference");
    if(launch_radius_>0)RCLCPP_WARN(get_logger(),"Declared launch clearance assumption: %s",launch_reference.c_str());
    const bool enable_planning=declare_parameter<bool>("enable_planning",true);
    if(enable_planning) {
      planner_clock_=std::make_shared<navigation::PlannerClock>([this]{return now().seconds();},
        [this](const std::string& m){RCLCPP_WARN(get_logger(),"%s",m.c_str());});
      const auto backend=declare_parameter<std::string>("route_backend","super_astar");
      if(backend!="super_astar")throw std::invalid_argument("unknown route backend");
      internal_grid_route_=backend=="super_astar";
      // larp_route_follower: the only difference from larp_navigation's
      // depth_map_node is the planner backend constructed here.
      planner_backend_=declare_parameter<std::string>("planner_backend","follower");
      if(planner_backend_!="follower")throw std::invalid_argument("larp_route_follower provides planner_backend follower only");
      larp_route_follower::FollowerSettings follower;
      follower.horizon=declare_parameter<double>("follower_horizon_m",follower.horizon);
      follower.replan_period=declare_parameter<double>("follower_replan_period_s",follower.replan_period);
      follower.scan_when_blocked=declare_parameter<bool>("follower_scan_when_blocked",true);
      follower.route_file=declare_parameter<std::string>("follower_route_file","");
      planner_=std::make_unique<larp_route_follower::FollowerRoutePlanner>(config_,map_,planner_clock_,follower,internal_grid_route_);
      if(internal_grid_route_ && !boundary_lower_.empty())
        planner_->setSearchBounds({boundary_lower_[0],boundary_lower_[1],boundary_lower_[2]},
                                 {boundary_upper_[0],boundary_upper_[1],boundary_upper_[2]});
      if(internal_grid_route_)route_output_=create_publisher<dronehacks_msgs::msg::RoutePolyline>("/navigation/route",1);
      const auto flight_lower=declare_parameter<std::vector<double>>("flight_envelope_lower",std::vector<double>{});
      const auto flight_upper=declare_parameter<std::vector<double>>("flight_envelope_upper",std::vector<double>{});
      if(!flight_lower.empty() || !flight_upper.empty()) {
        if(flight_lower.size()!=3 || flight_upper.size()!=3)throw std::invalid_argument("flight envelope needs three axes");
        planner_->setFlightEnvelope({flight_lower[0],flight_lower[1],flight_lower[2]},
          {flight_upper[0],flight_upper[1],flight_upper[2]});
      }
      path_=create_publisher<nav_msgs::msg::Path>("/navigation/committed_path",1);
      trajectories_=create_publisher<dronehacks_msgs::msg::CommittedTrajectory>("/navigation/trajectory",1);
      goals_=create_subscription<dronehacks_msgs::msg::NavigationGoal>("/navigation/goal",rclcpp::QoS(1).transient_local(),
        [this](const dronehacks_msgs::msg::NavigationGoal& m) {
          if(m.localization_epoch!=epoch_ || m.header.frame_id!="map")return;
          const auto& p=m.pose.position;const auto& q=m.pose.orientation;
          const double qn=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
          if(m.cancel) {if(m.goal_id>=planner_->goalId())planner_->cancel();return;}
          if(!std::isfinite(qn) || std::abs(qn-1)>.001)return;
          const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          planner_->setGoal(m.goal_id,{p.x,p.y,p.z},yaw);
        });
    } // enable_planning: map-only mode has no trajectory/goal interface.
    if(std::isfinite(max_depth_) && max_depth_>map_->getMapConfig().raycast_range_max) throw std::invalid_argument("depth range exceeds map ray range");
    snapshots_=create_publisher<dronehacks_msgs::msg::MapSnapshot>("/navigation/map_snapshot",rclcpp::QoS(1).transient_local());
    map_states_=create_publisher<dronehacks_msgs::msg::MapState>("/navigation/map_state",rclcpp::SensorDataQoS().keep_last(1));
    rejected_maps_=create_publisher<dronehacks_msgs::msg::MapSnapshot>("/navigation/rejected_map",1);
    execution_gated_=declare_parameter<bool>("execution_gated",false);
    phase_sub_=create_subscription<std_msgs::msg::String>("/flight_executive/state",rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String& m){
        execution_allowed_=m.data=="HOLD" || m.data=="TRACKING";
        if(execution_gated_ && !execution_allowed_ && planner_)planner_->suspendExecution();
      });
    health_=create_subscription<std_msgs::msg::Bool>("/navigation/vio_healthy",1,[this](const std_msgs::msg::Bool& m) {
      healthy_=m.data;health_received_=std::chrono::steady_clock::now();
    });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/navigation/odometry",rclcpp::SensorDataQoS().keep_last(4),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if(m->header.frame_id!="map" || m->child_frame_id!="base_link") return;
        const auto& p=m->pose.pose.position;const auto& q=m->pose.pose.orientation;
        navigation::TimedPose pose{rclcpp::Time(m->header.stamp).nanoseconds(),{p.x,p.y,p.z},{q.w,q.x,q.y,q.z}};
        if(!poses_.push(pose)) return;
        body_.p=pose.position;body_.q=pose.orientation;
        body_.v=body_.q*Eigen::Vector3d(m->twist.twist.linear.x,m->twist.twist.linear.y,m->twist.twist.linear.z);
        body_.a.setZero();body_.j.setZero();body_.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        body_.rcv_time=pose.stamp_ns*1e-9;body_.rcv=true;map_->setBody(body_);
      });
    if(!static_cad_only_) {
    info_=create_subscription<sensor_msgs::msg::CameraInfo>(declare_parameter<std::string>("depth_info_topic","/stereo/depth/camera_info"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        if(m->width && m->height && std::isfinite(m->k[0]+m->k[4]+m->k[2]+m->k[5]) && m->k[0]>0 && m->k[4]>0) calibration_=m;
      });
    depth_=create_subscription<sensor_msgs::msg::Image>(declare_parameter<std::string>("depth_topic","/stereo/depth/image"),rclcpp::SensorDataQoS().keep_last(3),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {if(pending_.size()==3)pending_.pop_front();pending_.push_back(m);});
    }
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{process();});
    if(enable_planning)planner_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{plan();});
  }
 private:
  void plan() {
    const int64_t t=now().nanoseconds();
    std_msgs::msg::Bool readiness;
    readiness.data=healthy_ && cad_prior_loaded_ && body_.rcv &&
      std::chrono::steady_clock::now()-health_received_<std::chrono::seconds(1) &&
      t*1e-9-body_.rcv_time>=-.005 && t*1e-9-body_.rcv_time<.3 &&
      (static_cad_only_ || (t>=last_depth_ns_ && t-last_depth_ns_<max_depth_age_ns_));
    // Check the launch column above the ground-contact region. The lower
    // section still requires the surveyed, physically clear launch fixture.
    if(readiness.data && !boundary_lower_.empty()) {
      const double top=body_.p.z()+takeoff_height_;
      if(top>boundary_upper_[2]-.4)readiness.data=false;
      for(double z=std::max(body_.p.z(),boundary_lower_[2]+.45);readiness.data && z<=top;z+=.05)
        if(!map_->isKnownFreeInflate({body_.p.x(),body_.p.y(),z}))readiness.data=false;
    }
    ready_->publish(readiness);
    // Dynamic-map certificates retain the sensor capture timestamp. An
    // explicitly immutable CAD map is revalidated at the current time; it
    // does not acquire a new observation age from an unused depth image.
    if(t-last_certificate_ns_<50'000'000)return;last_certificate_ns_=t;
    nav_msgs::msg::Path out;out.header.frame_id="map";out.header.stamp=now();
    dronehacks_msgs::msg::CommittedTrajectory command;
    command.header.frame_id="map";command.header.stamp=rclcpp::Time(last_depth_ns_);
    command.localization_epoch=epoch_;command.goal_id=planner_->goalId();
    command.map_revision=revision_;// ROS-time sequence survives a mapper-only restart in the same mission.
    // A backward clock remains fail-closed in the bridge.
    trajectory_sequence_=std::max(trajectory_sequence_+1,static_cast<uint64_t>(std::max<int64_t>(1,t)));
    command.sequence=trajectory_sequence_;
    auto invalidate=[&](const char* reason){
      if(!planner_->position().empty())planner_->suspendExecution();
      std_msgs::msg::Bool pending;route_ready_->publish(pending);
      path_->publish(out);trajectories_->publish(command);
      if(internal_grid_route_ && (!healthy_ || !planner_->active() || !map_ready_for_route_))publishGridRoute(false,reason);
      if(planner_->active())RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,
        "Trajectory certificate unavailable: %s (depth age %.3f s)",reason,(t-last_depth_ns_)*1e-9);
    };
    const bool map_ready=static_cad_only_?cad_prior_loaded_:(t-last_depth_ns_<=max_depth_age_ns_);
    map_ready_for_route_=map_ready && body_.rcv && t*1e-9-body_.rcv_time<=.3 &&
      t*1e-9-body_.rcv_time>=-.005 && std::chrono::steady_clock::now()-health_received_<=std::chrono::seconds(1);
    if(!planner_->active() || !map_ready || !body_.rcv ||
       t*1e-9-body_.rcv_time>.3 ||
       std::chrono::steady_clock::now()-health_received_>std::chrono::seconds(1)) {invalidate("health, goal, localization or map freshness");return;}
    if(execution_gated_ && !execution_allowed_) {
      planner_->suspendExecution();
      if(!warm_start_)warm_start_=navigation::Point(body_.p.x(),body_.p.y(),body_.p.z()+takeoff_height_);
      if(t-last_plan_ns_>=100'000'000) {
        last_plan_ns_=t;planner_->setMapRevision(revision_);
        std_msgs::msg::Bool cached;cached.data=planner_->prepareRoute(*warm_start_);
        route_ready_->publish(cached);
        publishGridRoute(true);
      }
      return;
    }
    int result=-1;
    if(t-last_plan_ns_>=100'000'000) {
      last_plan_ns_=t;planner_->setMapRevision(revision_);
      const auto started=std::chrono::steady_clock::now();
      result=int(planner_->plan());
      const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
      const auto& search=planner_->masterRoute();
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
        "Fixed replan: %.2f ms, search %.2f ms, pops %zu, checks %zu, result %d",
        elapsed,search.lastMs(),search.lastPops(),search.lastChecks(),result);
      // Skip elapsed periods rather than immediately processing catch-up work.
      if(elapsed>=100.)last_plan_ns_=now().nanoseconds();
      if(internal_grid_route_)publishGridRoute(now().seconds()-body_.rcv_time<=.3);

    }
    const auto recovery=planner_->recoveryCount();
    const bool safe=planner_->ensureSafeCommand();
    const auto traj=planner_->position();
    if(traj.empty()) {
      const auto* route=planner_->gridRoute();
      invalidate(route && route->reason()=="SUPER A* searching (progress retained)" ?
        "global route search pending; waiting for completion" : "no committed stopping curve");return;
    }
    command.start_time=rclcpp::Time(int64_t(traj.start_WT*1e9));
    for(const auto& piece:traj)command.position.push_back(serialize(piece,3));
    for(const auto& piece:planner_->yaw())command.yaw.push_back(serialize(piece,1));
    if(!safe) {
      // Record the precise rejected revision, not a nearby 4 Hz route snapshot.
      // Payload remains INVALID; consumers must never execute it.
      invalidate("committed curve no longer certified in latest map");
      if(rejected_maps_->get_subscription_count()>0 && rejected_revision_!=revision_) {
        rejected_maps_->publish(snapshot(last_depth_ns_,last_depth_pose_));rejected_revision_=revision_;
      }
      return;
    }
    if(planner_->recoveryCount()!=recovery && rejected_maps_->get_subscription_count()>0)
      rejected_maps_->publish(snapshot(last_depth_ns_,last_depth_pose_));
    // Check localization again after the potentially expensive optimization.
    // Immutable geometry permits a current validation stamp, never stale pose.
    if(static_cad_only_) {
      if(now().seconds()-body_.rcv_time>.3 || std::chrono::steady_clock::now()-health_received_>std::chrono::seconds(1)) {
        invalidate("localization expired during static-map validation");return;
      }
      command.header.stamp=now();
    }
    // Optimization is synchronous: never send a certificate that expired
    // while searching, and never refresh dynamic depth capture timestamps.
    if(!static_cad_only_ && (now().nanoseconds()-last_depth_ns_>=max_depth_age_ns_ ||
       now().seconds()-body_.rcv_time>.3)) {
      invalidate("sensor lease expired during planning");return;
    }
    command.status=command.VALID;
    trajectories_->publish(command);
    const double end=traj.getTotalDuration();
    // Keep the terminal hold visible after the moving portion finishes.
    const double begin=std::min(end,std::max(0.,now().seconds()-traj.start_WT));
    for(double s=begin;s<=end;s+=.05) {
      geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
      p.header.stamp=rclcpp::Time(int64_t((traj.start_WT+s)*1e9));p.pose.position=point(traj.getPos(s));p.pose.orientation.w=1;
      out.poses.push_back(p);
    }
    path_->publish(out);
    if(result!=-1)RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"%s result %d: validated %zu trajectory samples",planner_backend_.c_str(),result,out.poses.size());
  }
  void publishGridRoute(bool ready,const std::string& failure="") {
    dronehacks_msgs::msg::RoutePolyline msg;
    msg.header.frame_id="map";msg.header.stamp=now();msg.localization_epoch=epoch_;
    msg.goal_id=planner_->goalId();msg.map_revision=revision_;msg.route_id=++route_sequence_;
    const auto* grid=planner_->gridRoute();
    const auto cached_route=planner_->displayRoute();
    msg.reason=failure.empty()?planner_->routeReason():failure;
    if(ready && map_->isKnownFreeInflate(warm_start_.value_or(body_.p)) && planner_->active() && cached_route.goal_id==planner_->goalId() &&
       cached_route.map_revision==revision_ && cached_route.points.size()>=2) {
      msg.status=msg.VALID;
      for(const auto& p:cached_route.points)msg.points.push_back(point(p));
    }
    route_output_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"SUPER grid route: %s; searches=%lu last_search_ms=%.2f",
      msg.reason.c_str(),grid->searches(),grid->lastSearchMs());
  }
  static dronehacks_msgs::msg::PolynomialPiece serialize(const geometry_utils::Piece& piece,int dimensions) {
    dronehacks_msgs::msg::PolynomialPiece out;out.duration=piece.getDuration();out.degree=piece.getDegree();
    const auto& c=piece.getCoeffMat();
    for(int row=0;row<dimensions;++row)for(int col=0;col<c.cols();++col)out.coefficients.push_back(c(row,col));
    return out;
  }
  static geometry_msgs::msg::Point point(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point out;out.x=p.x();out.y=p.y();out.z=p.z();return out;
  }
  sensor_msgs::msg::PointCloud2 cloud(const super_utils::vec_Vec3f& points, const std_msgs::msg::Header& header) {
    sensor_msgs::msg::PointCloud2 msg;msg.header=header;
    sensor_msgs::PointCloud2Modifier mod(msg);mod.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);mod.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
    for(const auto& p:points) {*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}
    return msg;
  }
  void process() {
    if(!healthy_ || std::chrono::steady_clock::now()-health_received_>std::chrono::seconds(1)) {pending_.clear();return;}
    if(static_cad_only_) {
      // Geometry is latched once. Subsequent messages carry only a fresh body
      // pose and the geometry identity; they never renew dynamic observations.
      const int64_t stamp=now().nanoseconds();
      if(!body_.rcv || stamp*1e-9-body_.rcv_time>.3 ||
         stamp*1e-9-body_.rcv_time<-.005) return;
      if(!cad_prior_loaded_) {
        const auto count=map_->loadCadPrior(cad_prior_,true);
        cad_prior_loaded_=true;
        RCLCPP_INFO(get_logger(),"Loaded static CAD prior into %zu cells without depth",count);
        // Loading may be expensive. Consume fresh odometry/health before
        // publishing geometry or issuing a localization lease.
        return;
      }
      last_depth_pose_={int64_t(body_.rcv_time*1e9),body_.p,body_.q};
      last_depth_ns_=last_depth_pose_.stamp_ns;
      if(revision_==0) {
        // A mapper restart must not reuse an identity for different CAD.
        // ROS-clock rollback requires a new localization epoch as elsewhere.
        revision_=static_cast<uint64_t>(std::max<int64_t>(1,stamp));
        snapshots_->publish(snapshot(last_depth_ns_,last_depth_pose_));
        return;
      }
      dronehacks_msgs::msg::MapState state;
      state.header.frame_id="map";state.header.stamp=rclcpp::Time(last_depth_ns_);
      state.localization_epoch=epoch_;state.revision=revision_;
      state.body_pose.position=point(body_.p);
      auto& q=state.body_pose.orientation;
      q.w=body_.q.w();q.x=body_.q.x();q.y=body_.q.y();q.z=body_.q.z();
      map_states_->publish(state);
      return;
    }
    if(!calibration_ || pending_.empty() || !body_.rcv) return;
    // After a costly planner tick, prefer the newest image with a usable pose.
    // Keep the oldest only while the newer frame is still awaiting odometry.
    while(pending_.size()>1 && poses_.at(rclcpp::Time(pending_.back()->header.stamp).nanoseconds()))
      pending_.pop_front();
    auto m=pending_.front();
    const int64_t stamp=rclcpp::Time(m->header.stamp).nanoseconds();
    const auto age=now().nanoseconds()-stamp;
    if(stamp<=last_depth_ns_ || age>max_depth_age_ns_ || age< -5'000'000) {pending_.pop_front();return;}
    const auto pose=poses_.at(stamp);
    if(!pose) {if(age>150'000'000)pending_.pop_front();return;}
    pending_.pop_front();
    auto k=calibration_->k;
    if(!depth_frame_.empty()) {
      if(m->header.frame_id!=depth_frame_)return;
      // Rectified depth rays use the projection intrinsics, not raw lens K.
      const auto& p=calibration_->p;
      if(!(p[0]>0 && p[5]>0))return;
      k[0]=p[0];k[2]=p[2];k[4]=p[5];k[5]=p[6];
    }
    const int bytes=m->encoding=="16UC1"?2:m->encoding=="32FC1"?4:0;
    if(!bytes || m->is_bigendian || m->width!=calibration_->width || m->height!=calibration_->height ||
       m->header.frame_id!=calibration_->header.frame_id || m->step<m->width*bytes || m->data.size()!=size_t(m->height)*m->step) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Rejected depth geometry or encoding");return;
    }
    Eigen::Isometry3d world_body=Eigen::Isometry3d::Identity();
    world_body.linear()=pose->orientation.toRotationMatrix();world_body.translation()=pose->position;
    const Eigen::Isometry3d world_camera=world_body*body_camera_;
    rog_map::PointCloud points;
    points.reserve((m->width/stride_)*(m->height/stride_));
    if(!static_cad_only_) for(unsigned v=0;v<m->height;v+=stride_) for(unsigned u=0;u<m->width;u+=stride_) {
      // Ignore noisy rows only for mapping: preserve inference, image geometry and intrinsics.
      if(v<static_cast<unsigned>(ignore_top_rows_))continue;
      double d;
      if(bytes==2) {uint16_t raw;std::memcpy(&raw,&m->data[v*m->step+u*2],2);d=raw*.001;}
      else {float raw;std::memcpy(&raw,&m->data[v*m->step+u*4],4);d=raw;}
      // Invalid/no-return pixels are UNKNOWN. Never replace them with max range.
      if(!std::isfinite(d) || d<min_depth_ || d>max_depth_) continue;
      const Eigen::Vector3d ray((double(u)-k[2])*d/k[0],(double(v)-k[5])*d/k[4],d);
      if(ray.norm()>max_depth_) continue;
      const auto p=world_camera*ray;
      rog_map::PointType xyz;xyz.x=p.x();xyz.y=p.y();xyz.z=p.z();xyz.intensity=1;points.push_back(xyz);
    }
    if(points.empty() && !static_cad_only_) return;
    // Seed once before observations so initial rays obey the same three-frame
    // hit/clear rule as every later update (and pending hits are not reset).
    if(!cad_prior_.empty() && !cad_prior_loaded_) {
      const auto count=map_->loadCadPrior(cad_prior_,true);cad_prior_loaded_=true;
      RCLCPP_INFO(get_logger(),"Loaded revisable CAD prior into %zu cells",count);
    }
    map_->integrate(points,{world_camera.translation(),Eigen::Quaterniond(world_camera.linear())});
    if(!static_cad_only_ && launch_radius_>0 && !launch_declared_) {
      const auto count=map_->declareLaunchCylinder(launch_radius_,launch_min_z_,launch_max_z_);
      launch_declared_=true;RCLCPP_WARN(get_logger(),"Applied launch fixture assumption to %zu unknown cells once",count);
    }
    last_depth_ns_=stamp;last_depth_pose_=*pose;++revision_;
    dronehacks_msgs::msg::MapState state;
    state.header.frame_id="map";state.header.stamp=rclcpp::Time(stamp);
    state.localization_epoch=epoch_;state.revision=revision_;
    state.body_pose.position=point(pose->position);
    state.body_pose.orientation.w=pose->orientation.w();
    state.body_pose.orientation.x=pose->orientation.x();
    state.body_pose.orientation.y=pose->orientation.y();
    state.body_pose.orientation.z=pose->orientation.z();
    map_states_->publish(state);
    // Planner shares this map directly; full snapshots are visualization only.
    if(stamp-last_snapshot_ns_<1'000'000'000 || snapshots_->get_subscription_count()==0) return;
    auto msg=snapshot(stamp,*pose);
    snapshots_->publish(msg);last_snapshot_ns_=stamp;
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"ROG snapshot %lu: %u occupied, %u free cells",revision_,msg.occupied.width,msg.free.width);
  }
  dronehacks_msgs::msg::MapSnapshot snapshot(int64_t stamp,const navigation::TimedPose& pose) {
    dronehacks_msgs::msg::MapSnapshot snapshot;
    snapshot.header.stamp=rclcpp::Time(stamp);snapshot.header.frame_id="map";
    snapshot.localization_epoch=epoch_;snapshot.revision=revision_;snapshot.resolution=map_->getResolution();
    Eigen::Vector3d lo=map_->getLocalMapOrigin()-map_->getLocalMapSize()*.5;
    Eigen::Vector3d hi=map_->getLocalMapOrigin()+map_->getLocalMapSize()*.5;
    if(!boundary_lower_.empty()) {
      for(int i=0;i<3;++i){lo[i]=std::max(lo[i],boundary_lower_[i]-.2);hi[i]=std::min(hi[i],boundary_upper_[i]+.2);}
    }
    map_->boundBoxByLocalMap(lo,hi);snapshot.lower=point(lo);snapshot.upper=point(hi);
    super_utils::vec_Vec3f cells;
    map_->boxSearch(lo,hi,super_utils::OCCUPIED,cells);snapshot.occupied=cloud(cells,snapshot.header);
    if(snapshot_include_free_) {map_->boxSearch(lo,hi,super_utils::KNOWN_FREE,cells);snapshot.free=cloud(cells,snapshot.header);}
    snapshot.body_pose.header=snapshot.header;
    snapshot.body_pose.pose.position=point(pose.position);
    auto& q=snapshot.body_pose.pose.orientation;q.w=pose.orientation.w();q.x=pose.orientation.x();q.y=pose.orientation.y();q.z=pose.orientation.z();
    return snapshot;
  }
  bool snapshot_include_free_{true};
  double takeoff_height_{1.0};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  bool internal_grid_route_{false},map_ready_for_route_{false};uint64_t route_sequence_{0};
  rclcpp::Publisher<dronehacks_msgs::msg::RoutePolyline>::SharedPtr route_output_;
  std::string config_;
  double launch_radius_{0},launch_min_z_{.1},launch_max_z_{1.5};bool launch_declared_{false};
  std::shared_ptr<navigation::ObservedMap> map_;
  std::shared_ptr<navigation::PlannerClock> planner_clock_;
  std::string planner_backend_;
  std::unique_ptr<larp_route_follower::FollowerRoutePlanner> planner_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_ready_;
  bool execution_gated_{false},execution_allowed_{false};
  std::optional<navigation::Point> warm_start_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr phase_sub_;
  int64_t last_plan_ns_{0},last_certificate_ns_{0};
  uint64_t trajectory_sequence_{0};
  int64_t max_depth_age_ns_{500'000'000};
  rclcpp::Publisher<dronehacks_msgs::msg::CommittedTrajectory>::SharedPtr trajectories_;
  rclcpp::Subscription<dronehacks_msgs::msg::NavigationGoal>::SharedPtr goals_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  std::vector<double> boundary_lower_,boundary_upper_;
  std::string depth_frame_;
  std::string cad_prior_;bool cad_prior_loaded_{false},static_cad_only_{false};
  navigation::PoseBuffer poses_;
  navigation::TimedPose last_depth_pose_;
  uint64_t rejected_revision_{0};
  Eigen::Isometry3d body_camera_;
  super_utils::RobotState body_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr calibration_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> pending_;
  uint64_t epoch_{1},revision_{0};int stride_{4},ignore_top_rows_{0};double min_depth_{.3},max_depth_{6};
  int64_t last_depth_ns_{0},last_snapshot_ns_{0};bool healthy_{false};
  std::chrono::steady_clock::time_point health_received_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Publisher<dronehacks_msgs::msg::MapSnapshot>::SharedPtr snapshots_;
  rclcpp::Publisher<dronehacks_msgs::msg::MapState>::SharedPtr map_states_;
  rclcpp::Publisher<dronehacks_msgs::msg::MapSnapshot>::SharedPtr rejected_maps_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr planner_timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<DepthMapNode>());}
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
