#include <navigation/body_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>

class BodyOdometryNode : public rclcpp::Node {
 public:
  BodyOdometryNode() : Node("body_odometry") {
    world_frame_=declare_parameter<std::string>("world_frame", "map");
    if(world_frame_!="map" && world_frame_!="vio_odom")throw std::invalid_argument("unsupported odometry world frame");
    const auto file=declare_parameter<std::string>("rig_file", "");
    const auto rig=YAML::LoadFile(file);
    Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) t.matrix()(r,c)=rig["T_body_imu"][r][c].as<double>();
    const auto position=declare_parameter<std::vector<double>>("initial_position_m",{0.,0.,0.});
    const auto yaw=declare_parameter<double>("initial_yaw_rad",0.);
    if(position.size()!=3)throw std::invalid_argument("initial position requires three coordinates");
    adapter_=std::make_unique<navigation::BodyOdometryTransform>(t,Eigen::Vector3d(position[0],position[1],position[2]),yaw);
    output_=create_publisher<nav_msgs::msg::Odometry>("/navigation/odometry", 20);
    health_=create_publisher<std_msgs::msg::Bool>("/navigation/vio_healthy", 10);
    pose_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/poseimu", 20,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        if(m->header.frame_id=="global") {
          last_image_ns_=rclcpp::Time(m->header.stamp).nanoseconds(); ++image_updates_;
          last_image_wall_=std::chrono::steady_clock::now();
        }
      });
    input_=create_subscription<nav_msgs::msg::Odometry>("/odomimu", rclcpp::SensorDataQoS().keep_last(100),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ consume(*m); });
    timer_=create_wall_timer(std::chrono::milliseconds(100), [this] {
      std_msgs::msg::Bool status;
      const auto now=get_clock()->now().nanoseconds();
      const auto wall=std::chrono::steady_clock::now();
      // /clock and sensor data arrive on independent DDS readers. Permit at
      // most 5 ms of delivery skew, never arbitrary future or stale samples.
      const auto fresh=[now](int64_t t) { return t>0 && now-t>=-5'000'000 && now-t<300'000'000; };
      status.data=valid_ && fresh(last_output_ns_) && fresh(last_image_ns_) && image_updates_>=10 &&
                  wall-last_output_wall_<std::chrono::seconds(2) && wall-last_image_wall_<std::chrono::seconds(2);
      if(!status.data) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
        "VIO not ready: valid=%d odometry_age_ms=%.3f image_age_ms=%.3f image_updates=%zu",
        valid_, (now-last_output_ns_)*1e-6, (now-last_image_ns_)*1e-6, image_updates_);
      health_->publish(status);
    });
  }
 private:
  void consume(const nav_msgs::msg::Odometry& m) {
    const int64_t stamp=rclcpp::Time(m.header.stamp).nanoseconds();
    if(m.header.frame_id!="global" || m.child_frame_id!="imu") {
      valid_=false; return;
    }
    const auto& p=m.pose.pose.position; const auto& q=m.pose.pose.orientation;
    const auto& v=m.twist.twist.linear; const auto& w=m.twist.twist.angular;
    navigation::Matrix6 cp,ct;
    for(int r=0;r<6;++r) for(int c=0;c<6;++c) {
      cp(r,c)=m.pose.covariance[6*r+c]; ct(r,c)=m.twist.covariance[6*r+c];
    }
    try {
      auto b=adapter_->convert({p.x,p.y,p.z},{q.w,q.x,q.y,q.z},{v.x,v.y,v.z},{w.x,w.y,w.z},cp,ct);
      nav_msgs::msg::Odometry out;
      out.header=m.header; out.header.frame_id=world_frame_; out.child_frame_id="base_link";
      auto& op=out.pose.pose.position; op.x=b.position.x();op.y=b.position.y();op.z=b.position.z();
      auto& oq=out.pose.pose.orientation; oq.w=b.orientation.w();oq.x=b.orientation.x();oq.y=b.orientation.y();oq.z=b.orientation.z();
      auto& ov=out.twist.twist.linear; ov.x=b.linear_velocity.x();ov.y=b.linear_velocity.y();ov.z=b.linear_velocity.z();
      auto& ow=out.twist.twist.angular; ow.x=b.angular_velocity.x();ow.y=b.angular_velocity.y();ow.z=b.angular_velocity.z();
      for(int r=0;r<6;++r) for(int c=0;c<6;++c) {
        out.pose.covariance[6*r+c]=b.pose_covariance(r,c);
        out.twist.covariance[6*r+c]=b.twist_covariance(r,c);
      }
      valid_=b.pose_covariance.diagonal().head<3>().minCoeff()>=0 &&
             b.pose_covariance.diagonal().head<3>().maxCoeff()<2 &&
             b.pose_covariance.diagonal().tail<3>().minCoeff()>=0 &&
             b.pose_covariance.diagonal().tail<3>().maxCoeff()<.5;
      // Covariance health is diagnostic; PX4 decides whether to fuse the sample.
      output_->publish(out);
      last_output_ns_=stamp;
      last_output_wall_=std::chrono::steady_clock::now();
    } catch(const std::exception& e) {
      valid_=false;
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Invalid VIO sample skipped; next sample will be retried: %s",e.what());
    }
  }
  std::string world_frame_;
  std::unique_ptr<navigation::BodyOdometryTransform> adapter_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t last_output_ns_{0},last_image_ns_{0};
  std::chrono::steady_clock::time_point last_output_wall_{},last_image_wall_{};
  size_t image_updates_{0};
  bool valid_{false};
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<BodyOdometryNode>()); }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1; }
  rclcpp::shutdown();return 0;
}
