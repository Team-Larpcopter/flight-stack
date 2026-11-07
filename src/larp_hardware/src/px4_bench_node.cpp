// Disarmed end-to-end interface test: no arming or mode-command publisher.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <flight_executive_msgs/msg/trajectory_setpoint.hpp>
#include <larp_hardware/frames.hpp>
#include <larp_hardware/command_gate.hpp>
#include <chrono>
#include <iostream>
using namespace std::chrono_literals;
using Steady=std::chrono::steady_clock;
class Px4Bench:public rclcpp::Node {
 public:
  Px4Bench():Node("px4_bench_boundary") {
    enabled_=declare_parameter<bool>("enable_px4_output",false);
    aligned_=declare_parameter<bool>("alignment_verified",false);
    auto prefix=declare_parameter<std::string>("px4_topic_prefix","/fmu");
    if(prefix.empty() || prefix[0]!='/')throw std::invalid_argument("absolute PX4 prefix required");
    heading_=declare_parameter<double>("map_heading_ned",0.);
    auto origin=declare_parameter<std::vector<double>>("map_origin_ned",{0.,0.,0.});
    if(origin.size()!=3 || !std::isfinite(heading_))throw std::invalid_argument("invalid map alignment");
    origin_={origin[0],origin[1],origin[2]};if(!origin_.allFinite())throw std::invalid_argument("invalid origin");
    auto qos=rclcpp::SensorDataQoS().keep_last(10);
    status_=create_subscription<px4_msgs::msg::VehicleStatus>(prefix+"/out/vehicle_status_v1",qos,[this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m){
      disarmed_=m->arming_state==px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
      status_at_=Steady::now();if(!disarmed_)gate_.clear();});
    health_=create_subscription<std_msgs::msg::Bool>("/autonomy/vio_healthy",qos,[this](std_msgs::msg::Bool::ConstSharedPtr m){healthy_=m->data;health_at_=Steady::now();});
    vision_pub_=create_publisher<px4_msgs::msg::VehicleOdometry>(prefix+"/in/vehicle_visual_odometry",qos);
    command_pub_=create_publisher<px4_msgs::msg::TrajectorySetpoint>(prefix+"/in/trajectory_setpoint",qos);
    odom_=create_subscription<nav_msgs::msg::Odometry>("/autonomy/odometry",qos,[this](nav_msgs::msg::Odometry::ConstSharedPtr m){vision(*m);});
    trajectory_=create_subscription<flight_executive_msgs::msg::TrajectorySetpoint>("/autonomy/shadow/trajectory_in",qos,[this](flight_executive_msgs::msg::TrajectorySetpoint::ConstSharedPtr m){
      gate_.clear();if(!aligned_ || !m->certificate_required || !m->yaw_valid || m->header.frame_id!="local_ned")return;
      auto remaining=rclcpp::Time(m->valid_until).nanoseconds()-now().nanoseconds();
      const auto t=steady_ns();
      gate_.accept({t,t+remaining,{m->position.x,m->position.y,m->position.z},
        {m->velocity.x,m->velocity.y,m->velocity.z},{m->acceleration.x,m->acceleration.y,m->acceleration.z},m->yaw,m->yawspeed},t);
    });
    timer_=create_wall_timer(50ms,[this]{
      if(!admitted() || !aligned_)return;
      auto s=gate_.get(steady_ns(),true);if(!s)return;
      px4_msgs::msg::TrajectorySetpoint out;out.timestamp=now().nanoseconds()/1000;
      for(int i=0;i<3;++i){out.position[i]=s->position[i];out.velocity[i]=s->velocity[i];out.acceleration[i]=s->acceleration[i];out.jerk[i]=NAN;}
      out.yaw=s->yaw;out.yawspeed=s->yaw_rate;command_pub_->publish(out);
    });
    RCLCPP_INFO(get_logger(),"Bench boundary: output=%d; requires fresh DISARMED status; no arming or mode commands",enabled_);
  }
 private:
  static int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Steady::now().time_since_epoch()).count();}
  bool admitted()const{return enabled_ && disarmed_ && healthy_ && Steady::now()-status_at_<1s && Steady::now()-health_at_<300ms;}
  void vision(const nav_msgs::msg::Odometry& m) {
    if(!admitted() || m.header.frame_id!="map" || m.child_frame_id!="base_link")return;
    auto stamp=rclcpp::Time(m.header.stamp).nanoseconds(),age=now().nanoseconds()-stamp;
    if(age< -5'000'000 || age>300'000'000 || stamp<=last_stamp_)return;
    const auto& p=m.pose.pose.position;const auto& q=m.pose.pose.orientation;
    const auto& v=m.twist.twist.linear;const auto& w=m.twist.twist.angular;
    Eigen::Matrix3d cp,cq,cv;
    for(int r=0;r<3;++r)for(int c=0;c<3;++c){cp(r,c)=m.pose.covariance[r*6+c];cq(r,c)=m.pose.covariance[(r+3)*6+c+3];cv(r,c)=m.twist.covariance[r*6+c];}
    try {
      // Without surveyed north alignment, label the fixed vision frame FRD.
      auto f=larp_hardware::to_px4({p.x,p.y,p.z},{q.w,q.x,q.y,q.z},{v.x,v.y,v.z},{w.x,w.y,w.z},cp,cq,aligned_?heading_:0.,aligned_?origin_:Eigen::Vector3d::Zero());
      px4_msgs::msg::VehicleOdometry out;out.timestamp=now().nanoseconds()/1000;out.timestamp_sample=stamp/1000;
      out.pose_frame=aligned_?out.POSE_FRAME_NED:out.POSE_FRAME_FRD;out.velocity_frame=out.VELOCITY_FRAME_BODY_FRD;
      out.q={float(f.body_frd_in_ned.w()),float(f.body_frd_in_ned.x()),float(f.body_frd_in_ned.y()),float(f.body_frd_in_ned.z())};
      const Eigen::Matrix3d vb=larp_hardware::frd_from_flu()*cv*larp_hardware::frd_from_flu();
      for(int i=0;i<3;++i){out.position[i]=f.position_ned[i];out.velocity[i]=f.velocity_body_frd[i];out.angular_velocity[i]=f.angular_velocity_body_frd[i];out.position_variance[i]=f.position_covariance_ned(i,i);out.orientation_variance[i]=f.orientation_covariance_body_frd(i,i);out.velocity_variance[i]=vb(i,i);}
      out.quality=100;vision_pub_->publish(out);last_stamp_=stamp;
    }catch(const std::exception& e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Vision rejected: %s",e.what());}
  }
  bool enabled_=false,aligned_=false,disarmed_=false,healthy_=false;double heading_=0;Eigen::Vector3d origin_;
  int64_t last_stamp_=0;Steady::time_point status_at_,health_at_;larp_hardware::CommandGate gate_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<flight_executive_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vision_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr command_pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Px4Bench>());}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}rclcpp::shutdown();}
