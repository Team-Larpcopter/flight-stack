#include <navigation/heading_alignment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>

class EvSourceNode:public rclcpp::Node {
 public:
  EvSourceNode():Node("ev_source") {
    const double heading=declare_parameter<double>("heading_enu_rad",NAN);
    const auto reference=declare_parameter<std::string>("heading_reference","");
    if(reference.empty())throw std::invalid_argument("heading_reference must identify the surveyed launch heading");
    alignment_=std::make_unique<navigation::HeadingAlignment>(heading);
    RCLCPP_INFO(get_logger(),"ENU heading %.6f rad from declared reference: %s",heading,reference.c_str());
    output_=create_publisher<nav_msgs::msg::Odometry>("/navigation/odometry_enu",rclcpp::SensorDataQoS());
    health_=create_subscription<std_msgs::msg::Bool>("/navigation/vio_healthy",10,[this](const std_msgs::msg::Bool& m){
      healthy_=m.data;health_receipt_=std::chrono::steady_clock::now();
    });
    input_=create_subscription<nav_msgs::msg::Odometry>("/navigation/odometry",rclcpp::SensorDataQoS().keep_last(10),[this](const nav_msgs::msg::Odometry& m){
      const auto stamp=rclcpp::Time(m.header.stamp).nanoseconds(),age=now().nanoseconds()-stamp;
      if(!healthy_ || std::chrono::steady_clock::now()-health_receipt_>std::chrono::seconds(1) ||
         m.header.frame_id!="map" || m.child_frame_id!="base_link" || age< -5'000'000 || age>300'000'000 ||
         stamp-last_stamp_<33'333'333) return;
      const auto& p=m.pose.pose.position;const auto& q=m.pose.pose.orientation;
      const Eigen::Vector3d position(p.x,p.y,p.z);const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      navigation::Matrix6 cp;
      for(int r=0;r<6;++r)for(int c=0;c<6;++c)cp(r,c)=m.pose.covariance[r*6+c];
      if(!position.allFinite() || !rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>.001 || !cp.allFinite()) return;
      auto out=m;out.header.frame_id="enu";
      const auto enu_p=alignment_->position(position);const auto enu_q=alignment_->orientation(rotation);const auto covariance=alignment_->covariance(cp);
      auto& op=out.pose.pose.position;op.x=enu_p.x();op.y=enu_p.y();op.z=enu_p.z();
      auto& oq=out.pose.pose.orientation;oq.w=enu_q.w();oq.x=enu_q.x();oq.y=enu_q.y();oq.z=enu_q.z();
      for(int r=0;r<6;++r)for(int c=0;c<6;++c)out.pose.covariance[r*6+c]=covariance(r,c);
      // The twist is already in base_link. Rotating world axes must not rotate it.
      output_->publish(out);last_stamp_=stamp;
    });
  }
 private:
  std::unique_ptr<navigation::HeadingAlignment> alignment_;
  bool healthy_{false};int64_t last_stamp_{0};std::chrono::steady_clock::time_point health_receipt_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<EvSourceNode>());}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
