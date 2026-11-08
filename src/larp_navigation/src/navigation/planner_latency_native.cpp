// Passive DDS receipt observer. No publishers, sensors, or vehicle connections.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <dronehacks_msgs/msg/map_state.hpp>
#include <dronehacks_msgs/msg/route_polyline.hpp>
#include <dronehacks_msgs/msg/committed_trajectory.hpp>
#include <flight_executive_msgs/msg/trajectory_setpoint.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <thread>
#include <vector>
using Clock = std::chrono::steady_clock;
struct Stats {
  size_t received=0, invalid=0;
  Clock::time_point last{};
  int64_t last_stamp=0;
  std::vector<double> ages, gaps, stamp_gaps;
};
class Observer : public rclcpp::Node {
 public:
  Observer():Node("planner_latency_native") {
    duration=declare_parameter<double>("duration_seconds",30.);
    const auto warmup=declare_parameter<double>("warmup_seconds",2.);
    if(!std::isfinite(duration+warmup) || duration<=0 || duration>600 || warmup<0 || warmup>30)
      throw std::invalid_argument("duration must be (0,600], warmup [0,30] seconds");
    ready=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(warmup));
    watch<sensor_msgs::msg::Imu>("/autonomy/imu/data");
    watch<sensor_msgs::msg::CameraInfo>("/native_oak/left/camera_info");
    watch<sensor_msgs::msg::CameraInfo>("/native_oak/right/camera_info");
    watch<nav_msgs::msg::Odometry>("/autonomy/odomimu");
    watch<nav_msgs::msg::Odometry>("/autonomy/odometry");
    watch<geometry_msgs::msg::PoseWithCovarianceStamped>("/autonomy/poseimu");
    watch<dronehacks_msgs::msg::MapState>("/autonomy/map_state");
    watch<dronehacks_msgs::msg::RoutePolyline>("/autonomy/route",[](const auto& m){return m.status==m.VALID;});
    watch<dronehacks_msgs::msg::CommittedTrajectory>("/autonomy/committed_trajectory",[](const auto& m){return m.status==m.VALID;});
    watch<flight_executive_msgs::msg::TrajectorySetpoint>(declare_parameter<std::string>("trajectory_topic","/autonomy/shadow/trajectory_in"));
  }
  template<class T> void watch(const std::string& topic) {watch<T>(topic,[](const T&){return true;});}
  template<class T,class Valid> void watch(const std::string& topic,Valid valid) {
    stats.try_emplace(topic);
    subscriptions.push_back(create_subscription<T>(topic,rclcpp::SensorDataQoS().keep_last(2000),
      [this,topic,valid](typename T::ConstSharedPtr m){
        const auto received=Clock::now();if(received<ready)return;
        auto& s=stats.at(topic);++s.received;
        if(!valid(*m)){++s.invalid;return;}
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if(!s.ages.empty()) {
          s.gaps.push_back(std::chrono::duration<double,std::milli>(received-s.last).count());
          s.stamp_gaps.push_back((stamp-s.last_stamp)*1e-6);
        }
        s.last=received;s.last_stamp=stamp;
        s.ages.push_back((now().nanoseconds()-stamp)*1e-6);
      }));
  }
  void report() {
    const auto ended=Clock::now();
    const double elapsed=std::max(0.,std::chrono::duration<double>(ended-ready).count());
    auto percentile=[](std::vector<double> v,double p){
      if(v.empty())return std::string("null");
      std::sort(v.begin(),v.end());return std::to_string(v[size_t((v.size()-1)*p)]);
    };
    std::cout<<"{\"measurement\":\"ROS header age at native DDS receipt; not causal end-to-end latency\",\"duration_seconds\":"<<elapsed<<",\"topics\":{";
    bool comma=false;
    for(const auto& [topic,s]:stats) {
      if(comma)std::cout<<',';comma=true;
      double gap=s.ages.empty()?elapsed*1000:std::max(
        std::chrono::duration<double,std::milli>(ended-s.last).count(),
        std::chrono::duration<double,std::milli>(s.last-ready).count());
      // Interior gaps plus both silent edges of the observation window.
      if(!s.ages.empty()) {
        const double interior=std::accumulate(s.gaps.begin(),s.gaps.end(),0.);
        gap=std::max(std::chrono::duration<double,std::milli>(ended-s.last).count(),
          std::chrono::duration<double,std::milli>(s.last-ready).count()-interior);
        for(auto g:s.gaps)gap=std::max(gap,g);
      }
      std::cout<<std::quoted(topic)<<":{\"received_count\":"<<s.received<<",\"invalid_count\":"<<s.invalid
        <<",\"received_hz\":"<<(elapsed>0?s.received/elapsed:0.)
        <<",\"valid_hz\":"<<(elapsed>0?s.ages.size()/elapsed:0.)
        <<",\"age_ms_p50\":"<<percentile(s.ages,.5)<<",\"age_ms_p95\":"<<percentile(s.ages,.95)
        <<",\"age_ms_p99\":"<<percentile(s.ages,.99)<<",\"age_ms_max\":"<<percentile(s.ages,1)
        <<",\"max_valid_receipt_gap_including_silence_ms\":"<<gap
        <<",\"max_stamp_gap_ms\":"<<percentile(s.stamp_gaps,1)<<'}';
    }
    std::cout<<"}}\n";
  }
  double duration;
  Clock::time_point ready;
 private:
  std::map<std::string,Stats> stats;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<Observer>();
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    const auto end=node->ready+std::chrono::duration<double>(node->duration);
    while(rclcpp::ok() && Clock::now()<end) {
      executor.spin_some(std::chrono::milliseconds(2));
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    node->report();rclcpp::shutdown();return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
