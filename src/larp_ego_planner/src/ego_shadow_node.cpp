#include <ego_planner/planner_manager.h>
#include <larp_ego_planner/snapshot_grid.hpp>
#include <larp_ego_planner/msg/shadow_trajectory.hpp>
#include <dronehacks_msgs/msg/map_state.hpp>
#include <dronehacks_msgs/msg/navigation_goal.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

using Point = Eigen::Vector3d;
using Steady = std::chrono::steady_clock;
using Snapshot = dronehacks_msgs::msg::MapSnapshot;
using State = dronehacks_msgs::msg::MapState;
using Goal = dronehacks_msgs::msg::NavigationGoal;
using Candidate = larp_ego_planner::msg::ShadowTrajectory;

class EgoShadowNode : public rclcpp::Node {
 public:
  EgoShadowNode() : Node("ego_shadow") {
    radius_ = declare_parameter("robot_radius", .4);
    max_age_ = declare_parameter("max_input_age", .3);
    period_ = declare_parameter("replan_period", .2);
    if (!std::isfinite(radius_ + max_age_ + period_) || radius_ <= 0 || radius_ > 1 ||
        max_age_ <= 0 || max_age_ > 1 || period_ < .1 || period_ > 5)
      throw std::invalid_argument("invalid shadow input limits");
    if (get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("EGO shadow currently requires wall-clock inputs");
    output_ = create_publisher<Candidate>("/autonomy/ego/shadow/trajectory", 1);
    path_ = create_publisher<nav_msgs::msg::Path>("/autonomy/ego/shadow/path", 1);
  }
  void start() {
    auto self = shared_from_this();
    map_ = std::make_shared<GridMap>();
    map_->setExternalMap(.1, [this](const Point& p) { return grid_ ? grid_->occupied(p) : 1; });
    planner_.initPlanModules(self, std::make_shared<ego_planner::PlanningVisualization>(self), map_);
    planner_.deliverTrajToOptimizer();
    planner_.setDroneIdtoOpt();
    const auto& p = planner_.pp_;
    if (p.drone_id != 0 || !std::isfinite(p.max_vel_ + p.max_acc_ + p.ctrl_pt_dist + p.planning_horizen_ + p.feasibility_tolerance_) ||
        p.max_vel_ <= 0 || p.max_vel_ > 5 || p.max_acc_ <= 0 || p.max_acc_ > 10 ||
        p.ctrl_pt_dist < .1 || p.ctrl_pt_dist > 1 || p.planning_horizen_ < .5 || p.planning_horizen_ > 10 ||
        p.feasibility_tolerance_ < 0 || p.feasibility_tolerance_ > .5 ||
        get_parameter("optimization/order").as_int() != 3)
      throw std::invalid_argument("invalid EGO configuration; load config/ego.yaml (single drone_id=0)");
    for (const auto* name : {"optimization/lambda_smooth", "optimization/lambda_collision",
         "optimization/lambda_feasibility", "optimization/lambda_fitness", "optimization/dist0",
         "optimization/max_vel", "optimization/max_acc"}) {
      const double v = get_parameter(name).as_double();
      if (!std::isfinite(v) || v <= 0 || v > 100) throw std::invalid_argument("invalid optimization parameter");
    }
    snapshots_ = create_subscription<Snapshot>("/autonomy/map_snapshot", rclcpp::QoS(1).transient_local(),
      [this](Snapshot::ConstSharedPtr msg) {
        if (epoch_ && msg->localization_epoch != epoch_) { epoch_fault_ = true; clear(); return; }
        if (grid_ && msg->revision == revision_) return;
        clear(); grid_.reset(); failed_replans_ = 0;
        try {
          grid_ = std::make_unique<larp_ego_planner::SnapshotGrid>(*msg, radius_);
          epoch_ = msg->localization_epoch; revision_ = msg->revision;
          map_->setExternalMap(grid_->resolution(), [this](const Point& p) { return grid_ ? grid_->occupied(p) : 1; });
        } catch (const std::exception& e) { RCLCPP_WARN(get_logger(), "Rejected map: %s", e.what()); }
      });
    states_ = create_subscription<State>("/autonomy/map_state", rclcpp::SensorDataQoS().keep_last(1),
      [this](State::ConstSharedPtr msg) {
        if (epoch_ && msg->localization_epoch != epoch_) epoch_fault_ = true;
        state_ = msg; state_received_ = Steady::now();
      });
    odom_ = create_subscription<nav_msgs::msg::Odometry>("/autonomy/odometry", rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        body_ = msg; odom_received_ = Steady::now();
      });
    health_ = create_subscription<std_msgs::msg::Bool>("/autonomy/vio_healthy", 10,
      [this](const std_msgs::msg::Bool& msg) { healthy_ = msg.data; health_received_ = Steady::now(); });
    goals_ = create_subscription<Goal>("/autonomy/goal", rclcpp::QoS(1).transient_local(),
      [this](Goal::ConstSharedPtr msg) {
        if (msg->header.frame_id != "map" || !msg->goal_id || !msg->localization_epoch ||
            (epoch_ && msg->localization_epoch != epoch_) || msg->goal_id < goal_id_) return;
        if (msg->cancel) { goal_id_ = msg->goal_id; goal_.reset(); clear(); return; }
        if (msg->goal_id <= goal_id_ || !point(msg->pose.position).allFinite()) return;
        goal_id_ = msg->goal_id; goal_ = msg; failed_replans_ = 0; clear();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "EGO shadow ready: static map leases required; diagnostic output only");
  }
 private:
  static Point point(const geometry_msgs::msg::Point& p) { return {p.x, p.y, p.z}; }
  static geometry_msgs::msg::Point message(const Point& p) {
    geometry_msgs::msg::Point m; m.x = p.x(); m.y = p.y(); m.z = p.z(); return m;
  }
  bool fresh(const builtin_interfaces::msg::Time& stamp, Steady::time_point receipt) {
    const auto age = now().seconds() - rclcpp::Time(stamp).seconds();
    return age >= -.005 && age <= max_age_ &&
      std::chrono::duration<double>(Steady::now() - receipt).count() <= max_age_;
  }
  bool ready() {
    return !epoch_fault_ && grid_ && goal_ && state_ && body_ && healthy_ &&
      goal_->localization_epoch == epoch_ && state_->localization_epoch == epoch_ &&
      state_->revision == revision_ && state_->header.frame_id == "map" &&
      body_->header.frame_id == "map" && body_->child_frame_id == "base_link" &&
      fresh(state_->header.stamp, state_received_) && fresh(body_->header.stamp, odom_received_) &&
      std::chrono::duration<double>(Steady::now() - health_received_).count() <= max_age_;
  }
  Candidate candidate() {
    Candidate out; out.header.frame_id = "map"; out.header.stamp = now();
    out.localization_epoch = epoch_; out.map_revision = revision_; out.goal_id = goal_id_;
    return out;
  }
  void clear() {
    output_->publish(candidate());
    nav_msgs::msg::Path path; path.header.frame_id = "map"; path.header.stamp = now(); path_->publish(path);
  }
  void tick() {
    if (!ready()) { clear(); return; }
    if (std::chrono::duration<double>(Steady::now() - last_plan_).count() < period_) return;
    last_plan_ = Steady::now();
    const Point start = point(body_->pose.pose.position), end = point(goal_->pose.position);
    const auto& q = body_->pose.pose.orientation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const auto& v = body_->twist.twist.linear;
    const Point velocity = rotation * Point(v.x, v.y, v.z);
    if (!std::isfinite(rotation.squaredNorm()) || std::abs(rotation.squaredNorm() - 1) > .001 ||
        !velocity.allFinite() || velocity.norm() > 2 * planner_.pp_.max_vel_ ||
        grid_->occupied(start) || grid_->occupied(end)) { clear(); return; }
    Point target = end;
    const double distance = (end - start).norm();
    if (distance < .2) { clear(); return; }
    if (distance > planner_.pp_.planning_horizen_)
      target = start + (end - start) * (planner_.pp_.planning_horizen_ / distance);
    // Shadow replans begin at measured state. No simulated execution of old
    // candidates and no controller, swarm peer, yaw or mission-completion logic.
    // As in the upstream FSM, a failed polynomial seed gets a randomized seed
    // on a later tick. Keep one optimization attempt per tick. The upstream
    // random construction needs a nonvertical direction for its cross product.
    const bool random_seed = failed_replans_ > 0 && (target - start).head<2>().norm() > 1e-3;
    if (grid_->occupied(target) || !planner_.reboundReplan(start, velocity, Point::Zero(),
        target, Point::Zero(), true, random_seed) || !ready()) { ++failed_replans_; clear(); return; }
    auto& curve = planner_.local_data_.position_traj_;
    const double duration = curve.getTimeSum();
    if (!std::isfinite(duration) || duration <= 0 || duration > 60) { clear(); return; }
    auto out = candidate(); out.order = 3;
    const auto controls = curve.getControlPoint(); const auto knots = curve.getKnot();
    if (!controls.allFinite() || !knots.allFinite()) { clear(); return; }
    for (int i = 0; i < controls.cols(); ++i) out.control_points.push_back(message(controls.col(i)));
    out.knots.assign(knots.data(), knots.data() + knots.size());
    nav_msgs::msg::Path path; path.header = out.header;
    const int samples = static_cast<int>(std::ceil(duration / .02));
    for (int i = 0; i <= samples; ++i) {
      const double t = duration * i / samples;
      const auto p = curve.evaluateDeBoorT(t);
      if (grid_->occupied(p)) { ++failed_replans_; clear(); return; }
      geometry_msgs::msg::PoseStamped pose; pose.header = path.header;
      pose.header.stamp = rclcpp::Time(out.header.stamp) + rclcpp::Duration::from_seconds(t);
      pose.pose.position = message(p); pose.pose.orientation.w = 1.; path.poses.push_back(pose);
    }
    // Sampling is a diagnostic rejection check, not a continuous certificate.
    if (!ready()) { clear(); return; }
    failed_replans_ = 0;
    out.valid = true; output_->publish(out); path_->publish(path);
  }
  double radius_, max_age_, period_;
  uint64_t epoch_{0}, revision_{0}, goal_id_{0};
  bool healthy_{false}, epoch_fault_{false};
  uint64_t failed_replans_{0};
  std::unique_ptr<larp_ego_planner::SnapshotGrid> grid_;
  GridMap::Ptr map_;
  ego_planner::EGOPlannerManager planner_;
  Goal::ConstSharedPtr goal_; State::ConstSharedPtr state_;
  nav_msgs::msg::Odometry::ConstSharedPtr body_;
  Steady::time_point state_received_{}, odom_received_{}, health_received_{}, last_plan_{};
  rclcpp::Publisher<Candidate>::SharedPtr output_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Subscription<Snapshot>::SharedPtr snapshots_;
  rclcpp::Subscription<State>::SharedPtr states_;
  rclcpp::Subscription<Goal>::SharedPtr goals_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EgoShadowNode>(); node->start(); rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ego_shadow"), "%s", e.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
