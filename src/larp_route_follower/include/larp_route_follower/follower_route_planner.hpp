#pragma once
// Deterministic route follower behind the existing PlannerContract. It reuses
// the observed ROG map, SUPER's global A* route, the whole-curve validator and
// the certified stop from larp_navigation, and replaces trajectory
// optimization with closed-form quintic segments along the route horizon.
#include <navigation/planner_contract.hpp>
#include <navigation/planner_clock.hpp>
#include <navigation/certified_stop.hpp>
#include <navigation/ego_spline.hpp>
#include <navigation/route_guide.hpp>
#include <larp_route_follower/hermite.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace larp_route_follower {
using navigation::Point;
using navigation::Polyline;

struct FollowerSettings {
  double horizon{1.5};        // route look-ahead [m]; also capped by time
  double replan_period{.4};   // keep a live certified curve at least this long [s]
  std::string route_file;     // optional preloaded map-frame polyline (x y z per line)
};

class FollowerRoutePlanner final : public navigation::PlannerContract {
 public:
  FollowerRoutePlanner(const std::string& config, rog_map::ROGMap::Ptr map,
                       std::shared_ptr<navigation::PlannerClock> clock, FollowerSettings settings = {},
                       bool internal_grid_route = true)
      : map_(std::move(map)), clock_(std::move(clock)), limits_(navigation::PlannerLimits::load(config)),
        settings_(std::move(settings)),
        route_(config, map_, clock_, [this](const Point& a, const Point& b) { return lineFree(a, b); }),
        validator_([this](const Point& lo, const Point& hi) { return boxFree(lo, hi); }, limits_.v, limits_.a,
                   limits_.budget),
        yaw_validator_([](const Point&, const Point&) { return true; }, limits_.yaw_rate,
                       limits_.yaw_acceleration, limits_.budget),
        internal_grid_route_(internal_grid_route) {
    const auto root = YAML::LoadFile(config);
    cruise_ = root["traj_opt"]["boundary"]["max_vel"].as<double>();
    accel_ = root["traj_opt"]["boundary"]["max_acc"].as<double>();
    lead_ = root["super_planner"]["replan_forward_dt"].as<double>();
    goal_yaw_enabled_ = root["super_planner"]["goal_yaw_en"].as<bool>(false);
    if (!std::isfinite(lead_) || lead_ <= 0 || lead_ > .2) throw std::invalid_argument("invalid follower handoff lead");
    if (!std::isfinite(settings_.horizon) || settings_.horizon < .3 || settings_.horizon > 4.)
      throw std::invalid_argument("follower horizon must be [0.3,4] m");
    if (!std::isfinite(settings_.replan_period) || settings_.replan_period < .1 || settings_.replan_period > 2.)
      throw std::invalid_argument("follower replan period must be [0.1,2] s");
    if (!settings_.route_file.empty()) preload_ = loadRoute(settings_.route_file);
  }

  void setMapRevision(uint64_t revision) override { revision_ = revision; }
  void setSearchBounds(const Point& lo, const Point& hi) override { route_.setBounds(lo, hi); }
  const navigation::GlobalGridRoute* gridRoute() const override { return &route_; }
  void setFlightEnvelope(const Point& lo, const Point& hi) override {
    if (!lo.allFinite() || !hi.allFinite() || !(lo.array() < hi.array()).all())
      throw std::invalid_argument("invalid flight envelope");
    lower_ = lo;
    upper_ = hi;
  }
  bool setGoal(uint64_t id, const Point& goal, double yaw) override {
    if (clock_fault_ || !id || id <= goal_id_ || !goal.allFinite() || !std::isfinite(yaw)) return false;
    goal_id_ = id;
    goal_ = goal;
    goal_yaw_ = yaw;
    active_ = true;
    route_.clear();
    guide_.clear();
    guide_valid_ = false;
    horizon_scale_ = 1.;
    // A preloaded polyline is only a guide for its own goal, never a different one.
    if (preload_.size() >= 2 && (preload_.back() - goal_).norm() < .3) {
      Polyline points = preload_;
      points.back() = goal_;
      guide_valid_ = guide_.accept({"map", goal_id_, revision_, ++route_sequence_, clock_->getSimTime(), points},
                                   "map", goal_id_, revision_, clock_->getSimTime());
    }
    return true;
  }
  // Test/integration hook: an externally supplied route, as RoutePlanner::accept.
  bool accept(navigation::Route route, uint64_t current_revision) {
    guide_valid_ = guide_.accept(std::move(route), "map", goal_id_, current_revision, clock_->getSimTime());
    return guide_valid_;
  }
  void cancel() override {
    active_ = false;
    position_.clear();
    yaw_.clear();
    braking_ = false;
    route_.clear();
    guide_.clear();
    guide_valid_ = false;
  }
  bool active() const override { return active_; }
  uint64_t goalId() const override { return goal_id_; }
  uint64_t recoveryCount() const override { return recovery_count_; }
  bool braking() const { return braking_; }
  const std::string& lastFailure() const { return last_failure_; }
  geometry_utils::Trajectory position() override { return position_; }
  geometry_utils::Trajectory yaw() override { return yaw_; }

  bool ensureSafeCommand() override {
    if (!validClock() || !active_) return false;
    if (validator_(position_) && yaw_validator_(yaw_)) return true;
    auto stop = navigation::certifiedStop(position_, yaw_, clock_->getSimTime(), validator_);
    if (!stop || !yaw_validator_(stop->yaw)) return false;
    position_ = std::move(stop->position);
    yaw_ = std::move(stop->yaw);
    braking_ = true;
    ++recovery_count_;
    clock_->warn("Follower certified braking recovery");
    return true;
  }

  super_utils::RET_CODE plan() override {
    using namespace super_utils;
    if (!validClock() || !active_) return FAILED;
    const double now = clock_->getSimTime();
    const auto wall = std::chrono::steady_clock::now();
    const auto state = map_->getRobotState();
    if (!state.rcv || !state.p.allFinite() || !state.v.allFinite() || !state.a.allFinite() ||
        !std::isfinite(state.yaw + state.rcv_time) || now - state.rcv_time > .3 || now - state.rcv_time < -.005)
      return FAILED;
    const double end = position_.empty() ? now : position_.start_WT + position_.getTotalDuration();
    if (braking_ && (now < end + .2 || state.v.norm() > .1)) return NO_NEED;
    const bool live = !position_.empty() && now < end && !braking_;
    if (!live && state.v.norm() > .1) return FAILED;
    // Preserve a completed final command so the bridge can report is_final.
    if (!position_.empty() && !braking_ && (position_.getPos(position_.getTotalDuration()) - goal_).norm() < 1e-6 &&
        (state.p - goal_).norm() < .08 && state.v.norm() < .05)
      return NO_NEED;
    // Consistency: keep a live certified curve instead of re-deriving it every tick.
    if (live && now - last_commit_ < settings_.replan_period && end - now > 2 * settings_.replan_period + lead_)
      return NO_NEED;
    if (live && end - now < lead_ + .02) return NO_NEED;
    const double handoff = live ? now + lead_ : now;
    Point p = state.p, v = state.v, a = state.a;
    double y = state.yaw, yv = 0, ya = 0;
    geometry_utils::Trajectory prefix, prefix_yaw;
    if (live) {
      const double at = handoff - position_.start_WT;
      p = position_.getPos(at);
      v = position_.getVel(at);
      a = position_.getAcc(at);
      y = yaw_.getPos(at).x();
      yv = yaw_.getVel(at).x();
      ya = yaw_.getAcc(at).x();
      if (!navigation::polynomialSlice(position_, now - position_.start_WT, at, prefix) ||
          !navigation::polynomialSlice(yaw_, now - yaw_.start_WT, at, prefix_yaw))
        return failed("handoff slice failed");
    }
    // Cap look-ahead by time: a rest-to-rest segment lasts ~1.875 L/cruise, so
    // 8 s of cruise keeps even the slowest retry inside the validator's 30 s limit.
    // Continuous replanning extends the curve long before it decelerates.
    const double horizon = std::min(settings_.horizon * horizon_scale_, cruise_ * 8.);
    Polyline guide = guideFrom(p, horizon, now);
    if (guide.size() < 2) return failed("no certified route section from the current state");
    guide = compact(guide);
    // A guide cut at the edge of observed space can end just inside the body
    // clearance: retry this cycle with the end pulled back, then slower timing.
    std::vector<Polyline> guides{guide};
    for (const double pull : {.1, .25}) {
      Polyline shorter = trimEnd(guides.back(), pull);
      if (shorter.size() >= 2) guides.push_back(shorter);
    }
    for (const auto& [section, scale, corners] : attempts(guides)) {
      {
        geometry_utils::Trajectory position, yaw;
        if (!build(*section, p, v, a, y, yv, ya, handoff, !live, scale, corners, position, yaw)) continue;
        geometry_utils::Trajectory candidate = prefix, candidate_yaw = prefix_yaw;
        candidate.append(position);
        candidate.start_WT = now;
        candidate_yaw.append(yaw);
        candidate_yaw.start_WT = now;
        if (!validator_(candidate)) {
          last_failure_ = "position: " + validator_.failure();
          continue;
        }
        if (!yaw_validator_(candidate_yaw)) {
          last_failure_ = "yaw: " + yaw_validator_.failure();
          continue;
        }
        if (std::abs(candidate.getTotalDuration() - candidate_yaw.getTotalDuration()) > 1e-3) {
          last_failure_ = "yaw duration mismatch";
          continue;
        }
        // A moving handoff must be ready before its preserved prefix expires.
        if (live && std::chrono::duration<double>(std::chrono::steady_clock::now() - wall).count() >= lead_)
          return failed("handoff deadline exceeded");
        position_ = std::move(candidate);
        yaw_ = std::move(candidate_yaw);
        braking_ = false;
        last_commit_ = now;
        horizon_scale_ = std::min(1., horizon_scale_ * 1.25);
        return SUCCESS;
      }
      if (std::chrono::steady_clock::now() - wall > std::chrono::milliseconds(40)) break;
    }
    // Shorter look-ahead next cycle; the previous certified curve is untouched.
    horizon_scale_ = std::max(.25, horizon_scale_ * .5);
    return failed(last_failure_.empty() ? "no candidate passed certification" : last_failure_);
  }

  static Polyline loadRoute(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::invalid_argument("cannot open follower route file " + path);
    Polyline points;
    std::string line;
    while (std::getline(in, line)) {
      const auto hash = line.find('#');
      if (hash != std::string::npos) line.resize(hash);
      std::istringstream row(line);
      double x, y, z;
      if (!(row >> x)) continue;
      if (!(row >> y >> z)) throw std::invalid_argument("follower route rows must be x y z");
      const Point q(x, y, z);
      if (!q.allFinite()) throw std::invalid_argument("non-finite follower route point");
      if (points.empty() || (q - points.back()).norm() > 1e-6) points.push_back(q);
    }
    if (points.size() < 2) throw std::invalid_argument("follower route needs at least two points");
    return points;
  }

 private:
  Polyline guideFrom(const Point& p, double horizon, double now) {
    if ((p - goal_).norm() <= horizon && chordCertified(p, goal_)) return {p, goal_};
    Polyline guide;
    if (guide_valid_) {
      // The preloaded/external route stays valid while the map allows it; the
      // validity predicate truncates it at the first blocked or unobserved span.
      guide_.renew(now);
      guide = guide_.nearby(p, horizon, now, goal_id_, [this](const Point& a, const Point& b) { return lineFree(a, b); });
    }
    const double needed = std::min(.2, (p - goal_).norm());
    if ((guide.size() < 2 || length(guide) < needed) && internal_grid_route_)
      guide = route_.nearby(p, goal_, goal_id_, revision_, horizon);
    return guide;
  }
  // Remove only corners whose replacement chord passes the certifier's own
  // body-radius geometry; the inflated line test alone can be looser.
  Polyline compact(const Polyline& guide) const {
    Polyline out{guide.front()};
    for (size_t i = 0; i + 1 < guide.size();) {
      size_t next = guide.size() - 1;
      while (next > i + 1 && !chordCertified(guide[i], guide[next])) --next;
      out.push_back(guide[next]);
      i = next;
    }
    return out;
  }
  bool build(const Polyline& guide, const Point& p, const Point& v, const Point& a, double y, double yv, double ya,
             double start, bool from_rest, double scale, bool corners, geometry_utils::Trajectory& position,
             geometry_utils::Trajectory& yaw) const {
    Waypoints w(guide.begin(), guide.end());
    w.front() = p;
    // Heading follows the first meaningful travel direction; at the goal it
    // holds (or, when configured, turns to the goal yaw).
    double target = y;
    for (size_t i = 1; i < w.size(); ++i) {
      const Point d = w[i] - w[0];
      if (std::hypot(d.x(), d.y()) > .05) {
        target = std::atan2(d.y(), d.x());
        break;
      }
    }
    if (goal_yaw_enabled_ && (w.back() - goal_).norm() < 1e-6 && length(guide) < .2) target = goal_yaw_;
    const double delta = wrapAngle(target - y);
    const double yaw_rate = .8 * limits_.yaw_rate, yaw_accel = .8 * limits_.yaw_acceleration;
    const double yaw_time = scale * yawDuration(delta, yaw_rate, yaw_accel);
    position.clear();
    position.start_WT = start;
    // From rest with a large heading change: turn toward the route, then go,
    // so the camera observes the direction of travel before the body moves.
    const bool turn_first = from_rest && std::abs(delta) > .5;
    if (turn_first) position.emplace_back(yaw_time, quinticHermite(p, v, a, p, Point::Zero(), Point::Zero(), yaw_time));
    FollowOptions options{cruise_, accel_, scale, corners};
    const auto path = followWaypoints(w, turn_first ? Point::Zero() : v, turn_first ? Point::Zero() : a, start, options);
    if (path.empty()) return false;
    position.append(path);
    position.start_WT = start;
    const double total = position.getTotalDuration();
    yaw.clear();
    yaw.start_WT = start;
    yaw.emplace_back(std::min(yaw_time, total), yawPiece(y, yv, ya, y + delta, std::min(yaw_time, total)));
    if (total - yaw_time > 1e-6) {
      Eigen::MatrixXd hold = Eigen::MatrixXd::Zero(3, 6);
      hold(0, 5) = y + delta;
      yaw.emplace_back(total - yaw_time, hold);
    }
    return position.size() <= 64 && total <= 28.;
  }
  // Ordered attempts: full guide smooth/corner-stop, trimmed guides, then slower.
  static std::vector<std::tuple<const Polyline*, double, bool>> attempts(const std::vector<Polyline>& guides) {
    std::vector<std::tuple<const Polyline*, double, bool>> out;
    for (const double scale : {1., 1.5})
      for (const auto& g : guides)
        for (const bool corners : {false, true}) out.emplace_back(&g, scale, corners);
    return out;
  }
  // Remove the last `pull` metres of a polyline (never past its first segment start).
  static Polyline trimEnd(const Polyline& line, double pull) {
    double total = length(line);
    if (total - pull < .05) return {};
    Polyline out{line.front()};
    double left = total - pull;
    for (size_t i = 1; i < line.size() && left > 1e-9; ++i) {
      const double d = (line[i] - line[i - 1]).norm();
      if (d <= left) {
        out.push_back(line[i]);
        left -= d;
      } else {
        out.push_back(line[i - 1] + (line[i] - line[i - 1]) * (left / d));
        left = 0;
      }
    }
    return out;
  }
  static double length(const Polyline& line) {
    double out = 0;
    for (size_t i = 1; i < line.size(); ++i) out += (line[i] - line[i - 1]).norm();
    return out;
  }
  bool validClock() {
    const double now = clock_->getSimTime();
    if (!std::isfinite(now) || now < last_time_) {
      clock_fault_ = true;
      active_ = false;
      position_.clear();
      yaw_.clear();
    }
    last_time_ = now;
    return !clock_fault_;
  }
  super_utils::RET_CODE failed(const std::string& reason) {
    last_failure_ = reason;
    clock_->warn("Follower candidate rejected: " + reason);
    return super_utils::FAILED;
  }
  bool inside(const Point& p) const {
    return p.allFinite() && (p.array() >= lower_.array()).all() && (p.array() <= upper_.array()).all();
  }
  bool lineFree(const Point& a, const Point& b) const { return inside(a) && inside(b) && map_->isLineFree(a, b, true, true); }
  // Straight chord swept by the certified body sphere, in short boxes.
  bool chordCertified(const Point& a, const Point& b) const {
    if (!lineFree(a, b)) return false;
    const double distance = (b - a).norm();
    const int chunks = std::max(1, int(std::ceil(distance / .15)));
    for (int i = 0; i < chunks; ++i) {
      const Point lo_end = a + (b - a) * (double(i) / chunks), hi_end = a + (b - a) * (double(i + 1) / chunks);
      if (!boxFree(lo_end.cwiseMin(hi_end), lo_end.cwiseMax(hi_end))) return false;
    }
    return true;
  }
  bool boxFree(const Point& lo, const Point& hi) const {
    return navigation::certifiedBoxFree(map_, limits_.radius, lower_, upper_, lo, hi);
  }

  rog_map::ROGMap::Ptr map_;
  std::shared_ptr<navigation::PlannerClock> clock_;
  navigation::PlannerLimits limits_;
  FollowerSettings settings_;
  navigation::GlobalGridRoute route_;
  navigation::RouteGuide guide_;
  navigation::TrajectoryValidator validator_, yaw_validator_;
  geometry_utils::Trajectory position_, yaw_;
  Polyline preload_;
  Point goal_{Point::Zero()}, lower_{Point::Constant(-INFINITY)}, upper_{Point::Constant(INFINITY)};
  double cruise_{.06}, accel_{.04}, lead_{.1}, goal_yaw_{0}, horizon_scale_{1}, last_time_{-INFINITY},
      last_commit_{-INFINITY};
  uint64_t goal_id_{0}, revision_{0}, recovery_count_{0}, route_sequence_{0};
  bool active_{false}, braking_{false}, clock_fault_{false}, goal_yaw_enabled_{false}, guide_valid_{false},
      internal_grid_route_{true};
  std::string last_failure_;
};
}  // namespace larp_route_follower
