#pragma once
#include <ros_interface/ros_interface.hpp>
#include <functional>
#include <stdexcept>

namespace navigation {
// SUPER's portable interface uses the caller's clock. It never publishes /clock.
// Optional visualization is supplied by the owner, outside the optimization loop.
class PlannerClock final:public ros_interface::RosInterface {
 public:
  using Trajectory=geometry_utils::Trajectory;
  using Polytope=geometry_utils::Polytope;
  using PolytopeVec=geometry_utils::PolytopeVec;
  using Ellipsoid=geometry_utils::Ellipsoid;
  using Vec3f=super_utils::Vec3f;
  using Points=super_utils::vec_Vec3f;
  explicit PlannerClock(std::function<double()> clock,std::function<void(const std::string&)> log={})
    :clock_(std::move(clock)),log_(std::move(log)){}
  void debug(const std::string&) override {}
  void info(const std::string&) override {}
  void warn(const std::string& m) override {if(log_)log_(m);}
  void error(const std::string& m) override {if(log_)log_(m);}
  void fatal(const std::string& m) override {throw std::runtime_error(m);}
  void setSimTime(const double&) override {throw std::logic_error("planner cannot change the authoritative clock");}
  double getSimTime() override {return clock_();}
  void getSimTime(int32_t& sec,uint32_t& nsec) override {
    const int64_t ns=std::llround(clock_()*1e9);sec=ns/1000000000;nsec=ns%1000000000;
  }
  Points latest_frontend;
  void vizExpTraj(const Trajectory&,const std::string&) override {}
  void vizBackupTraj(const Trajectory&) override {}
  void vizFrontendPath(const Points& p) override {latest_frontend=p;}
  void vizExpSfc(const PolytopeVec&) override {}
  void vizBackupSfc(const Polytope&) override {}
  void vizGoalPath(const Points&) override {}
  void vizCommittedTraj(const Trajectory&,const double&) override {}
  void vizYawTraj(const Trajectory&,const Trajectory&) override {}
  void vizAstarBoundingBox(const Vec3f&,const Vec3f&) override {}
  void vizAstarPoints(const Vec3f&,const ::Color&,const std::string&,const double&,const int&) override {}
  void vizReplanLog(const Trajectory&,const Trajectory&,const Trajectory&,const Trajectory&,
    const PolytopeVec&,const Polytope&,const Points&,const int&) override {}
  void vizCiriSeedLine(const Vec3f&,const Vec3f&,const double&) override {}
  void vizCiriEllipsoid(const Ellipsoid&) override {}
  void vizCiriInfeasiblePoint(const Vec3f) override {}
  void vizCiriPolytope(const Polytope&,const std::string&) override {}
  void vizCiriPointCloud(const Points&) override {}
 private:
  std::function<double()> clock_;std::function<void(const std::string&)> log_;
};
}
