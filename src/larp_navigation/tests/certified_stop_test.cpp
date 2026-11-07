#include <navigation/certified_stop.hpp>
#include <traj_opt/yaw_traj_opt.h>
#include <iostream>
#include <stdexcept>
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main() {
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,6);
  c(0,0)=6/std::pow(5.,5);c(0,1)=-15/std::pow(5.,4);c(0,2)=10/std::pow(5.,3);c(2,5)=1;
  geometry_utils::Trajectory previous,yaw;previous.emplace_back(5.,c);previous.start_WT=10;
  yaw.emplace_back(5.,Eigen::MatrixXd::Zero(3,6));yaw.start_WT=10;
  navigation::TrajectoryValidator validate([](const auto& lo,const auto& hi){return lo.x()>-.2 && hi.x()<.35;},.6,1.,10);
  require(!validate(previous),"new obstacle did not invalidate old route");
  const auto stop=navigation::certifiedStop(previous,yaw,11.,validate);
  require(bool(stop),"failed to brake before future obstruction");
  const auto& s=stop->position;const double end=s.getTotalDuration();
  require((s.getPos(0)-previous.getPos(1)).norm()<1e-9,"braking position discontinuity");
  require((s.getVel(0)-previous.getVel(1)).norm()<1e-9,"braking velocity discontinuity");
  require((s.getAcc(0)-previous.getAcc(1)).norm()<1e-9,"braking acceleration discontinuity");
  require(s.getVel(end).norm()<1e-8 && s.getAcc(end).norm()<1e-8,"brake does not stop");
  require(validate(s),"braking curve failed full clearance/dynamics certificate");
  Eigen::MatrixXd rotating=Eigen::MatrixXd::Zero(3,6);rotating(0,4)=.2;rotating(0,5)=.3;
  yaw[0]=geometry_utils::Piece(5.,rotating);
  const auto turning_stop=navigation::certifiedStop(previous,yaw,11.,validate);
  require(bool(turning_stop),"bounded yaw-rate braking rejected");
  require(std::abs(turning_stop->yaw.getPos(0).x()-.5)<1e-9 &&
          std::abs(turning_stop->yaw.getVel(0).x()-.2)<1e-9,"yaw handoff is discontinuous");
  require(navigation::boundedStopYaw(turning_stop->yaw),"yaw stop exceeds complete-curve rate bound");
  navigation::TrajectoryValidator no_stop([](const auto&,const auto& hi){return hi.x()<.04;},.6,1.,10);
  require(!navigation::certifiedStop(previous,yaw,11.,no_stop),"accepted stop with unsafe initial position");
  require(!navigation::certifiedStop(previous,yaw,9.,validate),"accepted future initial trajectory");
  // Flight regression: cubic replanning preserved angle/rate but discarded
  // the incoming acceleration. Quintic interpolation must preserve all three.
  traj_opt::YawTrajOpt yaw_opt(1.);
  geometry_utils::Trajectory yaw_splice;
  super_utils::Vec4f yi(-.104327116,-.183946564,-.347978510,0),yf(0,0,0,0);
  auto short_position=navigation::stoppingPolynomial({0,0,1},{.1,0,0},{0,0,0},1.040577365,12.);
  require(yaw_opt.optimize(yi,yf,short_position,yaw_splice,5,false,false),"quintic yaw optimization failed");
  require(std::abs(yaw_splice.getPos(0).x()-yi[0])<1e-9 && std::abs(yaw_splice.getVel(0).x()-yi[1])<1e-9 &&
          std::abs(yaw_splice.getAcc(0).x()-yi[2])<1e-9,"quintic yaw splice lost incoming PVA");
  navigation::TrajectoryValidator yaw_check([](const auto&,const auto&){return true;},1.,2.,10);
  require(yaw_check(yaw_splice),"yaw curve failed full rate/acceleration certificate");
  // A tiny backward connector must not demand a pi-radian turn in <1 s.
  const super_utils::Vec4f resting_yaw(.4,0,0,0);
  auto tiny_backwards=navigation::stoppingPolynomial({0,0,1},{-.04,0,0},{0,0,0},.8,20.);
  require(yaw_opt.optimize(resting_yaw,yf,tiny_backwards,yaw_splice,5,false,true),"tiny connector yaw failed");
  require(yaw_check(yaw_splice) && std::abs(yaw_splice.getPos(.8).x()-.4)<1e-9,
          "free yaw spun toward a centimeter-scale connector");
  Eigen::MatrixXd side=Eigen::MatrixXd::Zero(3,6);side(1,4)=1.;side(2,5)=1.;
  geometry_utils::Trajectory lateral;lateral.emplace_back(.8,side);lateral.start_WT=20.;
  const super_utils::Vec4f moving_yaw(0,.2,.1,0);
  require(yaw_opt.optimize(moving_yaw,yf,lateral,yaw_splice,5,false,true) && yaw_check(yaw_splice),
          "short lateral travel must admit a bounded partial turn");
  require(std::abs(yaw_splice.getVel(0).x()-.2)<1e-9 && std::abs(yaw_splice.getAcc(0).x()-.1)<1e-9,
          "bounded partial turn changed initial yaw derivatives");
  const super_utils::Vec4f far_heading(3.,0,0,0);
  require(!yaw_opt.optimize(moving_yaw,far_heading,lateral,yaw_splice,5,false,false),
          "infeasible explicit terminal heading was silently relaxed");
  auto vertical=navigation::stoppingPolynomial({0,0,1},{0,0,.5},{0,0,0},2.,20.);
  require(yaw_opt.optimize(resting_yaw,yf,vertical,yaw_splice,5,false,true) && yaw_check(yaw_splice) &&
          std::abs(yaw_splice.getPos(2.).x()-.4)<1e-9,"vertical travel fabricated a horizontal yaw");
  require(yaw_opt.optimize(resting_yaw,yf,vertical,yaw_splice,5,false,false) &&
          std::abs(yaw_splice.getPos(2.).x())<1e-9,"explicit goal yaw was discarded");
  std::cout<<"C2 continuous, fully certified braking and infeasible-stop refusal passed\n";
}
