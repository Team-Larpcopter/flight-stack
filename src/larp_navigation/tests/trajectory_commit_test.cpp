#include <navigation/trajectory_validator.hpp>
#include <data_structure/cmd_traj.h>
#include <iostream>
#include <stdexcept>
using geometry_utils::Trajectory;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
Trajectory curve(double bow=0) {
  // x=10s^3-15s^4+6s^5, y=bow*64*s^3*(1-s)^3; s=t/5.
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,7);
  c(0,1)=6/std::pow(5.,5);c(0,2)=-15/std::pow(5.,4);c(0,3)=10/std::pow(5.,3);
  c(1,0)=-bow*64/std::pow(5.,6);c(1,1)=bow*192/std::pow(5.,5);
  c(1,2)=-bow*192/std::pow(5.,4);c(1,3)=bow*64/std::pow(5.,3);
  Trajectory t;t.emplace_back(5,c);t.start_WT=1;return t;
}
int main() {
  navigation::TrajectoryValidator validator([](const Eigen::Vector3d& lo,const Eigen::Vector3d& hi){
    return !(lo.x()<=.6 && hi.x()>=.4 && lo.y()<=.7 && hi.y()>=.4);
  },1.,2.,100);
  const auto straight=curve();require(validator(straight),"free stopping trajectory rejected");
  navigation::TrajectoryValidator open_space([](const auto&,const auto&){return true;},1.,2.,100);
  require(open_space(curve(.5)),"bowed test curve is dynamically invalid");
  require(!validator(curve(.5)),"accepted curve whose chord is free but polynomial hits obstacle");
  auto invalid=straight;invalid[0]=geometry_utils::Piece(5,Eigen::MatrixXd::Constant(3,6,NAN));
  require(!validator(invalid),"accepted non-finite polynomial");
  super_planner::CmdTraj committed;
  committed.setValidator([&](const Trajectory& p,const Trajectory&){return validator(p);});
  super_planner::ExpTraj exp;exp.setTrajectory(1,straight,straight);
  require(committed.setTrajectory(exp),"valid commit rejected");
  super_planner::ExpTraj bad;bad.setTrajectory(2,curve(.5),straight);
  require(!committed.setTrajectory(bad),"invalid expansion committed");
  require(committed.getStartWallTime()==1 && committed.getPos(2.5).y()==0,"rejection overwrote safe committed path");
  super_planner::BackupTraj backup;backup.setTrajectory(2,99,straight,straight);
  require(!committed.setTrajectory(exp,backup),"invalid backup splice committed");
  require(!committed.backupTrajAvilibale() && committed.getStartWallTime()==1,"failed splice changed commit metadata");
  std::cout<<"continuous curve clearance, stopping tail and atomic commit rejection passed\n";
}
