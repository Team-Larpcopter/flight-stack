#pragma once
#include <navigation/trajectory_validator.hpp>
#include <optional>

namespace navigation {
struct StopCommand {geometry_utils::Trajectory position,yaw;};

// Quintic Hermite stop, continuous in position, velocity and acceleration at
// handoff. It is a candidate only: the complete curve must pass certification.
inline geometry_utils::Trajectory stoppingPolynomial(const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,const Eigen::Vector3d& a,double duration,double now) {
  const double t=duration;
  const Eigen::Vector3d end=p+.5*t*v+(t*t/12.)*a;
  const Eigen::Vector3d d=end-p-t*v-.5*t*t*a,dv=-v-t*a,da=-a;
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,6);
  c.col(5)=p;c.col(4)=v;c.col(3)=.5*a;
  c.col(2)=10*d/std::pow(t,3)-4*dv/(t*t)+.5*da/t;
  c.col(1)=-15*d/std::pow(t,4)+7*dv/std::pow(t,3)-da/(t*t);
  c.col(0)=6*d/std::pow(t,5)-3*dv/std::pow(t,4)+.5*da/std::pow(t,3);
  geometry_utils::Trajectory out;out.emplace_back(t,c);out.start_WT=now;return out;
}

inline bool boundedStopYaw(const geometry_utils::Trajectory& trajectory) {
  const auto& piece=trajectory[0];const auto& c=piece.getCoeffMat();const double t=piece.getDuration();
  Eigen::Matrix<double,3,6> b;
  auto choose=[](int n,int k){double r=1;for(int i=1;i<=k;++i)r=r*(n-i+1)/i;return r;};
  for(int i=0;i<=5;++i) {
    b.col(i).setZero();
    for(int k=0;k<=i;++k)b.col(i)+=c.col(5-k)*std::pow(t,k)*choose(i,k)/choose(5,k);
  }
  if(!b.allFinite())return false;
  for(int i=0;i<5;++i)if((5*(b.col(i+1)-b.col(i))/t).norm()>1.)return false;
  return piece.getVel(t).norm()<1e-5 && piece.getAcc(t).norm()<1e-5;
}

inline std::optional<StopCommand> certifiedStop(const geometry_utils::Trajectory& previous,
    const geometry_utils::Trajectory& previous_yaw,double now,TrajectoryValidator& validate) {
  if(previous.empty() || previous_yaw.empty() || !std::isfinite(now) ||
     now<previous.start_WT || std::abs(previous.start_WT-previous_yaw.start_WT)>1e-5 ||
     std::abs(previous.getTotalDuration()-previous_yaw.getTotalDuration())>1e-3)return {};
  const double at=std::min(now-previous.start_WT,previous.getTotalDuration());
  const auto p=previous.getPos(at),v=previous.getVel(at),a=previous.getAcc(at);
  const auto y=previous_yaw.getPos(at),yv=previous_yaw.getVel(at),ya=previous_yaw.getAcc(at);
  if(!p.allFinite() || !v.allFinite() || !a.allFinite() || !y.allFinite() || !yv.allFinite() || !ya.allFinite())return {};
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(25);
  for(double duration:{.5,.75,1.,1.25,1.5,2.}) {
    if(std::chrono::steady_clock::now()>deadline)break;
    StopCommand stop{stoppingPolynomial(p,v,a,duration,now),stoppingPolynomial(y,yv,ya,duration,now)};
    if(boundedStopYaw(stop.yaw) && validate(stop.position) && std::chrono::steady_clock::now()<=deadline)return stop;
  }
  return {};
}
}
