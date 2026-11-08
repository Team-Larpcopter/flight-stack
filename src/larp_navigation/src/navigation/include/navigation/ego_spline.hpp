#pragma once
#include <data_structure/base/trajectory.h>
#include <stdexcept>

namespace navigation {
// SUPER's native slicing helper only supports degrees 5 and 7. Shift each
// polynomial's time origin explicitly so cubic EGO prefixes remain exact too.
inline bool polynomialSlice(const geometry_utils::Trajectory& source,double begin,double end,
    geometry_utils::Trajectory& out) {
  out.clear();out.start_WT=source.start_WT+begin;
  if(!std::isfinite(begin+end) || begin<0 || end<=begin || end>source.getTotalDuration()+1e-8)return false;
  auto choose=[](int n,int k){double value=1.;for(int i=1;i<=k;++i)value=value*(n-i+1)/i;return value;};
  double elapsed=0;
  for(const auto& piece:source) {
    const double lo=std::max(0.,begin-elapsed),hi=std::min(piece.getDuration(),end-elapsed);
    if(hi>lo+1e-9) {
      const auto& c=piece.getCoeffMat();const int n=c.cols()-1;
      if(n<0 || n>9 || !c.allFinite())return false;
      Eigen::MatrixXd shifted=Eigen::MatrixXd::Zero(3,n+1);
      for(int k=0;k<=n;++k)for(int j=k;j<=n;++j)
        shifted.col(n-k)+=c.col(n-j)*choose(j,k)*std::pow(lo,j-k);
      out.emplace_back(hi-lo,shifted);
    }
    elapsed+=piece.getDuration();
  }
  return !out.empty() && std::abs(out.getTotalDuration()-(end-begin))<1e-8;
}
// Exact uniform cubic B-spline to polynomial conversion. EGO's endpoint cost
// is soft: impose the command boundary conditions before independent validation.
inline geometry_utils::Trajectory egoPolynomial(Eigen::MatrixXd controls,double h,
    const Eigen::Vector3d& p,const Eigen::Vector3d& v,const Eigen::Vector3d& a,
    const Eigen::Vector3d& goal,double start) {
  if(controls.rows()!=3 || controls.cols()<6 || controls.cols()>67 || !controls.allFinite() ||
     !std::isfinite(h) || h<=0 || !p.allFinite() || !v.allFinite() || !a.allFinite() || !goal.allFinite())
    throw std::invalid_argument("invalid EGO spline");
  controls.col(0)=p-h*v+h*h*a/3.;
  controls.col(1)=p-h*h*a/6.;
  controls.col(2)=p+h*v+h*h*a/3.;
  for(int j=controls.cols()-3;j<controls.cols();++j)controls.col(j)=goal;
  geometry_utils::Trajectory out;out.start_WT=start;
  for(int i=0;i<controls.cols()-3;++i) {
    const Eigen::Vector3d q0=controls.col(i),q1=controls.col(i+1),q2=controls.col(i+2),q3=controls.col(i+3);
    Eigen::MatrixXd c(3,4);
    c.col(0)=(-q0+3*q1-3*q2+q3)/(6*h*h*h);
    c.col(1)=(q0-2*q1+q2)/(2*h*h);
    c.col(2)=(-q0+q2)/(2*h);
    c.col(3)=(q0+4*q1+q2)/6.;
    out.emplace_back(h,c);
  }
  return out;
}
inline geometry_utils::Trajectory terminalCapture(const Eigen::Vector3d& p,const Eigen::Vector3d& v,
    const Eigen::Vector3d& a,const Eigen::Vector3d& goal,double t,double start) {
  const Eigen::Vector3d d=goal-p-t*v-.5*t*t*a,dv=-v-t*a,da=-a;
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,6);
  c.col(5)=p;c.col(4)=v;c.col(3)=.5*a;
  c.col(2)=10*d/std::pow(t,3)-4*dv/(t*t)+.5*da/t;
  c.col(1)=-15*d/std::pow(t,4)+7*dv/std::pow(t,3)-da/(t*t);
  c.col(0)=6*d/std::pow(t,5)-3*dv/std::pow(t,4)+.5*da/std::pow(t,3);
  geometry_utils::Trajectory out;out.start_WT=start;out.emplace_back(t,c);return out;
}
}
