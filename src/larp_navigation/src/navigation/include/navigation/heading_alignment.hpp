#pragma once
#include <navigation/body_odometry.hpp>

namespace navigation {
// The launch-frame X axis has a DECLARED heading in ENU. This is an external
// calibration, never estimated from simulator truth or relabeled as north.
class HeadingAlignment {
 public:
  explicit HeadingAlignment(double heading) {
    if(!std::isfinite(heading))throw std::invalid_argument("explicit ENU heading calibration required");
    q_=Eigen::Quaterniond(Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()));
    jacobian_.setZero();jacobian_.topLeftCorner<3,3>()=q_.toRotationMatrix();jacobian_.bottomRightCorner<3,3>()=q_.toRotationMatrix();
  }
  Eigen::Vector3d position(const Eigen::Vector3d& p)const{return q_*p;}
  Eigen::Quaterniond orientation(const Eigen::Quaterniond& q)const{return q_*q;}
  Matrix6 covariance(const Matrix6& p)const{return jacobian_*p*jacobian_.transpose();}
 private:
  Eigen::Quaterniond q_;Matrix6 jacobian_;
};
}
