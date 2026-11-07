#include <navigation/body_odometry.hpp>
#include <iostream>
using namespace navigation;
void require(bool b, const char* s) { if(!b) throw std::runtime_error(s); }
int main() {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=Eigen::Vector3d(.1,0,0);
  t.linear()=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  BodyOdometryTransform adapter(t);
  Eigen::Matrix3d r_gb=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Quaterniond q_gi(r_gb*t.linear());
  Eigen::Vector3d p_gb(3,4,1);
  Eigen::Vector3d p_gi=p_gb+r_gb*t.translation();
  // Rotating around the body origin makes the displaced IMU translate.
  Eigen::Vector3d omega_b(0,0,2);
  Eigen::Vector3d v_i=t.linear().transpose()*omega_b.cross(t.translation());
  auto out=adapter.convert(p_gi,q_gi,v_i,t.linear().transpose()*omega_b,
                           Matrix6::Identity()*.01,Matrix6::Identity()*.01);
  require(out.position.norm()<1e-9,"anchor not at aircraft origin");
  require(out.orientation.angularDistance(Eigen::Quaterniond::Identity())<1e-9,"anchor yaw incorrect");
  require(out.linear_velocity.norm()<1e-9,"uncompensated IMU lever-arm velocity");
  require((out.angular_velocity-omega_b).norm()<1e-9,"angular velocity frame incorrect");
  auto moved=adapter.convert(p_gi+Eigen::Vector3d(0,2,0),q_gi,v_i,
                             t.linear().transpose()*omega_b, Matrix6::Identity()*.01,Matrix6::Identity()*.01);
  require((moved.position-Eigen::Vector3d(2,0,0)).norm()<1e-9,"world translation rotated incorrectly");
  require(std::abs(out.pose_covariance(1,1)-.0101)<1e-9,"orientation uncertainty did not propagate through lever arm");
  require((out.pose_covariance-out.pose_covariance.transpose()).norm()<1e-12,"covariance not symmetric");
  BodyOdometryTransform registered(t,Eigen::Vector3d(1,2,3),M_PI/2);
  auto registered_origin=registered.convert(p_gi,q_gi,v_i,t.linear().transpose()*omega_b,Matrix6::Identity()*.01,Matrix6::Identity()*.01);
  require((registered_origin.position-Eigen::Vector3d(1,2,3)).norm()<1e-9,"explicit map origin incorrect");
  require(registered_origin.orientation.angularDistance(Eigen::Quaterniond(r_gb))<1e-9,"explicit map heading incorrect");
  std::cout << "body origin, heading anchor, lever-arm velocity and covariance passed\n";
}
