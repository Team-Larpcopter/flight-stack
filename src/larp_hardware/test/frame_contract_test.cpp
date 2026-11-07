#include <larp_hardware/frames.hpp>
#include "utils/quat_ops.h"
#include <cassert>
using namespace larp_hardware;
int main(){
 const auto I=Eigen::Matrix3d::Identity();
 // sqrtVINS's JPL global->IMU numbers, interpreted as a ROS Hamilton
 // quaternion, represent IMU->global. Applying another inverse is wrong.
 for(int axis=0;axis<3;++axis){
  auto physical=Eigen::AngleAxisd(.7,Eigen::Vector3d::Unit(axis)).toRotationMatrix();
  auto jpl=ov_core::rot_2_quat(physical.transpose());
  Eigen::Quaterniond ros(jpl(3),jpl(0),jpl(1),jpl(2));
  assert((ros.toRotationMatrix()-physical).norm()<1e-12);
  assert((ov_core::quat_2_Rot(jpl)-physical.transpose()).norm()<1e-12);
 }
 auto q=Eigen::Quaterniond::Identity();
 auto p=to_px4({1,2,3},q,{4,5,6},{7,8,9},I,I,0,{10,20,30});
 assert((p.position_ned-Eigen::Vector3d(11,18,27)).norm()<1e-12);
 assert((p.velocity_body_frd-Eigen::Vector3d(4,-5,-6)).norm()<1e-12);
 assert((p.angular_velocity_body_frd-Eigen::Vector3d(7,-8,-9)).norm()<1e-12);
 assert(p.body_frd_in_ned.angularDistance(q)<1e-12);
 auto yaw=Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
 auto c=Eigen::Vector3d(1,2,3).asDiagonal().toDenseMatrix();
 auto turned=to_px4({1,0,0},yaw,{1,0,0},{0,0,1},c,c,M_PI/2,{0,0,0});
 assert((turned.position_ned-Eigen::Vector3d(0,1,0)).norm()<1e-12);
 assert(turned.body_frd_in_ned.angularDistance(q)<1e-12);
 assert(std::abs(turned.orientation_covariance_body_frd(0,0)-2)<1e-12);
 assert(std::abs(turned.position_covariance_ned(0,0)-2)<1e-12);
 // Noncommuting roll/pitch/yaw checks expose incorrect quaternion order.
 auto attitude=Eigen::Quaterniond(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-.3,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()));
 auto mixed=to_px4({0,0,0},attitude,{0,0,0},{0,0,0},c,c,.8,{0,0,0});
 for(int axis=0;axis<3;++axis){auto v=Eigen::Vector3d::Unit(axis);assert((mixed.body_frd_in_ned*v-ned_from_map(.8)*(attitude*(frd_from_flu()*v))).norm()<1e-12);}
 bool rejected=false;try{ned_from_map(NAN);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
}
