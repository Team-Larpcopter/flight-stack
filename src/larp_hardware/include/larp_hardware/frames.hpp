#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
namespace larp_hardware {
inline Eigen::Matrix3d frd_from_flu(){return Eigen::Vector3d(1,-1,-1).asDiagonal();}
// heading: heading of map +X clockwise from true north. Map is launch FLU,
// not ENU; unknown heading must never silently be labeled NED.
inline Eigen::Matrix3d ned_from_map(double heading) {
 if(!std::isfinite(heading))throw std::invalid_argument("unknown map heading");
 return Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ()).toRotationMatrix()*frd_from_flu();
}
struct Px4Frames {
 Eigen::Vector3d position_ned,velocity_body_frd,angular_velocity_body_frd;
 Eigen::Quaterniond body_frd_in_ned;
 Eigen::Matrix3d position_covariance_ned,orientation_covariance_body_frd;
};
inline Px4Frames to_px4(const Eigen::Vector3d& p_map,const Eigen::Quaterniond& q_map_body_flu,
 const Eigen::Vector3d& v_body_flu,const Eigen::Vector3d& w_body_flu,
 const Eigen::Matrix3d& position_cov_map,const Eigen::Matrix3d& orientation_cov_map,
 double heading,const Eigen::Vector3d& origin_ned) {
 if(!p_map.allFinite() || !origin_ned.allFinite() || !v_body_flu.allFinite() || !w_body_flu.allFinite()
    || !q_map_body_flu.coeffs().allFinite() || std::abs(q_map_body_flu.norm()-1)>1e-6
    || !position_cov_map.allFinite() || !orientation_cov_map.allFinite())throw std::invalid_argument("invalid frame input");
 const auto n_m=ned_from_map(heading),b_f=frd_from_flu();
 const Eigen::Matrix3d m_f=q_map_body_flu.toRotationMatrix();
 const Eigen::Matrix3d b_m=b_f*m_f.transpose();
 return {n_m*p_map+origin_ned,b_f*v_body_flu,b_f*w_body_flu,
   Eigen::Quaterniond(n_m*m_f*b_f),n_m*position_cov_map*n_m.transpose(),b_m*orientation_cov_map*b_m.transpose()};
}
}
