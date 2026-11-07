#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace navigation {
using Matrix6 = Eigen::Matrix<double, 6, 6>;
inline Eigen::Matrix3d skew(const Eigen::Vector3d& p) {
  Eigen::Matrix3d s;
  s << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
  return s;
}
struct BodyOdometry {
  Eigen::Vector3d position, linear_velocity, angular_velocity;
  Eigen::Quaterniond orientation;
  Matrix6 pose_covariance, twist_covariance;
};

// T_body_imu maps IMU coordinates to the FLU aircraft body. SqrtVINS's
// orientation error covariance is a LOCAL IMU rotation perturbation, while
// position error is global; its ROS publisher merely reorders these blocks.
class BodyOdometryTransform {
 public:
  explicit BodyOdometryTransform(const Eigen::Isometry3d& body_imu,
      const Eigen::Vector3d& initial_position=Eigen::Vector3d::Zero(), double initial_yaw=0.)
      : body_imu_(body_imu), initial_position_(initial_position),initial_yaw_(initial_yaw) {
    if(!initial_position_.allFinite() || !std::isfinite(initial_yaw_))throw std::invalid_argument("invalid map registration");
    const auto r = body_imu.linear();
    if (!body_imu.matrix().allFinite() || (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()>1e-6 ||
        std::abs(r.determinant()-1)>1e-6) throw std::invalid_argument("invalid IMU extrinsic");
  }
  BodyOdometry convert(const Eigen::Vector3d& p_gi, const Eigen::Quaterniond& q_gi,
                       const Eigen::Vector3d& v_i, const Eigen::Vector3d& w_i,
                       const Matrix6& pose_cov, const Matrix6& twist_cov) {
    if (!p_gi.allFinite() || !v_i.allFinite() || !w_i.allFinite() ||
        !q_gi.coeffs().allFinite() || std::abs(q_gi.norm()-1)>1e-3 ||
        !pose_cov.allFinite() || !twist_cov.allFinite())
      throw std::invalid_argument("invalid IMU odometry");
    const Eigen::Matrix3d r_gi = q_gi.normalized().toRotationMatrix();
    const Eigen::Matrix3d r_bi = body_imu_.linear();
    const Eigen::Vector3d p_bi = body_imu_.translation();
    const Eigen::Matrix3d r_gb = r_gi*r_bi.transpose();
    const Eigen::Vector3d p_gb = p_gi-r_gb*p_bi;
    if (!anchored_) {
      origin_ = p_gb;
      const double yaw = std::atan2(r_gb(1,0), r_gb(0,0));
      r_wg_ = Eigen::AngleAxisd(initial_yaw_-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      anchored_ = true;
    }
    BodyOdometry out;
    out.position = initial_position_+r_wg_*(p_gb-origin_);
    out.orientation = Eigen::Quaterniond(r_wg_*r_gb);
    out.angular_velocity = r_bi*w_i;
    out.linear_velocity = r_bi*v_i-out.angular_velocity.cross(p_bi);
    Matrix6 jp = Matrix6::Zero(), jt = Matrix6::Zero();
    jp.block<3,3>(0,0) = r_wg_;
    jp.block<3,3>(0,3) = r_wg_*r_gi*skew(r_bi.transpose()*p_bi);
    jp.block<3,3>(3,3) = r_wg_*r_gi;
    jt.block<3,3>(0,0) = r_bi;
    jt.block<3,3>(0,3) = skew(p_bi)*r_bi;
    jt.block<3,3>(3,3) = r_bi;
    out.pose_covariance = jp*pose_cov*jp.transpose();
    out.twist_covariance = jt*twist_cov*jt.transpose();
    return out;
  }
 private:
  Eigen::Isometry3d body_imu_;
  Eigen::Matrix3d r_wg_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initial_position_;double initial_yaw_;
  bool anchored_{false};
};
}
