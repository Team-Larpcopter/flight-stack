#pragma once
#include <Eigen/Core>
#include <cmath>
namespace navigation {
// Exact separation of a sphere swept over a centre-position AABB from a full
// occupied/unknown voxel. Touching is rejected. Radius is physical body plus
// the declared uncertainty allowance; grid resolution is geometry, not margin.
inline bool sphereBoxClear(const Eigen::Vector3d& lower,const Eigen::Vector3d& upper,
                           const Eigen::Vector3d& voxel,double resolution,double radius) {
  if(!lower.allFinite() || !upper.allFinite() || !voxel.allFinite() ||
     !std::isfinite(resolution+radius) || resolution<=0 || radius<0 ||
     (lower.array()>upper.array()).any())return false;
  const Eigen::Array3d separation=(lower.array()-voxel.array()-resolution*.5)
    .max(voxel.array()-resolution*.5-upper.array()).max(0.);
  return separation.matrix().squaredNorm()>radius*radius+1e-12;
}
}
