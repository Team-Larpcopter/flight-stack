#include <navigation/pose_buffer.hpp>
#include <iostream>
#include <stdexcept>
using namespace navigation;
void require(bool value,const char* why) { if(!value) throw std::runtime_error(why); }
int main() {
  PoseBuffer buffer;
  require(buffer.push({1'000'000'000,{0,0,0},Eigen::Quaterniond::Identity()}),"first pose");
  require(buffer.push({1'020'000'000,{2,0,0},Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()))}),"second pose");
  auto middle=buffer.at(1'010'000'000);
  require(bool(middle),"missing capture-time pose");
  require((middle->position-Eigen::Vector3d(1,0,0)).norm()<1e-9,"used latest pose instead of capture pose");
  auto axis=middle->orientation*Eigen::Vector3d::UnitX();
  require((axis-Eigen::Vector3d(std::sqrt(.5),std::sqrt(.5),0)).norm()<1e-9,"bad quaternion interpolation");
  require(!buffer.at(1'021'000'000),"extrapolated into future");
  require(!buffer.at(999'000'000),"extrapolated into past");
  require(!buffer.push({1'010'000'000,{0,0,0},Eigen::Quaterniond::Identity()}),"accepted reordered pose");
  require(buffer.push({1'200'000'000,{3,0,0},Eigen::Quaterniond::Identity()}),"pose after dropout");
  require(!buffer.at(1'100'000'000),"interpolated across dropout");
  buffer.clear();
  require(!buffer.at(1'200'000'000),"retained prior-epoch pose");
  std::cout << "capture-time interpolation, frame rotation, dropout and reset checks passed\n";
}
