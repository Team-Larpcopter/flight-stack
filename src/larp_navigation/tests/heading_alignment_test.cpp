#include <navigation/heading_alignment.hpp>
#include <iostream>
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(){
  navigation::HeadingAlignment aligned(M_PI/2);
  require((aligned.position({1,0,2})-Eigen::Vector3d(0,1,2)).norm()<1e-12,"north-facing launch X did not map to ENU north");
  require((aligned.position({0,1,0})-Eigen::Vector3d(-1,0,0)).norm()<1e-12,"launch left did not map to west");
  require((aligned.orientation(Eigen::Quaterniond::Identity())*Eigen::Vector3d::UnitX()-Eigen::Vector3d::UnitY()).norm()<1e-12,"heading quaternion wrong");
  navigation::Matrix6 c=navigation::Matrix6::Identity();c(0,0)=2;c(1,1)=3;c(3,3)=4;c(4,4)=5;
  auto rotated=aligned.covariance(c);
  require(std::abs(rotated(0,0)-3)<1e-12 && std::abs(rotated(1,1)-2)<1e-12 &&
    std::abs(rotated(3,3)-5)<1e-12 && std::abs(rotated(4,4)-4)<1e-12,"pose error covariance not rotated with world axes");
  bool rejected=false;try{navigation::HeadingAlignment bad(NAN);}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"missing heading silently accepted");
  std::cout<<"surveyed ENU heading, position, orientation and covariance checks passed\n";
}
