#include <navigation/perception_heading.hpp>
#include <cassert>
int main(){
 using V=Eigen::Vector3d;const V p(0,0,1),lo(-3,-3,0),hi(3,3,3);
 auto empty=[](const V&){return false;};
 assert(std::abs(navigation::perceptionHeading(p,.3,lo,hi,empty)-.3)<1e-9);
 auto wall=[](const V& q){return q.y()>=1. && q.y()<1.1;};
 const double yaw=navigation::perceptionHeading(p,0,lo,hi,wall);
 assert(yaw>.2 && yaw<=1.6);
 auto behind=[](const V& q){return q.x()<=-1. && q.x()>-1.1;};
 const double rear=navigation::perceptionHeading(p,0,lo,hi,behind,0.);
 assert(std::abs(rear)>1.0472); // useful surfaces outside the old +/-60 degree search
 auto net=[](const V& q){return q.x()>=2.95;};
 assert(std::abs(navigation::perceptionHeading(p,0,lo,hi,net))<1e-9);
}
