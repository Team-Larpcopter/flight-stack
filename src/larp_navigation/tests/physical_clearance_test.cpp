#include <navigation/physical_clearance.hpp>
#include <stdexcept>
#include <iostream>
using Eigen::Vector3d;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(){
 const Vector3d obstacle(.05,.05,.05);
 require(!navigation::sphereBoxClear({.2,.2,.3},{.3,.3,.4},obstacle,.1,.28),"legacy stencil diagonal hole allowed body collision");
 require(navigation::sphereBoxClear({.5,0,0},{.6,.1,.1},obstacle,.1,.35),"separated boxes rejected");
 require(!navigation::sphereBoxClear({.45,.05,.05},{.45,.05,.05},obstacle,.1,.35),"touching sphere accepted");
 require(!navigation::sphereBoxClear({0,0,0},{1,1,1},obstacle,.1,.35),"overlap accepted");
 // Rotation/reflection/permutation do not change Euclidean sphere geometry.
 const Vector3d p(.45,.35,.25),q(.55,.45,.35);
 const bool clear=navigation::sphereBoxClear(p,q,obstacle,.1,.28);
 require(clear==navigation::sphereBoxClear(-q,-p,-obstacle,.1,.28),"reflection changed clearance");
 require(clear==navigation::sphereBoxClear({p.z(),p.x(),p.y()},{q.z(),q.x(),q.y()},obstacle,.1,.28),"axis permutation changed clearance");
 std::cout<<"physical swept-sphere/full-voxel clearance checks passed\n";
}
