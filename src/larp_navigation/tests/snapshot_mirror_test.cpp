#include <navigation/observed_map.hpp>
#include <iostream>
#include <stdexcept>
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
int main(int argc,char** argv) {
 if(argc!=2)return 2;
 navigation::ObservedMap map(argv[1],[]{return 1.;},true);
 rog_map::PointCloud occupied,free;rog_map::PointType p;p.x=1.05;p.y=.05;p.z=1.05;p.intensity=1;free.push_back(p);p.x=2.05;occupied.push_back(p);
 map.replaceClassifications(occupied,free);
 require(map.isKnownFree(Eigen::Vector3d(1.05,.05,1.05)),"mirror lost free classification");
 require(map.isOccupied(Eigen::Vector3d(2.05,.05,1.05)),"mirror lost occupied classification");
 map.replaceClassifications(occupied,free);
 occupied.clear();free.clear();p.x=1.05;occupied.push_back(p);
 map.replaceClassifications(occupied,free);
 require(map.isOccupied(Eigen::Vector3d(1.05,.05,1.05)),"mirror did not replace free with occupied");
 require(map.isUnknown(Eigen::Vector3d(2.05,.05,1.05)),"absent snapshot cell retained stale obstacle");
 occupied.clear();free.push_back(p);map.replaceClassifications(occupied,free);
 require(map.isKnownFree(Eigen::Vector3d(1.05,.05,1.05)),"mirror did not accept authoritative obstacle removal");
 std::cout<<"Snapshot mirror preserves occupied/free/unknown revisions without accumulating evidence\n";
}
