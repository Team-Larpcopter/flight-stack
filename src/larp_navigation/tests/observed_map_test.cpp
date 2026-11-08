#include <navigation/observed_map.hpp>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <unistd.h>
void require(bool value,const char* why) {if(!value)throw std::runtime_error(why);}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  double now=1;
  navigation::ObservedMap map(argv[1],[&]{return now;},true);
  super_utils::RobotState body;
  body.p={-.1,0,1};body.v={.2,.3,.4};body.a.setZero();body.j.setZero();body.q.setIdentity();body.yaw=0;body.rcv=true;body.rcv_time=now;
  map.setBody(body);
  rog_map::PointCloud cloud;
  for(int y=-20;y<=20;++y)for(int z=-8;z<=8;++z) {
    rog_map::PointType p;p.x=3.05;p.y=y*.1+.05;p.z=1+z*.1+.05;p.intensity=1;cloud.push_back(p);
  }
  for(int i=0;i<2;++i) {
    map.integrate(cloud,{{0,0,1},Eigen::Quaterniond::Identity()});
    require(!map.isOccupied(Eigen::Vector3d(3.05,.05,1.05)),"unknown voxel confirmed in fewer than three frames");
  }
  for(int i=0;i<8;++i) map.integrate(cloud,{{0,0,1},Eigen::Quaterniond::Identity()});
  require((map.getRobotState().p-body.p).norm()<1e-9,"ray origin replaced body planning position");
  require((map.getRobotState().v-body.v).norm()<1e-9,"map update lost body velocity");
  const Eigen::Vector3d lo(-4,-4,0),hi(4,4,3);
  super_utils::vec_Vec3f free,occupied;
  map.boxSearch(lo,hi,super_utils::KNOWN_FREE,free);
  map.boxSearch(lo,hi,super_utils::OCCUPIED,occupied);
  require(free.size()>100 && occupied.size()>100,"missing classified ray cells");
  for(const auto& p:free)require(map.isKnownFree(p) && !map.isOccupied(p),"snapshot mislabeled free cell");
  for(const auto& p:occupied)require(map.isOccupied(p) && !map.isKnownFree(p),"snapshot mislabeled occupied cell");
  require(map.isUnknown(Eigen::Vector3d(-1.05,0,1)),"unseen rear volume marked free");
  require(map.isUnknown(Eigen::Vector3d(-.15,0,1)),"camera blind volume silently marked free");
  require(map.isUnknown(Eigen::Vector3d(4.05,0,1)),"space behind depth return marked free");
  require(!map.isKnownFreeInflate(Eigen::Vector3d(100,0,1)),"out-of-map query aliased known free storage");
  require(!map.isLineFree(Eigen::Vector3d(-1.05,0,1),Eigen::Vector3d(-1.05,0,1),true,true),"zero-length line accepted unknown endpoint");
  require(!map.isLineFree(Eigen::Vector3d(.5,0,1),Eigen::Vector3d(4,0,1),true,true),"accepted line through wall/unknown");
  require(map.declareLaunchCylinder(.8,.1,1.5)>0,"explicit launch fixture did not seed unknown cells");
  require(map.isKnownFree(Eigen::Vector3d(-.15,0,1)),"launch fixture not represented");
  require(map.isKnownFree(Eigen::Vector3d(-.05,.05,1.45)),"roundoff omitted full top cell inside launch fixture");
  require(map.isUnknown(Eigen::Vector3d(-.05,.05,1.55)),"launch fixture extended above declared boundary");
  require(map.isKnownFreeInflate(Eigen::Vector3d(-.05,.05,1.05)),"declared hover clearance lost at upper voxel boundary");
  require(map.isUnknown(Eigen::Vector3d(-1.05,0,1)),"launch fixture exceeded declared boundary");
  bool refused=false;try{map.declareLaunchCylinder(.8,.1,1.6);}catch(const std::invalid_argument&){refused=true;}
  require(refused,"launch assumption could be reapplied over new observations");
  const auto prior=std::filesystem::temp_directory_path()/std::string("cad-prior-test-"+std::to_string(getpid()));
  {std::ofstream f(prior);f<<"CAD_PRIOR_V1 0.1\n-1.050 0.050 1.050 1\n-1.650 0.050 1.050 0\n";}
  require(map.loadCadPrior(prior.string())==2,"prior was not initialized");
  std::filesystem::remove(prior);
  require(map.isOccupied(Eigen::Vector3d(-1.05,.05,1.05)),"CAD obstacle missing");
  require(map.isKnownFree(Eigen::Vector3d(-1.65,.05,1.05)),"CAD free volume missing");
  // A hundred pixels in one stereo exposure are one frame of evidence.
  rog_map::PointCloud duplicate;
  rog_map::PointType hit;hit.x=-1.65;hit.y=.05;hit.z=1.05;hit.intensity=1;
  for(int i=0;i<100;++i)duplicate.push_back(hit);
  map.integrate(duplicate,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(map.isUnknown(Eigen::Vector3d(-1.65,.05,1.05)),"correlated pixels saturated CAD-free voxel in one frame");
  map.integrate(duplicate,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(!map.isOccupied(Eigen::Vector3d(-1.65,.05,1.05)),"two frames confirmed occupancy too soon");
  map.integrate(duplicate,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(map.isOccupied(Eigen::Vector3d(-1.65,.05,1.05)),"three independent frames did not confirm occupancy");
  auto rear=cloud;for(auto& p:rear)p.x=-3.05;
  for(int i=0;i<2;++i) {
    map.integrate(rear,{{0,0,1},Eigen::Quaterniond::Identity()});
    require(map.isOccupied(Eigen::Vector3d(-1.65,.05,1.05)),"occupied voxel cleared before three empty views");
  }
  map.integrate(rear,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(map.isKnownFree(Eigen::Vector3d(-1.65,.05,1.05)),"three empty views did not clear occupied voxel");
  for(int i=0;i<17;++i)map.integrate(rear,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(map.isKnownFree(Eigen::Vector3d(-1.05,.05,1.05)),"observations could not clear outdated CAD wall");
  rog_map::PointCloud added;rog_map::PointType obstacle;obstacle.x=-1.65;obstacle.y=.05;obstacle.z=1.05;obstacle.intensity=1;added.push_back(obstacle);
  for(int i=0;i<20;++i)map.integrate(added,{{0,0,1},Eigen::Quaterniond::Identity()});
  require(map.isOccupied(Eigen::Vector3d(-1.65,.05,1.05)),"new obstacle could not override CAD free space");
  map.setHardBoundary({-2,-2,0},{2,2,2},.4);
  require(map.isOccupied(Eigen::Vector3d(2.1,0,1)),"boundary not occupied");
  require(!map.isKnownFree(Eigen::Vector3d(2.1,0,1)),"sensor free overrode boundary");
  require(map.isOccupiedInflate(Eigen::Vector3d(1.7,0,1)),"boundary body margin missing");
  require(!map.isLineFree(Eigen::Vector3d(0,0,1),Eigen::Vector3d(3,0,1),true,true),"route crossed boundary");
  for(int i=0;i<10;++i)map.integrate(cloud,{body.p,body.q});
  require(map.isOccupied(Eigen::Vector3d(2.1,0,1)),"ray clearing erased boundary");
  bool boundary_refused=false;try{map.setHardBoundary({-3,-3,0},{3,3,3},.4);}catch(const std::invalid_argument&){boundary_refused=true;}
  require(boundary_refused,"boundary could be changed in flight");
  std::cout<<"capture-origin rays, retained body state, classified cells and unseen-space checks passed\n";
}
