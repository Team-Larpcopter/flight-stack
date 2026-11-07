#include <navigation/observed_map.hpp>
#include <filesystem>
#include <unistd.h>
#include <iostream>
int main(int argc,char** argv) {
 if(argc!=2)return 2;
 navigation::ObservedMap map(argv[1],[]{return 1.;},true);
 map.setHardBoundary({-2,-2,0},{2,2,2},.4);
 const auto prior=std::filesystem::temp_directory_path()/std::string("static-cad-test-"+std::to_string(getpid()));
 {std::ofstream f(prior);f<<"CAD_PRIOR_V1 0.1\n-1.050 0.050 1.050 1\n-1.650 0.050 1.050 0\n";}
 const auto count=map.loadCadPrior(prior.string(),true);std::filesystem::remove(prior);
 if(count!=2 || !map.isOccupied(Eigen::Vector3d(-1.05,.05,1.05)) ||
    !map.isKnownFree(Eigen::Vector3d(-1.65,.05,1.05)) || !map.isOccupied(Eigen::Vector3d(2.1,0,1)))return 1;
 std::cout<<"Static CAD initialization without sensor hits preserves classification and geofence\n";
}
