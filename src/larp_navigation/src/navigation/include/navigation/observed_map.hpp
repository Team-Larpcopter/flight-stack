#pragma once
#include <rog_map/rog_map.h>
#include <functional>
#include <algorithm>
#include <limits>
#include <fstream>

namespace navigation {
// Mapping rays originate at the optical center. Planning uses the full body
// state, restored after every map update; ROG's pose-only callback cannot supply it.
class ObservedMap final : public rog_map::ROGMap {
 public:
  ObservedMap(const std::string& config, std::function<double()> clock, bool fixed_prior_volume=false) : clock_(std::move(clock)) {
    cfg_=rog_map::Config(config);if(fixed_prior_volume)cfg_.map_sliding_en=false;init();
    body_.p.setZero();body_.v.setZero();body_.a.setZero();body_.j.setZero();
    body_.q.setIdentity();body_.yaw=0;body_.rcv_time=0;body_.rcv=false;
    robot_state_=body_;
  }
  void setBody(const super_utils::RobotState& body) {body_=body;robot_state_=body;}
  void integrate(const rog_map::PointCloud& cloud, const super_utils::Pose& camera) {
    updateMap(cloud,camera);robot_state_=body_;
  }
  size_t loadCadPrior(const std::string& path, bool allow_unobserved_initialization=false) {
    if(prior_loaded_ || (map_empty_ && !allow_unobserved_initialization) || cfg_.map_sliding_en)throw std::invalid_argument("prior needs initialized map and can only load once");
    std::ifstream in(path);std::string magic;double resolution;
    if(!(in>>magic>>resolution) || magic!="CAD_PRIOR_V1" || std::abs(resolution-getResolution())>1e-8)
      throw std::invalid_argument("invalid CAD prior header/resolution");
    std::vector<std::pair<Eigen::Vector3d,int>> cells;
    double x,y,z;int occupied;
    while(in>>x>>y>>z>>occupied) {
      Eigen::Vector3d p(x,y,z),center;Eigen::Vector3i id;posToGlobalIndex(p,id);globalIndexToPos(id,center);
      if(!p.allFinite() || (occupied!=0 && occupied!=1) || !insideLocalMap(p) || (p-center).norm()>1e-6)
        throw std::invalid_argument("CAD cell outside fixed map or invalid lattice");
      cells.emplace_back(p,occupied);
      if(cells.size()>2000000)throw std::invalid_argument("CAD prior too large");
    }
    if(!in.eof() || cells.empty())throw std::invalid_argument("malformed/empty CAD prior");
    size_t count=0;
    for(const auto& cell:cells) {
      const auto& p=cell.first;if(outsideHardBoundary(p))continue;
      const int hash=getHashIndexFromPos(p);
      // Classified measurements obtained before initialization take precedence.
      if(!isUnknown(occupancy_buffer_[hash]))continue;
      resetCell(hash); // initialize finite prior odds in an unclassified cell
      if(cell.second)hitPointUpdate(p,hash,int(std::floor(cfg_.l_occ/cfg_.l_hit))+1);
      else missPointUpdate(p,hash,int(std::floor(cfg_.l_free/cfg_.l_miss))+1);
      ++count;
    }
    prior_loaded_=true;map_empty_=false;return count;
  }
  // Mirror validated authoritative classifications, with no ray inference.
  // Revisions preserve unknown space: previously classified cells absent from
  // the next snapshot become unknown. Identical cells do not accumulate odds.
  template<class Cloud> void replaceClassifications(const Cloud& occupied,const Cloud& free) {
    if(cfg_.map_sliding_en)throw std::invalid_argument("snapshot mirror requires fixed storage");
    if(snapshot_seen_.empty())snapshot_seen_.resize(occupancy_buffer_.size(),0);
    if(++snapshot_generation_==0) {std::fill(snapshot_seen_.begin(),snapshot_seen_.end(),0);snapshot_generation_=1;}
    std::vector<int> next;next.reserve(occupied.size()+free.size());
    auto apply=[&](const Cloud& cloud,bool hit) {
      for(const auto& q:cloud) {
        const Eigen::Vector3d p(q.x,q.y,q.z);
        if(outsideHardBoundary(p))continue;
        if(!insideLocalMap(p))throw std::invalid_argument("snapshot cell outside mirror storage");
        const int hash=getHashIndexFromPos(p);
        if(snapshot_seen_[hash]!=snapshot_generation_)next.push_back(hash);
        snapshot_seen_[hash]=snapshot_generation_;
        if((hit && isOccupied(occupancy_buffer_[hash])) || (!hit && isKnownFree(occupancy_buffer_[hash])))continue;
        resetCell(hash);
        if(hit)hitPointUpdate(p,hash,999);else missPointUpdate(p,hash,999);
      }
    };
    apply(free,false);apply(occupied,true);
    for(int hash:snapshot_cells_)if(snapshot_seen_[hash]!=snapshot_generation_)resetCell(hash);
    snapshot_cells_=std::move(next);map_empty_=false;
  }
  // One explicit, externally checked launch fixture. This is a start-condition
  // assumption, not an observation or an environment map. Never clear an
  // observed obstacle, and never reapply it as the robot moves.
  size_t declareLaunchCylinder(double radius,double min_z,double max_z) {
    if(launch_declared_ || map_empty_ || !std::isfinite(radius+min_z+max_z) || radius<=0 ||
       radius>2 || min_z>=max_z || max_z-min_z>3)throw std::invalid_argument("invalid/repeated launch declaration");
    size_t count=0;const double res=getResolution(),half=res*.5;
    // Decimal grid faces (e.g. 1.45 + .05) can round just above 1.5.
    // Tolerate arithmetic error, not any geometrically partial cell.
    const double eps=32*std::numeric_limits<double>::epsilon()*std::max({1.,radius,std::abs(min_z),std::abs(max_z)});
    Eigen::Vector3i lo,hi;posToGlobalIndex({-radius,-radius,min_z},lo);posToGlobalIndex({radius,radius,max_z},hi);
    for(int x=lo.x();x<=hi.x();++x)for(int y=lo.y();y<=hi.y();++y)
      for(int z=lo.z();z<=hi.z();++z) {
        Eigen::Vector3d p;globalIndexToPos({x,y,z},p);
        // Only full cells strictly inside the declared cylinder are admitted.
        if(std::hypot(std::abs(p.x())+half,std::abs(p.y())+half)>radius+eps ||
           p.z()-half<min_z-eps || p.z()+half>max_z+eps || !insideLocalMap(p))continue;
        const int hash=getHashIndexFromPos(p);
        if(isUnknown(occupancy_buffer_[hash])){missPointUpdate(p,hash,999);++count;}
      }
    launch_declared_=true;return count;
  }
 private:
  const double getSystemWalltimeNow() override {return clock_();}
  std::function<double()> clock_;
  super_utils::RobotState body_;
  std::vector<uint32_t> snapshot_seen_;std::vector<int> snapshot_cells_;uint32_t snapshot_generation_{0};
  bool launch_declared_{false},prior_loaded_{false};
};
}
