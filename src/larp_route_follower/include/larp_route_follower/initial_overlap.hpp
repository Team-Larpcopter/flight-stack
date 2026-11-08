#pragma once
#include <navigation/physical_clearance.hpp>
#include <data_structure/base/trajectory.h>
#include <Eigen/Core>
#include <array>
#include <map>
#include <vector>

namespace larp_route_follower {
// Exceptions are captured once per goal, from occupied cells only. They never
// delete map cells, include unknown cells, or grow as the aircraft moves.
class InitialOverlap {
 public:
  using Point=Eigen::Vector3d;
  using Key=std::array<int,3>;
  struct Cell {Point centre;double minimum;};
  static double distance(const Point& p,const Point& cell,double resolution) {
    return ((p-cell).array().abs()-resolution*.5).max(0.).matrix().norm();
  }
  void reset(){cells_.clear();captured_=false;}
  bool captured()const{return captured_;}
  bool active()const{return !cells_.empty();}
  size_t size()const{return cells_.size();}
  template<class Points> void capture(const Point& start,const Points& occupied,double resolution,double radius) {
    if(captured_)return;
    captured_=true;resolution_=resolution;radius_=radius;
    for(const auto& c:occupied) {
      const double d=distance(start,c,resolution);
      // A centre inside solid geometry has no unambiguous outward normal.
      if(d>1e-6 && d<=radius)cells_.emplace(key(c),Cell{c,d});
      if(cells_.size()>2048){cells_.clear();return;}
    }
  }
  void update(const Point& p) {
    for(auto i=cells_.begin();i!=cells_.end();) {
      const double d=distance(p,i->second.centre,resolution_);
      if(d>radius_+1e-5)i=cells_.erase(i);
      else {i->second.minimum=std::max(i->second.minimum,d);++i;}
    }
  }
  bool permitsBox(const Point& lo,const Point& hi,const Point& cell)const {
    const auto it=cells_.find(key(cell));if(it==cells_.end())return false;
    const auto sep=(lo.array()-cell.array()-resolution_*.5)
      .max(cell.array()-resolution_*.5-hi.array()).max(0.).matrix().eval();
    return sep.norm()+1e-6>=it->second.minimum;
  }
  bool permitsChord(const Point& a,const Point& b)const {
    for(const auto& entry:cells_) {
      const Point c=entry.second.centre;
      if(navigation::sphereBoxClear(a.cwiseMin(b),a.cwiseMax(b),c,resolution_,radius_))continue;
      if(distance(a,c,resolution_)>radius_+1e-5)return false; // no re-entry
      const Point nearest=a.cwiseMax(c-Point::Constant(resolution_*.5)).cwiseMin(c+Point::Constant(resolution_*.5));
      if((b-a).dot(a-nearest)<-1e-9)return false;
    }
    return true;
  }
  // Bernstein derivative controls prove outward motion for each overlapping
  // piece. Straight stop-at-corner pieces satisfy this without optimization.
  bool permitsTrajectory(const geometry_utils::Trajectory& trajectory)const {
    if(cells_.empty())return true;
    for(const auto& piece:trajectory) {
      const auto& coeff=piece.getCoeffMat();const int n=coeff.cols()-1;
      Eigen::MatrixXd b(3,n+1);const double t=piece.getDuration();
      auto choose=[](int n,int k){double r=1;for(int i=1;i<=k;++i)r=r*(n-i+1)/i;return r;};
      for(int i=0;i<=n;++i){b.col(i).setZero();for(int k=0;k<=i;++k)b.col(i)+=coeff.col(n-k)*std::pow(t,k)*choose(i,k)/choose(n,k);}
      const Point lo=b.rowwise().minCoeff(),hi=b.rowwise().maxCoeff();
      for(const auto& entry:cells_) {
        const Point c=entry.second.centre,a=b.col(0);
        if(navigation::sphereBoxClear(lo,hi,c,resolution_,radius_))continue;
        if(distance(a,c,resolution_)>radius_+1e-5)return false;
        const Point nearest=a.cwiseMax(c-Point::Constant(resolution_*.5)).cwiseMin(c+Point::Constant(resolution_*.5));
        for(int i=0;i<n;++i)if((b.col(i+1)-b.col(i)).dot(a-nearest)<-1e-9)return false;
      }
    }
    return true;
  }
 private:
  Key key(const Point& p)const{return {int(std::floor(p.x()/resolution_)),int(std::floor(p.y()/resolution_)),int(std::floor(p.z()/resolution_))};}
  bool captured_{false};double resolution_{.05},radius_{.3};std::map<Key,Cell> cells_;
};
}
