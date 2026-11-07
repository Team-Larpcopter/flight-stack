#pragma once
#include <data_structure/base/trajectory.h>
#include <functional>
#include <chrono>
#include <sstream>

namespace navigation {
// Certifies the complete polynomial curve using Bernstein convex hulls,
// including the stopping tail. A chord-only or sparse sample test misses bowed
// trajectories. Ambiguous boxes are subdivided; budget exhaustion rejects.
class TrajectoryValidator {
 public:
  using BoxFree=std::function<bool(const Eigen::Vector3d&,const Eigen::Vector3d&)>;
  TrajectoryValidator(BoxFree free,double max_v,double max_a,double budget_ms=10)
    : free_(std::move(free)),max_v_(max_v),max_a_(max_a),budget_ms_(budget_ms) {}
  bool operator()(const geometry_utils::Trajectory& traj) {
    failure_.clear();
    nodes_=0;deadline_=std::chrono::steady_clock::now()+std::chrono::microseconds(int64_t(budget_ms_*1000));
    if(traj.empty() || !std::isfinite(traj.start_WT) || !std::isfinite(traj.getTotalDuration()) ||
       traj.getTotalDuration()<=0 || traj.getTotalDuration()>30 || traj.size()>64) return reject("invalid duration or piece count");
    Eigen::Vector3d previous_p,previous_v,previous_a;bool first=true;
    for(const auto& piece:traj) {
      const auto& c=piece.getCoeffMat();const int n=c.cols()-1;const double duration=piece.getDuration();
      if(c.rows()!=3 || n<0 || n>9 || !c.allFinite() || !std::isfinite(duration) || duration<=0) return reject("invalid polynomial");
      Eigen::MatrixXd b(3,n+1);
      for(int i=0;i<=n;++i) {
        b.col(i).setZero();
        for(int k=0;k<=i;++k)b.col(i)+=c.col(n-k)*std::pow(duration,k)*choose(i,k)/choose(n,k);
      }
      const Eigen::Vector3d p=piece.getPos(0),v=piece.getVel(0),a=piece.getAcc(0);
      if(!first && ((p-previous_p).norm()>1e-3 || (v-previous_v).norm()>1e-3 || (a-previous_a).norm()>1e-2)) return reject("discontinuous polynomial pieces");
      if(!segment(b,duration,0)) return false;
      previous_p=piece.getPos(duration);previous_v=piece.getVel(duration);previous_a=piece.getAcc(duration);first=false;
    }
    return (previous_v.norm()<.05 && previous_a.norm()<.1) || reject("trajectory does not end at rest");
  }
  const std::string& failure()const{return failure_;}
 private:
  bool reject(const char* why){failure_=why;return false;}
  static double choose(int n,int k) {double r=1;for(int i=1;i<=k;++i)r=r*(n-i+1)/i;return r;}
  bool segment(const Eigen::MatrixXd& b,double duration,int depth) {
    if(++nodes_>4096 || std::chrono::steady_clock::now()>deadline_) return reject("validation time or subdivision budget exceeded");
    const int n=b.cols()-1;
    bool dynamics=true;
    for(int i=0;i<n;++i)if((n*(b.col(i+1)-b.col(i))/duration).norm()>max_v_)dynamics=false;
    for(int i=0;i<n-1;++i)if((n*(n-1)*(b.col(i+2)-2*b.col(i+1)+b.col(i))/(duration*duration)).norm()>max_a_)dynamics=false;
    const Eigen::Vector3d lo=b.rowwise().minCoeff(),hi=b.rowwise().maxCoeff();
    if(dynamics && free_(lo,hi))return true;
    if(depth>=14)return reject(dynamics ? "uncertified clearance boundary" : "hard velocity or acceleration limit exceeded");
    if(!free_(b.col(0),b.col(0)))return rejectPoint("curve endpoint outside observed free clearance",b.col(0));
    if(!free_(b.col(n),b.col(n)))return rejectPoint("curve endpoint outside observed free clearance",b.col(n));
    Eigen::MatrixXd q=b,left(3,n+1),right(3,n+1);
    left.col(0)=q.col(0);right.col(n)=q.col(n);
    for(int j=1;j<=n;++j) {
      for(int i=0;i<=n-j;++i)q.col(i)=.5*(q.col(i)+q.col(i+1));
      left.col(j)=q.col(0);right.col(n-j)=q.col(n-j);
    }
    if(!free_(left.col(n),left.col(n)))return rejectPoint("curve interior outside observed free clearance",left.col(n));
    return segment(left,duration*.5,depth+1) && segment(right,duration*.5,depth+1);
  }
  bool rejectPoint(const char* why,const Eigen::Vector3d& p) {
    std::ostringstream out;out<<why<<" at "<<p.transpose();failure_=out.str();return false;
  }
  BoxFree free_;double max_v_,max_a_,budget_ms_;size_t nodes_{0};
  std::string failure_;
  std::chrono::steady_clock::time_point deadline_;
};
}
