#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <functional>
#include <vector>
#include <limits>
namespace navigation {
// View preference only: no changes to positional routes or safety certification.
inline double perceptionHeading(const Eigen::Vector3d& p,double travel,
 const Eigen::Vector3d& lower,const Eigen::Vector3d& upper,
 const std::function<bool(const Eigen::Vector3d&)>& occupied, double current=std::numeric_limits<double>::quiet_NaN()) {
 if(!lower.allFinite() || !upper.allFinite())return travel;
 double best=travel,best_score=-1e9;
 for(int candidate=0;candidate<24;++candidate) {
  const double bias=candidate<=12 ? candidate*.2617993878 : (candidate-24)*.2617993878;
  const double heading=travel+bias;std::vector<Eigen::Vector2d> hits(13);
  std::vector<bool> valid(13,false);
  for(int i=0;i<13;++i) {
   const double angle=heading+(i-6)*.1163552835; // 80-degree horizontal view
   Eigen::Vector3d direction(std::cos(angle),std::sin(angle),0);
   for(double range=.2;range<=4.;range+=.05) {
    const Eigen::Vector3d q=p+range*direction;
    // Sparse boundary net is containment, not a guaranteed depth surface.
    if(q.x()<=lower.x()+.1 || q.y()<=lower.y()+.1 || q.x()>=upper.x()-.1 || q.y()>=upper.y()-.1)break;
    if(occupied(q)){hits[i]=q.head<2>();valid[i]=true;break;}
   }
  }
  Eigen::Matrix2d information=Eigen::Matrix2d::Zero();int count=0;
  for(int i=1;i<12;++i)if(valid[i-1]&&valid[i]&&valid[i+1]) {
   const Eigen::Vector2d tangent=hits[i+1]-hits[i-1];
   if(tangent.norm()<.02 || tangent.norm()>.8)continue;
   const Eigen::Vector2d normal(-tangent.y()/tangent.norm(),tangent.x()/tangent.norm());
   information+=normal*normal.transpose();++count;
  }
  const auto values=Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(information).eigenvalues();
  const double turn=std::isfinite(current)?std::atan2(std::sin(heading-current),std::cos(heading-current)):bias;
  const double score=2.*values[0]+.25*count-.08*bias*bias-.25*turn*turn;
  if(score>best_score){best_score=score;best=heading;}
 }
 return best;
}
}
