#include <navigation/ego_spline.hpp>
#include <navigation/trajectory_validator.hpp>
#include <stdexcept>
#include <iostream>
void require(bool v,const char* message){if(!v)throw std::runtime_error(message);}
int main() {
  Eigen::MatrixXd controls(3,10);
  for(int i=0;i<10;++i)controls.col(i)=Eigen::Vector3d(i*.08,std::sin(i)*.03,1.);
  const double h=.7;
  const Eigen::Vector3d p(0,0,1),v(.01,.02,0),a(.02,0,0),goal(.8,.1,1);
  auto curve=navigation::egoPolynomial(controls,h,p,v,a,goal,5.);
  require((curve.getPos(0)-p).norm()<1e-10 && (curve.getVel(0)-v).norm()<1e-10 &&
    (curve.getAcc(0)-a).norm()<1e-10,"incorrect initial boundary");
  require((curve.getPos(curve.getTotalDuration())-goal).norm()<1e-10 &&
    curve.getVel(curve.getTotalDuration()).norm()<1e-10 && curve.getAcc(curve.getTotalDuration()).norm()<1e-10,"incorrect terminal boundary");
  // Independent cardinal cubic basis for a span unaffected by boundary edits.
  for(double u:{0.,.13,.5,.91,1.}) {
    const Eigen::Vector3d expected=(std::pow(1-u,3)*controls.col(3)+
      (3*u*u*u-6*u*u+4)*controls.col(4)+(-3*u*u*u+3*u*u+3*u+1)*controls.col(5)+
      u*u*u*controls.col(6))/6.;
    require((curve[3].getPos(u*h)-expected).norm()<1e-10,"cubic conversion differs from B-spline basis");
  }
  for(size_t i=1;i<curve.size();++i) {
    require((curve[i-1].getPos(h)-curve[i].getPos(0)).norm()<1e-10,"position discontinuity");
    require((curve[i-1].getVel(h)-curve[i].getVel(0)).norm()<1e-10,"velocity discontinuity");
    require((curve[i-1].getAcc(h)-curve[i].getAcc(0)).norm()<1e-10,"acceleration discontinuity");
  }
  navigation::TrajectoryValidator validate([](const auto&,const auto&){return true;},1.,2.);
  require(validate(curve),"converted spline failed dynamics and continuity validation");
  geometry_utils::Trajectory sliced;
  require(navigation::polynomialSlice(curve,.23,2.31,sliced),"cubic slice failed");
  for(double t=0;t<sliced.getTotalDuration();t+=.071) {
    require((sliced.getPos(t)-curve.getPos(t+.23)).norm()<1e-10,"slice changed position");
    require((sliced.getVel(t)-curve.getVel(t+.23)).norm()<1e-10,"slice changed velocity");
    require((sliced.getAcc(t)-curve.getAcc(t+.23)).norm()<1e-10,"slice changed acceleration");
  }
  std::cout<<"Exact cubic conversion, C2 boundary conditions and stopping tail passed\n";
}
