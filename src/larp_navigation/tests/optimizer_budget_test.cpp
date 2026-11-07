#include <utils/optimization/lbfgs.h>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>
using math_utils::lbfgs;
double objective(void* data,const Eigen::VectorXd& x,Eigen::VectorXd& gradient) {
  if(data) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  gradient=2*x;
  return x.squaredNorm();
}
int main() {
  Eigen::VectorXd x=Eigen::VectorXd::Ones(3);
  double cost=0;
  lbfgs::lbfgs_parameter_t params;
  params.max_time_ms=1;
  int slow=1;
  auto rc=lbfgs::lbfgs_optimize(x,cost,objective,nullptr,nullptr,&slow,params);
  if(rc!=lbfgs::LBFGSERR_TIMEOUT || rc>=0) throw std::runtime_error("over-budget solution was accepted");
  params.max_time_ms=100;
  params.max_iterations=100;
  rc=lbfgs::lbfgs_optimize(x,cost,objective,nullptr,nullptr,nullptr,params);
  if(rc<0 || x.norm()>1e-6) throw std::runtime_error("valid optimization failed");
  std::cout << "timeout rejects candidate; normal optimization converges\n";
}
