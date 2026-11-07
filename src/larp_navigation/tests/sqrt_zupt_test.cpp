#include <state/State.h>
#include <state/StateHelper.h>
#include <update/UpdaterZeroVelocity.h>
#include <feat/FeatureDatabase.h>
#include <iostream>
#include <stdexcept>
using namespace ov_srvins;
int main() {
  // Static startup retains constructor calibration blocks. Their covariance
  // must match configured standard deviations, just as dynamic startup does.
  {
    StateOptions calibration;calibration.num_cameras=2;
    calibration.do_calib_camera_pose=true;calibration.do_calib_camera_intrinsics=true;
    calibration.do_calib_camera_timeoffset=true;
    InertialInitializerOptions prior;prior.init_prior_qc=.03;prior.init_prior_pc=.002;
    prior.init_prior_fc=3.;prior.init_prior_dc1=.01;prior.init_prior_dc2=1e-5;prior.init_prior_t=.004;
    auto calibrated=std::make_shared<State>(calibration,prior);
    for(int i=0;i<2;++i) {
      auto pose=StateHelper::get_marginal_covariance(calibrated,{calibrated->calib_IMUtoCAM.at(i)});
      auto camera=StateHelper::get_marginal_covariance(calibrated,{calibrated->cam_intrinsics.at(i)});
      if(std::abs(pose(0,0)-.03*.03)>1e-8 || std::abs(pose(3,3)-.002*.002)>1e-10 ||
         std::abs(camera(0,0)-9.)>1e-5 || std::abs(camera(4,4)-.0001)>1e-9 ||
         std::abs(camera(6,6)-1e-10)>1e-14)throw std::runtime_error("static calibration prior ignored");
    }
    auto dt=StateHelper::get_marginal_covariance(calibrated,{calibrated->calib_dt_CAMtoIMU});
    if(std::abs(dt(0,0)-.004*.004)>1e-10)throw std::runtime_error("time-offset prior ignored");
  }
  StateOptions opts;
  InertialInitializerOptions init;
  auto state=std::make_shared<State>(opts,init);
  state->is_initialized=true;
  state->update_timestamp(1.0);
  VecX x=VecX::Zero(16);x(3)=1;x(10)=.01;
  state->imu->set_value(x);state->imu->set_fej(x);
  StateHelper::set_initial_imu_square_root_covariance(state,VecX::Constant(15,.1));
  state->setup_matrix_buffer();
  NoiseManager noise;noise.sigma_w=.1;noise.sigma_a=.1;noise.sigma_wb=0;noise.sigma_ab=0;
  UpdaterOptions update;update.chi2_multipler=100;
  auto db=std::make_shared<ov_core::FeatureDatabase>();
  auto prop=std::make_shared<Propagator>(noise,9.81);
  UpdaterZeroVelocity zupt(update,noise,db,prop,9.81,1.,10.,.5);
  for(int i=0;i<=11;++i) {
    ov_core::ImuData imu;imu.timestamp=1.+i*.01;
    imu.wm.setZero();imu.am << 0,0,9.81;zupt.feed_imu(imu);
  }
  if(!zupt.try_update(state,1.1)) throw std::runtime_error("stationary update rejected");
  auto covariance=StateHelper::get_marginal_covariance(state,{state->imu});
  // Independent scalar Kalman posterior for gyro bias: P/(1+P*dt/(R*sigma^2)).
  const double expected=.01/(1.+.01*.1/(10.*.1*.1));
  if(std::abs(covariance(9,9)-expected)>2e-6)
    throw std::runtime_error("wrong gyro-bias posterior: joint factor or noise scaling");
  if(std::abs(state->imu->bias_g()(0)-expected)>2e-6)
    throw std::runtime_error("wrong gyro-bias correction");
  for(int i=3;i<9;++i) if(std::abs(covariance(i,i)-.01)>2e-6)
    throw std::runtime_error("zero-motion IMU factor altered independent position/velocity covariance");
  std::cout << "ZUPT joint covariance and variance scaling match analytic Kalman posterior\n";
}
