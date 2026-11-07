#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
namespace larp_hardware {
struct Sample { int64_t ns; std::array<double,3> value; };
struct AlignedImu { int64_t ns; std::array<double,3> acceleration,gyro; };
// Retain native gyro timestamps. Interpolate acceleration only between actual
// bracketing samples, never extrapolate or relabel a stale accelerometer sample.
class ImuAlignment {
 public:
  explicit ImuAlignment(int64_t max_gap_ns=20'000'000):gap_(max_gap_ns) {
    if(gap_<=0)throw std::invalid_argument("nonpositive alignment gap");
  }
  void accel(Sample s) { push(accel_,last_accel_,s); }
  void gyro(Sample s) { push(gyro_,last_gyro_,s); }
  std::optional<AlignedImu> pop() {
    while(!gyro_.empty() && !accel_.empty()) {
      const auto g=gyro_.front();
      if(g.ns<accel_.front().ns){gyro_.pop_front();++dropped_;continue;}
      while(accel_.size()>1 && accel_[1].ns<=g.ns)accel_.pop_front();
      if(accel_.front().ns==g.ns) {
        auto a=accel_.front();gyro_.pop_front();return AlignedImu{g.ns,a.value,g.value};
      }
      if(accel_.size()<2)return std::nullopt;
      auto a=accel_[0],b=accel_[1];
      if(b.ns-a.ns>gap_){gyro_.pop_front();++dropped_;continue;}
      double f=double(g.ns-a.ns)/double(b.ns-a.ns);std::array<double,3> v;
      for(int i=0;i<3;++i)v[i]=a.value[i]+f*(b.value[i]-a.value[i]);
      gyro_.pop_front();return AlignedImu{g.ns,v,g.value};
    }
    return std::nullopt;
  }
  uint64_t dropped() const{return dropped_;}
 private:
  void push(std::deque<Sample>& q,int64_t& last,Sample s) {
    if(s.ns<=0)throw std::invalid_argument("nonpositive sensor timestamp");
    for(double v:s.value)if(!std::isfinite(v))throw std::invalid_argument("nonfinite IMU");
    if(s.ns<=last)return; // repeated reports in DepthAI packets
    last=s.ns;q.push_back(s);if(q.size()>2048){q.pop_front();++dropped_;}
  }
  int64_t gap_,last_accel_=0,last_gyro_=0;uint64_t dropped_=0;std::deque<Sample> accel_,gyro_;
};
}
