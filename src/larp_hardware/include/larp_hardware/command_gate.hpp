#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
namespace larp_hardware {
struct CommandSample {
 int64_t receipt_steady_ns,deadline_steady_ns;
 std::array<double,3> position,velocity,acceleration;
 double yaw,yaw_rate;
};
// Native actuator admission: no arming/mode transitions. Clock-domain conversion
// is explicit at receipt, then expiry is enforced with a monotonic deadline.
class CommandGate {
 public:
 bool accept(CommandSample s,int64_t now) {
  sample_.reset();
  if(s.receipt_steady_ns>now || s.deadline_steady_ns<=now || s.deadline_steady_ns-now>305'000'000)return false;
  for(const auto& v:{s.position,s.velocity,s.acceleration})for(double x:v)if(!std::isfinite(x))return false;
  if(!std::isfinite(s.yaw)||!std::isfinite(s.yaw_rate))return false;
  sample_=s;return true;
 }
 std::optional<CommandSample> get(int64_t now,bool healthy) const {
  if(!healthy || !sample_ || now<sample_->receipt_steady_ns || now>=sample_->deadline_steady_ns)return {};
  return sample_;
 }
 void clear(){sample_.reset();}
 private:std::optional<CommandSample> sample_;
};
}
