#pragma once
#include "wheel_work_energy_control.h"

// Effective free-foot energy decrease, not an independently measured impact
// or friction loss. Fitted ONLY to 32 unpowered negative half-cycles in
// run 50881356. All energies here are mJ. Never add this loss to the old
// side-bias correction: this is a separate, mutually exclusive branch.
namespace walking_energy_loss {
constexpr char kRevision[]="walking_coast_four_runs_v4";
inline bool supported(const wheel_work_energy_control::State& s) {
  return s.absolute_reference && s.target_deg==8.f &&
      wheel_work_energy_control::freshFinite(s) &&
      s.rate_dps>=18.45f && s.rate_dps<=37.30f &&
      s.wheel_rpm>=362.f && s.wheel_rpm<=603.21f &&
      s.battery_mv>=7000.f && s.battery_mv<=8120.f &&
      s.previous_peak_deg>=4.3006f && s.previous_peak_deg<=6.90f;
}
inline float coastMj(float rate_dps) {
  return .09046158f+.00018973f*rate_dps*rate_dps;
}
class Feedback {
 public:
  unsigned count() const { return count_; }
  float biasMj() const {
    if(count_<2) return 0.f;
    float sum=0.f;for(unsigned i=0;i<count_;++i)sum+=residual_[i];
    return std::fmax(-.10f,std::fmin(.10f,sum/count_));
  }
  // Two matched coast arrivals first. Raise the cap only after two completed
  // matching powered arrivals below target at the current cap.
  uint16_t capMs() const { return count_>=2 && !overshot_ ? cap_ms_ : 0; }
  bool observe(float actual_mj,float raw_mj,float target_mj,uint16_t width_ms=0) {
    if(!std::isfinite(actual_mj)||!std::isfinite(raw_mj)||
       !std::isfinite(target_mj)||actual_mj<0.f||raw_mj<0.f||target_mj<=0.f) return false;
    overshot_=actual_mj>=target_mj;
    if(overshot_) { cap_ms_=cap_ms_>25 ? cap_ms_-5 : 25; limited_count_=0; }
    if(std::fabs(actual_mj-raw_mj)>.15f) { limited_count_=0; return false; }
    if(width_ms>0 && width_ms>=cap_ms_ && actual_mj<target_mj-.03f) {
      if(++limited_count_>=2) { if(cap_ms_<40)cap_ms_+=5;limited_count_=0; }
    } else if(width_ms>0) limited_count_=0;
    residual_[next_]=actual_mj-raw_mj;next_=(next_+1)%3;
    if(count_<3)++count_;
    return true;
  }
 private:
  float residual_[3]={};unsigned count_=0,next_=0,limited_count_=0;
  uint16_t cap_ms_=25;bool overshot_=false;
};
__attribute__((noinline)) inline wheel_work_energy_control::Choice choose(
    const lateral_energy::Geometry& g,const wheel_work_energy_control::State& s,
    const lateral_energy::CurrentTable& table,float initial_ma,float goal_ma,
    float target_mj,const Feedback& feedback,float cg_mm) {
  using namespace wheel_work_energy_control;
  Choice c;
  if(!supported(s)||!std::isfinite(target_mj)||target_mj<=0.f||
      !std::isfinite(initial_ma)||!std::isfinite(goal_ma)||goal_ma<=0.f||
      !std::isfinite(cg_mm)||std::fabs(cg_mm)>1e-6f) return c;
  const float previous_mj=1000.f*lateral_energy::potential(g,s.previous_peak_deg);
  const float passive=coastMj(s.rate_dps)+feedback.biasMj();
  // Signed Uprev-Ecoast may be negative from motion or energy exchange.
  const float loss=previous_mj-passive;
  if(!std::isfinite(loss)||passive<=0.f)return c;
  for(uint16_t ms=0;ms<=feedback.capMs();ms=ms==0 ? 10 : ms+1) {
    const float q=ms ? lateral_energy::charge(table,ms,initial_ma,goal_ma) : 0.f;
    if(!std::isfinite(q)||q<0.f||q>25.f)continue;
    Prediction p;
    // Same provisional incremental-gain transfer as the existing negative
    // assist model. Coast fit is measured; powered negative gain is NOT.
    const float gain=.22502117220f*.06189960458f*q*s.rate_dps*kRadPerDeg;
    p.energy_mj=previous_mj-loss+gain;p.free_mj=passive;
    p.valid=std::isfinite(p.energy_mj)&&p.energy_mj>=0.f;
    p.branch=10;p.cross_condition=true;p.support_weight=0.f;
    p.applied_bias_mj=feedback.biasMj();
    consider(c,300,ms,q,p,target_mj);
  }
  c.command_sign=1;c.allowed_cap_ms=feedback.capMs();c.walking_loss_mj=loss;
  finish(c,target_mj);return c;
}
}
