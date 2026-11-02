#pragma once
#include <cmath>
#include <cstdint>
// Experimental 8-degree walking startup. Two successive, fresh opposite-side
// peak pairs with usable wheel speed advance the requested target. A measured
// peak need not have reached the current target: otherwise a saturated pulse
// cap and an under-target peak can hold one another indefinitely. Invalid,
// stale, non-alternating or high-speed measurements never advance it. The
// caller remains responsible for physical angle/current/STOP guards.
namespace gradual_start {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  float target() const {return target_;}
  uint16_t __attribute__((noinline)) positiveCapMs() const {
    const float steps=std::floor((target_-4.5f)/.875f+1e-4f);
    return static_cast<uint16_t>(80+5*steps);
  }
  void __attribute__((noinline)) peak(int side,float angle,float rpm,bool fresh,uint32_t ms) {
    if(!fresh || !std::isfinite(angle) || !std::isfinite(rpm) ||
       angle<0 || (side!=1 && side!=-1)) {paired_=false;good_=0;return;}
    if(!paired_) {if(good_ && (ms<=ms_ || ms-ms_>1500))good_=0;side_=side;angle_=angle;rpm_=rpm;ms_=ms;paired_=true;return;}
    if(side==side_ || ms<=ms_ || ms-ms_>1500) {
      side_=side;angle_=angle;rpm_=rpm;ms_=ms;good_=0;return;
    }
    // Valid under-target peaks are evidence that the mechanism is still
    // oscillating; do not confuse model/goal error with sensor failure.
    const bool good=std::fabs(rpm_)<=500.f && std::fabs(rpm)<=500.f;
    paired_=false;ms_=ms;
    if(!good){good_=0;return;}
    // The first detected alternating pair can include the small kick/initial
    // half movement rather than established oscillation. It initializes the
    // history but cannot itself speed up startup.
    if(!have_initial_pair_) {have_initial_pair_=true;good_=0;return;}
    if(++good_>=2) {target_=std::fmin(8.f,target_+.5f);good_=0;}
  }
 private:
  float target_=4.5f,angle_=NAN,rpm_=NAN;
  uint32_t ms_=0;int side_=0;unsigned good_=0;bool paired_=false,have_initial_pair_=false;
};
}
