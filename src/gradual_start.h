#pragma once
#include <cmath>
#include <cstdint>
// Experimental 8-degree walking startup. One update requires two fresh,
// consecutive opposite-side peaks; no time-based forcing through high RPM.
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
    const bool good=angle_>=target_-.5f && angle>=target_-.5f &&
        std::fabs(rpm_)<=500.f && std::fabs(rpm)<=500.f;
    paired_=false;ms_=ms;
    if(!good){good_=0;return;}
    if(++good_>=2) {target_=std::fmin(8.f,target_+.5f);good_=0;}
  }
 private:
  float target_=4.5f,angle_=NAN,rpm_=NAN;
  uint32_t ms_=0;int side_=0;unsigned good_=0;bool paired_=false;
};
}
