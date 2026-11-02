#pragma once
#include <cmath>
#include <cstdint>

// Experimental symmetric tilt excitation for the 8 degree walking preset.
// An accepted command is not a measured current, an energy prediction, or a
// guarantee of the next peak. The mapping between +/- pitch and the robot's
// physical left/right is not assumed. Existing IMU, actuator and emergency
// protections remain the runner's responsibility.
namespace tilt_priority {
struct Input {
  uint32_t now_ms=0;
  float preset_deg=NAN;
  float target_deg=NAN;
  int8_t next_side=0;
  uint16_t hardware_cap_ms=0;
  bool hardware_healthy=false;
};
struct Choice {
  bool selected=false;
  int16_t current_ma=0;
  uint16_t width_ms=0;
  int8_t direction=0;
};

class Controller {
 public:
  void reset() { *this=Controller{}; }

  // Record every accepted crossing, including normal/coasting half-cycles.
  // Times are run-relative, in the same clock domain as the confirmed peaks.
  void crossing(int8_t side,uint32_t ms) {
    if((side!=1 && side!=-1) || !ms)return;
    auto& h=history_[index(side)];
    h.cross_ms=ms;
  }

  // A peak is used only after its own side's confirmed zero crossing. Its
  // elapsed time is historical evidence, not a prediction for the next
  // stronger pulse. Overdue, missing and out-of-order pairs are not used.
  void peak(int8_t side,uint32_t ms,float angle) {
    if((side!=1 && side!=-1) || !ms || !std::isfinite(angle) || angle<0.f)return;
    auto& h=history_[index(side)];
    if(h.peak_ms && ms<=h.peak_ms)return;
    h.peak_deg=angle;h.peak_ms=ms;
    if(h.cross_ms && ms>h.cross_ms && ms-h.cross_ms<=600) {
      h.elapsed_ms=static_cast<uint16_t>(ms-h.cross_ms);
      h.paired_ms=ms;
    } else {
      h.elapsed_ms=0;h.paired_ms=0;
    }
    h.cross_ms=0;
  }

  __attribute__((noinline)) Choice propose(const Input& in) const {
    Choice out;
    if(in.preset_deg!=8.f || !in.hardware_healthy ||
       (in.next_side!=1 && in.next_side!=-1) ||
       !std::isfinite(in.target_deg) || in.target_deg<=0.f ||
       !in.now_ms || !in.hardware_cap_ms)return out;
    const auto& h=history_[index(in.next_side)];
    const bool fresh=h.peak_ms && in.now_ms>=h.peak_ms &&
        in.now_ms-h.peak_ms<=1500 && std::isfinite(h.peak_deg);
    const float deficit=fresh ? in.target_deg-h.peak_deg : NAN;
    // Hand the choice back to the normal controller near/above the target.
    if(fresh && deficit<=.5f)return out;

    uint16_t cap=in.hardware_cap_ms<150 ? in.hardware_cap_ms : 150;
    const bool timing_fresh=fresh && h.paired_ms==h.peak_ms && h.elapsed_ms;
    if(timing_fresh) {
      if(h.elapsed_ms<=30)return out;
      const uint16_t timing_cap=static_cast<uint16_t>(h.elapsed_ms-30);
      if(cap>timing_cap)cap=timing_cap;
    } else if(cap>100) {
      cap=100;
    }
    if(!cap)return out;

    // When the measured side is already approaching its target, reduce
    // duration continuously rather than repeatedly applying the longest
    // pulse. Wide deficits deliberately use the available full duration.
    uint16_t width=cap;
    if(fresh && deficit<2.f && cap>70) {
      const float fraction=(deficit-.5f)/1.5f;
      const float scaled=70.f+(static_cast<float>(cap)-70.f)*fraction;
      width=static_cast<uint16_t>(std::floor(scaled+.5f));
      if(width>cap)width=cap;
    }
    out.selected=true;out.current_ma=1200;out.width_ms=width;out.direction=in.next_side;
    return out;
  }

 private:
  struct History {
    float peak_deg=NAN;
    uint32_t peak_ms=0,cross_ms=0,paired_ms=0;
    uint16_t elapsed_ms=0;
  };
  static unsigned index(int8_t side) {return side>0 ? 1u : 0u;}
  History history_[2];
};
}
