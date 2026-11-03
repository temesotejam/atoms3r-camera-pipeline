#pragma once
#include <cmath>
#include <cstdint>

// Experimental: preserve the early alternating wheel motion. A pair of
// fresh pre-input observations estimates its midpoint, not ground motion.
// Only shorten the pulse that would push an already biased wheel farther in
// the same direction; do not add torque, reverse torque or force a stop.
namespace wheel_cycle_guard {
class Controller {
 public:
  void reset() { *this=Controller{}; }
  uint16_t limit(int side, float rpm, bool valid, uint32_t ms, uint16_t width) {
    if (!valid || !std::isfinite(rpm) || !ms || (side!=1 && side!=-1)) {
      reset(); return width;
    }
    float midpoint=0.f; bool paired=false;
    if (last_side_==-side && ms>last_ms_ && ms-last_ms_<=1500) {
      midpoint=.5f*(last_rpm_+rpm); paired=true;
    }
    last_side_=side; last_rpm_=rpm; last_ms_=ms;
    if (!paired || midpoint*side<=150.f || !width) return width;
    const float strength=std::fmin(1.f,(midpoint*side-150.f)/250.f);
    // At most 25% of the original pulse; bounded and continuous at onset.
    uint16_t reduced=static_cast<uint16_t>(std::ceil(width*(1.f-.25f*strength)));
    return reduced<width ? reduced : width;
  }
 private:
  float last_rpm_=NAN;
  uint32_t last_ms_=0;
  int last_side_=0;
};
}
