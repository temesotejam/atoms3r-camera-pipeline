#pragma once
#include <cmath>
#include <cstdint>

// A bounded experimental command when ordinary negative-side selection is
// coast or unsupported. This is a heuristic, NOT an energy or gait-response
// prediction. The caller must report it separately from the fitted models,
// enforce all actuator/attitude/emergency guards, confirm the actual command,
// and retain negative-peak and wheel-speed observations for later evaluation.
namespace weak_side_assist {
struct Input {
  float target_preset_deg=NAN;      // operator's requested target
  float active_target_deg=NAN;      // currently ramped target, same frame as peaks
  int8_t physical_next_side=0;
  float last_negative_peak_deg=NAN;
  uint32_t last_negative_peak_ms=0;
  uint32_t now_ms=0;
  uint16_t negative_peak_count=0;   // actually confirmed in this run
  float crossing_rate_dps=NAN;
  float wheel_rpm=NAN;
  float battery_mv=NAN;
  float last_positive_peak_deg=NAN;
  bool hardware_healthy=false;      // includes no STOP, motor/IMU/roller readiness
  bool speed_fresh=false;           // actual measured, not modeled
  bool selector_valid=false;
  uint16_t selector_width_ms=0;
  uint16_t hardware_max_width_ms=0; // caller supplies tighter of existing limits
};
struct Choice {
  bool selected=false;
  uint16_t width_ms=0;
  int16_t current_ma=0;
  int8_t motion_sign=0;             // +1 relative to the zero-cross motion
  float deficit_deg=NAN;            // observation, NOT predicted response
};
// Keep current tested 300mA and existing negative hard limit. A pair of prior
// confirmed negative peaks prevents an initial seed/half step from masquerading
// as a developed weak side. No model-support cutoffs in rate, voltage, or rpm.
inline Choice choose(const Input& in) {
  Choice out;
  if(in.target_preset_deg!=8.f || in.physical_next_side!=-1 ||
     !in.hardware_healthy || !in.speed_fresh || in.negative_peak_count<2 ||
     !std::isfinite(in.active_target_deg) || in.active_target_deg<=0.f ||
     !std::isfinite(in.last_negative_peak_deg) || in.last_negative_peak_deg<=0.f ||
     !std::isfinite(in.last_positive_peak_deg) || in.last_positive_peak_deg<=0.f ||
     !std::isfinite(in.crossing_rate_dps) || in.crossing_rate_dps<=0.f ||
     !std::isfinite(in.wheel_rpm) || !std::isfinite(in.battery_mv) || in.battery_mv<=0.f ||
     !in.last_negative_peak_ms || in.now_ms<in.last_negative_peak_ms ||
     in.now_ms-in.last_negative_peak_ms>1500 ||
     in.last_negative_peak_deg>=16.f || in.last_positive_peak_deg>=16.f ||
     in.hardware_max_width_ms==0) return out;
  // A normal powered choice, whether excitation or braking, is left to its
  // own selector. The weak-side assist is only a replacement for no pulse.
  if(in.selector_valid && in.selector_width_ms>0)return out;
  const float deficit=in.active_target_deg-in.last_negative_peak_deg;
  if(!(deficit>.5f))return out;
  // Explicitly experimental proportional fallback: 20ms at 0.5 degrees of
  // deficit; 15ms/degree, capped by 70ms and the caller's hardware limit.
  const float raw=20.f+15.f*(deficit-.5f);
  const float limit=std::fmin(70.f,static_cast<float>(in.hardware_max_width_ms));
  const float bounded=std::fmin(limit,std::fmax(20.f,raw));
  out.width_ms=static_cast<uint16_t>(std::floor(bounded+.5f));
  out.selected=true;out.current_ma=300;out.motion_sign=1;out.deficit_deg=deficit;
  return out;
}
}
