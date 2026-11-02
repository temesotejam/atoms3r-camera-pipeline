#include "weak_side_assist.h"
#include <cassert>
#include <cmath>
#include <iostream>
using weak_side_assist::Input;
using weak_side_assist::choose;
int main() {
  Input x;x.target_preset_deg=8.f;x.active_target_deg=5.5f;x.physical_next_side=-1;
  x.last_negative_peak_deg=3.44f;x.last_negative_peak_ms=21098;x.now_ms=21726;
  x.negative_peak_count=29;x.crossing_rate_dps=22.6566f;x.wheel_rpm=264.3f;
  x.battery_mv=7870.f;x.last_positive_peak_deg=5.57611f;
  x.hardware_healthy=true;x.speed_fresh=true;x.selector_valid=true;x.selector_width_ms=0;x.hardware_max_width_ms=70;
  auto a=choose(x);assert(a.selected && a.width_ms==43 && a.current_ma==300 && a.motion_sign==1);
  // Reproduce low-rpm measured coast with NO learned-model range restriction.
  x.now_ms=24787;x.last_negative_peak_ms=24184;x.last_negative_peak_deg=3.64441f;x.wheel_rpm=171.72f;x.selector_valid=false;
  a=choose(x);assert(a.selected && a.width_ms==40);
  x.wheel_rpm=-10.f;assert(choose(x).selected); // negative wheel direction is NOT a support cutoff
  // A known powered normal choice is retained, even if it is opposing/braking.
  x.selector_valid=true;x.selector_width_ms=45;assert(!choose(x).selected);
  x.selector_width_ms=0;x.last_negative_peak_deg=5.1f;assert(!choose(x).selected);
  x.last_negative_peak_deg=5.0f;assert(!choose(x).selected); // equality at deadband
  x.last_negative_peak_deg=1.0f;a=choose(x);assert(a.selected && a.width_ms==70);
  x.hardware_max_width_ms=35;a=choose(x);assert(a.selected && a.width_ms==35);
  x.hardware_max_width_ms=19;assert(choose(x).selected && choose(x).width_ms==19);
  x.hardware_max_width_ms=0;assert(!choose(x).selected);
  x.hardware_max_width_ms=70;x.hardware_healthy=false;assert(!choose(x).selected);
  x.hardware_healthy=true;x.speed_fresh=false;assert(!choose(x).selected);
  x.speed_fresh=true;x.wheel_rpm=NAN;assert(!choose(x).selected);
  x.wheel_rpm=170.f;x.battery_mv=0;assert(!choose(x).selected);
  x.battery_mv=7870.f;x.crossing_rate_dps=NAN;assert(!choose(x).selected);
  x.crossing_rate_dps=25.f;x.physical_next_side=1;assert(!choose(x).selected);
  x.physical_next_side=-1;x.target_preset_deg=10.f;assert(!choose(x).selected);
  x.target_preset_deg=8.f;x.negative_peak_count=1;assert(!choose(x).selected);
  x.negative_peak_count=2;x.last_negative_peak_ms=x.now_ms+1;assert(!choose(x).selected);
  x.last_negative_peak_ms=x.now_ms-1501;assert(!choose(x).selected);
  x.last_negative_peak_ms=x.now_ms-1500;assert(choose(x).selected);
  x.last_positive_peak_deg=16.f;assert(!choose(x).selected);
  x.last_positive_peak_deg=5.f;x.last_negative_peak_deg=NAN;assert(!choose(x).selected);
  std::cout << "weak side assist: measured low-RPM cases, pulses, guards, stale peaks PASS\n";
}
