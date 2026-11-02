#include "tilt_priority.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  tilt_priority::Controller c;
  tilt_priority::Input in;
  in.now_ms=1000;in.preset_deg=8;in.target_deg=8;in.next_side=1;
  in.hardware_cap_ms=150;in.hardware_healthy=true;
  auto out=c.propose(in);
  assert(out.selected && out.current_ma==1200 && out.direction==1 && out.width_ms==100);
  // The completed plus cycle provides a 210-30=180 ms timing allowance,
  // which is still limited by the authorized hardware maximum of 150 ms.
  c.crossing(1,1000);c.peak(1,1210,5.f);in.now_ms=1600;
  out=c.propose(in);assert(out.selected && out.width_ms==150 && out.current_ma==1200);
  // The minus side has its own independent history and sign.
  in.next_side=-1;out=c.propose(in);
  assert(out.selected && out.current_ma==1200 && out.direction==-1 && out.width_ms==100);
  c.crossing(-1,1600);c.peak(-1,1776,4.f);in.now_ms=2000;
  out=c.propose(in);assert(out.selected && out.direction==-1 && out.width_ms==146);
  // As the same-side peak approaches 8 degrees, reduce the pulse width.
  c.crossing(-1,2000);c.peak(-1,2200,7.f);in.now_ms=2400;
  out=c.propose(in);assert(out.selected && out.width_ms==97 && out.current_ma==1200);
  c.crossing(-1,2400);c.peak(-1,2600,7.5f);in.now_ms=2700;
  assert(!c.propose(in).selected);
  // An actual crossing followed by an unusually early peak does not justify
  // emitting a longer pulse; an unpaired peak falls back to the 100 ms seed.
  c.crossing(-1,2700);c.peak(-1,2725,4.f);in.now_ms=2800;
  assert(!c.propose(in).selected);
  c.peak(-1,2820,4.f);in.now_ms=2900;
  assert(c.propose(in).selected && c.propose(in).width_ms==100);
  // Stale or future timestamps cannot be used as a calibrated time limit.
  in.now_ms=5000;assert(c.propose(in).width_ms==100);
  in.now_ms=100;assert(c.propose(in).width_ms==100);
  in.hardware_cap_ms=60;assert(c.propose(in).width_ms==60);
  in.hardware_cap_ms=0;assert(!c.propose(in).selected);
  in.hardware_cap_ms=150;in.hardware_healthy=false;assert(!c.propose(in).selected);
  in.hardware_healthy=true;in.preset_deg=10;assert(!c.propose(in).selected);
  in.preset_deg=8;in.next_side=0;assert(!c.propose(in).selected);
  in.next_side=1;in.target_deg=NAN;assert(!c.propose(in).selected);
  c.reset();in.target_deg=8;assert(c.propose(in).width_ms==100);
  std::cout << "tilt priority: independent peaks, timing and healthy guards PASS\n";
}
