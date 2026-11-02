#include "positive_left_boost.h"
#include <cassert>
#include <cmath>
#include <iostream>
using positive_left_boost::Controller;
using positive_left_boost::Input;

static Input example() {
  Input x;x.now_ms=1000;x.opposite_peak_ms=800;x.opposite_peak_deg=5;
  x.preset_deg=8;x.side_target_deg=10;x.next_side=1;
  x.wheel_rpm=120;x.battery_mv=8000;x.hardware_healthy=true;x.speed_fresh=true;
  x.normal_valid=true;x.normal_motion_aligned=true;x.normal_ma=400;
  x.normal_width_ms=100;x.normal_cap_ms=100;x.hardware_cap_ms=150;
  return x;
}
static void normalCycle(Controller& c,Input& x,float result) {
  assert(!c.propose(x).selected);
  c.confirmNormal(x,400,x.normal_width_ms,1,true);
  c.peak(1,x.now_ms+200,result,true);
  x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
}
static void baseline(Controller& c,Input& x) {
  normalCycle(c,x,5.0f);normalCycle(c,x,5.2f);
  assert(c.baselineCount()==2);
}
int main() {
  { Controller c;auto x=example();baseline(c,x);
    auto b=c.propose(x);assert(b.selected && b.current_ma==500 && b.width_ms==100);
    c.confirmTrial(x,500,100,1,true);assert(c.awaitingPeak());
    // The post-command opposite peak cannot substitute for the positive one.
    assert(!c.propose(x).selected);
    c.peak(1,x.now_ms+200,5.5f,true);assert(c.awaitingPeak());
    x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected);
    c.peak(-1,x.now_ms-200,4.8f,true);assert(c.accepted());
    b=c.propose(x);assert(b.selected && b.width_ms==125);
    c.confirmTrial(x,500,125,1,true);c.peak(1,x.now_ms+200,5.7f,true);
    c.peak(-1,x.now_ms+600,4.8f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    b=c.propose(x);assert(b.selected && b.width_ms==150);
    c.confirmTrial(x,500,150,1,true);c.peak(1,x.now_ms+220,5.8f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    b=c.propose(x);assert(b.selected && b.width_ms==150);
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.1f,true);
    c.peak(-1,x.now_ms+600,5.1f,true);
    assert(!c.accepted());x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected); // no positive improvement: normal fallback
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.8f,true);
    c.peak(-1,x.now_ms+600,3.0f,true);
    assert(!c.accepted());x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected); // opposite side badly deteriorated
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).width_ms==100);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.7f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(c.propose(x).width_ms==125);
    c.confirmTrial(x,500,125,1,true);c.peak(1,x.now_ms+200,5.3f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(c.propose(x).width_ms==100); // longer trial worsened: retain last good
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.7f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(c.propose(x).width_ms==100); // no immediate retry of failed 125
  }
  { Controller c;auto x=example();x.hardware_cap_ms=120;baseline(c,x);
    assert(c.propose(x).width_ms==100);c.confirmTrial(x,500,100,1,true);
    c.peak(1,x.now_ms+200,5.6f,true);c.peak(-1,x.now_ms+600,4.9f,true);
    x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;assert(c.propose(x).width_ms==120);
    c.confirmTrial(x,500,120,1,true);c.peak(1,x.now_ms+200,5.7f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(c.propose(x).width_ms==120); // physical cap never exceeded
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).width_ms==100);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,9.7f,true);
    c.peak(-1,x.now_ms+600,4.9f,true);x.now_ms+=800;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected); // no boost once target has been reached
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,false);assert(!c.awaitingPeak() && !c.accepted());
    c.peak(1,x.now_ms+200,7,true);assert(c.baselineCount()==0);
    assert(!c.propose(x).selected); // failed send contributes no evidence
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,400,100,1,true);assert(!c.awaitingPeak()&&!c.accepted());
  }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,-1,true);assert(!c.awaitingPeak()&&!c.accepted());
  }
  { Controller c;auto x=example();c.confirmNormal(x,400,100,1,false);
    c.peak(1,x.now_ms+200,4,true);assert(c.baselineCount()==0);
    c.confirmNormal(x,400,95,1,true);c.peak(1,x.now_ms+200,4,true);
    assert(c.baselineCount()==0); // actual width must match normal candidate
  }
  { Controller c;auto x=example();baseline(c,x);x.hardware_healthy=false;
    assert(!c.propose(x).selected);x.hardware_healthy=true;
    assert(!c.propose(x).selected); // reacquire two real peaks after interruption
  }
  { Controller c;auto x=example();baseline(c,x);x.speed_fresh=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.wheel_rpm=NAN;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.battery_mv=0;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.opposite_peak_ms=x.now_ms-1501;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.normal_width_ms=90;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.hardware_cap_ms=99;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.normal_motion_aligned=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.side_target_deg=9.5f;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.normal_valid=false;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.wheel_rpm+=101;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();baseline(c,x);x.opposite_peak_deg+=.81f;assert(!c.propose(x).selected); }
  { Controller c;auto x=example();c.confirmNormal(x,400,100,1,true);
    c.peak(1,x.now_ms+40,5,true);assert(c.baselineCount()==0&&!c.awaitingPeak()); }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,5.5f,true);
    c.peak(-1,x.now_ms+600,4.9f,false);assert(!c.accepted()); }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);x.now_ms+=1801;x.opposite_peak_ms=x.now_ms-200;
    assert(!c.propose(x).selected && !c.awaitingPeak()); }
  { Controller c;auto x=example();baseline(c,x);assert(c.propose(x).selected);
    c.confirmTrial(x,500,100,1,true);c.peak(1,x.now_ms+200,16.f,true);
    assert(!c.awaitingPeak() && !c.accepted()); }
  std::cout<<"positive left boost: matched 400 baseline, one 500 trial, feedback and guards PASS\n";
}
