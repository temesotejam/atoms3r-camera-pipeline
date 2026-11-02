#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static uint16_t tick(Controller& x,unsigned t,float l,float r,float peak=5.f,
                     uint16_t base=100,uint16_t cap=100,unsigned n=30,
                     bool complete=true,float target=1.f,int16_t ma=400,unsigned period=100) {
  const auto w=x.limit(t+period/2,base,ma,1,cap,true,target);
  x.confirm(t+period/2,true,ma,1,w);
  Cycle c;c.start_ms=t;c.end_ms=t+period;c.positive_ms=t+period/2;
  c.left=l;c.right=r;c.negative_peak=peak;c.target_mj=target;
  c.negative_key=0x80000000u | (300u<<16) | 0x8000u | n;
  c.positive_width=w;c.positive_ma=ma;c.positive_sign=1;
  c.complete=complete;c.negative_known=c.positive_executed=true;
  x.cycle(c);return w;
}
int main(){
  // A good normalized-ratio response with both weaker foot and body peak
  // retained permits another step. The observed timing includes one cycle
  // overlapping the command and two full later cycles.
  Controller good;assert(tick(good,100,3,1)==100);assert(tick(good,200,3,1)==100);
  assert(good.offset()==-10 && good.acceptedProbeCycles()==0);
  assert(tick(good,300,2.5,1.1)==90 && good.acceptedProbeCycles()==1);
  tick(good,400,2.5,1.1);assert(good.offset()==-10);
  tick(good,500,2.5,1.1);assert(good.offset()==-20);
  // Reducing both feet halves absolute disparity, but does not improve the
  // normalized ratio; the weaker foot is also lost. Undo the trial.
  Controller shrink;tick(shrink,100,3,1);tick(shrink,200,3,1);
  tick(shrink,300,1.5,.5);tick(shrink,400,1.5,.5);tick(shrink,500,1.5,.5);
  assert(shrink.offset()==0 && tick(shrink,600,3,1)==100);
  // Ratio improves slightly while weaker foot falls: must still unwind.
  Controller weak;tick(weak,100,3,1);tick(weak,200,3,1);
  tick(weak,300,2,.7f);tick(weak,400,2,.7f);tick(weak,500,2,.7f);
  assert(weak.offset()==0);
  // Body oscillation falls although observed ratio improves.
  Controller peak;tick(peak,100,3,1);tick(peak,200,3,1);
  tick(peak,300,2.5,1.1,4);tick(peak,400,2.5,1.1,4);tick(peak,500,2.5,1.1,4);
  assert(peak.offset()==0);
  // Stagnation reverses the next proposal. Under a non-saturated 80ms
  // ordinary baseline this is an actual -10 followed by +10 experiment.
  Controller reverse;tick(reverse,100,3,1,5,80,100);tick(reverse,200,3,1,5,80,100);
  assert(tick(reverse,300,3,1,5,80,100)==70);
  tick(reverse,400,3,1,5,80,100);tick(reverse,500,3,1,5,80,100);
  assert(reverse.offset()==0);assert(tick(reverse,600,3,1,5,80,100)==80);
  tick(reverse,700,3,1,5,80,100);assert(reverse.offset()==10);
  assert(tick(reverse,800,3,1,5,80,100)==90);
  // Fully blocked +10 at the upper cap explicitly tests the alternative.
  Controller capped;tick(capped,100,1,3,5,80,80);tick(capped,200,1,3,5,80,80);
  assert(capped.offset()==10);assert(tick(capped,300,1,3,5,80,80)==70);
  // Missing observation abandons the unverified candidate, then reacquires
  // without requesting an emergency stop.
  Controller missing;tick(missing,100,3,1);tick(missing,200,3,1);
  tick(missing,300,2.5,1.1);tick(missing,400,2.5,1.1,5,100,100,30,false);
  assert(missing.offset()==0 && !missing.fault());assert(tick(missing,500,3,1)==100);
  tick(missing,600,3,1);tick(missing,700,3,1);assert(!missing.fault());
  // The ordinary selector can jump or change current without the experiment
  // asserting a hardware fault. Incomparable observations undo the pending
  // candidate and later valid data may restart it.
  Controller modes;tick(modes,100,3,1,5,80,100);tick(modes,200,3,1,5,80,100);
  assert(tick(modes,300,2.5,1.1,5,80,100)==70);
  assert(tick(modes,400,2.5,1.1,5,95,100,45,true,1.f,300)>0);
  assert(!modes.fault());
  assert(tick(modes,500,3,1,5,95,100,30,true,1.f,300)>0);
  assert(!modes.fault());
  // A canceled candidate is not recorded as sent and cannot count as evidence.
  Controller cancel;tick(cancel,100,3,1);tick(cancel,200,3,1);
  assert(cancel.limit(350,100,400,1,100,true,1)==90);
  assert(cancel.cancelUnsent(100));cancel.confirm(350,true,400,1,100);
  assert(cancel.acceptedProbeCycles()==0 && !cancel.fault());
  // A failed actual experimental command keeps the pre-existing fault path.
  Controller fault;tick(fault,100,3,1);tick(fault,200,3,1);
  assert(fault.limit(350,100,400,1,100,true,1)==90);
  fault.confirm(350,false,400,1,0);assert(fault.requireNeutral());
  assert(fault.limit(450,100,400,1,100,true,1)==0);
  // Delayed old cycle cannot be counted after a newer confirmed adjustment.
  Controller late;tick(late,100,3,1);tick(late,200,3,1);
  assert(late.limit(350,100,400,1,100,true,1)==90);late.confirm(350,true,400,1,90);
  Cycle old;old.start_ms=200;old.end_ms=300;old.positive_ms=250;
  old.left=2;old.right=1;old.negative_peak=5;old.target_mj=1;old.negative_key=1;
  old.positive_width=100;old.positive_ma=400;old.positive_sign=1;
  old.complete=old.negative_known=old.positive_executed=true;late.cycle(old);
  assert(late.offset()==-10 && late.acceptedProbeCycles()==1);
  // Approximate 30-second event cadence and changing negative/ordinary inputs
  // based on 38921428. Values are replayed as fixed observations only; this
  // cannot predict how the physical robot would respond to changed outputs.
  Controller loglike;unsigned changes=0,held=0;
  const float left[]={3.104f,1.79119f,2.56245f,2.06484f,2.65809f,1.98468f,2.5591f,
    3.02064f,2.91008f,2.91475f,3.07414f,2.72823f,2.91936f,3.32827f,3.28772f,
    3.61458f,2.25551f,2.8821f,2.63781f,2.81539f,2.61764f,2.68611f,2.8618f,
    2.57976f,3.42457f,3.87727f,3.71694f,3.39015f,3.75461f};
  const float right[]={.86756f,.71473f,.65353f,.53508f,.61538f,.54957f,.40704f,
    .50698f,.38408f,.29898f,.34116f,.5016f,.36277f,.36078f,.27911f,.28798f,
    .3378f,.29122f,.3698f,.30871f,.15963f,.21935f,.23246f,.22305f,.47784f,
    .3307f,.30971f,.39436f,.5821f};
  for(unsigned i=0;i<29;++i){
    const uint16_t base=i<4?85:i<9?90:i<14?95:100;
    const unsigned neg=i<4?25:i<9?30:i<14?40:i<23?30:45;
    const auto w=tick(loglike,100+i*800,left[i],right[i],4.5f,base,100,neg,true,1.f,400,800);
    assert(w>0 && w<=100 && !loglike.fault());
    if(w!=base)++changes;else ++held;
  }
  assert(changes>0 && held>0 && loglike.acceptedProbeCycles()>0);
  std::cout<<"foot span: ratio response, weak side, time matching and recovery PASS\n";
}
