#include "foot_span_control.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace foot_span_control;
static Cycle cycle(uint32_t start,float left,float right,uint16_t width,
                   int16_t current=400,uint32_t negative=1,float peak=5.f) {
  Cycle x;x.start_ms=start;x.end_ms=start+100;x.positive_ms=start+50;
  x.left=left;x.right=right;x.negative_peak=peak;x.target_mj=1.f;
  x.negative_key=negative;x.positive_width=width;x.positive_ma=current;x.positive_sign=1;
  x.complete=x.negative_known=x.positive_executed=true;return x;
}
static uint16_t command(Controller& x,uint32_t when,uint16_t base=80,
                        int16_t ma=400,uint16_t cap=100,bool allowed=true) {
  auto n=x.limit(when,base,ma,1,cap,allowed,1.f);
  x.confirm(when,n>0,ma,1,n);return n;
}
int main(){
  Controller a;
  assert(command(a,150)==80);a.cycle(cycle(100,3,1,80));
  assert(command(a,250)==80);a.cycle(cycle(200,3,1,80));
  assert(a.offset()==-10);assert(a.acceptedProbeCycles()==0);assert(command(a,350)==70);
  assert(a.acceptedProbeCycles()==1); // actual matching command, not proposal
  a.cycle(cycle(300,3,1,70));assert(a.offset()==-10); // overlaps command
  assert(command(a,450)==70);a.cycle(cycle(400,3,1,70,400,2)); // different negative mode
  assert(a.offset()==-20);assert(command(a,550)==60);
  a.cycle(cycle(500,3,1,60,400,3));assert(command(a,650)==60);
  a.cycle(cycle(600,3,1,60,400,4));assert(a.offset()==-30);
  assert(command(a,750)==50);a.cycle(cycle(700,3,1,50));
  assert(command(a,850)==50);a.cycle(cycle(800,3,1,50));
  assert(a.offset()==-30); // conservative offset limit, no artificial model-domain latch
  // Multiple completed cycles delivered together must not accumulate two
  // adjustments before one altered motor command has actually been issued.
  Controller delayed;
  assert(command(delayed,150)==80);delayed.cycle(cycle(100,3,1,80));
  assert(command(delayed,250)==80);assert(command(delayed,350)==80);
  delayed.cycle(cycle(200,3,1,80));assert(delayed.offset()==-10);
  delayed.cycle(cycle(300,3,1,80));assert(delayed.offset()==-10);
  assert(command(delayed,450)==70);
  // Opposite measured asymmetry selects the opposite correction.
  Controller b;assert(command(b,150)==80);b.cycle(cycle(100,1,3,80));
  assert(command(b,250)==80);b.cycle(cycle(200,1,3,80));
  assert(b.offset()==10);assert(command(b,350)==90);
  // Equal spans do not force an intervention.
  Controller equal;assert(command(equal,150)==80);equal.cycle(cycle(100,2,2,80));
  assert(command(equal,250)==80);equal.cycle(cycle(200,2,2,80));assert(command(equal,350)==80);
  // Startup, missing data and fabricated or duplicate commands do not update.
  Controller missing;auto fake=cycle(100,3,1,80);missing.cycle(fake);
  assert(command(missing,250)==80);auto invalid=cycle(200,3,1,80);invalid.complete=false;missing.cycle(invalid);
  assert(command(missing,350)==80);missing.cycle(cycle(300,3,1,80));
  assert(missing.offset()==0);missing.cycle(cycle(300,3,1,80));assert(missing.offset()==0);
  // 2 consecutive weak peaks cause the applied offset to reduce, rather than
  // interpreting stationary or collapsed gait as a successful balance.
  Controller weak;assert(command(weak,150)==80);weak.cycle(cycle(100,3,1,80));
  assert(command(weak,250)==80);weak.cycle(cycle(200,3,1,80));
  assert(command(weak,350)==70);weak.cycle(cycle(300,3,1,70,400,1,4.f));
  assert(command(weak,450)==70);weak.cycle(cycle(400,3,1,70,400,1,4.f));
  assert(command(weak,550)==60);weak.cycle(cycle(500,3,1,60,400,1,4.f));
  assert(command(weak,650)==60);weak.cycle(cycle(600,3,1,60,400,1,4.f));
  assert(weak.offset()==-10); // one step toward baseline after two post-change weak peaks
  // The requested correction may legitimately lower the larger left span
  // while maintaining the right span. A decrease in their sum alone is not
  // gait collapse and must not undo a closing left-right difference.
  Controller closing;assert(command(closing,150)==80);closing.cycle(cycle(100,3,1,80));
  assert(command(closing,250)==80);closing.cycle(cycle(200,3,1,80));
  assert(command(closing,350)==70);closing.cycle(cycle(300,2.2f,1,70));
  assert(command(closing,450)==70);closing.cycle(cycle(400,2.2f,1,70));
  assert(closing.offset()==-20);assert(command(closing,550)==60);
  closing.cycle(cycle(500,1.6f,1,60));assert(command(closing,650)==60);
  closing.cycle(cycle(600,1.6f,1,60));assert(closing.offset()==-30);
  // Both feet falling to less than 35% of the baseline twice DOES trigger
  // reduction of the correction even if the body peak has not yet fallen.
  Controller collapse;assert(command(collapse,150)==80);collapse.cycle(cycle(100,3,1,80));
  assert(command(collapse,250)==80);collapse.cycle(cycle(200,3,1,80));
  assert(command(collapse,350)==70);collapse.cycle(cycle(300,.3f,.1f,70));
  assert(command(collapse,450)==70);collapse.cycle(cycle(400,.3f,.1f,70));
  assert(command(collapse,550)==60);collapse.cycle(cycle(500,.3f,.1f,60));
  assert(command(collapse,650)==60);collapse.cycle(cycle(600,.3f,.1f,60));
  assert(collapse.offset()==-10);
  // Two successive worsened observations invert the exploratory response
  // direction; the absolute outcome remains an empirical hardware question.
  Controller trend;assert(command(trend,150)==80);trend.cycle(cycle(100,3,1,80));
  assert(command(trend,250)==80);trend.cycle(cycle(200,3,1,80));
  assert(command(trend,350)==70);trend.cycle(cycle(300,3,1,70));
  assert(command(trend,450)==70);trend.cycle(cycle(400,3.3f,1,70));
  assert(trend.offset()==-20);assert(command(trend,550)==60);
  trend.cycle(cycle(500,3.3f,1,60));assert(command(trend,650)==60);
  trend.cycle(cycle(600,3.6f,1,60));assert(trend.offset()==-10); // reversed
  // Error reverses because the other foot has the larger span: the
  // correction must move back through zero and toward the other direction.
  Controller reversal;assert(command(reversal,150)==80);reversal.cycle(cycle(100,3,1,80));
  assert(command(reversal,250)==80);reversal.cycle(cycle(200,3,1,80));
  assert(command(reversal,350)==70);reversal.cycle(cycle(300,1,5,70));
  assert(command(reversal,450)==70);reversal.cycle(cycle(400,1,5,70));
  assert(reversal.offset()==0);assert(command(reversal,550)==80);
  // An output mismatch must immediately demand a neutral/fault action.
  Controller fault;assert(command(fault,150)==80);fault.cycle(cycle(100,3,1,80));
  assert(command(fault,250)==80);fault.cycle(cycle(200,3,1,80));
  assert(fault.limit(350,80,400,1,100,true,1.f)==70);fault.confirm(350,false,400,1,0);
  assert(fault.requireNeutral());assert(fault.limit(450,80,400,1,100,true,1.f)==0);
  fault.reset();assert(!fault.requireNeutral());
  // Missing camera evidence decays the offset to baseline without a permanent
  // lock. A later pair of valid cycles may restart adaptation.
  Controller stale;assert(command(stale,150)==80);stale.cycle(cycle(100,3,1,80));
  assert(command(stale,250)==80);stale.cycle(cycle(200,3,1,80));
  assert(command(stale,350)==70);assert(command(stale,3000)==80);assert(stale.offset()==0);
  // Narrower safety cap always wins; no request can exceed it.
  Controller cap;assert(command(cap,150)==80);cap.cycle(cycle(100,1,3,80));
  assert(command(cap,250)==80);cap.cycle(cycle(200,1,3,80));
  assert(cap.limit(350,80,400,1,75,true,1.f)<=75);
  // When the intended +10ms direction is entirely blocked by cap=baseline,
  // explicitly try -10ms and adapt further only after real feedback. This
  // is an experiment, not evidence that the opposite direction works.
  Controller windup;assert(command(windup,150,80,400,80)==80);windup.cycle(cycle(100,1,3,80));
  assert(command(windup,250,80,400,80)==80);windup.cycle(cycle(200,1,3,80));
  assert(windup.offset()==10 && windup.acceptedProbeCycles()==0);
  assert(command(windup,350,80,400,80)==70);
  assert(windup.offset()==-10 && windup.acceptedProbeCycles()==1);
  windup.cycle(cycle(300,1,3,70));assert(command(windup,450,80,400,80)==70);
  windup.cycle(cycle(400,1,3,70));assert(command(windup,550,80,400,80)==60);
  assert(windup.offset()==-20);assert(command(windup,650,80,400,100)==60);
  // Symmetric lower bound: never ask for a negative pulse; test +10ms.
  Controller lower;assert(command(lower,150,10,400,80)==10);lower.cycle(cycle(100,3,1,10));
  assert(command(lower,250,10,400,80)==10);lower.cycle(cycle(200,3,1,10));
  assert(lower.offset()==-10 && lower.acceptedProbeCycles()==0);
  assert(command(lower,350,10,400,80)==20 && lower.acceptedProbeCycles()==1);
  // Partial room applies only the reachable +5ms, and repeated attempts
  // blocked at that same cap must not count another confirmed change.
  Controller partial;assert(command(partial,150,80,400,85)==80);partial.cycle(cycle(100,1,3,80));
  assert(command(partial,250,80,400,85)==80);partial.cycle(cycle(200,1,3,80));
  assert(command(partial,350,80,400,85)==85 && partial.acceptedProbeCycles()==1);
  partial.cycle(cycle(300,1,3,85));assert(command(partial,450,80,400,85)==85);
  partial.cycle(cycle(400,1,3,85));assert(command(partial,550,80,400,85)==85);
  assert(partial.acceptedProbeCycles()==1);
  // A cap lowered after an applied offset must be confirmed as a separate
  // transition. A later blocked proposal can then explore the other side.
  Controller shrinking;assert(command(shrinking,150)==80);shrinking.cycle(cycle(100,1,3,80));
  assert(command(shrinking,250)==80);shrinking.cycle(cycle(200,1,3,80));
  assert(command(shrinking,350)==90 && shrinking.acceptedProbeCycles()==1);
  assert(command(shrinking,450,80,400,80)==80 && shrinking.acceptedProbeCycles()==2);
  shrinking.cycle(cycle(300,1,3,90));shrinking.cycle(cycle(400,1,3,80));
  assert(command(shrinking,550,80,400,80)==80);shrinking.cycle(cycle(500,1,3,80));
  assert(shrinking.offset()==10);assert(command(shrinking,650,80,400,80)==70);
  assert(shrinking.acceptedProbeCycles()==3);
  // A hard cap always limits the result even if the normal selector or
  // an offset causes a >10ms change from the previous physical command.
  Controller hard;assert(command(hard,150)==80);hard.cycle(cycle(100,3,1,80));
  assert(command(hard,250)==80);hard.cycle(cycle(200,3,1,80));
  assert(command(hard,350)==70);assert(hard.selectorUnavailable(450));
  hard.confirm(450,true,400,1,100);assert(hard.limit(550,80,400,1,75,true,1.f)==65);
  hard.confirm(550,true,400,1,65);assert(!hard.fault());
  // Current and model changes do not create a permanent out-of-range stop.
  Controller modes;assert(command(modes,150)==80);modes.cycle(cycle(100,3,1,80));
  assert(command(modes,250)==80);modes.cycle(cycle(200,3,1,80));
  assert(command(modes,350)==70);assert(modes.selectorUnavailable(450));
  modes.confirm(450,true,300,1,73);assert(!modes.requireNeutral());
  assert(modes.limit(550,73,300,1,100,true,1.f)==63);
  modes.confirm(550,true,300,1,63);assert(!modes.requireNeutral());
  // Current changes in 304's ordinary selector (400->300->400->350)
  // must not reset a maintained correction or suppress ordinary walking.
  // Same width at different current does not imply equal physical work.
  assert(a.offset()==-30);assert(a.limit(950,73,300,1,100,true,1.f)==43);
  a.confirm(950,true,300,1,43);a.cycle(cycle(900,3,1,43,300));
  assert(a.offset()==-30);assert(a.limit(1050,85,400,1,100,true,1.f)==55);
  a.confirm(1050,true,400,1,55);a.cycle(cycle(1000,3,1,55,400));
  assert(a.offset()==-30);assert(a.limit(1150,82,350,1,100,true,1.f)==52);
  a.confirm(1150,true,350,1,52);a.cycle(cycle(1100,3,1,52,350));
  assert(a.offset()==-30 && !a.fault());
  // Slow but consistent worsening (>0.20 degrees across several complete
  // observations) must reverse direction; no 0.15-degree individual jump.
  Controller slow;assert(command(slow,150)==80);slow.cycle(cycle(100,3,1,80));
  assert(command(slow,250)==80);slow.cycle(cycle(200,3,1,80));
  assert(command(slow,350)==70);slow.cycle(cycle(300,3.04f,1,70));
  assert(command(slow,450)==70);slow.cycle(cycle(400,3.04f,1,70));
  assert(command(slow,550)==60);slow.cycle(cycle(500,3.08f,1,60));
  assert(command(slow,650)==60);slow.cycle(cycle(600,3.08f,1,60));
  assert(command(slow,750)==50);slow.cycle(cycle(700,3.12f,1,50));
  assert(command(slow,850)==50);slow.cycle(cycle(800,3.12f,1,50));
  assert(command(slow,950)==50);slow.cycle(cycle(900,3.16f,1,50));
  assert(command(slow,1050)==50);slow.cycle(cycle(1000,3.20f,1,50));
  assert(slow.offset()>-30); // polarity reversed despite gradual deterioration
  assert(command(slow,1150)==60);slow.cycle(cycle(1100,3.24f,1,60));
  // A baseline-width change is an actual input change too: the delayed cycle
  // containing or preceding it cannot be used as post-change feedback.
  Controller timing;assert(command(timing,150)==80);timing.cycle(cycle(100,3,1,80));
  assert(command(timing,250)==80);timing.cycle(cycle(200,3,1,80));
  assert(command(timing,350)==70);timing.cycle(cycle(300,3,1,70));
  assert(command(timing,450,85)==75);timing.cycle(cycle(400,3,1,75));
  assert(timing.offset()==-20); // ordinary selector change still allows feedback
  assert(command(timing,550,85)==65);timing.cycle(cycle(500,3,1,65));
  assert(timing.offset()==-20); // cycle overlaps feedback adjustment at 550
  // A future ordinary baseline command must not cause an earlier complete
  // cycle delivered late to be discarded. Feedback causality uses the epoch
  // of the actual command inside that cycle, not a global last-input time.
  Controller late;assert(command(late,150)==80);late.cycle(cycle(100,3,1,80));
  assert(command(late,250)==80);late.cycle(cycle(200,3,1,80));
  assert(command(late,350)==70);late.cycle(cycle(300,3,1,70));
  assert(command(late,450)==70);
  assert(command(late,550,81)==71); // future ordinary +1 ms baseline change
  late.cycle(cycle(400,3,1,70));assert(late.offset()==-20);
  // A series of ordinary 1ms selector changes is allowed to supply feedback;
  // they are tracked as physical input changes, not attributed causes.
  Controller jitterbase;assert(command(jitterbase,150)==80);jitterbase.cycle(cycle(100,3,1,80));
  assert(command(jitterbase,250)==80);jitterbase.cycle(cycle(200,3,1,80));
  assert(command(jitterbase,350)==70);jitterbase.cycle(cycle(300,3,1,70));
  assert(command(jitterbase,450,81)==71);jitterbase.cycle(cycle(400,3,1,71));
  assert(jitterbase.offset()==-20);
  // A cancelled, unsent candidate falls back to the ordinary choice.
  Controller cancel;assert(command(cancel,150)==80);cancel.cycle(cycle(100,3,1,80));
  assert(command(cancel,250)==80);cancel.cycle(cycle(200,3,1,80));
  assert(cancel.limit(350,80,400,1,100,true,1.f)==70);
  assert(cancel.cancelUnsent(80));cancel.confirm(350,true,400,1,80);assert(!cancel.fault());
  assert(cancel.acceptedProbeCycles()==0);
  // A cancelled candidate is never treated as a confirmed adaptation or
  // accumulated into a subsequent larger step.
  assert(cancel.offset()==0);assert(command(cancel,450)==80);
  // The ordinary selector may itself change by >10ms; the experimental
  // offset limit does not suppress or claim to bound that baseline change.
  Controller jumps;assert(command(jumps,150)==80);jumps.cycle(cycle(100,3,1,80));
  assert(command(jumps,250)==80);jumps.cycle(cycle(200,3,1,80));
  assert(command(jumps,350)==70);assert(jumps.selectorUnavailable(450));
  jumps.confirm(450,true,400,1,100);
  assert(jumps.limit(550,80,400,1,100,true,1.f)==70);
  jumps.confirm(550,true,400,1,70);assert(!jumps.fault());
  std::cout<<"foot span controller: accumulating feedback, feedback sign, fault and recovery PASS\n";
}
