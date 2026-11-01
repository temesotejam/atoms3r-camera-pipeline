#include "../src/foot_phase_cap.h"
#include <cassert>
#include <iostream>
using namespace foot_phase_cap;
static void pair(Controller& c,unsigned end,float left=2.2f,float right=-.1f,float neg=4.7f,bool neg_output=true) {
  c.pair(end,end-800,left,right,neg,true,neg_output,true);
}
static unsigned limit(Controller& c,unsigned now,unsigned cap=80,float target=.4f){return c.limit(now,80,400,1,cap,true,target);}
static void confirm(Controller& c,unsigned now,unsigned width){c.confirm(now,true,400,1,width);}
static void baseline(Controller& c,unsigned now){assert(limit(c,now)==80);confirm(c,now,80);}
static void prepare(Controller& c){baseline(c,900);pair(c,1000);baseline(c,1700);pair(c,1800);}
static void heldCycle(Controller& c,unsigned end,unsigned width,float l,float r,float n=4.7f) {
  unsigned t=end-750;assert(limit(c,t)==width);confirm(c,t,width);pair(c,end,l,r,n);
}
int main(){
  Controller waiting;baseline(waiting,900);pair(waiting,1000);
  assert(limit(waiting,1100)==80);baseline(waiting,1700);pair(waiting,1800);
  assert(waiting.limit(1850,80,400,1,80,false,.4f)==80 && waiting.state()==State::Waiting);
  // Start after two valid consecutive pairs and an actual 80ms baseline.
  Controller c;prepare(c);assert(limit(c,1850)==75);confirm(c,1850,75);
  pair(c,2600); // starts before change: cannot be used as counterfactual evidence
  heldCycle(c,3400,75,1.8f,.4f,4.6f);heldCycle(c,4200,75,1.8f,.4f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,4250)==70);confirm(c,4250,70);
  pair(c,5000,1.8f,.4f,4.6f); // overlapping transition pair is excluded
  heldCycle(c,5800,70,1.4f,.8f,4.6f);heldCycle(c,6600,70,1.4f,.8f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,6650)==65);confirm(c,6650,65);
  pair(c,7400,1.4f,.8f,4.6f);
  heldCycle(c,8200,65,1.2f,1.f,4.6f);heldCycle(c,9000,65,1.2f,1.f,4.6f);
  assert(c.state()==State::Hold);assert(limit(c,9050)==65);confirm(c,9050,65);
  c.tick(11500);assert(c.state()==State::Release && c.reason()==Reason::Stale);
  assert(limit(c,11501)==70);confirm(c,11501,70);
  assert(limit(c,11502)==75);confirm(c,11502,75);
  assert(limit(c,11503)==80);confirm(c,11503,80);assert(c.state()==State::Locked);
  // No response after two complete cycles restores the baseline.
  Controller flat;prepare(flat);assert(limit(flat,1850)==75);confirm(flat,1850,75);pair(flat,2600);
  heldCycle(flat,3400,75,2.2f,-.1f);heldCycle(flat,4200,75,2.2f,-.1f);
  assert(flat.state()==State::Release && flat.reason()==Reason::Inconclusive);
  assert(limit(flat,4250)==80);confirm(flat,4250,80);
  // Retain signed measurements, reject the false success where the weak foot
  // gets worse and both feet's forward movement collapses.
  Controller weakfoot;prepare(weakfoot);assert(limit(weakfoot,1850)==75);confirm(weakfoot,1850,75);pair(weakfoot,2600);
  heldCycle(weakfoot,3400,75,.5f,-.3f);
  assert(weakfoot.state()==State::Release && weakfoot.reason()==Reason::Worsened);
  Controller signedfoot;baseline(signedfoot,900);pair(signedfoot,1000,2,-.63f);baseline(signedfoot,1700);
  pair(signedfoot,1800,2,-.21f);assert(limit(signedfoot,1850)==75);
  // No probing after negative no-output, low-side trend collapse, duplicated
  // or overlapping pairs, absent baseline confirmation or missing sensors.
  Controller no_neg;baseline(no_neg,900);pair(no_neg,1000,2,0,4.7,false);baseline(no_neg,1700);
  pair(no_neg,1800,2,0,4.7,false);assert(limit(no_neg,1850)==80);
  Controller trend;baseline(trend,900);pair(trend,1000,2,0,4.7);baseline(trend,1700);
  pair(trend,1800,2,0,4.2);assert(limit(trend,1850)==80);
  Controller duplicate;baseline(duplicate,900);pair(duplicate,1000);pair(duplicate,1000);
  assert(limit(duplicate,1100)==80);duplicate.pair(1800,400,2,0,4.7,true,true,true);
  assert(limit(duplicate,1850)==80);
  Controller no_actual;pair(no_actual,1000);pair(no_actual,1800);assert(limit(no_actual,1850)==80);
  Controller lost;prepare(lost);assert(limit(lost,1850)==75);confirm(lost,1850,75);
  lost.negativeDecision(2000,true,false);
  assert(lost.state()==State::Release && lost.reason()==Reason::NegativeLoss);
  Controller weak;prepare(weak);assert(limit(weak,1850)==75);confirm(weak,1850,75);pair(weak,2600);
  heldCycle(weak,3400,75,2.2f,-.1f,4.1f);heldCycle(weak,4200,75,2.2f,-.1f,4.0f);
  assert(weak.state()==State::Release && weak.reason()==Reason::WeakPeak);
  // Hold must continue monitoring deterioration, rather than holding forever.
  Controller later;prepare(later);assert(limit(later,1850)==75);confirm(later,1850,75);pair(later,2600);
  heldCycle(later,3400,75,1.8f,.4f);heldCycle(later,4200,75,1.8f,.4f);
  assert(later.state()==State::Hold);later.negativeDecision(4250,true,false);
  assert(later.state()==State::Release && later.reason()==Reason::NegativeLoss);
  // A stricter existing cap can force an immediate drop. After confirming its
  // actual 60ms pulse, any increase must start from 60ms, not the old 75ms.
  Controller safety;prepare(safety);assert(limit(safety,1850)==75);confirm(safety,1850,75);
  assert(limit(safety,1900,60)==60);confirm(safety,1900,60);
  assert(limit(safety,1901)==65);confirm(safety,1901,65);assert(limit(safety,1902)==70);
  // An unexpected width must also become the restoration reference.
  Controller mismatch;prepare(mismatch);assert(limit(mismatch,1850)==75);mismatch.confirm(1850,true,400,1,70);
  assert(mismatch.state()==State::Release);assert(limit(mismatch,1851)==75);
  Controller bypass;prepare(bypass);assert(limit(bypass,1850)==75);confirm(bypass,1850,75);
  bypass.confirm(1900,true,400,1,80);assert(bypass.state()==State::Release);
  // A changed current makes a width-only safe restoration impossible; report
  // neutral-required so the caller can stop output rather than invent a pulse.
  Controller current;prepare(current);assert(limit(current,1850)==75);current.confirm(1850,true,350,1,75);
  assert(current.requireNeutral() && limit(current,1851)==0);
  // A changed baseline current must be blocked *before* a different-current
  // pulse can be sent; a change in target ends identification and restores.
  Controller current_before;prepare(current_before);assert(limit(current_before,1850)==75);confirm(current_before,1850,75);
  assert(current_before.limit(1900,80,350,1,80,true,.4f)==0 && !current_before.requireNeutral());
  current_before.confirm(1900,false,0,1,0);
  assert(current_before.limit(2700,80,350,1,80,true,.4f)==80);
  Controller branch;prepare(branch);assert(limit(branch,1850)==75);confirm(branch,1850,75);
  branch.modeChanged();branch.confirm(1900,false,0,1,0);
  assert(branch.state()==State::Locked && branch.reason()==Reason::ModeChanged && !branch.requireNeutral());
  assert(limit(branch,2700)==80);
  assert(branch.limit(2800,80,350,1,80,false,.4f)==80 && branch.state()==State::Locked);
  assert(branch.limit(2900,80,400,1,80,true,NAN)==80 && branch.state()==State::Locked);
  assert(limit(branch,3000)==80 && branch.state()==State::Locked);
  Controller changed_target;prepare(changed_target);assert(limit(changed_target,1850)==75);confirm(changed_target,1850,75);
  assert(limit(changed_target,1900,80,.41f)==80 && changed_target.reason()==Reason::TargetChanged);
  confirm(changed_target,1900,80);assert(changed_target.state()==State::Locked);
  // Different startup targets cannot be pooled as the two-cycle baseline.
  Controller differing;baseline(differing,900);pair(differing,1000);
  assert(limit(differing,1700,80,.42f)==80);confirm(differing,1700,80);pair(differing,1800);
  assert(limit(differing,1850,80,.42f)==80);
  // A stale pulse from a previous cycle cannot qualify a fresh baseline pair.
  Controller repeated;baseline(repeated,900);pair(repeated,1000);pair(repeated,1800);
  assert(limit(repeated,1850)==80);
  // A delayed completed image cycle can be delivered after the next positive
  // pulse. Match by decision timestamp, not whichever pulse arrived last.
  Controller delayed;prepare(delayed);assert(limit(delayed,1850)==75);confirm(delayed,1850,75);
  assert(limit(delayed,2650)==75);confirm(delayed,2650,75);pair(delayed,2600);
  assert(limit(delayed,3450)==75);confirm(delayed,3450,75);pair(delayed,3400,1.8f,.4f);
  assert(limit(delayed,4250)==75);confirm(delayed,4250,75);pair(delayed,4200,1.8f,.4f);
  assert(delayed.state()==State::Hold);
  // A decision exactly on the cycle boundary cannot be counted twice.
  Controller boundary;baseline(boundary,200);pair(boundary,1000);
  assert(limit(boundary,1000)==80);confirm(boundary,1000,80);pair(boundary,1800);
  assert(limit(boundary,1850)==80);
  Controller nan_wait;assert(nan_wait.limit(1000,80,400,1,80,true,NAN)==80 && !nan_wait.requireNeutral());
  Controller neutral;prepare(neutral);assert(limit(neutral,1850)==75);confirm(neutral,1850,75);
  assert(limit(neutral,1900)==75);neutral.confirm(1900,true,350,1,75);
  assert(neutral.requireNeutral());
  assert(neutral.limit(1901,80,400,1,80,false,.4f)==0);
  std::cout<<"Foot phase cap: consecutive measured cycles, executed actions, signed feet, response and recovery PASS\n";
}
