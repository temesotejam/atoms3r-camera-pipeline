#include "foot_span_control.h"
#include <cassert>
#include <iostream>
using namespace foot_span_control;
static Cycle c(uint32_t s,uint32_t e,float l,float r,uint16_t width,uint32_t neg=1) {
  Cycle x;x.start_ms=s;x.end_ms=e;x.positive_ms=(s+e)/2;x.left=l;x.right=r;x.negative_peak=5.f;x.target_mj=1.f;
  x.negative_key=neg;x.positive_width=width;x.positive_ma=400;x.positive_sign=1;
  x.complete=x.negative_known=x.positive_executed=true;return x;
}
static void command(Controller& x,uint32_t t,uint16_t expected,uint16_t base=80,uint16_t cap=100,bool allowed=true) {
  auto got=x.limit(t,base,400,1,cap,allowed,1.f); if(got!=expected) std::cerr<<"t="<<t<<" got="<<got<<" expected="<<expected<<" state="<<int(x.state())<<"\n"; assert(got==expected);
  x.confirm(t,true,400,1,expected);
}
static void baseline(Controller& x) {
  command(x,150,80);x.cycle(c(100,200,3,1,80));
  command(x,250,80);x.cycle(c(200,300,3,1,80));
}
int main() {
  // Smaller input improves camera span discrepancy: hold the minus probe.
  Controller a;baseline(a);command(a,350,75);assert(a.state()==State::ProbeMinus);
  command(a,450,75);a.cycle(c(400,500,2.8f,1.2f,75));
  command(a,550,75);a.cycle(c(500,600,2.8f,1.2f,75));
  assert(a.state()==State::Hold);command(a,650,75);
  // Unchanged or worsened observation restores, then tries the other direction.
  Controller b;baseline(b);command(b,350,75);
  command(b,450,75);b.cycle(c(400,500,3,1,75));
  command(b,550,75);b.cycle(c(500,600,3,1,75));
  assert(b.state()==State::Restore);command(b,650,80);assert(b.state()==State::Collect);
  // Opposite-direction trial must wait for TWO new complete baseline cycles.
  command(b,750,80);b.cycle(c(700,800,3,1,80));
  command(b,850,80);b.cycle(c(800,900,3,1,80));
  command(b,950,85);assert(b.state()==State::ProbePlus);
  command(b,1050,85);b.cycle(c(1000,1100,2.8f,1.2f,85));
  command(b,1150,85);b.cycle(c(1100,1200,2.8f,1.2f,85));assert(b.state()==State::Hold);
  // A large apparent error improvement caused by collapsed movement or
  // weakened body oscillation is rejected, not mistaken for improvement.
  Controller collapse;baseline(collapse);command(collapse,350,75);command(collapse,450,75);
  collapse.cycle(c(400,500,.4f,.4f,75));assert(collapse.state()==State::Restore);
  Controller weak;baseline(weak);command(weak,350,75);command(weak,450,75);
  auto lowpeak=c(400,500,3,1,75);lowpeak.negative_peak=4.4f;weak.cycle(lowpeak);
  assert(weak.state()==State::Restore);
  // A changed negative decision cannot be counted as a comparable response.
  Controller d;baseline(d);command(d,350,75);command(d,450,75);
  d.cycle(c(400,500,2.8f,1.2f,75,2));assert(d.acceptedProbeCycles()==0);
  command(d,550,75);d.cycle(c(500,600,2.8f,1.2f,75));assert(d.acceptedProbeCycles()==1);
  // Missing evidence and unexecuted/forged command cannot create baseline.
  Controller f;auto fake=c(100,200,3,1,80);f.cycle(fake);f.cycle(c(200,300,3,1,80));
  assert(f.limit(350,80,400,1,100,true,1.f)==80);assert(f.state()==State::Collect);
  Controller g;baseline(g);assert(g.limit(350,80,400,1,100,true,1.f)==75);
  g.confirm(350,false,400,1,0);assert(g.fault());assert(g.requireNeutral());assert(g.limit(450,80,400,1,100,true,1.f)==0);
  g.reset();assert(!g.fault());
  // Disallowed control and tighter cap win over experiment; re-acquire later.
  Controller h;baseline(h);command(h,350,75);assert(h.limit(450,80,400,1,70,true,1.f)==70);
  h.confirm(450,true,400,1,70);assert(h.state()==State::Collect);h.reset();baseline(h);command(h,350,75);
  assert(h.limit(450,80,400,1,100,false,1.f)==0 && h.state()==State::Restore);
  h.confirm(450,false,0,0,0);command(h,550,80,95,100);assert(h.state()==State::Restore);
  // No supported negative pair for a long interval: bounded timeout restores
  // instead of permanent adaptation lock, and can attempt another probe.
  Controller k;baseline(k);command(k,350,75);assert(k.limit(2351,80,400,1,100,true,1.f)==80);
  k.confirm(2351,true,400,1,80);assert(k.state()==State::Collect);
  // Missing measurements reset unproven reference; reference needs 2 new cycles.
  Controller m;command(m,150,80);m.cycle(c(100,200,3,1,80));
  auto missing=c(200,300,3,1,80);missing.complete=false;m.cycle(missing);
  command(m,350,80);m.cycle(c(300,400,3,1,80));
  assert(m.limit(450,80,400,1,100,true,1.f)==80);
  // Hold expires on absent comparable observation, restoring rather than
  // indefinitely carrying an experimental command through missing data.
  Controller stale;baseline(stale);command(stale,350,75);command(stale,450,75);
  stale.cycle(c(400,500,2.8f,1.2f,75));command(stale,550,75);
  stale.cycle(c(500,600,2.8f,1.2f,75));assert(stale.state()==State::Hold);
  assert(stale.limit(2601,80,400,1,100,true,1.f)==80);
  stale.confirm(2601,true,400,1,80);assert(stale.state()==State::Collect);
  // Delayed data from before the first probe must not trigger a guard.
  Controller old;baseline(old);command(old,350,75);
  auto prior=c(300,400,.1f,.1f,75);old.cycle(prior);assert(old.state()==State::ProbeMinus);
  // Baseline jumps while probing: restore through +5ms confirmed steps.
  Controller jump;baseline(jump);command(jump,350,75);
  command(jump,450,80,100,100);assert(jump.state()==State::Restore);
  command(jump,550,85,100,100);command(jump,650,90,100,100);
  command(jump,750,95,100,100);command(jump,850,100,100,100);assert(jump.state()==State::Collect);
  // An incomparable negative command cannot be misattributed as probe harm.
  Controller nc;baseline(nc);command(nc,350,75);command(nc,450,75);
  nc.cycle(c(400,500,.1f,.1f,75,2));assert(nc.state()==State::ProbeMinus);
  // Variable baseline must NOT launch a probe from a misleading 2-cycle mean.
  Controller noisy;command(noisy,150,80);noisy.cycle(c(100,200,3,1,80));
  command(noisy,250,80);noisy.cycle(c(200,300,3.5f,1,80));
  assert(noisy.limit(350,80,400,1,100,true,1.f)==80);noisy.confirm(350,true,400,1,80);
  noisy.cycle(c(300,400,3.45f,1,80));command(noisy,450,75);
  command(noisy,550,75);noisy.cycle(c(500,600,3.2f,1.2f,75));
  command(noisy,650,75);noisy.cycle(c(600,700,3.4f,1.0f,75));assert(noisy.state()==State::Restore);
  // Reject a spurious improvement when it is smaller than observed variation.
  Controller jitter;baseline(jitter);command(jitter,350,75);command(jitter,450,75);
  jitter.cycle(c(400,500,3,1.07f,75));command(jitter,550,75);
  jitter.cycle(c(500,600,2.9f,1.07f,75));assert(jitter.state()==State::Restore);
  // Unsent candidate can return to the confirmed safe baseline without fault.
  Controller unsent;baseline(unsent);assert(unsent.limit(350,80,400,1,100,true,1.f)==75);
  assert(unsent.cancelUnsent(80));unsent.confirm(350,true,400,1,80);assert(!unsent.requireNeutral());
  // A candidate which cannot be formed during a wide baseline change must
  // skip that crossing and retain the last actual width for gradual restore.
  Controller skipped;baseline(skipped);command(skipped,350,75);
  assert(skipped.limit(450,95,400,1,100,true,1.f)==80);
  assert(!skipped.cancelUnsent(95));skipped.confirm(450,false,0,0,0);
  command(skipped,550,80,95,100);assert(skipped.state()==State::Restore);
  command(skipped,650,85,95,100);command(skipped,750,90,95,100);
  command(skipped,850,95,95,100);assert(skipped.state()==State::Collect);
  // Once already balanced within the evidence threshold, keep baseline.
  Controller balanced;command(balanced,150,80);balanced.cycle(c(100,200,2,2,80));
  command(balanced,250,80);balanced.cycle(c(200,300,2,2,80));
  assert(balanced.limit(350,80,400,1,100,true,1.f)==80 && balanced.state()==State::Collect);
  // A valid ordinary selector switching 400mA/75ms -> 300mA/73ms must not
  // repeatedly suppress positive output or be falsely confirmed as a probe.
  Controller mode;baseline(mode);command(mode,350,75);
  assert(mode.limit(450,73,300,1,100,true,1.f)==73);
  assert(mode.state()==State::Collect && !mode.requireNeutral());
  mode.confirm(450,true,300,1,73);
  assert(mode.limit(550,73,300,1,100,true,1.f)==73);
  mode.confirm(550,true,300,1,73);
  auto not400=c(400,500,3,1,73);not400.positive_ma=300;mode.cycle(not400);
  assert(mode.state()==State::Collect);
  // A selector/model-mode switch is not a sensor fault; handing control back
  // to the ordinary fallback never latches a neutral request.
  Controller modebranch;baseline(modebranch);command(modebranch,350,75);
  assert(modebranch.selectorUnavailable(450));assert(modebranch.state()==State::Collect);
  assert(!modebranch.requireNeutral());modebranch.confirm(450,true,300,1,73);
  assert(modebranch.selectorUnavailable(550));modebranch.confirm(550,true,300,1,73);
  assert(!modebranch.requireNeutral());
  // All three ordinary selector currents probe at the SAME current as the
  // two actual baseline cycles; current changes re-acquire references.
  for(int16_t ma: {int16_t(300),int16_t(350),int16_t(400)}) {
    Controller same;
    assert(same.limit(150,80,ma,1,100,true,1.f)==80);same.confirm(150,true,ma,1,80);
    auto first=c(100,200,3,1,80);first.positive_ma=ma;same.cycle(first);
    assert(same.limit(250,80,ma,1,100,true,1.f)==80);same.confirm(250,true,ma,1,80);
    auto second=c(200,300,3,1,80);second.positive_ma=ma;same.cycle(second);
    assert(same.anchorCurrent()==ma && same.limit(350,80,ma,1,100,true,1.f)==75);
    same.confirm(350,true,ma,1,75);assert(!same.requireNeutral());
    assert(same.limit(450,73,ma==300?350:300,1,100,true,1.f)==73);
    assert(same.state()==State::Collect && !same.requireNeutral());
  }
  Controller wrong_ma;baseline(wrong_ma);assert(wrong_ma.limit(350,80,400,1,100,true,1.f)==75);
  wrong_ma.confirm(350,true,300,1,75);assert(wrong_ma.requireNeutral());
  std::cout<<"foot span controller: probes, confirmed outputs, incomparable cycles and reacquisition PASS\n";
}
