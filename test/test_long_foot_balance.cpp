#include "long_foot_balance.h"
#include <cassert>
using long_foot_balance::Controller;
static void sample(Controller& c,unsigned n,float l,float r,float p=7.f,float m=7.f) {
  c.observe(n-100,n,l,r,p,m,true,true);
}
int main(){
  Controller c;
  for(unsigned n=1000;n<=4000;n+=1000)sample(c,n,3.f,1.f);
  assert(c.propose(4100,60,100,true)==60); // first four periods untouched
  c.observe(4100,4500,0,0,0,0,false,true); // missing is not zero
  sample(c,5000,3.f,1.f);
  assert(c.propose(5100,60,100,true)==65);
  c.confirm(5100,45,45,true); // guard removed candidate
  assert(c.offset()==0 && c.state()==Controller::State::Collect);
  c.confirm(5200,45,49,false); // command not accepted
  assert(c.offset()==0);
  c.confirm(5300,45,49,true);
  assert(c.offset()==5 && c.state()==Controller::State::Trial);
  c.observe(5000,5400,3.f,1.f,7.f,7.f,true,true); // overlaps intervention
  assert(c.samples()==0);
  for(unsigned n=6000;n<=10000;n+=1000)sample(c,n,3.f,1.5f);
  assert(c.propose(10100,60,100,true)==65); // improved: hold for a further five periods
  for(unsigned n=11000;n<=15000;n+=1000)sample(c,n,3.f,1.5f);
  assert(c.propose(15100,60,100,true)==70);
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.proposedTrialReferenceOffset()==5);
  c.confirm(15100,49,49,true); // next proposal blocked; active +5 is still compared to zero
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.trialGeneration()==1);
  c.observe(15001,15120,3.f,1.5f,7.f,7.f,true,true,false);
  assert(c.trialActive() && c.trialReferenceOffset()==0);
  c.confirm(15200,49,53,true);
  assert(c.trialReferenceOffset()==5 && c.trialGeneration()==2);
  for(unsigned n=16000;n<=20000;n+=1000)sample(c,n,3.f,.7f);
  assert(c.state()==Controller::State::Restore);
  assert(c.propose(20100,60,100,true)==65);
  c.confirm(20150,49,49,true); // same physical output; discard stale offset
  assert(c.offset()==5 && c.state()==Controller::State::Hold);
  assert(c.trialActive() && c.trialReferenceOffset()==0 && c.trialGeneration()==3);
  c.observe(20200,20300,3.f,1.f,7.f,7.f,true,true,false);
  assert(c.samples()==0); // old +5 must still be effective after restore
  for(unsigned n=21000;n<=25000;n+=1000)sample(c,n,3.f,1.f);
  assert(c.propose(25100,60,100,true)==60); // reverse failed direction
  c.confirm(25200,49,45,true);
  sample(c,26000,3.f,1.f,3.f,7.f); // serious peak drop unwinds immediately
  assert(c.state()==Controller::State::Collect);
  assert(c.propose(26100,60,100,true)==60);
  assert(c.propose(40000,60,100,true)==60); // stale observations: baseline
  assert(c.offset()==0);
  c.observe(40100,40200,3.f,1.f,7.f,7.f,true,false);
  assert(c.propose(40300,60,100,true)==60);
  Controller safety;
  for(unsigned n=1000;n<=5000;n+=1000)sample(safety,n,3.f,1.f);
  safety.confirm(5300,60,65,true);
  safety.observe(5900,6000,0,0,3.f,7.f,false,true); // IMU still sees weak swing with no foot frame
  assert(safety.state()==Controller::State::Restore);
  assert(safety.propose(6100,60,100,true)==60);
  Controller flip;
  for(unsigned n=1000;n<=5000;n+=1000)sample(flip,n,3.f,1.f);
  flip.confirm(5300,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(flip,n,2.f,2.f);
  assert(flip.state()==Controller::State::Hold && flip.propose(10100,60,100,true)==65);
  for(unsigned n=11000;n<=15000;n+=1000)sample(flip,n,1.f,3.f);
  assert(flip.propose(15100,60,100,true)==60); // sign reversed while holding
  Controller overlap;
  for(unsigned n=1000;n<=5000;n+=1000)sample(overlap,n,3.f,1.f);
  overlap.confirm(5300,60,65,true);
  overlap.observe(5000,5400,3.f,1.f,3.f,7.f,true,true);
  assert(overlap.state()==Controller::State::Restore && overlap.samples()==0);
  Controller age;
  sample(age,1000,3.f,1.f);sample(age,5000,3.f,1.f);sample(age,9000,3.f,1.f);
  sample(age,13000,3.f,1.f);sample(age,17000,3.f,1.f);
  assert(age.samples()<5 && age.propose(17100,60,100,true)==60);
  // The latest five measurements replace a stale startup reference while a
  // candidate waits. A blocked old proposal never gains renewed freshness.
  Controller rolling;
  for(unsigned n=1000;n<=5000;n+=1000)sample(rolling,n,1.1f,.24f);
  assert(rolling.error()>.5f && rolling.propose(5100,60,100,true)==65);
  rolling.confirm(5100,60,60,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(rolling,n,1.75f,.585f);
  assert(rolling.error()>.45f && rolling.error()<.49f);
  assert(rolling.propose(11600,60,100,true)==60); // proposal older than 1.5s
  sample(rolling,12000,1.75f,.585f);
  assert(rolling.propose(12100,60,100,true)==65);
  rolling.confirm(12100,60,65,true);
  assert(rolling.trialActive() && rolling.trialGeneration()==1 && rolling.trialReferenceOffset()==0);
  rolling.observe(12200,12500,1.75f,1.f,7.f,7.f,true,true,false);
  assert(rolling.samples()==0); // guard removed the changed output
  for(unsigned n=13000;n<=17000;n+=1000)sample(rolling,n,1.75f,1.f);
  assert(rolling.state()==Controller::State::Hold);
  assert(rolling.propose(17100,60,100,true)==65); // no immediate next step
  Controller growth;
  for(unsigned n=1000;n<=5000;n+=1000)sample(growth,n,1.f,.2f,5.f,5.f);
  growth.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(growth,n,2.f,.5f,7.f,7.f);
  assert(growth.state()==Controller::State::Trial); // growth is inconclusive
  assert(growth.propose(10100,60,100,true)==65);
  Controller timed;
  for(unsigned n=1000;n<=5000;n+=1000)sample(timed,n,3.f,1.f);
  timed.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(timed,n,3.f,1.5f);
  for(unsigned n=11000;n<=15000;n+=1000)sample(timed,n,3.f,1.5f);
  assert(timed.propose(15100,60,100,true)==70);
  timed.confirm(15100,65,70,true);
  for(unsigned n=16000;n<=27000;n+=1000)sample(timed,n,3.f,1.6f,9.f,9.f);
  assert(timed.propose(27101,60,100,true)==65); // inconclusive: restore +5, not zero
  timed.confirm(27101,70,65,true);
  assert(timed.offset()==5 && timed.trialActive() && timed.trialReferenceOffset()==0);
  Controller safety_ref;
  for(unsigned n=1000;n<=5000;n+=1000)sample(safety_ref,n,3.f,1.f);
  safety_ref.confirm(5100,60,65,true);
  for(unsigned n=6000;n<=10000;n+=1000)sample(safety_ref,n,3.f,1.5f);
  for(unsigned n=11000;n<=15000;n+=1000)sample(safety_ref,n,3.f,1.5f,6.f,6.f);
  assert(safety_ref.trialActive() && safety_ref.propose(15100,60,100,true)==70);
  safety_ref.observe(15120,15200,0.f,0.f,4.4f,6.f,false,true,false);
  assert(safety_ref.state()==Controller::State::Restore); // queued refs never weaken active safety
  // Actual run 38414363 peak trajectory: the early reference grows from
  // roughly 3.1 to 5.1 degrees, while the later peaks settle near 5 degrees.
  Controller replay;
  const float early[]={3.065f,3.910f,3.877f,4.499f,5.120f};
  const float early_m[]={6.237f,6.439f,5.688f,5.643f,5.974f};
  for(unsigned i=0;i<5;++i)sample(replay,1000+i*1000,1.1f,.24f,early[i],early_m[i]);
  assert(replay.propose(5100,60,100,true)==60);
  const float settled[]={5.087f,5.077f,4.924f,4.933f,4.946f};
  const float settled_m[]={5.587f,6.155f,5.282f,5.995f,5.534f};
  for(unsigned i=0;i<5;++i)sample(replay,6000+i*1000,1.75f,.585f,settled[i],settled_m[i]);
  assert(replay.propose(10100,60,100,true)==65);
}
