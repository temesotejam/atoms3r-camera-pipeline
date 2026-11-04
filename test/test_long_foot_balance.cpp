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
  assert(c.propose(10100,60,100,true)==70); // weak foot improved: hold and one next step
  c.confirm(10200,49,53,true);
  for(unsigned n=11000;n<=15000;n+=1000)sample(c,n,3.f,.7f);
  assert(c.state()==Controller::State::Restore);
  assert(c.propose(15100,60,100,true)==65);
  c.confirm(15150,49,49,true); // same physical output; discard stale offset
  assert(c.offset()==5 && c.state()==Controller::State::Collect);
  for(unsigned n=16000;n<=20000;n+=1000)sample(c,n,3.f,1.f);
  assert(c.propose(20100,60,100,true)==60); // reverse failed direction
  c.confirm(20200,49,45,true);
  sample(c,21000,3.f,1.f,3.f,7.f); // serious peak drop unwinds immediately
  assert(c.state()==Controller::State::Collect);
  assert(c.propose(21100,60,100,true)==60);
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
}
