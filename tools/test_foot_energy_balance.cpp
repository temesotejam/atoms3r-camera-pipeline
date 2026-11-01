#include "foot_energy_balance.h"
#include <cassert>
#include <cmath>
#include <iostream>
using foot_energy_balance::Controller;
static void cycle(Controller& c,uint16_t id,unsigned t,float left,float right) {
  c.frame(id,t-25,true,true,left,right);c.peak(1,t);
  c.frame(id,t+25,true,true,left,right);
  c.frame(id,t+375,true,true,left,right);c.peak(-1,t+400);
  c.frame(id,t+425,true,true,left,right);
}
int main() {
  Controller c;c.reset(7);c.update(10);assert(!c.valid() && c.bias()==0);
  for(unsigned i=0;i<3;i++)cycle(c,7,1000+i*800,20,2);
  c.update(3800);assert(c.valid());assert(c.bias()>.019f && c.bias()<.021f);
  assert(c.factor(1)<1 && c.factor(-1)>1);
  assert(std::fabs(c.factor(1)+c.factor(-1)-2)<1e-6f);
  c.update(4900);assert(c.bias()>.039f);
  c.update(7900);assert(!c.valid() && c.bias()<.021f);
  c.update(9000);assert(std::fabs(c.bias())<1e-6f);
  c.reset(8);for(unsigned i=0;i<3;i++)cycle(c,8,1000+i*800,1,20);
  c.update(3800);assert(c.valid() && c.bias()<0);
  c.reset(9);for(unsigned i=0;i<3;i++)cycle(c,8,1000+i*800,20,2);
  c.update(3800);assert(!c.valid() && c.bias()==0);
  c.reset(10);for(unsigned i=0;i<3;i++)cycle(c,10,1000+i*800,0,0);
  c.update(3800);assert(c.valid() && c.bias()==0);
  c.reset(11);for(unsigned i=0;i<3;i++)cycle(c,11,1000+i*800,20,2);
  c.update(3800);assert(c.valid());
  c.update(4500);assert(c.bias()<.021f); // update rate is limited
  c.reset(12);
  for(unsigned i=0;i<3;i++) {
    const unsigned t=1000+i*800;
    c.frame(12,t-30,true,true,20,2);c.frame(12,t+20,true,true,20,2);
    c.frame(12,t+150,true,true,99,99);c.peak(1,t); // peak confirmed after newer frame
    c.frame(12,t+370,true,true,20,2);c.frame(12,t+420,true,true,20,2);
    c.frame(12,t+550,true,true,99,99);c.peak(-1,t+400);
  }
  c.update(3800);assert(c.valid() && c.bias()>0);
  assert(std::fabs(c.leftMean()-20)<1e-6f && std::fabs(c.rightMean()-2)<1e-6f);
  std::cout<<"Foot energy: nearest peak, matched run, bounded opposite allocations, stale return and no-motion PASS\n";
}
