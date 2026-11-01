#include "gradual_start.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  gradual_start::Controller c;
  assert(c.target()==4.5f && c.positiveCapMs()==80);
  auto pair=[&](unsigned t,float a,float b,float rpm){c.peak(1,a,rpm,true,t);c.peak(-1,b,rpm,true,t+400);};
  pair(1000,4.3f,4.2f,450);assert(c.target()==4.5f);
  pair(2000,4.3f,4.2f,450);assert(c.target()==5.f);
  pair(3000,6,6,501);pair(4000,6,6,450);assert(c.target()==5.f);
  c.peak(1,6,450,false,5000);c.peak(-1,6,450,true,5400);assert(c.target()==5.f);
  c.reset();unsigned t=1000;
  for(int i=0;i<14;++i){pair(t,8,8,450);t+=1000;}
  assert(c.target()==8 && c.positiveCapMs()==100);
  for(int i=0;i<4;++i){pair(t,8,8,450);t+=1000;}assert(c.target()==8);
  c.reset();c.peak(1,8,450,true,1000);c.peak(-1,8,450,true,3001);assert(c.target()==4.5f);
  std::cout<<"Gradual startup: paired peaks, speed and freshness gates, staged cap and reset PASS\n";
}
