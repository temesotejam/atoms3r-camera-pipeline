#include "wheel_cycle_guard.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  wheel_cycle_guard::Controller c;
  // Initial alternating samples from run 464 are kept intact.
  assert(c.limit(-1,270.97f,true,417,59)==59);
  assert(c.limit(1,-287.44f,true,814,43)==43);
  // At the first modest drift only a small correction is allowed.
  c.reset();c.limit(1,-346.53f,true,2360,26);
  auto modest=c.limit(-1,4.18f,true,2748,45);
  assert(modest>=43 && modest<=45);
  // Later negative wheel bias limits only the negative-going command.
  c.reset();c.limit(1,-566.3f,true,18748,32);
  auto limited=c.limit(-1,-1.88f,true,19217,110);
  assert(limited<110 && limited>=83);
  assert(c.limit(1,-647.55f,true,19585,32)==32);
  // Symmetry, stale data, invalid speed, and coast do not create torque.
  c.reset();c.limit(-1,600,true,100,30);
  assert(c.limit(1,0,true,500,100)<100);
  c.reset();c.limit(1,-600,true,100,30);
  assert(c.limit(-1,0,true,2000,100)==100);
  c.reset();c.limit(1,-600,true,100,30);
  assert(c.limit(-1,NAN,false,500,100)==100);
  assert(c.limit(1,-600,true,600,0)==0);
  std::cout << "wheel cycle guard PASS\n";
}
