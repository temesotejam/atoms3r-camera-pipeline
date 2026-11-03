#include "tilt_priority.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using tilt_priority::Controller;
int main(){
  Controller c;
  c.peak(1,220,2.46524f);c.peak(-1,598,5.0033f);
  c.peak(1,991,6.16416f);
  assert(c.negativeEarlyThreshold(1200,1)==0.f);
  c.peak(-1,6880,4.19616f);c.peak(1,7238,7.2264f);
  c.footCycle(6053,6880,3.58f,.56f,true);
  assert(c.negativeEarlyThreshold(7300,1)==1.f); // >= 3 deg body gap
  c.footCycle(6880,7724,3.87f,.48f,true);
  c.peak(-1,7724,4.05603f);c.peak(1,8091,7.03083f);
  assert(c.negativeEarlyThreshold(8250,1)==1.f); // persistent camera asymmetry
  assert(c.negativeEarlyThreshold(8250,-1)==0.f);
  assert(c.negativeEarlyThreshold(12000,1)==0.f); // stale peaks never trigger
  c.footCycle(7724,8534,2.8f,.77f,false);
  c.peak(-1,8534,5.f);c.peak(1,8908,7.f);
  assert(c.negativeEarlyThreshold(9000,1)==0.f); // invalid camera and gap <3
  c.footCycle(8534,9368,3.f,.4f,true);
  assert(c.negativeEarlyThreshold(9400,1)==0.f); // one cycle is not enough
  c.footCycle(9368,10235,4.f,.5f,true);
  c.peak(-1,10235,4.f);c.peak(1,10598,6.f);
  assert(c.negativeEarlyThreshold(10650,1)==1.f);
  c.footCycle(10235,11213,3.f,2.5f,true);
  assert(c.negativeEarlyThreshold(11300,1)==0.f); // balance restored
  c.peak(-1,13729,3.86261f);c.peak(1,14106,8.38403f);
  assert(c.negativeEarlyThreshold(14403,1)==1.f); // last measured return
  // Run 590 returned from +8.38 degrees, from +1.04 to +0.97 while moving
  // inward; it never crossed absolute zero in that half-cycle. Replaying
  // this observed pair tests the decision, not the resulting physical path.
  const float threshold=c.negativeEarlyThreshold(14403,1);
  const bool inward=-14.63f<0.f;
  const bool would_cross=inward && 1.04f>threshold && .97f<=threshold;
  const bool original_zero=inward && 1.04f>0.f && .97f<=0.f;
  assert(would_cross && !original_zero);
  c.reset();assert(c.negativeEarlyThreshold(14403,1)==0.f);
  std::puts("body asymmetry and camera-based early timing: PASS");
}
