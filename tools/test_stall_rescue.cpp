#include "stall_rescue.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using stall_rescue::Detector;
int main(){
  Detector d;
  // Measured near-stall episode from run 307, held inward and confirmed.
  assert(!d.observe(17207,17203,16796,8.91235f,2.92f,2.91f,-3.19f,-2.76f));
  assert(!d.observe(17214,17207,16796,8.91235f,2.91f,2.88f,-2.76f,-2.40f));
  assert(d.observe(17219,17214,16796,8.91235f,2.88f,2.87f,-2.40f,-1.42f));
  // One-sample noise, true reversal, stale data, and motion away from centre
  // never cause a rescue. A fresh trigger transition must be observed again.
  assert(!d.observe(17207,17204,16796,8.91f,2.93f,2.91f,-3.4f,-2.8f));
  assert(!d.observe(17210,17207,16796,8.91f,2.91f,2.90f,-2.8f,+.2f));
  assert(!d.observe(17213,17210,16796,8.91f,2.90f,2.89f,-2.5f,-2.0f));
  assert(!d.observe(17250,17210,16796,8.91f,2.90f,2.89f,-3.2f,-2.0f));
  assert(!d.observe(17260,17257,16796,8.91f,2.80f,2.90f,-3.2f,-2.0f));
  assert(!d.observe(17270,17267,16796,8.91f,2.93f,2.91f,-3.2f,NAN));
  assert(!d.observe(16900,16897,16796,8.91f,3.1f,3.0f,-3.2f,-2.0f));
  assert(stall_rescue::kMaxPulseMs==15);
  std::puts("inward stall rescue: confirmed low speed, stale/reversal rejection PASS");
}
