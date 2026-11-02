#include "foot_stroke_observer.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace foot_stroke_observer;
static bool near(float a,float b) { return std::fabs(a-b)<1e-5f; }

int main() {
  // Initial center -> forward (2), then back (-3) -> forward (2) = 5.
  // Reversal is confirmed on a later sample, never at the extremum itself.
  Observer a(+1,.5f,500);
  assert(a.observe(0,0,true).kind==Kind::None);
  assert(a.observe(10,1,true).kind==Kind::None);
  assert(a.observe(20,2,true).kind==Kind::None);
  auto half=a.observe(30,1.4f,true);
  assert(half.kind==Kind::InitialToForward && near(half.width,2));
  assert(half.from_ms==0 && half.to_ms==20 && half.confirmed_ms==30);
  a.observe(40,-3,true); a.observe(50,-2.4f,true); a.observe(60,2,true);
  auto full=a.observe(70,1.4f,true);
  assert(full.kind==Kind::BackToForward && near(full.width,5));
  assert(full.from_ms==40 && full.to_ms==60 && full.confirmed_ms==70);

  // Opposite camera direction, with an arbitrary offset, is equivalent.
  Observer b(-1,.5f,500);
  b.observe(0,100,true); b.observe(10,101,true); b.observe(20,103,true);
  b.observe(30,102.4f,true); b.observe(40,98,true);
  auto opposite=b.observe(50,98.6f,true);
  assert(opposite.kind==Kind::BackToForward && near(opposite.width,5));
  assert(opposite.from_ms==20 && opposite.to_ms==40);

  // No forward sign means no invented movement; invalid configuration is
  // visible to the caller instead of guessing from observed trajectories.
  Observer unknown(0,.5f,500);
  assert(unknown.observe(0,0,true).kind==Kind::None);
  assert(unknown.status()==Status::InvalidConfiguration);
  Observer bad_threshold(1,0.f,500);
  assert(bad_threshold.observe(0,0,true).kind==Kind::None);
  assert(bad_threshold.status()==Status::InvalidConfiguration);

  // Invalid, reordered, or missing samples start a new tracking epoch and
  // cannot complete a stroke using an old foot extremum.
  Observer c(1,.5f,100);
  c.observe(0,0,true); c.observe(10,-3,true); c.observe(20,-2,true);
  c.observe(30,NAN,false); assert(c.status()==Status::InvalidSample);
  c.observe(40,2,true); assert(c.observe(50,1,true).kind==Kind::None);
  c.observe(60,0,true); c.observe(400,10,true); assert(c.status()==Status::GapReset);
  assert(c.observe(390,11,true).kind==Kind::None);
  assert(c.status()==Status::NonMonotonicTime);

  // Only new FULL forward strokes can be paired; the initial half and an
  // out-of-time pair cannot masquerade as equal, valid gait steps.
  Pairer pair;
  pair.left(half); pair.right(opposite);
  assert(!pair.take(70,100,100).valid);
  pair.left(full);
  auto cmp=pair.take(70,100,30);
  assert(cmp.valid && near(cmp.difference,0));
  assert(!pair.take(70,100,30).valid); // consumed once
  pair.left(full); pair.right(opposite);
  assert(!pair.take(70,100,5).valid); // peak skew 20ms
  pair.invalidateLeft();
  assert(!pair.take(70,100,100).valid);
  pair.left(full);
  assert(!pair.take(1000,100,100).valid); // stale
  auto late=opposite; late.to_ms=1; late.confirmed_ms=1000;
  auto current=full; current.to_ms=990; current.confirmed_ms=1000;
  pair.reset(); pair.left(current); pair.right(late);
  assert(!pair.take(1000,100,2000).valid); // fresh confirmation, old peak

  std::cout << "foot stroke observer tests passed\n";
}
