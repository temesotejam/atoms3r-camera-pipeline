#include "foot_phase_observer.h"
#include <cmath>
#include <iostream>

// Input is delivery/notification-time-ordered. Camera frame and peak timestamps
// must both be measured from the same measurement/run epoch. This replay does
// not predict how a changed motor input would alter subsequent observations.
// R run-id
// F notification-ms frame-ms left-valid right-valid left-deg right-deg
// P notification-ms peak-ms side
int main() {
  foot_phase_observer::Observer observer;
  char op;unsigned run=0,now=0,ms=0;uint32_t sequence=0;
  while(std::cin>>op) {
    if(op=='R') {std::cin>>run;observer.reset(static_cast<uint16_t>(run));sequence=0;continue;}
    if(op=='B') {
      int lv,rv,zero;float left_x,right_x,left,right;
      if(!(std::cin>>now>>ms>>lv>>rv>>zero>>left_x>>right_x>>left>>right))return 1;
      observer.boundedFrame(static_cast<uint16_t>(run),ms,lv,rv,zero,left_x,right_x,left,right);
    } else if(op=='P') {
      int side;if(!(std::cin>>now>>ms>>side))return 1;
      observer.peak(static_cast<uint16_t>(run),side,ms);
    } else return 1;
    if(observer.sequence()!=sequence) {
      sequence=observer.sequence();
      std::cout<<now<<' '<<observer.cycleStart()<<' '<<observer.cycleTime()<<' '
               <<observer.count()<<' '<<observer.leftMean()<<' '<<observer.rightMean()<<' '
               <<observer.leftLatest()<<' '<<observer.rightLatest()<<' '
               <<observer.valid(now)<<'\n';
    }
  }
}
