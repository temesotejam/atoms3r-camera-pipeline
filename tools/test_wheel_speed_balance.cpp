#include "wheel_speed_balance.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
int main(int argc,char**){
 if(argc>1){wheel_speed_balance::Controller c;int side,valid;unsigned t;float rpm,peak;unsigned limited=0;
  while(std::cin>>side>>rpm>>valid>>t>>peak){c.peak(1,peak);c.crossing(side,rpm,valid,t);if(side==1 && c.capMs()<100)++limited;}
  assert(limited>=5);std::cout<<"Recorded speed pairs trigger guard "<<limited<<" times\n";return 0;}

 wheel_speed_balance::Controller c; c.peak(1,6.f);
 for(int i=0;i<8;i++){unsigned t=1000+i*800;c.crossing(-1,380,true,t);c.crossing(1,49,true,t+400);}
 assert(c.capMs()==80);
 c.crossing(-1,380,true,8000);c.crossing(1,49,true,8400);assert(c.capMs()==80);
 c.crossing(-1,50,true,9000);c.crossing(1,-30,true,9400);assert(c.capMs()==85);
 c.peak(1,4.9f);c.crossing(-1,380,true,10000);c.crossing(1,49,true,10400);assert(c.capMs()==100);
 c.reset();c.peak(1,6);c.crossing(-1,380,true,1000);c.crossing(1,49,true,3000);assert(c.capMs()==100);
 c.crossing(-1,380,true,4000);c.crossing(1,49,false,4400);assert(c.capMs()==100);
 std::cout<<"Wheel speed bias: consecutive paired evidence, bounded cap, weak swing and stale data PASS\n";
}
