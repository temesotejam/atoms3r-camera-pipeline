#include "walking_energy_loss.h"
#include <cassert>
#include <iostream>
int main(int argc,char** argv) {
  using namespace walking_energy_loss;
  using wheel_work_energy_control::State;
  const lateral_energy::Geometry g;
  lateral_energy::CurrentTable table;assert(table.prepare(.06f));
  const float target=1000.f*lateral_energy::potential(g,8.f);
  Feedback f;State s{32,560,7840,true,5.5f};s.absolute_reference=true;
  assert(supported(s));
  State alternate{30,520,7690,true,5.5f};alternate.absolute_reference=true;
  assert(supported(alternate));
  State low{35.5324f,514.35f,7610,true,4.30066f};low.absolute_reference=true;
  auto low_choice=choose(g,low,table,0,280,target,f,0);
  assert(low_choice.valid && low_choice.walking_loss_mj<0.f);
  State low_rpm{30,446.55f,7610,true,5.2f};low_rpm.absolute_reference=true;
  assert(choose(g,low_rpm,table,0,280,target,f,0).valid);
  assert(choose(g,alternate,table,0,280,target,f,0).valid);
  alternate.battery_mv=7000;assert(supported(alternate));
  alternate.battery_mv=8120;assert(supported(alternate));
  alternate.battery_mv=8121;assert(!supported(alternate));
  State new_coast{13.8f,277.8f,7510.f,true,7.37f};new_coast.absolute_reference=true;
  assert(supported(new_coast));assert(choose(g,new_coast,table,0,280,target,f,0).valid);
  auto c=choose(g,s,table,0,280,target,f,0);
  assert(c.valid&&c.branch==10&&c.width_ms==0&&c.walking_loss_mj>0);
  assert(c.cross_condition&&c.support_weight==0);
  assert(std::fabs(c.free_mj+c.walking_loss_mj-1000.f*lateral_energy::potential(g,s.previous_peak_deg))<1e-5f);
  const float raw=coastMj(s.rate_dps);
  assert(f.observe(raw-.03f,raw,target));assert(f.capMs()==0);
  assert(f.observe(raw-.03f,raw,target));assert(f.capMs()==25);
  c=choose(g,s,table,0,280,target,f,0);
  assert(c.valid&&c.width_ms==25&&c.current_ma==300&&c.command_sign==1);
  assert(c.target_above_reachable&&c.q_mas>0&&c.q_mas<=25);
  assert(std::fabs(c.free_mj-(raw-.03f))<1e-5f); // residual exactly once
  assert(std::fabs(c.energy_mj-(c.free_mj+.22502117220f*.06189960458f*c.q_mas*s.rate_dps*wheel_work_energy_control::kRadPerDeg))<1e-5f);
  assert(!f.observe(NAN,raw,target));assert(!f.observe(raw+1,raw,target));
  for(int i=0;i<20;++i)f.observe(raw,raw,target);
  assert(f.capMs()==25); // no unsupported escalation
  for(int i=0;i<9;++i) {
    State bad=s;
    if(i==0)bad.wheel_fresh=false;
    if(i==1)bad.rate_dps=13.f;
    if(i==2)bad.rate_dps=38.f;
    if(i==3)bad.wheel_rpm=620;
    if(i==4)bad.battery_mv=6900;
    if(i==5)bad.previous_peak_deg=7.7f;
    if(i==6)bad.target_deg=10;
    if(i==7)bad.absolute_reference=false;
    if(i==8)bad.rate_dps=NAN;
    assert(!choose(g,bad,table,0,280,target,f,0).valid);
  }
  assert(!choose(g,s,table,NAN,280,target,f,0).valid);
  assert(!choose(g,s,table,0,280,target,f,.1f).valid);
  Feedback over;assert(over.observe(target,target,target));
  assert(over.observe(target,target,target));assert(over.capMs()==0);
  assert(!f.observe(target+.5f,raw,target));assert(f.capMs()==0);
  Feedback adaptive;
  assert(adaptive.observe(raw,raw,target));assert(adaptive.observe(raw,raw,target));
  assert(adaptive.capMs()==25);
  assert(adaptive.observe(raw,raw,target,25));assert(adaptive.capMs()==25);
  assert(adaptive.observe(raw,raw,target,25));assert(adaptive.capMs()==30);
  for(int i=0;i<20;++i)assert(adaptive.observe(raw,raw,target,adaptive.capMs()));
  assert(adaptive.capMs()==60);
  assert(adaptive.observe(target,target,target,60));assert(adaptive.capMs()==0);
  assert(adaptive.observe(raw,raw,target));assert(adaptive.capMs()==55);
  // Extension requires fresh, positive arrival speed and three matched
  // capped arrivals. Missing or low speed returns the cap to 60 ms.
  Feedback speed;assert(speed.observe(raw,raw,target));assert(speed.observe(raw,raw,target));
  for(int i=0;i<14;++i)assert(speed.observe(raw,raw,target,speed.capMs()));
  assert(speed.capMs()==60);
  assert(speed.observe(raw,raw,target,60,40.f,true));
  assert(!speed.observe(NAN,raw,target,60,40.f,true));
  assert(speed.observe(raw,raw,target,60,40.f,true));
  assert(speed.observe(raw,raw,target,60,40.f,true));assert(speed.capMs()==60);
  assert(speed.observe(raw,raw,target,60,40.f,true));assert(speed.capMs()==65);
  assert(speed.observe(raw,raw,target,65,NAN,false));assert(speed.capMs()==60);
  for(int i=0;i<9;++i)assert(speed.observe(raw,raw,target,speed.capMs(),40.f,true));
  assert(speed.capMs()==75);
  assert(speed.observe(raw,raw,target,75,29.f,true));assert(speed.capMs()==60);
  assert(speed.observe(target,target,target,60,40.f,true));assert(speed.capMs()==0);
  if(argc>1) {
    float rate,rpm,battery,previous,peak;unsigned count=0;float squared=0;
    while(std::cin>>rate>>rpm>>battery>>previous>>peak) {
      State recorded{rate,rpm,battery,true,previous};recorded.absolute_reference=true;
      Feedback coast;auto pred=choose(g,recorded,table,0,280,target,coast,0);
      assert(pred.valid&&pred.width_ms==0);
      const float error=pred.energy_mj-1000.f*lateral_energy::potential(g,peak);
      squared+=error*error;++count;
    }
    if(argv[1][0]=='f') assert(count==18&&std::sqrt(squared/count)<.043f);
    else if(argv[1][0]=='t') assert(count==31&&std::sqrt(squared/count)<.031f);
    else assert(count==32&&std::sqrt(squared/count)<.019f);
  }
  std::cout<<"Walking loss balance, one residual, strict domain, bounded assist and recorded coast replay PASS\n";
}
