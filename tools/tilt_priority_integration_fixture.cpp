#define FOOT_TREND_CYCLES 2
#include "tilt_priority.h"
#include "stall_rescue.h"
#include "long_foot_balance.h"
#include "negative_cap_recovery.h"
#include "wheel_cycle_guard.h"
#include "foot_span_control.h"
#include "peak_error_adaptation.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
namespace Config {
constexpr uint16_t WALKING_TARGET8_MAX_PULSE_MS=150;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_CURRENT_WRITE_FAILED=12;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_NONE=0;
constexpr uint8_t ENERGY_CONTROL_AUTONOMOUS_REASON_VALID_NO_OUTPUT=1;
}
enum class ExperimentState { RUNNING_BATCH_SWEEP, STOPPED };
uint32_t millis(){return 10000;}
namespace PsramLogger {
struct EnergyControlAutonomousZeroCrossEvent {
 uint32_t zero_cross_time_ms=999;int8_t physical_next_peak_side=1,q_command_direction=0;
 float target_peak_deg=8,pre_input_wheel_speed_rpm=20,target_energy_j=.001f,zero_cross_abs_rate_dps=10;
 uint16_t vbat_mV=7500;bool pre_input_wheel_speed_valid=true;uint32_t pre_input_wheel_speed_age_us=100;
 bool side_mismatch_diagnostic=false,energy_model_extrapolated=false,work_prediction_cross_condition=false;
 uint8_t work_control_branch=0,reason=0;uint16_t work_brake_cap_ms=0,solver_selected_integer_width_ms=0;
 uint16_t wheel_guard_legacy_ms=0,wheel_guard_extra_ms=0;bool wheel_guard_trial_eligible=false;
 uint16_t weak_probe_baseline_ms=0,weak_probe_requested_ms=0;bool weak_probe_executed=false;
 float passive_energy_j=1,free_next_peak_amplitude_deg=1,predicted_energy_j=1,predicted_observed_energy_j=1;
 float predicted_next_peak_amplitude_deg=1,predicted_observed_next_peak_deg=1,predicted_wheel_work_j=1;
 float predicted_input_work_j=1,predicted_post_input_work_j=1,predicted_work_change_j=1;
 float q_command_mA_s=1,q_effective_pred_mA_s=1,predicted_current_goal_mA=1;
 float predicted_current_rise_tau_s=1,aligned_initial_current_mA=1;
 bool command_matches_zero_cross_motion=false,valid=false,output_executed=false;
 int16_t command_current_mA=0;uint16_t pulse_width_ms=0;uint32_t pulse_start_ms=0,pulse_end_ms=0;
 enum FootSpanFlag : uint16_t { Span_selected=2, NegativeRecoveryStateBit0=1u<<12, NegativeRecoveryStateBit1=1u<<13 };
 uint16_t foot_span_baseline_ms=0,foot_span_requested_ms=0,foot_span_flags=0;
 void setFootSpanFlag(FootSpanFlag f,bool v){if(v)foot_span_flags|=f;else foot_span_flags&=~f;}
};
}
struct Status {bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;
 float current_audit_q_target_mA_s=1,current_audit_q_pred_mA_s=1;};
struct Imu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct Roller {bool good=true;bool ok()const{return good;}};
struct Resettable {int resets=0;void reset(){++resets;}void interrupt(){++resets;}};
struct Adaptive {bool broken=false;void breakPair(){broken=true;}};
class ExperimentRunner {public:
 bool stall_rescue_event_=false;
 Status status_;Imu imu_obj;Roller roller_obj;Imu* imu_=&imu_obj;Roller* roller_=&roller_obj;
 tilt_priority::Controller tilt_priority_;foot_span_control::Controller foot_span_control_;
 wheel_cycle_guard::Controller wheel_cycle_guard_;
 long_foot_balance::Controller long_foot_balance_;
 negative_cap_recovery::Controller negative_cap_recovery_;
 uint32_t negative_recovery_last_applied_ms_=0;
 uint32_t foot_trial_pending_ms_=0,foot_trial_pending_generation_=0;
 bool foot_trial_pending_effective_=false;
 Resettable weak_side_assist_,walking_loss_feedback_,walking_low_speed_guard_;
 Adaptive adaptive_cg_;bool mode=true,begin_ok=true;unsigned calls=0;int sign=0,ma=0;uint16_t width=0;
 float run_cg_shift_mm_=0,pending_cg_shift_mm_=0,pending_target_energy_j_=NAN;
 bool energy_control_autonomous_pending_peak_=false,energy_control_autonomous_pending_probe_=false;
 int8_t energy_control_autonomous_pending_next_side_=0;
 float energy_control_autonomous_pending_target_deg_=0,energy_control_autonomous_pending_q_command_mA_s_=0;
 float energy_control_autonomous_pending_model_energy_j_=0,energy_control_autonomous_pending_wheel_rpm_=0;
 uint32_t energy_control_autonomous_pending_wheel_age_us_=0,energy_control_autonomous_pending_zero_ms_=0,run_start_ms_=100;
 bool energy_control_autonomous_pending_wheel_valid_=false,energy_control_autonomous_pending_saturated_upper_=true;
 bool energy_control_autonomous_pending_saturated_lower_=true,work_positive_pending_=true,work_negative_pending_=true;
 bool work_walking_pending_=true,work_walking_low_pending_=true;
 peak_error_adaptation::Mode energy_control_autonomous_pending_mode_=peak_error_adaptation::Coast;
 bool wheelWorkMode()const{return mode;}
 bool beginEnergyControlAutonomousPulse(uint32_t,uint32_t,int8_t d,uint16_t w,int16_t m){
   ++calls;sign=d;width=w;ma=m;return begin_ok;
 }
 bool tryTiltPriority(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
 bool tryTiltPriorityImpl(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,bool*);
 bool tryTiltPriorityRescue(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
};
// METHOD_SOURCE
using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
int main(){
 // Seed the real tilt controller with an overshoot-induced negative cap of
 // 30ms and a completed 8-degree startup; no actuator shortcuts are used.
 auto prepare_negative=[](ExperimentRunner& r) {
  uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  for(int n=0;n<3;++n)for(int side:{-1,1}) {
   tilt_priority::Choice c;c.selected=true;c.direction=static_cast<int8_t>(side);
   c.width_ms=r.tilt_priority_.widthLimit(side);c.current_ma=1200;c.target_deg=r.tilt_priority_.target();
   r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
   r.tilt_priority_.confirm(static_cast<int8_t>(side),t,c,true);
   r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,side<0?8.f:3.f);t+=300;
  }
  assert(r.tilt_priority_.widthLimit(-1)==30);
  for(int n=0;n<14;++n)for(int side:{-1,1}) {
   tilt_priority::Choice c;c.selected=true;c.coast=true;c.direction=static_cast<int8_t>(side);c.target_deg=r.tilt_priority_.target();
   r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
   r.tilt_priority_.confirm(static_cast<int8_t>(side),t,c,true);
   r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,3.f);t+=300;
  }
  assert(r.tilt_priority_.target()==8.f);
  for(int i=0;i<6;++i) {
   const uint32_t start=t-4300+i*700,end=start+700;
   r.negative_cap_recovery_.peak(1,start+300,6.f,true,true);
   r.negative_cap_recovery_.peak(-1,end,5.f,true,true);
   r.negative_cap_recovery_.footCycle(start,end,3.f,.5f,true,true,true);
  }
  assert(r.negative_cap_recovery_.candidate(t,8.f,30,true,true));
  return t;
 };
 // Even when all old negative-recovery evidence is present, the ordinary
 // protected command is sent and the trial cannot start.
 {ExperimentRunner r;auto t=prepare_negative(r);Event e;e.physical_next_peak_side=-1;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==30);
  assert(!e.weak_probe_executed && !r.negative_cap_recovery_.active());
  Event held;held.physical_next_peak_side=-1;assert(r.tryTiltPriority(held,t+300));
  assert(held.pulse_width_ms==30 && !held.weak_probe_executed);}
 // A rejected send cannot start the trial or advance its output history.
 {ExperimentRunner r;auto t=prepare_negative(r);r.begin_ok=false;Event e;e.physical_next_peak_side=-1;
  const auto guard_before=r.wheel_cycle_guard_;
  assert(r.tryTiltPriority(e,t));assert(!e.output_executed && !e.weak_probe_executed);
  assert(!r.negative_cap_recovery_.active() && !r.negative_cap_recovery_.attempted());
  auto reference=guard_before;assert(r.wheel_cycle_guard_.limit(1,300.f,true,t+100,60)==reference.limit(1,300.f,true,t+100,60));}
 // Guard-reduced baseline and candidate still use the same prior history.
 {ExperimentRunner r;auto t=prepare_negative(r);
  r.wheel_cycle_guard_.limit(1,-400.f,true,t-100,60);auto reference=r.wheel_cycle_guard_;
  assert(reference.limit(-1,-400.f,true,t,35)==27);
  Event e;e.physical_next_peak_side=-1;e.pre_input_wheel_speed_rpm=-400.f;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==23);
  assert(!e.weak_probe_executed && !r.negative_cap_recovery_.active());}
 // A timing cap that makes both candidates equal is not a successful probe.
 {ExperimentRunner r;auto t=prepare_negative(r);
  tilt_priority::Choice c;c.selected=true;c.direction=-1;c.width_ms=30;c.current_ma=1200;c.target_deg=8;
  r.tilt_priority_.crossing(-1,t-400);r.tilt_priority_.confirm(-1,t-400,c,true);
  r.tilt_priority_.peak(-1,t-340,3.f);
  Event e;e.physical_next_peak_side=-1;assert(r.tryTiltPriority(e,t));
  assert(e.output_executed && e.pulse_width_ms==30);
  assert(!e.weak_probe_executed);
  assert(!r.negative_cap_recovery_.active() && !r.negative_cap_recovery_.attempted());}
 // A legacy active probe is cancelled and cannot block the positive path.
 {ExperimentRunner r;auto t=prepare_negative(r);
  r.negative_cap_recovery_.command(t-100,30,35,true,true);
  assert(r.negative_cap_recovery_.active());
  Event positive;positive.physical_next_peak_side=1;assert(r.tryTiltPriority(positive,t));
  assert(positive.output_executed && !r.negative_cap_recovery_.active());}
 // The real dispatch records the old limit and bounded additional change.
 {ExperimentRunner r;
  r.wheel_cycle_guard_.peak(-1,5.3f,100,true);r.wheel_cycle_guard_.peak(1,5.4f,450,true);
  r.wheel_cycle_guard_.peak(-1,5.51f,800,true);r.wheel_cycle_guard_.peak(1,5.23f,1150,true);
  Event a;a.physical_next_peak_side=1;a.pre_input_wheel_speed_rpm=-270.f;
  assert(r.tryTiltPriority(a,1200));
  Event b;b.physical_next_peak_side=-1;b.pre_input_wheel_speed_rpm=14.f;
  assert(r.tryTiltPriority(b,1500));
  assert(b.output_executed && b.wheel_guard_trial_eligible && b.wheel_guard_extra_ms>0);
  assert(b.wheel_guard_extra_ms<=5 && b.pulse_width_ms+b.wheel_guard_extra_ms==b.wheel_guard_legacy_ms);
 }
 // No 1200mA foot-width trial interrupts the 4.5 degree startup stage.
 {ExperimentRunner r;Event e;assert(r.tryTiltPriority(e,500));
  assert(e.output_executed && e.pulse_width_ms==60 && e.foot_span_baseline_ms==0);
  assert(!(e.foot_span_flags & Event::Span_selected));}
 // Five valid observations qualify a proposal. These inputs verify dispatch,
 // not physical foot response. Guard comparisons use identical wheel history.
 auto prepare_foot=[](ExperimentRunner& r) {
  uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  for(int n=0;n<6;++n)for(int side: {-1,1}) {
    tilt_priority::Choice coast;coast.selected=true;coast.coast=true;
    coast.direction=static_cast<int8_t>(side);coast.target_deg=r.tilt_priority_.target();
    r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
    r.tilt_priority_.confirm(static_cast<int8_t>(side),t,coast,true);
    r.tilt_priority_.peak(static_cast<int8_t>(side),t+100,3.f);t+=300;
  }
  assert(r.tilt_priority_.target()==6.f);
  for(int i=0;i<5;++i)r.long_foot_balance_.observe(t-600+i*100,t-500+i*100,3.f,1.f,6.f,5.f,true,true);
  return t;
 };
 // Production camera path groups six adjacent complete cycles into three
 // paired observations before the actual protected output is selected.
 {ExperimentRunner r;auto t=prepare_foot(r);r.long_foot_balance_.reset();
  for(int i=0;i<6;++i) {
   const uint32_t start=t-1800+i*300,end=start+300;
   r.long_foot_balance_.observeCycle(start,end,3.f,1.f,6.f,5.f,true,true,true);
  }
  Event e;assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==65);
  assert(r.long_foot_balance_.offset()==5 && r.foot_trial_pending_effective_);}
 {ExperimentRunner r;auto t=prepare_foot(r);Event e;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==65);
  assert(e.foot_span_baseline_ms==60 && e.foot_span_requested_ms==65);
  assert(e.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_ms_==t);
  assert(r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());
  Event held;assert(r.tryTiltPriority(held,t+200));
  assert(held.pulse_width_ms==65 && held.foot_span_baseline_ms==60 && held.foot_span_requested_ms==65);
  assert(held.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());}
 {ExperimentRunner r;auto t=prepare_foot(r);
  r.wheel_cycle_guard_.limit(-1,400.f,true,t-100,60);
  auto expected=r.wheel_cycle_guard_;assert(expected.limit(1,400.f,true,t,65)==49);
  Event e;e.pre_input_wheel_speed_rpm=400.f;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==49);
  assert(e.foot_span_baseline_ms==45 && e.foot_span_requested_ms==49);
  assert(e.foot_span_flags & Event::Span_selected);assert(r.long_foot_balance_.offset()==5);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());
  assert(r.wheel_cycle_guard_.limit(-1,200.f,true,t+300,40)==expected.limit(-1,200.f,true,t+300,40));}
 // A later observed cycle may have no intervention left after the timing
 // cap, even though the controller still holds its previously sent offset.
 // Such a cycle is recorded, but cannot serve as post-trial evidence.
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));assert(r.foot_trial_pending_effective_);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t+200);r.tilt_priority_.confirm(1,t+200,coast,true);
  r.tilt_priority_.peak(1,t+285,3.f);
  Event clipped;assert(r.tryTiltPriority(clipped,t+300));assert(clipped.output_executed);
  assert(r.long_foot_balance_.trialActive());
  assert(clipped.weak_probe_baseline_ms==clipped.weak_probe_requested_ms);
  assert(!r.foot_trial_pending_effective_ && !clipped.weak_probe_executed);}
 // A second trial compares +10 against the already accepted +5, not against
 // ordinary 0. No post-confirm generation may reuse the older reference.
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));const auto first_generation=r.long_foot_balance_.trialGeneration();
  for(int i=0;i<5;++i){const uint32_t ms=t+500+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  assert(r.long_foot_balance_.state()==long_foot_balance::Controller::State::Hold);
  for(int i=0;i<5;++i){const uint32_t ms=t+2200+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  const uint32_t t2=t+3700;r.tilt_priority_.peak(-1,t2-50,3.f);
  Event second;assert(r.tryTiltPriority(second,t2));assert(second.output_executed);
  assert(r.long_foot_balance_.trialGeneration()!=first_generation);
  assert(r.long_foot_balance_.trialReferenceOffset()==5 && r.long_foot_balance_.offset()==10);
  assert(second.weak_probe_baseline_ms==65 && second.weak_probe_requested_ms==70);
  assert(r.foot_trial_pending_effective_ && r.foot_trial_pending_generation_==r.long_foot_balance_.trialGeneration());}
 {ExperimentRunner r;auto t=prepare_foot(r);Event first;
  assert(r.tryTiltPriority(first,t));const auto gen=r.long_foot_balance_.trialGeneration();
  for(int i=0;i<5;++i){const uint32_t ms=t+500+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  for(int i=0;i<5;++i){const uint32_t ms=t+2200+i*300;
   r.long_foot_balance_.observe(ms-100,ms,3.f,1.5f,6.f,5.f,true,true,true);}
  const uint32_t t2=t+3700;r.tilt_priority_.peak(-1,t2-140,3.f);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t2-100);r.tilt_priority_.confirm(1,t2-100,coast,true);
  r.tilt_priority_.peak(1,t2-15,3.f);
  Event blocked;assert(r.tryTiltPriority(blocked,t2));assert(blocked.output_executed);
  assert(r.long_foot_balance_.trialGeneration()==gen && r.long_foot_balance_.offset()==5);
  assert(blocked.weak_probe_baseline_ms==blocked.weak_probe_requested_ms);
  assert(!r.foot_trial_pending_effective_ && !blocked.weak_probe_executed);}
 {ExperimentRunner r;auto t=prepare_foot(r);r.begin_ok=false;Event e;
  r.wheel_cycle_guard_.limit(-1,300.f,true,t-100,60);auto untouched=r.wheel_cycle_guard_;
  assert(r.tryTiltPriority(e,t));assert(!e.output_executed && !(e.foot_span_flags & Event::Span_selected));
  assert(r.long_foot_balance_.offset()==0);
  assert(!r.foot_trial_pending_effective_ && r.foot_trial_pending_ms_==0);
  assert(r.wheel_cycle_guard_.limit(1,300.f,true,t+100,60)==untouched.limit(1,300.f,true,t+100,60));}
 {ExperimentRunner r;auto t=prepare_foot(r);
  tilt_priority::Choice coast;coast.selected=true;coast.coast=true;coast.direction=1;coast.target_deg=6;
  r.tilt_priority_.crossing(1,t-100);r.tilt_priority_.confirm(1,t-100,coast,true);
  r.tilt_priority_.peak(1,t-10,3.f);
  Event e;assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms==60);
  assert(e.foot_span_baseline_ms==60 && e.foot_span_requested_ms==60);
  assert(!(e.foot_span_flags & Event::Span_selected));assert(r.long_foot_balance_.offset()==0);}
 // The optional camera trial must never silently cap an ordinary 1200 mA
 // positive pulse at 100 ms. Grow the existing ramp through actual paired
 // peak/command confirmations, then test the real runner dispatch.
 {ExperimentRunner r;uint32_t t=1000;r.tilt_priority_.peak(1,100,2.f);
  auto action=[&](int side){
    tilt_priority::Input in;in.now_ms=t;in.preset_deg=8;in.target_deg=r.tilt_priority_.target();
    in.crossing_rate_dps=10;in.next_side=static_cast<int8_t>(side);in.hardware_cap_ms=150;in.hardware_healthy=true;
    auto choice=r.tilt_priority_.propose(in);assert(choice.selected&&!choice.coast);
    r.tilt_priority_.crossing(static_cast<int8_t>(side),t);
    r.tilt_priority_.confirm(static_cast<int8_t>(side),t,choice,true);
    r.tilt_priority_.peak(static_cast<int8_t>(side),t+200,3.f);t+=400;
  };
  for(int n=0;n<12;++n){action(-1);action(1);}
  action(-1);assert(r.tilt_priority_.widthLimit(1)>100);
  Event e;e.physical_next_peak_side=1;e.zero_cross_abs_rate_dps=10;
  assert(r.tryTiltPriority(e,t));assert(e.output_executed && e.pulse_width_ms>100);
  assert(e.foot_span_baseline_ms==0 && !(e.foot_span_flags & Event::Span_selected));
 }
 // Replay the measured startup peaks and crossings from run 302. The new
 // output widths are not a prediction of the resulting physical peaks.
 {ExperimentRunner r;r.tilt_priority_.peak(1,220,2.21185f);
  Event e;e.physical_next_peak_side=-1;e.zero_cross_time_ms=414;e.zero_cross_abs_rate_dps=18.645f;
  assert(r.tryTiltPriority(e,416));
  assert(e.valid&&e.output_executed&&e.work_control_branch==15);
  assert(r.sign==-1&&r.ma==1200&&r.width==59&&e.pulse_end_ms==475);
  assert(r.energy_control_autonomous_pending_probe_&&r.energy_control_autonomous_pending_next_side_==-1);
  assert(r.energy_control_autonomous_pending_target_deg_==4.5f);
  assert(std::isnan(r.energy_control_autonomous_pending_model_energy_j_));
  assert(!r.work_positive_pending_&&!r.work_negative_pending_&&r.adaptive_cg_.broken);
  r.tilt_priority_.peak(-1,641,6.57367f);
  Event e2;e2.zero_cross_time_ms=865;e2.zero_cross_abs_rate_dps=36.6997f;assert(r.tryTiltPriority(e2,867));
  assert(e2.valid&&!e2.output_executed&&e2.reason==1&&r.calls==1);
  assert(e2.command_current_mA==0&&e2.pulse_width_ms==0&&e2.pulse_start_ms==0);
  assert(e2.q_command_direction==0&&e2.q_command_mA_s==0);
  assert(r.energy_control_autonomous_pending_peak_&&r.energy_control_autonomous_pending_probe_);
  assert(r.energy_control_autonomous_pending_mode_==peak_error_adaptation::Coast);
  // Synthetic return peak after one coast permits a short recovery pulse.
  r.tilt_priority_.peak(1,1098,3.8f);
  Event e3;e3.physical_next_peak_side=-1;assert(r.tryTiltPriority(e3,1300));
  assert(e3.valid&&e3.output_executed&&e3.pulse_width_ms==15&&r.calls==2);
  r.tilt_priority_.peak(-1,1500,3.5f);
  Event e4;assert(r.tryTiltPriority(e4,1700));
  assert(e4.output_executed&&r.calls==3&&e4.command_current_mA==1200);
 }
 {ExperimentRunner r;Event e;r.begin_ok=false;assert(r.tryTiltPriority(e,1000));
  assert(!e.valid&&!e.output_executed&&e.pulse_width_ms==0&&e.reason==12);
  assert(!r.energy_control_autonomous_pending_peak_);
  r.tilt_priority_.peak(1,1190,1.f);
  assert(r.tilt_priority_.widthLimit(1)==60);}
 // A failed rescue preserves the measured-controller history and must not
 // interrupt the independent weak-side adaptation. The next ordinary
 // decision matches an otherwise identical untouched runner.
 {ExperimentRunner r,control; r.stall_rescue_event_=true;r.begin_ok=false;
  Event trial;trial.physical_next_peak_side=-1;trial.zero_cross_abs_rate_dps=2;
  assert(r.tryTiltPriority(trial,1000));
  assert(!trial.output_executed && trial.reason==12 && r.weak_side_assist_.resets==0);
  r.stall_rescue_event_=false;r.begin_ok=true;
  Event a,b;a.physical_next_peak_side=b.physical_next_peak_side=-1;
  assert(r.tryTiltPriority(a,1300) && control.tryTiltPriority(b,1300));
  assert(a.output_executed && b.output_executed && a.pulse_width_ms==b.pulse_width_ms);
  assert(r.tilt_priority_.widthLimit(-1)==control.tilt_priority_.widthLimit(-1));
  assert(r.wheel_cycle_guard_.limit(1,300.f,true,1600,40)==
         control.wheel_cycle_guard_.limit(1,300.f,true,1600,40));
 }
 // Run 803's recorded 1858 ms crossing and immediately prior peak,
 // evaluated as an isolated hypothetical decision at the initial stage.
 {ExperimentRunner r;r.tilt_priority_.peak(1,1698,4.67317f);
  Event e;e.zero_cross_time_ms=1858;e.physical_next_peak_side=-1;e.zero_cross_abs_rate_dps=41.0152f;
  assert(r.tryTiltPriority(e,1860));
  assert(e.valid&&e.output_executed&&e.pulse_width_ms==34&&r.sign==-1);}
 // Signed direction remains determined by the crossing side; the rate is
 // the positive interpolated magnitude, for both signs.
 {ExperimentRunner slow,medium,fast,negative;Event a,b,c,d;
  a.zero_cross_abs_rate_dps=15;b.zero_cross_abs_rate_dps=30;c.zero_cross_abs_rate_dps=60;
  d.zero_cross_abs_rate_dps=30;d.physical_next_peak_side=-1;
  assert(slow.tryTiltPriority(a,1000)&&medium.tryTiltPriority(b,1000));
  assert(fast.tryTiltPriority(c,1000)&&negative.tryTiltPriority(d,1000));
  assert(a.pulse_width_ms==60&&b.pulse_width_ms<60&&b.pulse_width_ms>20);
  assert(c.valid&&c.output_executed&&c.pulse_width_ms==20&&fast.calls==1);
  assert(d.pulse_width_ms==b.pulse_width_ms&&negative.sign==-1);
 }
 {ExperimentRunner r;Event e;e.zero_cross_abs_rate_dps=NAN;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.zero_cross_abs_rate_dps=0;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;r.status_.emergency_stop=true;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;r.imu_obj.good=false;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.pre_input_wheel_speed_age_us=999999;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 {ExperimentRunner r;Event e;e.pre_input_wheel_speed_rpm=1200.f;assert(r.tryTiltPriority(e,1000));
  assert(e.valid&&e.output_executed&&r.ma==1200);}
 {ExperimentRunner r;Event e;e.target_peak_deg=10;assert(!r.tryTiltPriority(e,1000)&&r.calls==0);}
 std::cout<<"actual runner staged output, coast recovery and sensor guards PASS\n";
}
