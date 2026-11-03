#include "tilt_priority.h"
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
 float passive_energy_j=1,free_next_peak_amplitude_deg=1,predicted_energy_j=1,predicted_observed_energy_j=1;
 float predicted_next_peak_amplitude_deg=1,predicted_observed_next_peak_deg=1,predicted_wheel_work_j=1;
 float predicted_input_work_j=1,predicted_post_input_work_j=1,predicted_work_change_j=1;
 float q_command_mA_s=1,q_effective_pred_mA_s=1,predicted_current_goal_mA=1;
 float predicted_current_rise_tau_s=1,aligned_initial_current_mA=1;
 bool command_matches_zero_cross_motion=false,valid=false,output_executed=false;
 int16_t command_current_mA=0;uint16_t pulse_width_ms=0;uint32_t pulse_start_ms=0,pulse_end_ms=0;
};
}
struct Status {bool emergency_stop=false,pulse_active=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;
 float current_audit_q_target_mA_s=1,current_audit_q_pred_mA_s=1;};
struct Imu {bool good=true;bool acquisitionHealthy()const{return good;}bool stale(uint32_t)const{return !good;}};
struct Roller {bool good=true;bool ok()const{return good;}};
struct Resettable {int resets=0;void reset(){++resets;}void interrupt(){++resets;}};
struct Adaptive {bool broken=false;void breakPair(){broken=true;}};
class ExperimentRunner {public:
 Status status_;Imu imu_obj;Roller roller_obj;Imu* imu_=&imu_obj;Roller* roller_=&roller_obj;
 tilt_priority::Controller tilt_priority_;Resettable foot_span_control_,weak_side_assist_,walking_loss_feedback_,walking_low_speed_guard_;
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
};
// METHOD_SOURCE
using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
int main(){
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
  // Synthetic later peaks test that coast remains in control and output can
  // resume after the old large peak has been replaced by low measurements.
  r.tilt_priority_.peak(1,1098,3.8f);
  Event e3;e3.physical_next_peak_side=-1;assert(r.tryTiltPriority(e3,1300));
  assert(e3.valid&&!e3.output_executed&&r.calls==1);
  r.tilt_priority_.peak(-1,1500,3.5f);
  Event e4;assert(r.tryTiltPriority(e4,1700));
  assert(e4.output_executed&&r.calls==2&&e4.command_current_mA==1200);
 }
 {ExperimentRunner r;Event e;r.begin_ok=false;assert(r.tryTiltPriority(e,1000));
  assert(!e.valid&&!e.output_executed&&e.pulse_width_ms==0&&e.reason==12);
  assert(!r.energy_control_autonomous_pending_peak_);
  r.tilt_priority_.peak(1,1190,1.f);
  assert(r.tilt_priority_.widthLimit(1)==60);}
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
