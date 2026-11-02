"""Execute the real target-energy method with a host-side geometry double.

This checks wiring and peak/preset bookkeeping, not physical left/right mapping,
the fitted energy model, embedded timing, or hardware motor response.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
runner=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
start='void ExperimentRunner::applyFootEnergyTarget('
end='\nvoid ExperimentRunner::capturePendingWheelWork('
assert runner.count(start)==1 and runner.count(end)==1
method=start+runner.split(start,1)[1].split(end,1)[0]
select=runner.split('bool ExperimentRunner::selectFootPhaseOutput(',1)[1].split(
    '\nvoid ExperimentRunner::choosePeakMpc(',1)[0]
assert select.index('if(!phase_sensors_fresh)') < select.index('positivePeakAtLeast(')
assert select.index('positivePeakAtLeast(') < select.index('const uint16_t limited=')
protect=select.split('if(!gradual_start_.positivePeakAtLeast(',1)[1].split(
    'const uint16_t limited=',1)[0]
assert 'foot_span_control_.selectorUnavailable(t_test_ms)' in protect
assert 'event.foot_span_requested_ms=event.foot_span_baseline_ms;' in protect
assert 'return false;' in protect
pending='energy_control_autonomous_pending_target_deg_ = work_mode && event.target_peak_deg==8.f\n      ? event.foot_balance_target_deg : event.target_peak_deg;'
assert pending in runner
peak=runner.split('bool ExperimentRunner::recordEnergyControlAutonomousPeak(',1)[1].split(
    '\nvoid ExperimentRunner::',1)[0]
assert 'energyControlPeakTarget(physical_side)' in peak
target=runner.split('float ExperimentRunner::energyControlPeakTarget(',1)[1].split('\n}',1)[0]
assert 'physical_side == energy_control_autonomous_pending_next_side_' in target
assert '? energy_control_autonomous_pending_target_deg_' in target
assert 'gradual_start::sideTargetDeg(gradual_start_.target(),physical_side)' in target
assert 'state.target_deg=event.target_peak_deg;' in runner
feedback=runner.split('void ExperimentRunner::observeWheelWorkPeak(',1)[1].split(
    '\nvoid ExperimentRunner::captureWheelWorkDecision(',1)[0]
assert 'const float preset_deg=energy_control_autonomous_target_peak_deg_;' in feedback
assert 'work_positive_pending_branch_==0 && preset_deg==8.f' in feedback
assert 'negativeFeedbackIndex(preset_deg,work_negative_pending_assist_)' in feedback
fixture=r'''#include "gradual_start.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
namespace PsramLogger { struct EnergyControlAutonomousZeroCrossEvent {
 int8_t physical_next_peak_side=0;
 float target_energy_j=0,foot_balance_target_deg=0,foot_energy_bias=0,
 foot_energy_left_px=0,foot_energy_right_px=0,low_peak_bias_mj=0;
 bool foot_energy_valid=false,low_peak_active=false;
 int foot_energy_phase=0,foot_energy_reason=0;
}; }
struct Foot {float bias()const{return .2f;}float leftMean()const{return 2;}
 float rightMean()const{return 1;}bool valid()const{return true;}
 int phase()const{return 1;}int reason()const{return 0;}};
struct Low {float bias()const{return 0;}bool active()const{return false;}};
struct Ramp {float value=8;float target()const{return value;}};
class ExperimentRunner {public:
 Ramp gradual_start_;Foot foot_energy_;Low low_peak_feedback_;
 float controlPotential(float signed_deg)const{return signed_deg*signed_deg*.001f;}
 void applyFootEnergyTarget(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t);
};
// METHOD
int main(){
 ExperimentRunner r;PsramLogger::EnergyControlAutonomousZeroCrossEvent e;
 e.physical_next_peak_side=1;r.applyFootEnergyTarget(e,100);
 assert(std::fabs(e.foot_balance_target_deg-10)<1e-6f);
 assert(std::fabs(e.target_energy_j-.100f)<1e-6f);
 e.physical_next_peak_side=-1;r.applyFootEnergyTarget(e,100);
 assert(std::fabs(e.foot_balance_target_deg-8)<1e-6f);
 assert(std::fabs(e.target_energy_j-.064f)<1e-6f);
 r.gradual_start_.value=4.5f;e.physical_next_peak_side=1;r.applyFootEnergyTarget(e,100);
 assert(std::fabs(e.foot_balance_target_deg-4.5f)<1e-6f);
 assert(std::fabs(e.target_energy_j-.02025f)<1e-6f);
 std::cout<<"actual runner side target and energy wiring PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='side_target_runner_') as td:
 source=Path(td)/'side_target.cpp';exe=Path(td)/'side_target.exe'
 source.write_text(fixture.replace('// METHOD',method),encoding='utf-8')
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'src'),
                 str(source),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
 
# Compile and execute the real post-selector guard with host sensor doubles.
select_method='bool ExperimentRunner::selectFootPhaseOutput('+select.split(
    'void ExperimentRunner::choosePeakMpc(',1)[0]
selector_fixture=r'''#include "gradual_start.h"
#include "foot_span_control.h"
#include "foot_span_candidate.h"
#include "peak_error_adaptation.h"
#include <cassert>
#include <cstdint>
#include <iostream>
uint32_t millis(){return 100000;}
namespace Config {constexpr int ENERGY_CONTROL_AUTONOMOUS_REASON_ESTOP_OR_STATE=1;
 constexpr int ENERGY_CONTROL_AUTONOMOUS_REASON_ROLLER_NOT_READY=2;
 constexpr int ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE=3;}
enum class ExperimentState {RUNNING_BATCH_SWEEP,OTHER};
namespace PsramLogger {struct EnergyControlAutonomousZeroCrossEvent {
 enum {Span_selected=2};uint16_t foot_span_baseline_ms=0,foot_span_requested_ms=0;
 float target_peak_deg=8,foot_balance_target_deg=10,target_energy_j=.001f;
 float pre_input_wheel_speed_rpm=200;bool pre_input_wheel_speed_valid=true;
 uint32_t pre_input_wheel_speed_age_us=1000;uint16_t vbat_mV=7800;
 int8_t physical_next_peak_side=1;bool side_mismatch_diagnostic=false;int reason=0;
 void setFootSpanFlag(int,bool){}
};}
struct Roller {bool ok()const{return true;}};
struct Imu {bool acquisitionHealthy()const{return true;}bool stale(uint32_t)const{return false;}};
struct Status {bool emergency_stop=false;ExperimentState state=ExperimentState::RUNNING_BATCH_SWEEP;};
class ExperimentRunner {public:
 Roller roller_storage;Imu imu_storage;Roller* roller_=&roller_storage;Imu* imu_=&imu_storage;
 Status status_;gradual_start::Controller gradual_start_;foot_span_control::Controller foot_span_control_;
 lateral_energy::CurrentTable positive_current_probe_tables_[2],lateral_current_table_;
 bool wheelWorkMode()const{return true;}
 void requestEmergencyStop(const char*){status_.emergency_stop=true;}
 wheel_work_energy_control::Choice chooseFootPhaseWidth(
     const PsramLogger::EnergyControlAutonomousZeroCrossEvent&,const wheel_work_energy_control::Choice& c,uint16_t,float){return c;}
 bool selectFootPhaseOutput(PsramLogger::EnergyControlAutonomousZeroCrossEvent&,uint32_t,float,
   wheel_work_energy_control::Choice&,int16_t&,const lateral_energy::CurrentTable*&);
};
// METHOD
int main(){
 using Event=PsramLogger::EnergyControlAutonomousZeroCrossEvent;
 using Choice=wheel_work_energy_control::Choice;
 Choice choice;choice.valid=true;choice.width_ms=80;choice.allowed_cap_ms=100;
 choice.branch=5;choice.current_ma=400;choice.command_sign=1;
 int16_t current=400;const lateral_energy::CurrentTable* table=nullptr;
 {ExperimentRunner r;Event e;
  // Develop a proposed -10ms correction, then demonstrate that an under-
  // target positive peak cancels it before it can reduce the normal output.
  auto sample=[&](uint32_t start){
   auto width=r.foot_span_control_.limit(start+50,80,400,1,100,true,1);
   r.foot_span_control_.confirm(start+50,true,400,1,width);
   foot_span_control::Cycle c;c.start_ms=start;c.end_ms=start+100;c.positive_ms=start+50;
   c.left=3;c.right=1;c.negative_peak=5;c.target_mj=1;c.positive_width=width;
   c.positive_ma=400;c.positive_sign=1;c.complete=c.positive_executed=true;
   r.foot_span_control_.cycle(c);
  };
  sample(100);sample(200);assert(r.foot_span_control_.offset()==-10);
  r.gradual_start_.peak(1,5,300,true,300);
  assert(!r.selectFootPhaseOutput(e,350,7.8,choice,current,table));
  assert(r.foot_span_control_.offset()==0 && e.foot_span_requested_ms==80);
  assert(choice.width_ms==80 && !r.status_.emergency_stop);
 }
 {ExperimentRunner r;Event e;e.pre_input_wheel_speed_valid=false;
  assert(r.selectFootPhaseOutput(e,350,7.8,choice,current,table));
  assert(e.reason==Config::ENERGY_CONTROL_AUTONOMOUS_REASON_NONFINITE_STATE);
 }
 {ExperimentRunner r;Event e;r.gradual_start_.peak(1,11,300,true,300);
  assert(!r.selectFootPhaseOutput(e,350,7.8,choice,current,table));
  assert(!r.status_.emergency_stop);
 }
 std::cout<<"actual runner positive-target priority and sensor gate PASS\n";
}
'''
with tempfile.TemporaryDirectory(prefix='side_priority_runner_') as td:
 source=Path(td)/'priority.cpp';exe=Path(td)/'priority.exe'
 source.write_text(selector_fixture.replace('// METHOD',select_method),encoding='utf-8')
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+str(root/'src'),
                 str(source),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
