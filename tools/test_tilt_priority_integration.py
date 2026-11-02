"""Execute the real tilt-priority runner with hardware doubles.

This checks dispatch and telemetry, not ESP timing or physical response.
"""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
runner=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
fixture=(root/'tools/tilt_priority_integration_fixture.cpp').read_text(encoding='utf-8')
start='bool ExperimentRunner::tryTiltPriority('
end='\nbool ExperimentRunner::tryWeakSideAssist('
assert runner.count(start)==1 and runner.count(end)==1
method=start+runner.split(start,1)[1].split(end,1)[0]
at_zero=runner.split('void ExperimentRunner::updateEnergyControlAutonomousAtZeroCross(',1)[1]
assert at_zero.index('tryTiltPriority(event,t_test_ms)') < at_zero.index('work_choice=chooseWheelWork(')
assert 'energy_control_autonomous_pending_probe_=true' in method
assert 'energy_control_autonomous_pending_model_energy_j_=NAN' in method
assert 'event.work_control_branch=15' in method
assert 'tilt_priority_.crossing(in.next_side,t_test_ms)' in method
assert 'tilt_priority_.peak(physical_side,peak_ms,amplitude_deg)' in runner
assert 'std::fabs(event.pre_input_wheel_speed_rpm)' not in method
assert fixture.count('// METHOD_SOURCE')==1
fixture=fixture.replace('// METHOD_SOURCE',method)
with tempfile.TemporaryDirectory(prefix='tilt_priority_runner_') as td:
 source=Path(td)/'fixture.cpp';exe=Path(td)/'fixture'
 source.write_text(fixture,encoding='utf-8')
 subprocess.run(['g++','-std=c++11','-O2','-Wall','-Wextra','-Werror',
                 '-I'+str(root/'src'),str(source),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
