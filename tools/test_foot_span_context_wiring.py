"""Check actual runner's camera context, 1200mA dispatch and logging."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'src/experiment_runner.cpp').read_text(encoding='utf8')
cycle=s.split('void ExperimentRunner::processFootSpanCycle()',1)[1].split('\nvoid ExperimentRunner::recordFootPhasePeak(',1)[0]
assert 'cycle.left=span.left.trimmed_span;cycle.right=span.right.trimmed_span' in cycle
for x in ('negative->trigger_deg','negative->rate_dps','negative->rpm','context_ok'):
    assert x in cycle
decision=s.split('void ExperimentRunner::finishFootEnergyDecision(',1)[1].split('\nbool ExperimentRunner::update',1)[0]
for x in ('d.trigger_deg=', 'd.rate_dps=event.zero_cross_abs_rate_dps',
          'event.pre_input_wheel_speed_valid && event.pre_input_wheel_speed_age_us<=50000',
          'foot_span_control_.confirm(', 'event.work_control_branch==15 && (event.pulse_width_ms>100 || tilt_priority_.target()<6.f)'):
    assert x in decision
runner=s.split('bool ExperimentRunner::tryTiltPriority(',1)[1].split('\nbool ExperimentRunner::tryWeakSideAssist(',1)[0]
assert 'foot_span_control_.reset()' not in runner
assert 'foot_span_control_.limit(' in runner and 'choice.width_ms<=100' in runner
assert 'tilt_priority_.target()>=6.f' in runner and 'preferHighCurrentFirst()' in runner
assert 'tilt_priority_.timingCap(' in runner and 'Span_selected' in runner
metadata=(root/'src/psram_logger.cpp').read_text(encoding='utf8')
assert 'confirmed_1200ma_positive_width_trial_v1' in metadata
print('1200mA camera feedback, comparable context and log wiring PASS')
