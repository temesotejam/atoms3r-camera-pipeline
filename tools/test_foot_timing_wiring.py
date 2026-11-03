"""Check actual source scope and the telemetry contract for timing experiment."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
runner=(root/'src/experiment_runner.cpp').read_text(encoding='utf-8')
policy=runner.split('void ExperimentRunner::updateEnergyControlAutonomousMotion(',1)[1].split('\nvoid ExperimentRunner::updateEnergyControlAutonomousPeakTracker(',1)[0]
assert 'energy_control_autonomous_input_peak_percent_==0 && peak_side==1' in policy
assert 'threshold_deg=tilt_priority_.negativeEarlyThreshold(t_test_ms,peak_side)' in policy
assert 'before_deg > threshold_deg && detector_relative_angle_deg <= threshold_deg' in policy
assert 'rate_dps * peak_side < 0.0f' in policy
cycle=runner.split('void ExperimentRunner::processFootSpanCycle()',1)[1].split('\nvoid ExperimentRunner::recordFootPhasePeak(',1)[0]
assert 'span.left.trimmed_span,span.right.trimmed_span,timing_quality' in cycle
assert '!span.left.single_outlier && !span.right.single_outlier' in cycle
assert '!span.left.edge_unverified && !span.right.edge_unverified' in cycle
logger=(root/'src/psram_logger.cpp').read_text(encoding='utf-8')
assert 'target8_zero_percent_imbalanced_positive_return_may_use_positive_early_threshold_up_to_3deg' in logger
print('foot timing motion and telemetry wiring PASS')
