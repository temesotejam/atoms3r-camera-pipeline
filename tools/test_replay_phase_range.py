#!/usr/bin/env python3
"""Check that historical phase replay applies the same range gate as runtime."""
from replay_phase_cap import events_for

frame = {
    "measurement_time_us": 1000, "frame_us": 2000, "delivered_us": 2200,
    "processing_us": 100, "frame_valid": True, "zero_ready": True,
    "left_valid": True, "right_valid": True, "left_in_range": True,
    "right_in_range": True, "left_deg": 1, "right_deg": 2,
}

def flags(**updates):
    f = {**frame, **updates}
    m = {"led_sync_start_total_ms": 0, "foot_frames": [f],
         "energy_control_autonomous_peak_events": [],
         "energy_control_autonomous_zero_cross_events": []}
    tokens = events_for({}, m, 0, 0)[0][2].split()
    return int(tokens[3]), int(tokens[4])

assert flags() == (1, 1)
assert flags(left_in_range=False) == (0, 1)
assert flags(right_in_range=False) == (1, 0)
assert flags(left_in_range=None) == (0, 1)
assert flags(right_valid=False) == (1, 0)
print("phase replay calibration-range gate PASS")
