#!/usr/bin/env python3
"""Check that historical phase replay applies the same range gate as runtime."""
from replay_phase_cap import events_for
import subprocess
import sys

frame = {
    "measurement_time_us": 1000, "frame_us": 2000, "delivered_us": 2200,
    "processing_us": 100, "frame_valid": True, "zero_ready": True,
    "left_valid": True, "right_valid": True, "left_in_range": True,
    "right_in_range": True, "left_deg": 1, "right_deg": 2,
    "left_x": 202.5, "right_x": 204.0,
}

def flags(**updates):
    f = {**frame, **updates}
    m = {"led_sync_start_total_ms": 0, "foot_frames": [f],
         "energy_control_autonomous_peak_events": [],
         "energy_control_autonomous_zero_cross_events": []}
    tokens = events_for({}, m, 0, 0)[0][2].split()
    assert tokens[0] == 'B'
    return int(tokens[3]), int(tokens[4]), int(tokens[5])

assert flags() == (1, 1, 1)
# Strict flags are preserved in the log; the C++ phase-only gate receives raw x.
assert flags(left_in_range=False) == (1, 1, 1)
assert flags(right_in_range=False) == (1, 1, 1)
assert flags(right_valid=False) == (1, 0, 1)
assert flags(left_x=float('nan')) == (0, 1, 1)
assert flags(right_deg=float('nan')) == (1, 0, 1)
assert flags(zero_ready=False) == (1, 1, 0)
print("phase replay calibration-range gate PASS")

# End-to-end: Python forwards recorded raw coordinates to the same C++
# boundedFrame used by the runner, rather than implementing a parallel gate.
def replay(left_x, right_x, **updates):
    lines=['R 1']
    for i, side in enumerate((-1, 1, -1, 1, -1)):
        t=1000+i*400
        lines.append(f'P {t} {t} {side}')
        for offset, active in [(10, True),(110, False)]:
            f={**frame, **updates, 'measurement_time_us': (t+offset)*1000,
               'frame_us': (t+offset)*1000, 'delivered_us': (t+offset)*1000,
               'processing_us': 0, 'left_x': left_x, 'right_x': right_x,
               'left_valid': active and updates.get('left_valid',True),
               'right_valid': active and updates.get('right_valid',True),
               'left_deg': updates.get('left_deg',4 if side==1 else 1),
               'right_deg': updates.get('right_deg',1 if side==1 else 2)}
            m={'led_sync_start_total_ms':0,'foot_frames':[f],
               'energy_control_autonomous_peak_events':[],'energy_control_autonomous_zero_cross_events':[]}
            lines.append(events_for({},m,0,0)[0][2])
    result=subprocess.run([sys.argv[1]],input='\n'.join(lines)+'\n',text=True,capture_output=True,check=True)
    return [s.split() for s in result.stdout.splitlines()]

assert len(sys.argv)==2, 'pass compiled test_foot_phase_run_replay executable'
assert len(replay(201.6569+2,203.0819+2))==2
assert len(replay(63.6918-2,67.5192-2))==2
assert not replay(201.6569+3,203.0819)
assert not replay(201.6569,203.0819+3)
assert not replay(201.6569+2.01,203.0819)
assert not replay(201.6569,203.0819+2.01)
# A valid right-only coordinate cannot be accepted as a left coordinate.
assert not replay(203.8,203.0)
assert len(replay(201.0,203.8))==2
assert not replay(float('nan'),203.0819)
assert not replay(201.6569,float('nan'))
assert not replay(None,203.0819)
assert not replay(201.6569,203.0819,left_deg=float('nan'))
assert not replay(201.6569,203.0819,zero_ready=False)
assert not replay(201.6569,203.0819,frame_valid=False)
assert not replay(201.6569,203.0819,right_valid=False)
print('phase replay to live C++ boundary and missing-data gate PASS')
