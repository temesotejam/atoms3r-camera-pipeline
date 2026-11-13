# Foot phase control: normal coast and observation coverage

Run 147354797 used firmware 1.0.61 and passed RWLOG CRC validation. For the
first time, the new foot-phase controller selected two positive-side 400 mA
commands at 75 ms rather than the ordinary 80 ms at 6.210 and 6.962 s.
At 7.311 s the negative-side selector returned VALID_NO_OUTPUT; the previous
controller treated the ordinary coast as a loss of output, released the trial,
returned to 80 ms at 7.768 s and remained locked for the rest of the run.
The recorded command is not by itself proof of actual motor current or motion.

The 5–20 s recorded phase-matched marker movement was approximately 15.70 px
left and 1.46 px right, compared with 19.46 px and 0.96 px in run 42865708.
These are signed camera-relative movement metrics, not ground steps or heading.
The measurements are from different runs with different conditions; two
modified commands and the concurrent negative-side change do not establish
the effect of the intervention.

In run 147, of 297 running frames where both markers were detected, 245 were
within 2 px of both calibration supports and 296 were within 3 px; one was
farther than 3 px. Recorded-input reconstruction produced 16 complete phase
cycles under the former boundary and 37 under the latter; the observer's
additional validity checks accepted 12 and 35 respectively. These numbers are
from the unmodified recorded trajectory and do not predict movement after a
different motor command. The wider range uses limited endpoint extrapolation,
not independently validated calibration; ordinary diagnostic range flags are
unchanged.

Version 1.0.62 distinguishes a valid, zero-width model-selected negative coast
from unsupported, failed or inconsistent decisions. A valid coast alone does
not immediately end the trial, while an unknown or failed decision does. A
completed cycle with a different negative mode, current, direction, duration
or branch than its two matching baseline cycles is not used to declare
improvement. Two consecutive non-comparable cycles, or a 5 s trial timeout,
end an unresolved trial and return to the normal selector. Existing peak,
freshness, target and safety conditions retain priority. The positive-side
trial remains limited to one 5 ms reduction; no additional stage is enabled.
The existing two-cycle, 0.1-degree response condition is provisional and
cannot establish a causal real-world improvement. Hardware response and
heading need further measurement.
