# Foot-phase observation with bounded endpoint extrapolation

Run 46657774 used firmware 1.0.59, completed the 30-second test and passed the
RWLOG CRC check. The logged phase controller remained in Waiting and did not
select any modified candidate. Its right-foot boot zero was 203.92249 px, while
the right calibration's upper endpoint is 203.0819 px. Both markers were
detected but simultaneous strict in-range observations were uncommon. In the
5–20 s window, peak-matched camera-relative movement was approximately 18.6 px
on the left and 0.6 px on the right. This comparison uses body-camera-relative
motion, not ground-relative step length or directly observed heading.

Version 1.0.60 allows the phase observer alone to use a marker position at most
2 px outside either calibrated endpoint. This is limited endpoint
extrapolation, not a new calibration. The usual diagnostic range flags do not
change. Both markers, the frame and the boot zero must be valid, coordinates
and angles must be finite, and positions outside that margin are rejected. The
precision of extrapolated angles has not been independently established.

An experimental positive-side intervention is limited to one 5 ms reduction
from the ordinary 400 mA branch-5 baseline. It can remain active if the
existing short-term response guards permit, but does not automatically
increase to 10 or 15 ms. A change in baseline candidate ends the trial;
existing tighter caps and stop conditions take precedence during recovery.
The previous two-cycle, 0.1-degree rule remains a provisional online condition,
not proof that the motor caused any change. Negative-side normal no-output
still interrupts this trial rather than being treated as a measured output.

Recorded-input replay of run 46657774, under six combinations of added peak
notification delay (0, 25 or 75 ms) and camera polling delay (0 or 25 ms),
reaches the first 80-to-75 ms candidate at 7.682 s. The replay stops at that
first counterfactual change; it cannot predict the actual subsequent movement,
motor current, heading, or stability. Exposure timing and real closed-loop
behavior remain unverified. A subsequent real run is required to determine
whether the candidate is actually sent and whether the imbalance changes.
