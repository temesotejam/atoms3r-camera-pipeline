# Negative-side one-peak energy inverse

This experimental mode is selectable as `bridge_inverse` on the device page.
It is limited to fixed fore-aft legs, an 8° target, a 300 mA current command,
and a saved upright reference. The 14-run fit used an absolute MEKF pitch of
1.932539° at that pose, but the controller uses angles relative to the saved
reference. Re-saving the same physical upright pose may yield a different
absolute MEKF pitch, so the absolute value is not a model input or gate.
The positive half-cycle retains the previous bridge controller.

At the negative-side zero crossing, the controller reads body rate, the latest
wheel speed, the previous positive peak, and battery voltage. For each measured
command duration (0, 10, 15, 20, 25 ms), the model predicts wheel work during
the command and after it, then computes the next-peak position energy:

`E_peak(T) = E_free(state) + 0.4229016404 [W_on(T) + W_after(T) - W_after(0)] + residual`.

The firmware chooses the duration whose predicted energy is closest to the
position energy at 8°. The `residual` is the mean of up to three completed
negative-peak errors against the original fit; only pulses within the measured
0–25 ms range update it. This is a within-run correction, not a measurement of
electrical energy or a direct measurement of shaft work.

The fitted state envelope is 57.2522–76.2778 °/s, 4.28–550.04 rpm,
5.0288–8.52741° previous peak, and 7.22–8.00 V. The wheel-speed sample must
be at most 50 ms old. If any requirement fails, the negative-side command is
skipped and the event records the invalid decision.

Measured durations do not always reach 8°. When 25 ms is still predicted to
leave too much energy, a completed negative peak above target permits the next
negative pulse to extend by 1 ms, at most 30 ms. A preceding positive peak
below 7° or a negative peak below 5° resets the extension. The 25–30 ms
prediction uses the 20–25 ms slope and is flagged as extrapolated. The first
closed-loop run must be reviewed before treating this range as validated.
Firmware 1.0.19 corrects the feedback sign: `peak_error_deg` is target minus
measured peak, so a negative value below -0.5° triggers an extra millisecond
of braking. The first 1.0.18 run exposed that the earlier condition was
reversed and could not extend a 25 ms brake when the peak was too large.

The log stores the predicted next-peak energy, measured peak energy, residual,
input width, extrapolation flag, wheel speed, and voltage. A missing supported
state means no negative pulse rather than an inferred response outside the fit.
