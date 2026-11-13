# Zero-independent foot phase observation

Run 42865708 used firmware 1.0.60, passed its RWLOG CRC check and finished the
30-second test. The phase controller remained in Waiting and never selected a
modified input. Both foot markers were detected on most frames but no foot
angle zero was established. The stored zero_reason alone does not establish
whether the robot actually moved at a particular time.

In the 5–20 s portion, 18 negative-positive-negative cycles gave approximate
mean signed camera-relative marker movements of 19.46 px on the left and
0.96 px on the right. This is not ground-relative distance or heading. In
run 46657774 the corresponding values were 18.64 px and 0.62 px; neither run
actually used the phase correction, and changing conditions prevent a causal
comparison.

For one foot, let g(x) be its fixed piecewise-linear calibration function.
The existing zero-referenced angle is a(x,z)=g(x)-g(z). The phase observation
is the difference between the positive peak and the mean of the two adjacent
negative peaks, or its opposite for the other foot. The common g(z) term
cancels, including when g is nonlinear. Version 1.0.61 therefore computes this
phase-only quantity from detected marker x values against a fixed calibration
reference. It does not change the ordinary zero-referenced angle, its valid
flag, other controllers or the range diagnostics. The original per-foot
calibration and the provisional 2 px endpoint extension still apply.

Using run 42865708, a counterfactual recorded-input replay allowing valid raw
marker positions but not changing any recorded state reached a 400 mA,
80-to-75 ms positive-side candidate under six notification/poll-delay cases.
The candidate was at 7.656 or 10.694 s depending on camera polling delay.
Replays stop at the first changed candidate; those times are not observed
motor output and cannot predict subsequent motion, wheel current, foot step,
heading or stability. The actual capture-to-observer latency is not known.

The 1.0.60 maximum 5 ms experimental correction and its gates are retained.
An unsupported negative-side decision, loss of observations, changed
baseline/target, and other existing conditions can still end the trial for
the run. In particular, the old recorded run had an unsupported negative
decision after the earlier counterfactual candidate; the true post-intervention
decision may differ. The 0.1-degree two-cycle criterion is provisional and
does not establish causal improvement. Real closed-loop outcome remains
unverified.
