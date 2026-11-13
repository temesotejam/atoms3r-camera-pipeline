# Target 12: state-dependent work and contextual input limits (1.0.40)

This is a fixed-leg experimental controller for the absolute-angle +/-12 degree
package. The 8 and 10 degree packages retain their existing selectors.

## Energy and work

The physical accounting is

`E_next = E_now + W_on + W_post - L`.

`W_on` is reaction work during the command and `W_post` is work from command
release to the next body peak. Wheel spin kinetic energy is not body work.
The offline labels integrate `-J * body_rate * d(wheel_absolute_rate)` using
position-derived wheel speed, with provisional `J = 2.8e-5 kg m^2`.
They inherit the inertia, axis sign, timing and planar-motion assumptions.

The implemented, loss-inclusive arrival approximation is

`E_pred(s,q) = E_free(s) + eta_mode * [W_pred(s,q)-W_pred(s,0)] + b_context`.

`s` contains body rate squared, signed pre-input wheel rpm, battery voltage,
and the previous opposite peak. `q` is predicted signed current integral in
the body excitation direction. It is calculated from candidate current,
duration, the voltage-dependent current goal, rise time and residual current.
`q` is not an energy measurement.

The work model predicts total and on-command work separately. Post-command
work is their difference, so `W_on + W_post = W_total` by construction.
The coast intercept is shared by assist and brake, and the dose terms vanish
at q=0. No action-dependent constant jumps at that boundary.
The signed dose slopes depend on rpm and voltage. The retained-work factors
are empirical arrival coefficients, not mechanical efficiencies. Free arrival
includes ordinary loss and coast work; losses have not been individually
identified. This does not claim a first-principles identification of all losses.

The candidate solver minimizes absolute target-energy error. Target energy
still uses the signed CG-corrected potential; matching actual peaks update
causal context-weighted residuals expressed using the current equivalent CG.
The CG value is a model-relative correction, not a unique physical survey.

## Calibration and transition

Three recent 30-second logs supply 78 positive and 77 negative complete
responses after 5 seconds, with at least 95 percent observed-work coverage.
Ridge fits use only pre-input state and assigned dose as predictors. Observed
work and peaks are training labels only. No measured future work enters the
runtime inverse.

Core voltage coverage is 7380--7560 mV positive and 7380--7550 mV negative.
Positive rates are 66.266--89.0184 deg/s and rpm -18.01--230.94; negative
rates are 70.0554--89.6155 deg/s and rpm 0--339.47. These marginal ranges
do not establish coverage of every joint state or hypothetical command.
Voltage also changes between runs, so its fitted term is not an isolated
causal identification of battery effects. Farther voltages use the smooth
historical-model transition instead of unrestricted voltage extrapolation.
The measured negative durations stop at 45 ms. Longer assist commands remain
staged extrapolations. All new predictions retain the cross-condition flag.

State-only smooth weights blend into the previous predictor outside the
core. A single state's blend weight is identical for every candidate dose,
so model-transition offsets do not create an artificial dose optimum.
Sparse online residuals shrink toward zero and nearby rpm/voltage observations
receive more weight. Calibration residual learning requires weight >=0.9;
early fallback-only startup outcomes do not contaminate the new model.

## Input limits

Negative assistance is armed by two confirmed low peaks. Six cap histories
cover rpm anchors 0, 150, 330 and voltage anchors 7380, 7550 mV. Smooth
weights interpolate the histories; there is no hard rpm-bin switch.
Measured near-stopped inputs seed 35/40 ms, middle-speed 35 ms, fast-rotor
25 ms. A selected pulse can be shorter or zero: these are exploration ceilings.

Confirmed under-target, cap-limited assist responses add weighted evidence.
At 1.5 units of evidence a cell expands 5 ms. Evidence survives coast periods.
An over-target assist reduces only nearby context caps, weighted by similarity.
Thus a high-rpm excess does not shrink the stopped-rotor cap. Under/over
thresholds use signed potential-energy differences at 11.7/12.3 degrees.

Hard limits remain assist 300 mA / 100 ms / 25 mA s, positive 300/350/400 mA
and 100 ms, negative brake 70 ms, and the 16-degree stop. Fresh sensor and
voltage gates remain active. Large positive underprediction retains a nearby
context energy guard; peaks above 15 degrees or excessive residuals suppress
positive excitation until an acceptable observation is recorded.

## Verification and use

Synthetic host tests check work/energy identities, signed braking, zero-dose
continuity, smooth state transitions, contextual cap separation and hard gates.
Private run-separated fitting and chronological replay distinguish prediction
accuracy from tracking accuracy. Original-action replay is not a simulation of
the modified closed loop, and does not establish actual target tracking.

Flash 1.0.40, keep the same fixed-leg arrangement, and use the existing
12-degree / 30-second button. Absolute zero is unchanged; no upright capture
is required. Save each RWLOG before clearing the run.
