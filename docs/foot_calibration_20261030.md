# Fixed-leg five-pose foot-angle calibration

The legs stayed at their upright positions while the camera/IMU-bearing body
rotated about the leg joint. The resulting camera-to-marker geometry represents
the leg angle relative to the body. Reference angle is baseline MEKF roll minus
pose MEKF roll, not the lateral sway angle used by the RW controller.

| Reference angle (deg) | Right marker X (px) | Left marker X (px) |
| ---: | ---: | ---: |
| 0 | 203.0819 | 201.6569 |
| 3.688465 | 182.4695 | 173.8169 |
| 8.371281 | 152.5749 | 144.3562 |
| 14.109958 | 111.0505 | 102.7030 |
| 20.678285 | 67.8096 | 62.8279 |

Each foot has its own piecewise-linear position-to-angle mapping. The current
per-boot neutral remains zero by subtracting the mapped angle at that foot's
measured neutral X. Values between knots are interpolated. Outside either end,
the nearest segment is extrapolated: no clipping, zero replacement, or rejection
solely because the observation lies outside calibration support.

`right_in_range` / `left_in_range` still identify the measured X envelope.
`right_valid` / `left_valid` remain independent of those flags. Valid extrapolated
angles and marker positions are retained in live diagnostics, captures and RWLOG.
Pose comparison retains the `outside_range` warning but that warning alone no
longer makes the pose comparison unusable. Invalid/ambiguous detections and
unlocked zero are still distinguished from usable angle measurements.

These are five hand-held snapshot pairs, not averaged static holds or independent
validation. Small motion, frame-delivery/IMU timing, and up to about 2.57 degrees
of cross-axis change can affect calibration. The exact fit at the five supplied
knots must not be reported as independent angular accuracy. Endpoint extrapolation
is an estimate, including small negative relative angles not covered by these
forward-angle poses. Mounting geometry and rotation center must stay consistent.

Source values and capture identities are retained in
`tools/fixtures/foot_calibration_20261030.json`. Historical fixtures keep their
original calibration semantics. This change affects vision observation only;
the RW amplitude controller is unchanged.


## 2026-11-01 repeat endpoint

Two additional supplied vision diagnostics (1224, 1519) were incorporated provisionally, assuming the same fixed-foot, rotating body/camera protocol. Their body-roll change is 20.574717 degrees. Their observed marker displacement is normalized to the original upright marker positions, then the resulting endpoint and reference angle are averaged with the original endpoint. The three intermediate points and the per-boot zero procedure are retained. The new evidence and formula are recorded in `tools/fixtures/foot_calibration_20261101.json`. The repeat captures are hand-supported and cannot independently establish angular accuracy, marker-centering accuracy, or whether the feet moved in the real world.
