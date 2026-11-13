# Investigation plan: end-to-end camera latency

## 1. Objective

Reduce the age of a foot-marker observation at the instant the control task consumes it. Do not optimize a proxy such as detector execution time alone.

The production path currently uses:
- GC0308, QVGA 320x240 grayscale
- continuous camera receiver and XCLK during alignment and START/MEASURE/END
- camera driver task pinned to Core 0, priority 3
- FootObserver pinned to Core 0, priority 1
- RunControl pinned to Core 1, priority 4
- 15 FPS target (66.7 ms nominal frame period)
- a latest-frame snapshot consumed by the controller
- marker detector that scans 17 vertical templates (dy=-32..+32, step 4); each template evaluates the 320-pixel horizontal profile for each lane

Source of truth: `temesotejam/atoms3r-lateral-energy-control`, baseline commit `ec5faa49766187cb552523d8c234f6a6bba61224` (verified 2026-11-13).

## 2. Historical work that must not be blindly repeated

The earlier camera coexistence investigation included one-shot capture, receiver/XCLK gating, continuous capture, camera task placement/priority inspection, frame queue handling, and preview-copy suppression. The current integrated runtime already keeps receiver/XCLK active through active observation to avoid the 20 ms warm-up on every frame.

A separate historical latency experiment reported approximately 72.7 ms in its capture-to-consumer path, with detector work around 9 ms. Those numbers belong to that earlier test configuration; they are NOT a verified measurement of the current integrated build.

Previous modes documented by the project:
- Mode 0 baseline
- Mode 1 wait-shortening
- Mode 2 early framebuffer return
- buffer-overwrite mitigation experiment

Do not repeat these as-is. Recover their exact source revisions and raw measurement definitions before proposing any related change. An earlier run also showed that the frame timestamp is a driver timestamp, not a verified physical exposure timestamp.

## 3. First task: instrumentation only

Add low-overhead timestamps/counters to distinguish these stages:

| Symbol | Event |
|---|---|
| T0 | driver frame timestamp (`fb->timestamp`; semantics explicitly marked unverified exposure) |
| T1 | return from `cam_take()` / `acquireContinuous()` |
| T2 | FootObserver starts marker processing |
| T3 | both marker detectors finish |
| T4 | latest FootFrame is published to shared snapshot |
| T5 | control task reads a new sequence and calls `runner.observeFootFrame()` |

Report:
- T1-T0: driver timestamp to frame delivery (diagnostic only; timestamp semantics caveat)
- T3-T2: detector processing time
- T4-T3: post-processing/publication cost
- T5-T4: consumer wait
- T5-T0: timestamp-to-control-consumption age
- frame interval, dropped/invalid frames, sequence gaps
- median, p95, p99, max; count and sample window

Use fixed-size counters/rings, no per-frame String/JSON formatting, no serial printing during active runs, and no dynamic allocation in the hot path. Export diagnostics after the run.

## 4. Hypotheses to test (not assumptions)

1. The 25 ms polling gate in `main.cpp` may add up to one polling interval before the controller consumes a newly published foot frame. Verify actual task cadence and whether the controller can safely consume a newer sequence every 5 ms.
2. The 15 FPS capture cadence may dominate observation age. Measure actual frame interval and queue behavior before changing it.
3. `FootObserver::loop()` has a nominal 66,667 us pacing calculation. Determine whether this pacing is redundant with camera frame production or causes an additional wait. Do not remove it until actual frame cadence and CPU behavior are measured.
4. Detector work may be reducible. Current detector searches all 17 vertical offsets and all 320 horizontal pixels per lane. Test a bounded horizontal ROI only against recorded frames and preserve a full-width reacquisition fallback.
5. PSRAM bandwidth/cache contention may affect cross-core latency. Treat as a hypothesis; do not infer it solely from correlation.

## 5. Experiment sequence

A. Reproduce current baseline without source behavior changes.
B. Add instrumentation only; compare timing distributions and verify diagnostics do not perturb control deadlines.
C. Change only controller consumption cadence (if T5-T4 is material); run host tests and compare the 2.5 ms deadline counters.
D. Change only detector ROI (if T3-T2 is material); replay the same frames and require identical or explicitly bounded marker outputs, validity, and reacquisition behavior.
E. Consider capture scheduling/driver changes only if T1-T0 or frame interval is the dominant measured term and historical experiments have been fully reconciled.

Each experiment gets its own branch/commit and a written before/after report.

## 6. Acceptance gates

- No changes to IMU sampling, MEKF, wheel/roller output, energy controller, pulse timing, or motor limits.
- No relaxation of the 2.5 ms control deadline.
- No increase in camera invalid-frame rate or left/right marker loss.
- No loss of calibration range behavior or marker reacquisition.
- Existing runtime tests and ESP32-S3 build pass.
- Hardware improvement is claimed only after the same device, lighting, movement, and test procedure are repeated and raw logs are compared.

## 7. Current state

Repository initialized with this plan. No firmware source has been modified or flashed yet.
