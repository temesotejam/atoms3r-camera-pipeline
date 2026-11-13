# Baseline source map

## Production integration baseline

Repository: https://github.com/temesotejam/atoms3r-lateral-energy-control
Commit inspected: `ec5faa49766187cb552523d8c234f6a6bba61224`
Inspection date: 2026-11-13

Relevant files:
- `src/camera_coexistence.cpp`: camera initialization, continuous capture, `cam_take()`, frame timestamp wrapper, capture counters
- `src/camera_coexistence.h`: camera snapshot and API
- `src/foot_observer.cpp`: capture-consumer loop, detector invocation, timestamps, shared snapshot publication, optional preview
- `src/foot_observer.h`: FootFrame and FootSnapshot data model
- `src/white_marker_tracker.cpp`: current white marker detector
- `src/white_marker_tracker.h`: detector API/state
- `src/foot_tracking_config.h`: 320x240 dimensions, lane rows, vertical search radius/step, detector thresholds
- `src/main.cpp`: control-side foot observation polling and `runner.observeFootFrame()`
- `src/run_control_worker.h`: permanent controller task on Core 1 / priority 4

Important current details:
- Camera driver task: Core 0 / priority 3.
- Foot observer: Core 0 / priority 1.
- RunControl: Core 1 / priority 4.
- Camera target: 15 FPS.
- The observer's nominal frame pacing uses 66,667 us.
- Controller checks for a new foot sequence behind a 25 ms gate.
- `frame_us` is documented as a camera-driver timestamp, not a verified physical exposure timestamp.
- The marker detector checks 17 vertical offsets at 4 px spacing and scans 320 horizontal pixels for each offset and each marker lane.
- During active runs, `publishPreview()` returns immediately; preview image copying is already suppressed while running.

## Original vision lineage

### `atoms3r-visual-pose-tracker`

https://github.com/temesotejam/atoms3r-visual-pose-tracker

The earlier general visual tracker included ArUco, template, pyramid and local tracking experiments. Its later lightweight direction used current-frame sparse horizontal scans for white markers.

### `atoms3r-foot-angle-tracker`

https://github.com/temesotejam/atoms3r-foot-angle-tracker

This split out foot-angle sensing. The main runtime uses GC0308 QVGA grayscale and a sparse-line white-marker detector. It explicitly avoids ArUco, previous-frame image buffers, template matching, pyramids and Otsu in its normal path.

## Historical local investigation records

The following files were found in the user's Library and should be treated as historical evidence, not as proof of current behavior:

- `ATOMS3RCAM_04716_timing_operation_analysis.md` — timing analysis of older control runs; notes that camera processing overlap was correlated with control overruns but did not prove camera causation.
- `ATOMS3RCAM_04716_program_cause_review.md` — source-level review of older control overruns and candidate causes.
- `rw_progress_20261110.pdf` — reports earlier image interval median 100.0 ms and detector processing median 12.4 ms in that measurement.

The exact historical "Mode 0 / Mode 1 / Mode 2 / buffer overwrite" experiment source revision and raw metrics still need to be recovered before any related experiment is considered.
