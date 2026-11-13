# AtomS3R Camera Pipeline

低遅延カメラパイプラインの独立検証リポジトリです。

## Goal

AtomS3R-CAM (ESP32-S3 / GC0308) のカメラ撮影から、左右の白色マーカー認識結果を制御側が利用できるまでの遅延を、既存の歩行制御ファームウェアを壊さずに調査・改善します。

対象経路:

1. GC0308 frame production / DMA
2. camera driver frame queue and `cam_take()`
3. FootObserver frame delivery
4. white-marker detection
5. latest-result publication
6. RunControl consumption

## Safety and compatibility constraints

- The production repository remains the source of truth until a change is validated.
- Do not repeat previously tested one-shot / wait-shortening / early-buffer-return experiments without new evidence.
- Preserve camera continuous-capture ownership and the existing Core 0 / Core 1 separation unless measurements justify a controlled experiment.
- Do not weaken the 2.5 ms control deadline or alter IMU, motor, estimator, or walking-control behavior.
- Preserve marker validity, calibration behavior, frame timestamps, and RWLOG compatibility.
- Never claim hardware latency improvement from host tests or builds alone.

## Baseline source

The initial investigation baseline is the production repository:
https://github.com/temesotejam/atoms3r-lateral-energy-control

Relevant original vision lineage:
- https://github.com/temesotejam/atoms3r-visual-pose-tracker
- https://github.com/temesotejam/atoms3r-foot-angle-tracker

See [docs/INVESTIGATION_PLAN.md](docs/INVESTIGATION_PLAN.md) for the measurement plan and known historical experiments.
