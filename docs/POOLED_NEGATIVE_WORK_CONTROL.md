[1.0.31 adaptive lateral CG energy update](ADAPTIVE_LATERAL_CG.md)

## 1.0.29: separate side energy models and balanced input authority

### What the latest run showed

Run 130765421 used 7.92-7.95 V. Positive input reverted to a 300 mA bridge because the original work model admitted only 7.28-7.40 V. Its 0.12 mJ residual limit was active on 17/22 late positive decisions. Negative 40 ms inputs predicted 9.78 degrees and measured 9.73 degrees on average: reaching 8 degrees was outside the allowed candidate set. Positive peaks below 7.5 degrees reset the negative cap twice. These are actuator/model restrictions, not evidence of IMU mounting error.

Correct IMU mounting does not imply symmetric mechanics. Foot contact geometry, center-of-mass offset, friction and the fixture can change effective inertia, potential and losses between sides. Current observations do not identify those causes separately. The reference potential U_ref(abs(angle)) remains the existing geometric coordinate; the revised side-specific models and residuals predict arrival energy in this coordinate. They do not claim to measure a new asymmetric physical potential or isolate each loss.

### Positive model

Fit 8 runs and 180 completed positive arrivals, including the five randomized 300/350/400 mA runs and runs 107786469, 69890231 and 130765421. Provisional wheel-work labels use J=2.8e-5 kg m2 and the timestamped position estimator. Only pre-input rate, wheel RPM, voltage and candidate predicted current integral enter runtime prediction. Future peak and observed work enter offline labels only.

Let omega=rate*pi/180, n=RPM/500, v=(battery_mV-7500)/1000, Q=predicted current integral in mA s. All energy/work below is mJ:

```
W_zero = 0.36045438418 - 0.05256293271*n - 0.10535792788*v
W_total = W_zero + 0.06189960458*Q*omega
W_on = 0.09858037463*Q*omega
W_post = W_total - W_on
E_free = 0.07353697995 + 0.41423900771*omega^2 + 0.22502117220*W_zero
E_peak = E_free + 0.22502117220*(W_total-W_zero) + b_positive
```

The work coefficient is an effective empirical multiplier, not mechanical efficiency. Search 300,350,400 mA with each current's prepared rise-time table and voltage-dependent goal current, at 0 or 10-100 ms. Prefer a lower current if it already reaches within 0.005 mJ of U_ref(8 degrees). Otherwise choose the candidate minimizing arrival-energy error. Outside the expanded work-state range, the startup bridge also searches all three currents; it no longer falls back to 300 mA only.

Work-state admission: rate 48.3821-72.7405 deg/s, RPM -1 to 400, battery 7.00-8.12 V, fresh speed. Q above the observed 15.33208 mA s and unmeasured combinations are flagged, not presented as validated. The command envelope remains at most 400 mA and 100 ms. Positive residual correction is the last-three mean after two completed matched peaks, now bounded at +/-0.25 mJ. Bridge and work histories remain separate, reset per run, and compare to the raw model without double counting the applied correction.

Across the eight held-out runs, raw RMSE ranges from 0.1594 to 0.8708 degrees, and causal residual correction ranges from 0.1319 to 0.4872 degrees; this is not a universal geometry identification. Latest whole-run held-out prediction RMSE is 0.3961 degrees; using only earlier measured residuals from that held-out run reduces it to 0.2074 degrees. Final training RMSE on the latest run is 0.1636 degrees. These predict unchanged recorded inputs and are not closed-loop target-achievement results. The actual candidate solver selects 350/400 mA on 14 of the latest 26 stored states with zero initial residual (and zero residual-current assumption in that selection test). See `positive_work_8runs_validation.json`, fixture/source manifest, and `test_balanced_energy.cpp`.

### Negative input authority

Keep the 31-run pulse/post-pulse work coefficients and the full-charge coast calibration. Continue minimizing abs(E_peak(input)-U_ref(8 degrees)); duration is not forced to the cap. Start the cap at 25 ms. Two completed matched cap-length inputs with excess above 0.03 mJ can advance it by 5 ms. A previous positive peak must be at least 7.5 degrees through 40 ms, and at least 7.8 degrees to go above 40 ms. The absolute maximum is now 70 ms at 300 mA. Below 7.8 degrees an extended cap is limited/reset to 40 ms; below 7.5 it resets to 25 ms; below 7 degrees braking is skipped. A negative energy undershoot reduces the cap by 5 ms.

The latest measured states require approximately 56-70 ms under this extrapolated model; some reach the new limit. All inputs above 40 ms are explicitly unmeasured extrapolations. Every such candidate is logged as cross-condition/extrapolated; no measured data above 40 ms are claimed. Finite-energy and fresh-state checks, previous-side protection and the existing emergency stop remain. Stronger input is contingent on completed peaks, not enabled at startup. Real response and simultaneous left/right target feasibility still require the next run.

### Measurement

Flash 1.0.29-balanced-energy-inputs. Keep the same fixed-leg fixture and saved upright reference. Use the existing 30-second work-control button at 8 degrees. The current and duration are selected automatically. Log branch 6 means expanded positive work, 5 expanded positive bridge, and 4 negative work; raw/corrected energy, bias/history size and candidate bounds remain recorded. Other presets and the IMU/reference implementation are unchanged.

---

## 1.0.28: full-charge coast calibration and negative control

Run 69890231 recorded 8.09-8.11 V. Its 34 negative decisions were skipped by the previous 8.00 V ceiling. The stable 10-30 s interval supplies 22 no-command negative peaks. Only their pre-crossing rate, wheel speed, previous positive peak and voltage are runtime inputs; the subsequent peak is an offline label.

Keep the original pulse and post-pulse work equations and the original 31-run arrival-energy coefficients. Modify the free arrival energy, in mJ:

`E_free_new = E_free_old - 0.05972251687 * clamp((battery_mV - 8000)/100, 0, 1)`

`E_peak(input) = E_free_new + 0.44113787836 * Delta_W(input) + b_negative`

Choose current duration by minimizing the difference to the target potential energy. The extra term is an empirical arrival-energy calibration, not an independently identified physical loss. The input-effect multiplier is unchanged. At or below 8.00 V the calibration is zero, preserving the original 31-run predictions.

First fit on 10-20 s, then test on unused 20-30 s: angle RMSE decreases from 0.2991 to 0.1587 degrees (11 test events). The final coefficient includes all 22 events; its training RMSE is 0.2423 degrees. Reproduce with `python tools/fit_full_charge_negative.py`; see `full_charge_negative_validation.json`, the CSV fixture and hashed source manifest.

Voltage admission is now 6.21-8.12 V; the upper 10 mV is one telemetry step above the whole-run observed maximum. Other state limits, fresh-speed requirement and opposite-peak protection remain. Every nonzero negative input above 8.00 V is marked `work_prediction_cross_condition`, including 10-25 ms. Even a no-input prediction at the unmeasured 8.12 V margin is flagged. Logs separate the calibrated baseline from predicted pulse and post-pulse work.

High-voltage braking has not been measured in this run. The 25/30/35/40 ms cap continues to advance only after completed matched peaks and a healthy previous positive peak. A target outside the permitted energy range remains saturated; duration never exceeds 40 ms. Prediction improvement is not proof of reaching 8 degrees. The 30-second preset and separate positive energy feedback remain.

---

## 1.0.27: positive energy feedback and 30-second run

The work-control preset now records 30 seconds. Other normal and comparison presets retain their existing durations. Countdown, run termination and RWLOG metadata use the same duration.

For each positive prediction branch (historical angle-energy bridge or work model), store the last three completed matched residuals `r = U(actual positive peak) - E_raw_prediction` in mJ. After two observations use `b = clamp(mean(r), -0.12, +0.12) mJ`. Both histories reset each run. Failed writes and unmatched peaks are excluded.

For the work model minimize `abs(E_raw(input) + b - U(target))` over existing supported candidates. For the bridge fallback invert `E_raw(input) = U(target) - b`, retaining the current-response table and input limits. A negative residual requests more energy. The prior target-error integral is disabled in work-control mode to avoid adapting twice. Current integral is not treated as energy.

Store residuals against the raw prediction: subtract the applied bias when the work prediction already includes it. Bridge `predicted_energy_j` remains raw; `predicted_observed_energy_j` includes the correction. `prediction_bias_j` and `work_bias_samples` record both positive branches. Negative prediction and the 25/30/35/40 ms completed-peak gate are unchanged.

Causal replay of run 107786469 uses only earlier completed peaks. For six positive decisions after 10 seconds, energy prediction RMSE decreases from 0.05999 to 0.01668 mJ. See `positive_energy_feedback_replay.json` and the runtime math test. These are unchanged recorded inputs, not a closed-loop simulation or proof of reaching 8 degrees. Real response to changed inputs remains to be measured.

---

# 入力後の仕事を含む負側制御（1.0.26）

## 実機での手順

`1.0.26-pooled-negative-work`を書き込む。脚の前後移動を同じように固定し、保存した直立基準を確認して「仕事に基づく制御で15秒測定を開始」を押す。目標8°、負側300 mA。最初は1回測定してRWLOGを保存する。

## 今回のデータを反映した内容

77054774では負側25 msを9回実行し、後半5件の到達角は平均10.4646°だった。旧式の予測は平均10.1850°で、0.2796°小さく、RMSEは0.3187°だった。通電中の仕事は平均−0.3077 mJ、入力後からピークまでの仕事は+0.2195 mJ。正味の減速仕事を予測するには両方が必要である。

既存19回の0～25 ms測定412件、30～40 ms等を含む2群10回の248件、今回の25 ms後半5件、直前の無入力ログ42884626の後半5件を合わせ、31回670件で予測式を作った。今回の5件を丸ごと学習から外す検証のRMSEは0.1301°、31回のうち1回ずつを外す検証は全670件で0.3079°。最後に今回の5件も学習に含めた。どちらの検証も、入力後の実測仕事や次ピークを予測の入力に使わない。

## エネルギーと仕事の式

単位はmJ。`ω`はZEROクロス時の角速度[rad/s]、`r=(角速度[°/s]−70)/10`、`n=(ホイールrpm−200)/200`、`v=(電圧mV−7300)/500`、`p=直前の正側ピーク[°]−8`、`t=候補通電時間ms/25`。

```text
W_on  = −0.181207841t −0.110979567t² −0.013450859tr
        +0.029755850tn −0.002338393tv
W_off = 0.268238323 +0.029919620r +0.116990081n +0.003480068v
        −0.112391697t +0.043908940t² +0.007892802tr −0.063873554tn
ΔW    = W_on(t) + W_off(t) − W_off(0)
E_free = 0.290175920 +0.680478920ω² −0.014368250n
         +0.005002621p +0.012113272v
E_peak = E_free +0.441137878ΔW + b
```

`E_free`は無入力なら次の頂点に残るエネルギーの予測で、自然なホイールの減速の仕事も含む。`ΔW`は入力で変わる仕事で、無入力時の仕事を引いて二重計上を避ける。実測仕事と到達エネルギーの間の残差を`R=(0.441137878−1)ΔW+b`と置けば、`E_peak=E_free+ΔW+R`という収支になる。係数は暫定慣性、接触損失、計測誤差等を含む実験的な補正であり、独立に求めた機械効率ではない。

`b`は、完了した負側ピークの実測位置エネルギーと、補正前の予測エネルギーの差の直近3件平均。正側が7.5°以上保たれた有効な対応付きピークを使い、2件集まってから適用し、±0.12 mJに制限する。前回すでに加えた`b`を予測から除いて残差を計算するため、補正を二重に足さない。測定ごとに初期化する。

候補は0 ms、または10 msから現在の上限まで1 ms刻み。各候補で上記の式を計算し、`|U(8°)−E_peak|`が最も小さい入力を選ぶ。`U(8°)`は既存足裏円弧・重心モデルで約0.7334 mJ。入力後の仕事も含んで候補を比較するため、通電中だけで必要な仕事を満たしたとして計算を終えない。

## 強い入力への進め方

各測定の初期上限は25 ms。上限を使った入力の後でも、実測した負側のエネルギーが目標を0.03 mJ超えており、直前の正側ピークが7.5°以上だったことを2回続けて確認すると、候補の上限を25→30→35→40 msと1段階ずつ広げる。上限は選べる候補の範囲であり、上限の時間を必ず出力する指令ではない。

直前の正側が7.5°未満なら、その回の上限は25 ms。弱い正側に続く負側ピークが確認された場合も内部上限を25 msへ戻す。7°未満では従来どおり負側入力を見送る。実測で目標より0.03 mJ以上減らしすぎた場合は、上限を1段階下げる。40 msを超える指令は出さない。

## 適用範囲と未確認の部分

各変数の記録範囲は角速度57.2522～76.2778°/s、回転数4.28～575.79 rpm、電圧6.21～8.00 V、直前の正側ピーク5.0288～8.56178°。全組合せが測定済みという意味ではない。特に今回の低電圧で25 msを超える入力は、他条件の30～40 ms測定から推定する新しい組合せである。

25 msを超え、従来の30～40 ms測定の状態範囲に入らない候補は`work_prediction_cross_condition=true`とし、`energy_model_extrapolated`にも反映する。段階的な上限と正側の保護条件を使って、実機で確認する。オフライン検証は同じ条件の強い入力で8°を実現した証明ではない。今回の入力前状態を固定して40 msを予測しても、約9.6～9.9°である。閉ループでの変化は次の実機ログで確認する。

データの範囲外や古いホイール速度では負側の入力を見送る。正側は既存の制御を使う。計算はZEROクロス時の少数候補の評価と、ピーク確定時の3件平均だけで、マイコンで再学習しない。

## 記録と再現

`work_control_branch=4`が新しい負側の式。`prediction_bias_j`が適用した`b`、`work_bias_samples`が補正の観測数、`work_brake_cap_ms`がその回の上限。通電中・入力後・合計・無入力との差の予測仕事を別々に記録する。ピークの`model_residual_j`は適用後の予測との差で、補正用に内部で使う元の式との差とは区別する。

再現は`python tools/fit_pooled_negative_work.py`（numpy使用）。正規化した670行は`tools/fixtures/pooled_negative_31runs.csv`、検証と係数は`pooled_negative_31runs_validation.json`、元ログとSHA-256は`pooled_negative_31runs_sources.json`。元のRWLOGは変更しない。
