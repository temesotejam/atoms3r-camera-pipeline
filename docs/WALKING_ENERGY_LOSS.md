# 歩行時の実効エネルギー低下（1.0.46）

元ログは `energy_control_autonomous_run_1_50881356.rwlog`（1.0.41、目標8°）。
5〜30秒の無入力の負側半周期32回を使用。抽出した生値と元ファイルSHA256は
`tools/fixtures/walking_loss_50881356.json` に保存。

従来は負側の角速度が既存モデルの範囲を下回り、32回とも
`WORK_OUTSIDE_MEASURED_SUPPORT` で入力を見送っていた。
今回の低振幅をすべて追加摩擦で説明することはできない。

## エネルギー収支

既存の機体形状の位置エネルギー座標 U を使用する。
無入力半周期の実効低下量は `L = U(前ピーク) - U(次ピーク)`。
32回の平均は0.140129 mJ、範囲0.024220〜0.281165 mJ。
重心・姿勢の偏り、脚の運動、モデル誤差、RWとの交換も含む量であり、
衝突や摩擦単独の散逸エネルギーではない。

前進に伴う前後傾斜では、RWの角運動量と機体の角速度の外積による
ジャイロ反力が旋回など他軸の運動に結びつく可能性がある。
理想的なジャイロ項そのものは散逸ではなく、他軸への結合として扱う。
今回の単一ログからジャイロ項や左右の接地損失を個別には同定できない。
本変更はジャイロ補償やRW平均回転数の制御を追加するものではない。

現在のZEROクロスの角速度 r [deg/s] から、次の無入力ピークを予測する。

`E_coast [mJ] = 0.09046158 + 0.00018973 r²`

学習した補正 b を使い、`L_walk = Uprev - (E_coast+b)`、
`E_next = Uprev - L_walk + ΔE_input` として選択する。
予測された低下量は符号付きで扱う。負の場合は、足の動き、重心の偏り、RWとのエネルギー交換、モデル誤差を含む見かけの増加を示し、散逸が負という意味ではない。
同じ32点への当てはめRMSEは0.018482 mJ。独立した検証ではない。

## 適用条件

目標8°、負側、絶対角基準、重心補正0 mm、ホイール速度の鮮度30 ms以内。
全条件を同時に満たす場合のみ新分岐10を使う。

|量|範囲|
|---|---|
|ZEROクロス角速度|18.45〜37.30 deg/s|
|ホイール回転数|362〜603.21 rpm|
|電池電圧|7000〜8120 mV|
|直前の正側ピーク|4.3006〜6.90°|

追加ログ39418856の無入力31回（7690〜7700 mV）では、元の式の予測誤差RMSEは0.0200 mJ。追加ログ47647215の無入力31回では、既存式の平均予測誤差は−0.0201 mJ、RMSEは0.0298 mJ。角速度・直前ピーク・ホイール回転数の範囲は3試行の実測を含む。7000〜8120 mVは従来の電流モデルの範囲を転用しており、歩行中に全域を検証したものではない。電圧に応じた電流予測は既存の計算を使う。
電圧や振幅がこの範囲から離れると追加モデルは使わず、従来の条件判定に戻る。
広い電圧範囲や目標10°・12°には今回のデータを転用しない。

## 補償入力と記録

最初の2回は無入力。新分岐の予測と対応する完了ピークが一致した場合だけ学習し、
補正は直近3回の平均、最大±0.10 mJ。残差が±0.15 mJを超える値は学習しない。
2回の受理後に、無入力と300 mA・10〜25 msの励振から目標エネルギーに近いものを選ぶ。
同じ上限の入力後に目標不足のピークが2回続き、モデル残差が受理された場合、上限を5 msずつ最大40 msまで広げる。目標を超えた場合は上限を5 ms下げ、次の入力を抑える。25 msを超える歩行時の効果は未検証。

`ΔE_input [mJ] = 0.22502117220 × 0.06189960458 × Q[mA s] × r[rad/s]`

これは既存の負側補助と同じ、正側から転用した暫定増分ゲインである。
今回のログには負側の入力がなく、実測で同定した負側ゲインではない。
従来の左右別補正や重心推定とは同時に学習せず、二重補正を防ぐ。

ログには分岐10、専用モデル名、`predicted_loss_j`、予測ピーク、実測ピーク、
入力と飽和を保存する。目標に届かなくても予測を目標値に書き換えない。
足角度の補間・外挿と範囲外保存は1.0.41を継承。

この変更で歩行中の±8°を達成できるかは実機未検証。
正側は元ログで既に400 mA・100 msの上限に達しており、この追加だけで
正側の入力不足まで解消するとはいえない。

追加ログ42306015には無入力18回と25 ms入力13回があり、無入力の既存式RMSEは0.0416 mJ。異なる条件であるため両群の差は入力効果の独立した同定ではない。足の左右差による目標調整を引き継ぐ。


### 2026-11-01: run 55836070

Eight unpowered negative arrivals at 323.90–360.33 rpm show coast prediction RMSE 0.0249 mJ; this extends the lower rpm bound to 323.9. These are observations under walking, not proof of the loss mechanism. The run also reached 40 ms fifteen times while the negative-side peaks remained below target. The cap may increase by 5 ms after each two accepted below-target capped arrivals, up to 60 ms; 45–60 ms remains unvalidated on the robot, and an overshoot stops the assist and reduces the cap. The motor power, output, freshness and other domain guards remain in force.


### 2026-11-01: runs 31767392, 27579224 and 33892778

Thirteen previously unsupported unpowered arrivals add measured low-speed coast evidence down to 13.8 deg/s and 274.92 rpm and up to a previous peak of 7.64 degrees. The model's held-out coast error is checked against these records. The domain is extended to rate 13.75–37.80 deg/s, wheel speed 274.90–603.21 rpm and previous peak 4.3006–7.65 degrees. Powered response in the new low-speed domain remains unvalidated. Wheel speed bias correction releases the positive-side cap when the most recent negative-side peak is less than 4 degrees, giving priority to sustaining oscillation; both mechanisms remain provisional.


### 2026-11-01: runs 50202353 and 275876204

These closed-loop trials repeatedly saturated negative assist at 60 ms. Wheel speed at the next negative peak stayed positive in many samples while the peak remained below target. The experimental controller can extend its 60 ms cap by 5 ms to at most 75 ms after three matched capped under-target arrivals with fresh wheel speed at least +30 rpm at the peak. Missing, stale, or lower arrival speed restores the 60 ms cap. Overshoot and existing geometry, energy, charge, motor-current and stop guards remain in force. Arrival speed is a delayed diagnostic, not a guarantee of no zero crossing during a pulse. Neither the sign nor magnitude of additional body energy, nor yaw improvement, is validated for 65–75 ms.
