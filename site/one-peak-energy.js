/* Observation-only one-peak model refitted to 14 fixed-leg runs on
 * 2026-10-21. Inputs are exclusively values logged at the ZERO crossing and
 * the already assigned pulse width. The observed peak is used only to score
 * the prediction after the run. Nothing here writes a motor command. */
(function(root) {
  'use strict';
  const model = Object.freeze({
    revision:'negative_one_peak_energy_20261021_fourteen_runs_v2',
    training_events:304,
    input_current_mA:300,
    widths_ms:[0,10,15,20,25],
    reference_abs_pitch_deg:1.932539,
    reference_tolerance_deg:0.05,
    support:{rate_dps:[57.2522,76.2778],wheel_rpm:[4.28,550.04],
      previous_peak_deg:[5.0288,8.52741],battery_mV:[7220,8000]},
    on:[-0.11374238610590627,-0.2137163629606094,-0.0121297577897634,0.002750431639219765],
    off:[0.22670010497624232,0.007915741915865374,0.01891249166239664,
      -0.12244123666267166,0.07027683583380318],
    arrival:[1.396542982188298,0.0986473699053286,-0.024852205530032204,
      -0.04795626985252528,0.004742852774672139,0.42290164040184]
  });
  function potentialMj(deg) {
    const a=Math.abs(deg)*Math.PI/180, r=.15, x=.005, h=.12, m=.1997, g=9.80665;
    const c=Math.sqrt(r*r-x*x);
    const dh=a<=Math.asin(x/r) ? -2*h*Math.sin(a/2)**2+x*Math.sin(a) :
      r-c+2*(c-h)*Math.sin(a/2)**2;
    return 1000*m*g*dh;
  }
  function inversePotentialMj(energy) {
    const upper=Math.asin(.045/.15)*180/Math.PI;
    if(!Number.isFinite(energy) || energy<0 || energy>potentialMj(upper)) return null;
    let lo=0,hi=upper;
    for(let i=0;i<45;++i) {
      const mid=(lo+hi)/2;
      if(potentialMj(mid)<energy) lo=mid; else hi=mid;
    }
    return (lo+hi)/2;
  }
  function dot(a,b) {return a.reduce((s,v,i)=>s+v*b[i],0);}
  function predict(state,widthMs) {
    const rate=(state.rate_dps-72.5)/3;
    const rpm=(state.wheel_rpm-80)/50;
    const prev=(state.previous_peak_deg-8)/.5;
    const voltage=(state.battery_mV-7000)/1000;
    const t=widthMs/25;
    const on=dot(model.on,[t,t*t,t*rate,t*rpm]);
    const off=dot(model.off,[1,rate,rpm,t,t*t]);
    const offZero=dot(model.off,[1,rate,rpm,0,0]);
    const workChange=on+off-offZero;
    const free=dot(model.arrival.slice(0,5),[1,rate,rpm,prev,voltage]);
    const energy=free+model.arrival[5]*workChange;
    return {free_energy_mj:free,predicted_work_on_mj:on,
      predicted_work_after_mj:off,predicted_work_change_mj:workChange,
      residual_factor:model.arrival[5],predicted_energy_mj:energy,
      predicted_peak_deg:inversePotentialMj(energy)};
  }
  function expandTable(value) {
    if(Array.isArray(value)) return value;
    if(!value || !Array.isArray(value.fields) || !Array.isArray(value.rows) ||
       value.rows.some(row=>!Array.isArray(row) || row.length!==value.fields.length))
      throw Error('RWLOGのイベント表の形式が不正です。');
    return value.rows.map(row=>Object.fromEntries(value.fields.map((field,i)=>[field,row[i]])));
  }
  function analyze(metadata) {
    if(!metadata || !metadata.negative_probe_batch ||
       metadata.energy_control_autonomous_normal_current_mA!==model.input_current_mA)
      throw Error('この予測式は固定脚・負側5条件の一括実験（300 mA）のRWLOG専用です。');
    if(metadata.metadata_event_detail_truncated)
      throw Error('RWLOGのイベント詳細が途中で切れています。予測と実測を対応付けられません。');
    if(!metadata.energy_control_autonomous_zero_cross_events ||
       !metadata.energy_control_autonomous_peak_events)
      throw Error('ZEROクロスまたはピークの記録が見つかりません。');
    const events=expandTable(metadata.energy_control_autonomous_zero_cross_events);
    const peaks=expandTable(metadata.energy_control_autonomous_peak_events);
    const negatives=peaks.filter(p=>p.physical_peak_side===-1).sort((a,b)=>a.peak_time_ms-b.peak_time_ms);
    const rows=[];
    for(const e of events) {
      if(e.negative_probe_batch_index==null) continue;
      const width=e.negative_probe_batch_planned_width_ms;
      const state={rate_dps:Math.abs(e.zero_cross_rate_dps),
        wheel_rpm:e.pre_input_wheel_speed_rpm,
        previous_peak_deg:e.previous_peak_amplitude_deg,battery_mV:e.vbat_mV};
      const reasons=[];
      if(e.physical_next_peak_side!==-1) reasons.push('負側以外');
      if(e.zero_cross_time_ms<10000) reasons.push('開始10秒未満');
      if(e.negative_probe_batch_rate_skipped) reasons.push('角速度条件で入力中止');
      if(!e.valid) reasons.push('ZEROクロス無効');
      if(!model.widths_ms.includes(width)) reasons.push('入力時間が学習条件外');
      if(e.pulse_width_ms!==width || (width>0 && !e.output_executed)) reasons.push('指令が計画通り実行されていない');
      if(!e.pre_input_wheel_speed_valid) reasons.push('入力前ホイール速度が無効');
      if(!Number.isFinite(metadata.upright_reference_abs_pitch_deg) ||
         Math.abs(metadata.upright_reference_abs_pitch_deg-model.reference_abs_pitch_deg)>
           model.reference_tolerance_deg) reasons.push('直立基準が学習条件外');
      for(const [key,limits] of Object.entries(model.support))
        if(!Number.isFinite(state[key])||state[key]<limits[0]||state[key]>limits[1])
          reasons.push(key+'が学習範囲外');
      const finite=Object.values(state).every(Number.isFinite) && model.widths_ms.includes(width);
      const prediction=finite ? predict(state,width) : null;
      const candidates=finite ? model.widths_ms.map(ms=>({ms,...predict(state,ms)})) : [];
      const validCandidates=candidates.filter(c=>c.predicted_peak_deg!==null);
      const target=metadata.energy_control_autonomous_target_peak_deg;
      const targetEnergy=Number.isFinite(target) ? potentialMj(target) : null;
      const best=targetEnergy!==null && validCandidates.length ?
        validCandidates.reduce((a,b)=>{
          const aError=Math.abs(a.predicted_energy_mj-targetEnergy);
          const bError=Math.abs(b.predicted_energy_mj-targetEnergy);
          return bError<aError || (bError===aError && b.ms<a.ms) ? b : a;
        }) : null;
      const next=negatives.find(p=>p.peak_time_ms>e.zero_cross_time_ms);
      if(!next) reasons.push('次の負側ピークが未記録');
      const observed=next?.peak_amplitude_deg ?? null;
      const targetReachable=Number.isFinite(target) && validCandidates.length===5 ?
        Math.min(...validCandidates.map(c=>c.predicted_peak_deg))<=target &&
        target<=Math.max(...validCandidates.map(c=>c.predicted_peak_deg)) : null;
      rows.push({model_revision:model.revision,firmware_revision:metadata.firmware_revision,
        event_index:e.event_index,batch_index:e.negative_probe_batch_index,
        zero_cross_time_ms:e.zero_cross_time_ms,assigned_width_ms:width,
        executed_width_ms:e.pulse_width_ms,event_valid:e.valid,
        rate_skipped:e.negative_probe_batch_rate_skipped,
        output_executed:e.output_executed,rate_dps:state.rate_dps,
        wheel_rpm:state.wheel_rpm,previous_peak_deg:state.previous_peak_deg,
        battery_mV:state.battery_mV,target_deg:target,
        ...(prediction||{}),predicted_best_width_ms:best?.ms??null,
        predicted_best_peak_deg:best?.predicted_peak_deg??null,
        predicted_best_energy_error_mj:best&&targetEnergy!==null ?
          best.predicted_energy_mj-targetEnergy : null,
        target_reachable_with_tested_widths:targetReachable,
        next_peak_time_ms:next?.peak_time_ms??null,observed_next_peak_deg:observed,
        error_deg:prediction?.predicted_peak_deg!=null && Number.isFinite(observed) ?
          prediction.predicted_peak_deg-observed : null,
        within_training_support:reasons.length===0,exclusions:reasons.join(' / ')});
    }
    const scored=rows.filter(r=>r.within_training_support && r.error_deg!==null);
    const diagnostic=rows.filter(r=>r.zero_cross_time_ms>=10000 && r.event_valid &&
      !r.rate_skipped && r.assigned_width_ms===r.executed_width_ms &&
      (r.assigned_width_ms===0 || r.output_executed) && r.error_deg!==null);
    const outside=diagnostic.filter(r=>!r.within_training_support);
    const score=group=>({n:group.length,
      rmse_deg:group.length?Math.sqrt(group.reduce((s,r)=>s+r.error_deg**2,0)/group.length):null,
      mean_error_deg:group.length?group.reduce((s,r)=>s+r.error_deg,0)/group.length:null});
    const rmse=scored.length ? Math.sqrt(scored.reduce((s,r)=>s+r.error_deg**2,0)/scored.length) : null;
    const mae=scored.length ? scored.reduce((s,r)=>s+Math.abs(r.error_deg),0)/scored.length : null;
    return {model,rows,summary:{events:rows.length,scored:scored.length,rmse_deg:rmse,mae_deg:mae,
      outside_support:score(outside),training_held_out_run_rmse_deg:.289}};
  }
  function describe(report) {
    const {summary,rows}=report;
    const voltage=rows.map(r=>r.battery_mV).filter(Number.isFinite);
    const line=(x,unit)=>x==null?'対象なし':x.toFixed(3)+unit;
    return [`固定した式：${model.revision}`,
      `条件付きで評価したイベント：${summary.scored}/${summary.events}`,
      `新しいログの次ピーク誤差：RMSE ${line(summary.rmse_deg,'°')} / 平均絶対誤差 ${line(summary.mae_deg,'°')}`,
      `学習範囲外の参考値：${summary.outside_support.n}件 / RMSE ${line(summary.outside_support.rmse_deg,'°')} / 予測−実測の平均 ${line(summary.outside_support.mean_error_deg,'°')}`,
      '参考：学習に使った14回を1回ずつ除いて検証したRMSEは0.289°',
      voltage.length ? `このログの記録電圧：${(Math.min(...voltage)/1000).toFixed(2)}〜${(Math.max(...voltage)/1000).toFixed(2)} V` : '電圧記録なし',
      '直立基準や学習範囲が異なる予測は検証精度として扱いません。CSVには予測値と理由を残します。',
      '停止後の仕事は予測値です。実測ピークを予測計算に使っていません。',
      'このページの予測結果はホイールへの指令に使用しません。'].join('\n');
  }
  root.OnePeakEnergy={model,potentialMj,inversePotentialMj,predict,analyze,describe};
  if(typeof module!=='undefined') module.exports=root.OnePeakEnergy;
})(typeof globalThis!=='undefined'?globalThis:this);
