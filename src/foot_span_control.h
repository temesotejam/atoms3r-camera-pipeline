#pragma once
#include <cmath>
#include <cstdint>

// Experimental control on camera-relative complete-foot spans. It does not
// measure ground stride or yaw. The caller must provide cycle measurements
// matched to the ACTUAL commands and enforce motor/attitude/emergency guards.
namespace foot_span_control {
struct Cycle {
  uint32_t start_ms=0, end_ms=0, positive_ms=0;
  float left=NAN, right=NAN, negative_peak=NAN, target_mj=NAN;
  uint32_t negative_key=0; // actual negative command: coast or current/sign/width; never model-support label
  uint16_t positive_width=0;
  int16_t positive_ma=0;
  int8_t positive_sign=0;
  bool complete=false, negative_known=false, positive_executed=false;
};
enum class State : uint8_t { Collect, ProbeMinus, Restore, ProbePlus, Hold, Fault };
class Controller {
 public:
  void reset() { *this=Controller{}; }
  State state() const { return state_; }
  bool fault() const { return state_==State::Fault; }
  bool requireNeutral() const { return state_==State::Fault; }
  uint16_t requested() const { return requested_; }
  uint16_t anchor() const { return anchor_; }
  int16_t anchorCurrent() const { return anchor_ma_; }
  int direction() const { return direction_; }
  // Called only if a proposed candidate was never issued. Returning false
  // requests skipping this crossing; the caller must not silently send a
  // baseline that jumps more than 5ms from the confirmed command.
  // A changed/unavailable selector is not a motor fault. Give control back to
  // the caller's ordinary selector/fallback; discard trial and re-acquire.
  bool selectorUnavailable(uint32_t now) {
    if(state_==State::Fault)return false;
    resetCollect();baseline_after_ms_=now;request_target_mj_=NAN;return true;
  }
  bool cancelUnsent(uint16_t baseline) {
    if(state_==State::Fault)return false;
    const bool can_return=!last_actual_valid_ || last_actual_ma_!=request_ma_ ||
        (last_actual_width_<=baseline+5 && baseline<=last_actual_width_+5);
    if(!can_return) {beginRestore(false);return false;}
    resetCollect();await_baseline_confirmation_=true;return true;
  }
  float referenceError() const { return reference_error_; }
  unsigned acceptedProbeCycles() const { return probe_count_; }
  // Consume each strictly chronological cycle at most once. Incomplete or
  // mismatched observations never become zero or evidence of improvement.
  void cycle(const Cycle& c) {
    if(c.end_ms<=last_end_) return;
    last_end_=c.end_ms;
    if(state_==State::Fault) return;
    if(!c.complete || !c.negative_known || !c.positive_executed ||
       !supportedCurrent(c.positive_ma) || c.positive_sign!=1 ||
       c.start_ms>=c.positive_ms || c.positive_ms>=c.end_ms ||
       !std::isfinite(c.left) || !std::isfinite(c.right) ||
       c.left<0.f || c.right<0.f || !std::isfinite(c.negative_peak) || c.negative_peak<=0.f ||
       !std::isfinite(c.target_mj) || c.target_mj<=0.f) {
      if(state_==State::Collect) clearReference();
      return;
    }
    const Actual* actual=nullptr;unsigned actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=actual_[(actual_next_+8-actual_count_+i)%8];
      if(a.ms>c.start_ms && a.ms<c.end_ms) {actual=&a;++actions;}
    }
    if(actions!=1 || !actual || actual->ms!=c.positive_ms || !actual->executed ||
       actual->ma!=c.positive_ma || actual->sign!=c.positive_sign || actual->width!=c.positive_width ||
       !std::isfinite(actual->target_mj) || std::fabs(actual->target_mj-c.target_mj)>.001f) {
      if(state_==State::Collect) clearReference();
      return;
    }
    const float error=std::fabs(c.left-c.right);
    if(state_==State::Collect) {
      if(baseline_after_ms_ && c.start_ms<=baseline_after_ms_) return;
      if(reference_count_ && (c.start_ms!=reference_end_ || c.negative_key!=negative_key_ ||
          c.positive_width!=anchor_ || c.positive_ma!=anchor_ma_ || std::fabs(c.target_mj-target_)>.001f)) clearReference();
      if(reference_count_==0) { anchor_=c.positive_width; anchor_ma_=c.positive_ma; negative_key_=c.negative_key; target_=c.target_mj; reference_sum_=0.f; reference_total_sum_=0.f; reference_peak_sum_=0.f; }
      const float total=c.left+c.right;
      if(reference_count_ && std::fabs(error-last_reference_error_)>.3f) {
        // Reacquire a stable adjacent reference pair; never lock adaptation.
        clearReference();reference_sum_=error;reference_total_sum_=total;
        reference_peak_sum_=c.negative_peak;reference_count_=1;
      } else if(reference_count_<2) {
        if(reference_count_) reference_spread_=std::fabs(error-last_reference_error_);
        reference_sum_+=error;reference_total_sum_+=total;reference_peak_sum_+=c.negative_peak;++reference_count_;
      } else {
        reference_spread_=std::fabs(error-last_reference_error_);
        reference_sum_=last_reference_error_+error;reference_total_sum_=last_reference_total_+total;reference_peak_sum_=last_reference_peak_+c.negative_peak;
      }
      last_reference_error_=error;last_reference_total_=total;last_reference_peak_=c.negative_peak;reference_end_=c.end_ms;
      if(reference_count_>=2) {reference_error_=reference_sum_/2.f;reference_total_=reference_total_sum_/2.f;reference_peak_=reference_peak_sum_/2.f;}
      return;
    }
    if(state_==State::Restore) return;
    // Never judge a pre-intervention or incomparable cycle as a response.
    if(c.negative_key!=negative_key_ || std::fabs(c.target_mj-target_)>.001f ||
       c.positive_ma!=anchor_ma_ || c.positive_width!=desired_ || !probe_started_ || c.start_ms<=probe_started_) return;
    last_comparable_end_=c.end_ms;
    // A smaller left-right difference is not success if overall motion or
    // the associated body peak falls significantly. These guard thresholds
    // are inherited experimental criteria, not proof of ground gait safety.
    if(c.left+c.right<reference_total_-.3f || c.negative_peak<reference_peak_-.5f) {
      beginRestore(false);return;
    }
    if(state_==State::Hold) {
      if(error>reference_error_+.3f) beginRestore(false);
      return;
    }
    if(state_!=State::ProbeMinus && state_!=State::ProbePlus) return;
    if(probe_last_end_ && c.start_ms!=probe_last_end_) {probe_count_=0;probe_sum_=0.f;}
    if(probe_count_==0) probe_min_=probe_max_=error;
    else {probe_min_=std::fmin(probe_min_,error);probe_max_=std::fmax(probe_max_,error);}
    probe_last_end_=c.end_ms;probe_sum_+=error;++probe_count_;
    if(probe_count_>=2) {
      const float spread=probe_max_-probe_min_;
      const float required=.1f+.5f*(reference_spread_+spread);
      if(spread<=.3f && probe_sum_/static_cast<float>(probe_count_) <= reference_error_-required) {
        state_=State::Hold;
      } else beginRestore(state_==State::ProbeMinus);
    }
  }
  // The candidate is bounded by the caller's existing hard cap and differs
  // from the matching baseline by no more than one 5ms step. Unsafe/changed
  // modes return the caller's baseline with no experimental intervention.
  uint16_t limit(uint32_t now,uint16_t baseline,int16_t ma,int sign,
                 uint16_t existing_cap,bool allowed,float target_mj) {
    const uint16_t safe=baseline<existing_cap ? baseline:existing_cap;
    requested_=safe; pending_=false; request_target_mj_=target_mj;request_ma_=ma;
    if(state_==State::Fault) {requested_=0;return 0;}
    if(!supportedCurrent(ma) || sign!=1 || (state_!=State::Collect && ma!=anchor_ma_)) {
      // An experimental perturbation cannot be transferred to a new
      // current/direction. Return to the independently chosen, capped normal
      // selector immediately; never relabel it as a confirmed probe.
      resetCollect();baseline_after_ms_=now;return safe;
    }
    if(!allowed || !std::isfinite(target_mj) || target_mj<=0.f) {
      // Actual hardware/sensor guard still wins. The caller must suppress
      // output for unsafe hardware and distinguish it from model coverage.
      const bool active=state_!=State::Collect;
      if(active) {beginRestore(false);requested_=0;return 0;}
      resetCollect();return safe;
    }
    if(state_!=State::Collect && state_!=State::Restore &&
       (baseline!=anchor_ || safe!=baseline || std::fabs(target_mj-target_)>.001f)) {
      beginRestore(false);
    }
    if((state_==State::ProbeMinus || state_==State::ProbePlus) && probe_started_) {
      const uint32_t fresh=last_comparable_end_?last_comparable_end_:probe_started_;
      if(now<fresh || now-fresh>2000) beginRestore(false);
      else if(now<probe_started_ || now-probe_started_>6000) beginRestore(state_==State::ProbeMinus);
    }
    if(state_==State::Hold && (!last_comparable_end_ || now<last_comparable_end_ ||
       now-last_comparable_end_>2000 || now<probe_started_ || now-probe_started_>6000)) beginRestore(false);
    if(state_==State::Collect) {
      if(reference_count_<2 || now<reference_end_ || now-reference_end_>2000 ||
         baseline!=anchor_ || ma!=anchor_ma_ || std::fabs(target_mj-target_)>.001f || safe!=baseline) return safe;
      // Do not perturb an already balanced signal below its observed variation.
      if(reference_error_<=.1f+.5f*reference_spread_) return safe;
      if(next_direction_>0) {
        if(anchor_+5<=existing_cap && anchor_+5<=100) startProbe(1);
        else {next_direction_=-1;if(anchor_>=15) startProbe(-1);else return safe;}
      } else if(anchor_>=15) startProbe(-1);
      else if(anchor_+5<=existing_cap && anchor_+5<=100) startProbe(1);
      else return safe;
    }
    if(state_==State::Restore) {
      restore_target_=safe;
      // Never grow the width by more than 5ms from the actual prior command;
      // a tighter existing hard cap may require a larger immediate decrease.
      requested_=safe;
      if(requested_==0) {resetCollect();baseline_after_ms_=now;return 0;}
      if(last_actual_valid_ && last_actual_ma_==ma && requested_>last_actual_width_+5)
        requested_=static_cast<uint16_t>(last_actual_width_+5);
      pending_=true;return requested_;
    }
    if(desired_>existing_cap || desired_>100 || desired_<10) {resetCollect();return safe;}
    requested_=desired_;pending_=true;return requested_;
  }
  // Confirm actual command; an unexecuted or contradictory command prevents
  // future probing until explicit run reset. Confirmation is command-level,
  // not a measurement of motor current or robot response.
  void confirm(uint32_t now,bool executed,int16_t ma,int sign,uint16_t width) {
    Actual& a=actual_[actual_next_];
    a.ms=now;a.width=width;a.ma=ma;a.sign=sign;a.executed=executed;a.target_mj=request_target_mj_;
    actual_next_=(actual_next_+1)%8;if(actual_count_<8)++actual_count_;
    if(executed && width>0) {
      if(supportedCurrent(ma) && sign==1) {last_actual_width_=width;last_actual_ma_=ma;last_actual_valid_=true;}
      else last_actual_valid_=false;
    }
    if(await_baseline_confirmation_) {
      if(executed && supportedCurrent(ma) && sign==1 && width>0) baseline_after_ms_=now;
      await_baseline_confirmation_=false;
    }
    if(!pending_)return;
    pending_=false;
    if(!executed || ma!=request_ma_ || sign!=1 || width!=requested_) {state_=State::Fault;return;}
    if(state_==State::Restore) {
      if(width<restore_target_)return;
      const bool next_plus=try_plus_after_restore_ && restore_target_==anchor_ && anchor_+5<=100;
      resetCollect();baseline_after_ms_=now;next_direction_=next_plus?1:-1;
      return;
    }
    if((state_==State::ProbeMinus || state_==State::ProbePlus) && !probe_started_)probe_started_=now;
  }
 private:
  static bool supportedCurrent(int16_t ma) { return ma==300 || ma==350 || ma==400; }
  void clearReference() {reference_count_=0;reference_sum_=0.f;reference_end_=0;}
  void resetCollect() {state_=State::Collect;clearReference();pending_=false;probe_started_=probe_last_end_=last_comparable_end_=0;baseline_after_ms_=0;probe_count_=0;probe_sum_=0.f;direction_=0;}
  void startProbe(int dir) {direction_=dir;desired_=static_cast<uint16_t>(static_cast<int>(anchor_)+dir*5);
    state_=dir<0?State::ProbeMinus:State::ProbePlus;probe_started_=probe_last_end_=last_comparable_end_=0;baseline_after_ms_=0;probe_sum_=0.f;probe_count_=0;}
  void beginRestore(bool try_plus) {state_=State::Restore;try_plus_after_restore_=try_plus;pending_=false;}
  struct Actual {uint32_t ms=0;uint16_t width=0;int16_t ma=0;int sign=0;bool executed=false;float target_mj=NAN;};
  Actual actual_[8];unsigned actual_next_=0,actual_count_=0;
  State state_=State::Collect;
  uint32_t last_end_=0,reference_end_=0,negative_key_=0,probe_started_=0,probe_last_end_=0,last_comparable_end_=0,baseline_after_ms_=0;
  uint16_t anchor_=0,desired_=0,requested_=0,restore_target_=0,last_actual_width_=0;
  int16_t anchor_ma_=0,request_ma_=0,last_actual_ma_=0;
  float target_=NAN,request_target_mj_=NAN,reference_sum_=0.f,reference_error_=0.f,last_reference_error_=0.f,probe_sum_=0.f;
  float reference_total_sum_=0.f,reference_total_=0.f,last_reference_total_=0.f;
  float reference_peak_sum_=0.f,reference_peak_=0.f,last_reference_peak_=0.f;
  float reference_spread_=0.f,probe_min_=0.f,probe_max_=0.f;
  unsigned reference_count_=0,probe_count_=0;
  int direction_=0,next_direction_=-1;
  bool pending_=false,try_plus_after_restore_=false,last_actual_valid_=false,await_baseline_confirmation_=false;
};
}
