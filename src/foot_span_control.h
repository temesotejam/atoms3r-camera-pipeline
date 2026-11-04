#pragma once
#include <cmath>
#include <cstdint>

// Online experimental correction of body-relative camera marker spans. These
// values are not ground step lengths, and the initial control polarity is an
// empirically motivated hypothesis, not a guaranteed actuator response.
namespace foot_span_control {
// A single sampled extremum may be a fast real swing or a false detection.
// Neither the raw nor trimmed span alone can decide which; withhold feedback
// when discarding that extremum changes the result substantially.
inline bool spanReliable(float raw,float trimmed) {
  return std::isfinite(raw) && std::isfinite(trimmed) && raw>=0.f && trimmed>=0.f &&
      !(raw-trimmed>1.5f && trimmed<raw*.5f);
}
struct Cycle {
  uint32_t start_ms=0, end_ms=0, positive_ms=0;
  float left=NAN, right=NAN, negative_peak=NAN, target_mj=NAN;
  // Context of the negative input: comparisons must not silently treat
  // different early-trigger angles or incoming mechanical states as equal.
  float negative_trigger_deg=0.f,negative_rate_dps=0.f,negative_rpm=0.f;
  float positive_rate_dps=0.f,positive_rpm=0.f;
  uint32_t negative_key=0;
  uint16_t positive_width=0;
  int16_t positive_ma=0;
  int8_t positive_sign=0;
  bool complete=false, negative_known=false, positive_executed=false;
};
enum class State : uint8_t { Collect, ProbeMinus, Restore, ProbePlus, Hold, Fault };
class Controller {
 public:
  void reset() { *this=Controller{}; }
  State state()const{return state_;}
  bool fault()const{return state_==State::Fault;}
  bool requireNeutral()const{return fault();}
  uint16_t requested()const{return requested_;}
  uint16_t anchor()const{return baseline_ms_;}
  int16_t anchorCurrent()const{return request_ma_;}
  int direction()const{return offset_ms_<0?-1:offset_ms_>0?1:0;}
  float referenceError()const{return filtered_error_;}
  unsigned acceptedProbeCycles()const{return updates_;}
  int offset()const{return offset_ms_;}
  // Earlier 400mA runs tried the negative offset repeatedly while right
  // turning remained. The 1200mA branch starts its own exploratory trial in
  // the opposite direction; its response is not assumed to be the same.
  void preferHighCurrentFirst(){if(!high_current_initial_){polarity_=1;high_current_initial_=true;}}
  bool selectorUnavailable(uint32_t now) {
    if(fault())return false;
    // A different selector is not a hardware fault. Cancel experimental
    // feedback and reacquire observation before trying another candidate.
    (void)now;invalidateObservation();pending_=false;request_eligible_=false;return true;
  }
  bool cancelUnsent(uint16_t baseline) {
    (void)baseline;
    if(fault())return false;
    // Candidate not sent: restore the ordinary selector; do not pretend that
    // its resulting command tested the requested experimental width.
    if(trial_active_ && confirmed_offset_!=0)retry_after_restore_=true;
    pending_=false;request_changed_=false;offset_ms_=0;requested_=baseline;trial_active_=false;have_observation_=false;
    proposal_sign_=0;
    pending_change_=confirmed_offset_!=0;
    consecutive_=0;last_usable_end_=0;have_reference_=false;state_=State::Collect;return true;
  }
  void cycle(const Cycle& c) {
    if(c.end_ms<=last_end_ || fault())return;
    last_end_=c.end_ms;
    if(!c.complete || !c.negative_known || !c.positive_executed || !supportedCurrent(c.positive_ma) ||
       c.positive_sign!=1 || c.start_ms>=c.positive_ms || c.positive_ms>=c.end_ms ||
       !std::isfinite(c.left) || !std::isfinite(c.right) || c.left<0 || c.right<0 ||
       !std::isfinite(c.negative_peak) || c.negative_peak<=0 ||
       !std::isfinite(c.target_mj) || c.target_mj<=0 ||
       !std::isfinite(c.negative_trigger_deg) || !std::isfinite(c.negative_rate_dps) ||
       !std::isfinite(c.negative_rpm) || !std::isfinite(c.positive_rate_dps) ||
       !std::isfinite(c.positive_rpm)) {
      invalidateObservation();return;
    }
    const Actual* action=nullptr;unsigned actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=actual_[(actual_next_+8-actual_count_+i)%8];
      if(a.ms>c.start_ms && a.ms<c.end_ms){action=&a;++actions;}
    }
    if(actions!=1 || !action || action->ms!=c.positive_ms || !action->executed ||
       action->ma!=c.positive_ma || action->sign!=c.positive_sign ||
       action->width!=c.positive_width || !std::isfinite(action->target_mj) ||
       std::fabs(action->target_mj-c.target_mj)>.001f) {invalidateObservation();return;}
    // Each actual command retains the input-change and feedback-change
    // timestamps that were known at that instant. Future ordinary commands
    // must not retrospectively invalidate this earlier camera cycle.
    // An older feedback generation after a new intervention is not a
    // response to the new intervention and must not increment it again.
    if(action->feedback_changed_ms!=last_feedback_changed_ms_)return;
    // After an interrupted applied trial has actually returned to baseline,
    // allow the ordinary controller to settle before reacquiring evidence.
    // Only cycles that start after the wait count toward a new proposal.
    if(retry_wait_) {
      const uint32_t elapsed=c.start_ms-retry_start_ms_;
      if(elapsed>=0x80000000u || elapsed<5000u) {invalidateObservation();return;}
      retry_wait_=false;
    }
    const bool continuous=last_usable_end_ && c.start_ms==last_usable_end_;
    consecutive_=continuous?consecutive_+1:1;last_usable_end_=c.end_ms;
    // Normalize asymmetry so shrinking both feet cannot look like progress.
    // A small constant avoids unstable division when both signals are small;
    // the camera width is not an absolute ground stride.
    if(c.left+c.right<.3f) {invalidateObservation();return;}
    const float e=(c.left-c.right)/(c.left+c.right+.1f);
    // A sign reversal between adjacent complete cycles is not evidence for a
    // sustained side imbalance. Reacquire two consecutive same-sign cycles
    // before proposing a new offset; rollback remains independent of this.
    const int sign=e>.10f ? 1 : e<-.10f ? -1 : 0;
    if(sign && sign==imbalance_sign_ && continuous) {
      if(imbalance_count_<2)++imbalance_count_;
    } else {imbalance_sign_=sign;imbalance_count_=sign ? 1 : 0;}
    // A queued exploratory change needs a still-sustained imbalance. A new
    // complete cycle that is balanced or reverses the imbalance cancels the
    // unsent proposal, while a safety rollback remains independent.
    if(pending_change_ && proposal_sign_ &&
       (imbalance_count_<2 || imbalance_sign_!=proposal_sign_)) {
      offset_ms_=confirmed_offset_;pending_change_=false;proposal_sign_=0;
      state_=offset_ms_==0 ? State::Collect : State::Hold;
    }
    observed_left_=c.left;observed_right_=c.right;observed_peak_=c.negative_peak;
    observed_error_=e;observed_current_=c.positive_ma;observed_target_=c.target_mj;
    observed_negative_key_=c.negative_key;
    observed_negative_trigger_=c.negative_trigger_deg;
    observed_negative_rate_=c.negative_rate_dps;observed_negative_rpm_=c.negative_rpm;
    observed_positive_rate_=c.positive_rate_dps;observed_positive_rpm_=c.positive_rpm;
    observed_baseline_=action->baseline;
    have_observation_=true;
    if(!have_reference_) {
      filtered_error_=e;reference_peak_=c.negative_peak;
      reference_left_=c.left;reference_right_=c.right;have_reference_=true;return;
    }
    filtered_error_=.5f*filtered_error_+.5f*e;
    if(offset_ms_==0) {
      reference_peak_=.8f*reference_peak_+.2f*c.negative_peak;
      reference_left_=.8f*reference_left_+.2f*c.left;
      reference_right_=.8f*reference_right_+.2f*c.right;
    }
    if(pending_change_ || consecutive_<2 ||
       (action->feedback_changed_ms && c.start_ms<=action->feedback_changed_ms))return;
    // Capture two complete observations after the actual changed command.
    // Negative-side input/contact can change too: this is trend evidence,
    // never a claim that the pulse caused the measured difference.
    if(trial_active_) {
      const int base_delta=static_cast<int>(action->baseline)-static_cast<int>(trial_baseline_);
      if(!trial_reference_valid_ || c.positive_ma!=trial_current_ || std::fabs(c.target_mj-trial_target_)>.05f ||
         base_delta>5 || base_delta< -5 || !comparableNegative(c.negative_key,trial_negative_key_) ||
         std::fabs(c.negative_trigger_deg-trial_negative_trigger_)>.5f ||
         std::fabs(c.negative_rate_dps-trial_negative_rate_)>10.f ||
         std::fabs(c.negative_rpm-trial_negative_rpm_)>100.f ||
         std::fabs(c.positive_rate_dps-trial_positive_rate_)>15.f ||
         std::fabs(c.positive_rpm-trial_positive_rpm_)>150.f) {
        invalidateObservation();return;
      }
      const float weak=trial_right_?c.right:c.left;
      trial_ratio_sum_+=std::fabs(e);trial_weak_sum_+=weak;trial_peak_sum_+=c.negative_peak;
      if(++trial_count_<2)return;
      const float ratio=trial_ratio_sum_*.5f,weak_mean=trial_weak_sum_*.5f,peak=trial_peak_sum_*.5f;
      trial_active_=false;
      const bool weak_kept=weak_mean>=trial_weak_*.8f && weak_mean>=trial_weak_-.10f;
      // Balance is only a useful success when the initially weaker foot has
      // actually increased. A shrinking stronger foot alone must not be
      // mistaken for a response to the changed command. This is a small
      // observation threshold, not a precision or ground-stride guarantee.
      const bool weak_improved=weak_mean>=trial_weak_*1.05f && weak_mean>=trial_weak_+.05f;
      const bool peak_kept=peak>=trial_peak_*.85f && peak>=trial_peak_-.5f;
      // No clear ratio improvement, loss of the smaller foot, or loss of
      // negative oscillation: unwind this offset step and explore oppositely.
      if(!weak_kept || !weak_improved || !peak_kept || ratio+.02f>=trial_ratio_) {
        polarity_=-polarity_;
        const int restored=towardZero(offset_ms_);
        state_=restored==0?State::Collect:State::Restore;
        if(restored!=offset_ms_){offset_ms_=restored;pending_change_=true;proposal_sign_=0;}
        return;
      }
    }
    // Maintain a weaker-side/peak guard even after the offset reaches a cap.
    const float weak=reference_right_<=reference_left_?c.right:c.left;
    const float refweak=std::fmin(reference_left_,reference_right_);
    const bool weak_loss=weak<refweak*.8f && weak<refweak-.05f;
    const bool peak_loss=c.negative_peak<reference_peak_-.5f;
    // Baseline motion alone is not the result of an applied trial. Do not
    // reverse the exploratory polarity when there is nothing to roll back.
    if(offset_ms_!=0 && (weak_loss || peak_loss))++weak_count_;else weak_count_=0;
    if(weak_count_>=2) {
      weak_count_=0;polarity_=-polarity_;
      const int restored=towardZero(offset_ms_);
      state_=restored==0?State::Collect:State::Restore;
      if(restored!=offset_ms_) {offset_ms_=restored;pending_change_=true;proposal_sign_=0;}
      return;
    }
    if(std::fabs(filtered_error_)<=.10f){state_=State::Hold;return;}
    if(imbalance_count_<2 || (filtered_error_>0.f ? 1 : -1)!=imbalance_sign_)return;
    const int step=(filtered_error_>0.f ? 1 : -1)*polarity_*10;
    const int proposal=offset_ms_+step;
    const int bounded=proposal < -30 ? -30 : proposal > 30 ? 30 : proposal;
    state_=bounded<0?State::ProbeMinus:bounded>0?State::ProbePlus:State::Collect;
    if(bounded!=offset_ms_) {offset_ms_=bounded;pending_change_=true;proposal_sign_=imbalance_sign_;}
  }

  uint16_t limit(uint32_t now,uint16_t baseline,int16_t ma,int sign,
                 uint16_t existing_cap,bool allowed,float target_mj,
                 float positive_rate_dps=0.f,float positive_rpm=0.f) {
    const uint16_t cap=existing_cap<100?existing_cap:100;
    const uint16_t safe=baseline<cap?baseline:cap;
    baseline_ms_=baseline;request_ma_=ma;request_target_mj_=target_mj;
    requested_=safe;pending_=false;request_changed_=false;request_eligible_=false;
    if(fault()){requested_=0;return 0;}
    // The ordinary selector may change current every step. Keep the feedback
    // offset and track the actual current/width; a time offset at a different
    // current is not claimed to deliver the same physical energy.
    if(!allowed || !supportedCurrent(ma) || sign!=1 || !std::isfinite(target_mj) || target_mj<=0 || safe<10) {
      // The caller suppresses real hardware faults and uses the normal
      // selector for unsupported modes, rather than latching a zero output.
      state_=State::Collect;return safe;
    }
    request_eligible_=true;
    // A candidate proposed from an older camera cycle must not be submitted
    // after a missing/invalid observation. Ordinary output remains available.
    if(!have_observation_ && offset_ms_!=0)invalidateObservation();
    if(last_usable_end_ && (now<last_usable_end_ || now-last_usable_end_>2500)) {
      // Stale camera data cannot justify retaining any experimental offset.
      // Continue with the ordinary selector while observations reacquire.
      invalidateObservation();
    }
    if(!last_usable_end_)return safe;
    // Do not submit a proposed change if its own pre-input state already
    // differs from the reference. Checking only later cycles would allow the
    // intervention itself to occur at a quite different wheel/body speed.
    if(pending_change_ && offset_ms_!=0) {
      const int delta=static_cast<int>(observed_baseline_)-static_cast<int>(baseline);
      if(!have_observation_ || !std::isfinite(positive_rate_dps) || !std::isfinite(positive_rpm) ||
         observed_current_!=ma || std::fabs(observed_target_-target_mj)>.05f ||
         delta>5 || delta< -5 || std::fabs(observed_positive_rate_-positive_rate_dps)>15.f ||
         std::fabs(observed_positive_rpm_-positive_rpm)>150.f) {
        invalidateObservation();return safe;
      }
    }
    // If a newly requested first step has no room in that direction, try the
    // other direction explicitly instead of repeatedly counting a blocked
    // proposal. This is an exploratory intervention, not proof that the
    // opposite sign improves gait. Never exceed one 10ms feedback step.
    if(pending_change_ && confirmed_offset_==0 && offset_ms_==10 && safe==cap && safe>=20) {
      polarity_=-polarity_;offset_ms_=-10;state_=State::ProbeMinus;
    } else if(pending_change_ && confirmed_offset_==0 && offset_ms_==-10 && safe==10 && cap>=20) {
      polarity_=-polarity_;offset_ms_=10;state_=State::ProbePlus;
    }
    int wanted=static_cast<int>(safe)+offset_ms_;
    // Anti-windup: a correction blocked by a physical cap is not an
    // applied intervention. Do not hide an unexecuted offset that could
    // suddenly reappear when the ordinary cap later increases.
    if(wanted<10) {wanted=10;offset_ms_=10-static_cast<int>(safe);}
    if(wanted>cap) {wanted=cap;offset_ms_=static_cast<int>(cap)-safe;}
    // A changing cap can itself remove an already-applied offset. Record the
    // offset transition only after the resulting command is confirmed too.
    if(offset_ms_!=confirmed_offset_)pending_change_=true;
    if(offset_ms_==0 && (state_==State::ProbeMinus || state_==State::ProbePlus))state_=State::Collect;
    // Only each feedback OFFSET update is limited to 10ms. The ordinary
    // selector can change its own width or mode by more than 10ms; do not
    // suppress that normal output merely because its model branch changed.
    // The real absolute hardware/gradual-start cap still takes precedence.
    requested_=static_cast<uint16_t>(wanted);
    request_changed_=requested_!=safe;
    pending_=request_changed_;
    return requested_;
  }
  void confirm(uint32_t now,bool executed,int16_t ma,int sign,uint16_t width) {
    Actual& a=actual_[actual_next_];a.ms=now;a.width=width;a.ma=ma;a.sign=sign;
    a.executed=executed;a.target_mj=request_target_mj_;a.baseline=baseline_ms_;
    actual_next_=(actual_next_+1)%8;if(actual_count_<8)++actual_count_;
    if(pending_ && (!executed || ma!=request_ma_ || sign!=1 || width!=requested_)) {
      state_=State::Fault;pending_=false;return;
    }
    if(executed && supportedCurrent(ma) && sign==1 && width>0) {
      const bool applied_feedback=pending_change_ && request_eligible_ &&
          ma==request_ma_ && sign==1 && width==requested_ && offset_ms_!=confirmed_offset_;
      // Count only confirmed feedback transitions.
      if(applied_feedback){
        last_feedback_changed_ms_=now;confirmed_offset_=offset_ms_;proposal_sign_=0;++updates_;
        if(confirmed_offset_==0 && retry_after_restore_) {
          retry_after_restore_=false;retry_wait_=true;retry_start_ms_=now;
        }
        trial_active_=offset_ms_!=0 && have_observation_;
        if(trial_active_) {
          trial_ratio_=std::fabs(observed_error_);
          trial_right_=observed_right_<=observed_left_;
          trial_weak_=trial_right_?observed_right_:observed_left_;
          trial_peak_=observed_peak_;trial_current_=ma;trial_target_=request_target_mj_;trial_baseline_=baseline_ms_;
          trial_negative_key_=observed_negative_key_;
          trial_negative_trigger_=observed_negative_trigger_;
          trial_negative_rate_=observed_negative_rate_;trial_negative_rpm_=observed_negative_rpm_;
          trial_positive_rate_=observed_positive_rate_;trial_positive_rpm_=observed_positive_rpm_;
          const int before_delta=static_cast<int>(observed_baseline_)-static_cast<int>(baseline_ms_);
          trial_reference_valid_=observed_current_==ma && std::fabs(observed_target_-request_target_mj_)<=.05f &&
              before_delta<=5 && before_delta>= -5;
          trial_ratio_sum_=trial_weak_sum_=trial_peak_sum_=0.f;trial_count_=0;
        }
      }
      if(request_eligible_ && ma==request_ma_ && sign==1 && width==requested_)pending_change_=false;
    }
    a.feedback_changed_ms=last_feedback_changed_ms_;
    pending_=false;
  }
 private:
  void invalidateObservation() {
    // Drop both an active trial and a proposal not yet sent. Keeping the
    // proposal after clearing the reference would issue an untracked pulse.
    if(trial_active_ && confirmed_offset_!=0)retry_after_restore_=true;
    consecutive_=0;last_usable_end_=0;weak_count_=0;have_reference_=false;
    imbalance_count_=0;imbalance_sign_=0;proposal_sign_=0;
    have_observation_=false;trial_active_=false;offset_ms_=0;
    pending_change_=confirmed_offset_!=0;
    state_=pending_change_?State::Restore:State::Collect;
  }
  static bool supportedCurrent(int16_t ma){return ma==300 || ma==350 || ma==400 || ma==1200;}
  // Keep distinct negative inputs out of the same response comparison.
  // A 10 ms tolerance accommodates observed normal negative pulse changes;
  // similar commands still do not establish an isolated causal response.
  static bool comparableNegative(uint32_t a,uint32_t b){
    if(a==1 || b==1)return a==1 && b==1;
    if(!(a&0x80000000u) || !(b&0x80000000u) || ((a^b)&0x7fff8000u))return false;
    const int delta=static_cast<int>(a&0x7fffu)-static_cast<int>(b&0x7fffu);
    return delta>=-10 && delta<=10;
  }
  static int towardZero(int value){return value>10?value-10:value< -10?value+10:0;}
  struct Actual {uint32_t ms=0,feedback_changed_ms=0;
    uint16_t width=0,baseline=0;int16_t ma=0;int8_t sign=0;bool executed=false;float target_mj=NAN;};
  Actual actual_[8];unsigned actual_next_=0,actual_count_=0;
  State state_=State::Collect;
  uint32_t last_end_=0,last_usable_end_=0,last_feedback_changed_ms_=0,retry_start_ms_=0;
  uint32_t observed_negative_key_=0,trial_negative_key_=0;
  uint16_t baseline_ms_=0,requested_=0;
  int16_t request_ma_=0;
  float request_target_mj_=NAN,filtered_error_=0,reference_peak_=0,reference_left_=0,reference_right_=0;
  float observed_left_=0,observed_right_=0,observed_peak_=0,observed_error_=0,observed_target_=0;
  float observed_negative_trigger_=0,observed_negative_rate_=0,observed_negative_rpm_=0;
  float trial_negative_trigger_=0,trial_negative_rate_=0,trial_negative_rpm_=0;
  float observed_positive_rate_=0,observed_positive_rpm_=0;
  float trial_positive_rate_=0,trial_positive_rpm_=0;
  float trial_ratio_=0,trial_weak_=0,trial_peak_=0,trial_target_=0;
  float trial_ratio_sum_=0,trial_weak_sum_=0,trial_peak_sum_=0;
  int16_t observed_current_=0,trial_current_=0;uint16_t observed_baseline_=0,trial_baseline_=0;unsigned trial_count_=0;
  bool have_observation_=false,trial_active_=false,trial_right_=true,trial_reference_valid_=false;
  int offset_ms_=0,confirmed_offset_=0,polarity_=-1;
  int imbalance_sign_=0,proposal_sign_=0;unsigned imbalance_count_=0;
  unsigned consecutive_=0,weak_count_=0,updates_=0;
  bool have_reference_=false,pending_=false,request_changed_=false,pending_change_=false,request_eligible_=false;
  bool high_current_initial_=false,retry_after_restore_=false,retry_wait_=false;
};
}
