#pragma once
#include <cmath>
#include <cstdint>

// Online experimental correction of body-relative camera marker spans. These
// values are not ground step lengths, and the initial control polarity is an
// empirically motivated hypothesis, not a guaranteed actuator response.
namespace foot_span_control {
struct Cycle {
  uint32_t start_ms=0, end_ms=0, positive_ms=0;
  float left=NAN, right=NAN, negative_peak=NAN, target_mj=NAN;
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
  bool selectorUnavailable(uint32_t now) {
    if(fault())return false;
    // A different ordinary selector is not a hardware fault. Carry no pending
    // request; the same learned offset may be used once a candidate can be
    // formed and a fresh new observation is available.
    pending_=false;request_eligible_=false;have_time_=true;last_limit_=now;state_=State::Collect;return true;
  }
  bool cancelUnsent(uint16_t baseline) {
    (void)baseline;
    if(fault())return false;
    // Candidate not sent: restore the ordinary selector; do not pretend that
    // its resulting command tested the requested experimental width.
    pending_=false;request_changed_=false;offset_ms_=0;requested_=baseline;
    pending_change_=confirmed_offset_!=0;
    consecutive_=0;last_usable_end_=0;have_reference_=false;state_=State::Collect;return true;
  }
  void cycle(const Cycle& c) {
    if(c.end_ms<=last_end_ || fault())return;
    last_end_=c.end_ms;
    if(!c.complete || !c.positive_executed || !supportedCurrent(c.positive_ma) ||
       c.positive_sign!=1 || c.start_ms>=c.positive_ms || c.positive_ms>=c.end_ms ||
       !std::isfinite(c.left) || !std::isfinite(c.right) || c.left<0 || c.right<0 ||
       !std::isfinite(c.negative_peak) || c.negative_peak<=0 ||
       !std::isfinite(c.target_mj) || c.target_mj<=0) {
      consecutive_=0;return;
    }
    const Actual* action=nullptr;unsigned actions=0;
    for(unsigned i=0;i<actual_count_;++i) {
      const Actual& a=actual_[(actual_next_+8-actual_count_+i)%8];
      if(a.ms>c.start_ms && a.ms<c.end_ms){action=&a;++actions;}
    }
    if(actions!=1 || !action || action->ms!=c.positive_ms || !action->executed ||
       action->ma!=c.positive_ma || action->sign!=c.positive_sign ||
       action->width!=c.positive_width || !std::isfinite(action->target_mj) ||
       std::fabs(action->target_mj-c.target_mj)>.001f) {consecutive_=0;return;}
    // Each actual command retains the input-change and feedback-change
    // timestamps that were known at that instant. Future ordinary commands
    // must not retrospectively invalidate this earlier camera cycle.
    // An older feedback generation after a new intervention is not a
    // response to the new intervention and must not increment it again.
    if(action->mode_generation!=mode_generation_ ||
       action->feedback_changed_ms!=last_feedback_changed_ms_)return;
    const bool continuous=last_usable_end_ && c.start_ms==last_usable_end_;
    consecutive_=continuous?consecutive_+1:1;last_usable_end_=c.end_ms;
    const float e=c.left-c.right, magnitude=std::fabs(e);
    if(!have_reference_) {filtered_error_=e;reference_peak_=c.negative_peak;
      reference_left_=c.left;reference_right_=c.right;have_reference_=true;last_error_magnitude_=magnitude;return;}
    filtered_error_=.5f*filtered_error_+.5f*e;
    if(offset_ms_==0) {
      reference_peak_=.8f*reference_peak_+.2f*c.negative_peak;
      reference_left_=.8f*reference_left_+.2f*c.left;
      reference_right_=.8f*reference_right_+.2f*c.right;
    }
    // A pending offset must first be applied and confirmed before any newer
    // observation is allowed to accumulate another increment.
    if(pending_change_ || consecutive_<2 ||
       (action->feedback_changed_ms && c.start_ms<=action->feedback_changed_ms))return;
    // These safeguards guard observed loss of oscillation, not evidence of
    // contact or real-world walking performance. Two bad cycles avoid a
    // single camera glitch instantly suppressing adaptation.
    const bool both_collapsed=(reference_left_+reference_right_>.4f) &&
        c.left<.35f*reference_left_ && c.right<.35f*reference_right_;
    if(c.negative_peak<reference_peak_-.8f || both_collapsed)++weak_count_;
    else weak_count_=0;
    if(weak_count_>=2) {
      const int restored=towardZero(offset_ms_);
      state_=restored==0?State::Collect:State::Restore;weak_count_=0;
      if(restored!=offset_ms_) {offset_ms_=restored;pending_change_=true;}
      return;
    }
    // Accumulate small successive worsening, not only a single large jump.
    // The same signed trend is required twice so one noisy frame does not
    // automatically reverse the control polarity.
    const float delta=magnitude-last_error_magnitude_;
    if(delta>0.f) {worse_accum_+=delta;++worse_count_;}
    else {worse_count_=0;worse_accum_=std::fmax(0.f,worse_accum_+delta);}
    last_error_magnitude_=magnitude;
    if(worse_count_>=2 && worse_accum_>=.20f && offset_ms_!=0) {
      polarity_=-polarity_;worse_count_=0;worse_accum_=0.f;
    }
    // +/-0.15 degree is a temporary deadband, NOT a support cutoff. On an
    // exact balance we retain the correction and continue observing.
    if(std::fabs(filtered_error_)<=.15f){state_=State::Hold;return;}
    const int step=(filtered_error_>0.f ? 1 : -1)*polarity_*10;
    const int proposal=offset_ms_+step;
    const int bounded=proposal < -30 ? -30 : proposal > 30 ? 30 : proposal;
    state_=bounded<0?State::ProbeMinus:bounded>0?State::ProbePlus:State::Collect;
    if(bounded!=offset_ms_) {offset_ms_=bounded;pending_change_=true;}
  }
  uint16_t limit(uint32_t now,uint16_t baseline,int16_t ma,int sign,
                 uint16_t existing_cap,bool allowed,float target_mj) {
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
    if(last_usable_end_ && (now<last_usable_end_ || now-last_usable_end_>2500)) {
      offset_ms_=towardZero(offset_ms_);state_=State::Restore;pending_change_=true;
    }
    if(!last_usable_end_)return safe;
    // If a newly requested first step has no room in that direction, try the
    // other direction explicitly instead of repeatedly counting a blocked
    // proposal. This is an exploratory intervention, not proof that the
    // opposite sign improves gait. Never exceed one 10ms feedback step.
    if(pending_change_ && confirmed_offset_==0 && offset_ms_==10 && safe==cap && safe>=20) {
      polarity_=-polarity_;offset_ms_=-10;state_=State::ProbeMinus;
      worse_count_=0;worse_accum_=0.f;
    } else if(pending_change_ && confirmed_offset_==0 && offset_ms_==-10 && safe==10 && cap>=20) {
      polarity_=-polarity_;offset_ms_=10;state_=State::ProbePlus;
      worse_count_=0;worse_accum_=0.f;
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
    have_time_=true;last_limit_=now;
    return requested_;
  }
  void confirm(uint32_t now,bool executed,int16_t ma,int sign,uint16_t width) {
    Actual& a=actual_[actual_next_];a.ms=now;a.width=width;a.ma=ma;a.sign=sign;
    a.executed=executed;a.target_mj=request_target_mj_;
    actual_next_=(actual_next_+1)%8;if(actual_count_<8)++actual_count_;
    if(pending_ && (!executed || ma!=request_ma_ || sign!=1 || width!=requested_)) {
      state_=State::Fault;pending_=false;return;
    }
    if(executed && supportedCurrent(ma) && sign==1 && width>0) {
      const bool changed=!last_actual_valid_ || last_actual_ma_!=ma || last_actual_sign_!=sign || width!=last_actual_width_;
      const bool applied_feedback=pending_change_ && request_eligible_ &&
          ma==request_ma_ && sign==1 && width==requested_ && offset_ms_!=confirmed_offset_;
      // Track all actual width/current changes, while distinguishing changes
      // in the experimental feedback offset from ordinary baseline updates.
      if(changed)last_changed_ms_=now;
      if(applied_feedback){last_feedback_changed_ms_=now;confirmed_offset_=offset_ms_;++updates_;}
      if(request_eligible_ && ma==request_ma_ && sign==1 && width==requested_)pending_change_=false;
      last_actual_width_=width;last_actual_ma_=ma;last_actual_sign_=sign;last_actual_valid_=true;
    } else if(executed && width>0) {
      const bool changed=!last_actual_valid_ || last_actual_ma_!=ma || last_actual_sign_!=sign || width!=last_actual_width_;
      if(changed)last_changed_ms_=now;
      last_actual_width_=width;last_actual_ma_=ma;last_actual_sign_=sign;last_actual_valid_=true;
    } else {
      if(last_actual_valid_)last_changed_ms_=now;
      last_actual_valid_=false;
    }
    a.input_changed_ms=last_changed_ms_;a.feedback_changed_ms=last_feedback_changed_ms_;
    a.mode_generation=mode_generation_;
    pending_=false;
  }
 private:
  static bool supportedCurrent(int16_t ma){return ma==300 || ma==350 || ma==400;}
  static int towardZero(int value){return value>10?value-10:value< -10?value+10:0;}
  struct Actual {uint32_t ms=0,input_changed_ms=0,feedback_changed_ms=0,mode_generation=0;
    uint16_t width=0;int16_t ma=0;int sign=0;bool executed=false;float target_mj=NAN;};
  Actual actual_[8];unsigned actual_next_=0,actual_count_=0;
  State state_=State::Collect;
  uint32_t last_end_=0,last_usable_end_=0,last_changed_ms_=0,last_feedback_changed_ms_=0,last_limit_=0,mode_generation_=0;
  uint16_t baseline_ms_=0,requested_=0,last_actual_width_=0;
  int16_t request_ma_=0,last_actual_ma_=0;
  int last_actual_sign_=0;
  float request_target_mj_=NAN,filtered_error_=0,reference_peak_=0,reference_left_=0,reference_right_=0,last_error_magnitude_=0,worse_accum_=0;
  int offset_ms_=0,confirmed_offset_=0,polarity_=-1;
  unsigned consecutive_=0,weak_count_=0,worse_count_=0,updates_=0;
  bool have_reference_=false,last_actual_valid_=false,pending_=false,request_changed_=false,pending_change_=false,have_time_=false,request_eligible_=false;
};
}
