#pragma once
#include <cmath>
#include <cstdint>

// Experimental feedback from body-relative camera motion, not ground stride.
// Physical response direction is learned by bounded trials, not assumed.
namespace long_foot_balance {
class Controller {
 public:
  static constexpr unsigned kWindow=5;
  static constexpr uint32_t kMaxAgeMs=12000;
  static constexpr int kStepMs=5;
  static constexpr int kMaxOffsetMs=20;
  enum class State : uint8_t { Collect, Trial, Hold, Restore };
  void reset(){*this=Controller{};}
  State state()const{return state_;}
  unsigned samples()const{return n_;}
  int offset()const{return offset_;}
  int requestedOffset()const{return requested_;}
  float error()const{return error_;}
  // 'valid' is an image-quality decision; a missing image is not zero swing.
  // 'healthy' is independent actuator/IMU/run safety, not an image miss.
  void observe(uint32_t start,uint32_t now,float left,float right,float plus_peak,float minus_peak,bool valid,bool healthy) {
    if(!healthy) {clearForSafety();return;}
    if(!now || start>=now || (changed_at_ && now<=changed_at_) ||
       !std::isfinite(plus_peak) || !std::isfinite(minus_peak) || plus_peak<=0 || minus_peak<=0)return;
    // A serious drop in either rocking peak must unwind without waiting for
    // a complete foot-response window. Existing emergency limits remain primary.
    if((offset_!=0 || state_==State::Trial) && ref_plus_>0 && ref_minus_>0 &&
       (plus_peak<ref_plus_*.65f || minus_peak<ref_minus_*.65f)) {
      if(offset_==0) {clearForSafety();return;}
      requested_=0;state_=State::Restore;pending_=true;clearSamples();return;
    }
    if(changed_at_ && start<=changed_at_)return;
    if(!valid || !std::isfinite(left) || !std::isfinite(right) || left<0 || right<0)return;
    if(last_sample_ && now<=last_sample_)return;
    if(last_sample_ && now-last_sample_>kMaxAgeMs)clearSamples();
    last_sample_=now;
    if(pending_)return;
    if(n_ && (now<window_start_ || now-window_start_>kMaxAgeMs))clearSamples();
    if(!n_)window_start_=start;
    left_sum_+=left;right_sum_+=right;plus_sum_+=plus_peak;minus_sum_+=minus_peak;
    ++n_;
    if(n_<kWindow || state_==State::Restore)return;
    const float inv=1.f/static_cast<float>(n_);
    const float l=left_sum_*inv,r=right_sum_*inv,p=plus_sum_*inv,m=minus_sum_*inv;
    // Sum spans before dividing so tiny noisy cycles do not have equal
    // influence to strong, well-observed foot motion.
    const float e=(left_sum_-right_sum_)/(left_sum_+right_sum_+.1f*kWindow);error_=e;
    if(l+r<.3f) {clearSamples();return;}
    if(state_==State::Trial) {
      const float weak=right_weak_ ? r:l;
      const bool better=std::fabs(e)+.02f<std::fabs(ref_error_) && weak>=ref_weak_+.05f &&
        weak>=ref_weak_*1.05f && p>=ref_plus_*.85f && m>=ref_minus_*.85f;
      if(!better) {requested_=previous_;state_=State::Restore;pending_=true;polarity_=-polarity_;clearSamples();return;}
      state_=State::Hold;
    }
    // Deadband and bounded integral steps. Only an actual confirmed change
    // starts the new observation period; no credit for a clipped proposal.
    if(std::fabs(e)>.10f) {
      const int sign=e>0.f ? 1 : -1;
      if(decision_sign_ && sign!=decision_sign_)polarity_=-polarity_;
      decision_sign_=sign;
      const int next=offset_+polarity_*kStepMs;
      if(next>=-kMaxOffsetMs && next<=kMaxOffsetMs) {
        ref_error_=e;right_weak_=r<=l;ref_weak_=right_weak_?r:l;ref_plus_=p;ref_minus_=m;
        previous_=offset_;requested_=next;pending_=true;proposal_at_=now;
      }
    }
    clearSamples();
  }
  // The returned value is a pre-guard candidate. Always compare against the
  // ordinary baseline passed through an independent copy of the same guard.
  uint16_t propose(uint32_t now,uint16_t baseline,uint16_t cap,bool healthy) {
    if(!healthy || !last_sample_ || now<last_sample_ || now-last_sample_>kMaxAgeMs) {
      clearForSafety();return baseline;
    }
    if(state_==State::Trial && changed_at_ &&
       (now<changed_at_ || now-changed_at_>kMaxAgeMs)) {
      clearForSafety();return baseline;
    }
    if(pending_ && state_!=State::Restore && proposal_at_ &&
       (now<proposal_at_ || now-proposal_at_>kMaxAgeMs)) {
      requested_=offset_;pending_=false;state_=State::Collect;clearSamples();
    }
    const int delta=pending_?requested_:offset_;
    int out=static_cast<int>(baseline)+delta;
    if(out<0)out=0;
    if(out>static_cast<int>(cap))out=cap;
    return static_cast<uint16_t>(out);
  }
  uint16_t current(uint16_t baseline,uint16_t cap)const {
    int out=static_cast<int>(baseline)+offset_;
    if(out<0)out=0;
    if(out>static_cast<int>(cap))out=cap;
    return static_cast<uint16_t>(out);
  }
  // current and actual are AFTER identical guards; acceptance indicates
  // command start accepted, not measured current or completed motor motion.
  void confirm(uint32_t now,uint16_t current_guarded,uint16_t actual,bool accepted) {
    if(!pending_ || !accepted || !now)return;
    if(state_==State::Restore) {
      if((requested_>offset_ && actual<current_guarded) ||
         (requested_<offset_ && actual>current_guarded))return;
      offset_=requested_;pending_=false;state_=State::Collect;changed_at_=now;clearSamples();
      return;
    }
    // A candidate clipped to the same guarded command is no experiment.
    if((requested_>offset_ && actual<=current_guarded) ||
       (requested_<offset_ && actual>=current_guarded))return;
    const int actual_change=static_cast<int>(actual)-static_cast<int>(current_guarded);
    if(actual_change>kStepMs || actual_change< -kStepMs)return;
    offset_=requested_;pending_=false;state_=State::Trial;changed_at_=now;clearSamples();
  }
 private:
  void clearSamples(){n_=0;window_start_=0;left_sum_=right_sum_=plus_sum_=minus_sum_=0;}
  void clearForSafety(){*this=Controller{};}
  uint32_t last_sample_=0,changed_at_=0,proposal_at_=0,window_start_=0;unsigned n_=0;
  float left_sum_=0,right_sum_=0,plus_sum_=0,minus_sum_=0,error_=0;
  float ref_error_=0,ref_weak_=0,ref_plus_=0,ref_minus_=0;
  int offset_=0,requested_=0,previous_=0,polarity_=1,decision_sign_=0;
  bool pending_=false,right_weak_=false;
  State state_=State::Collect;
};
}
