#pragma once
#include <cmath>
#include <cstdint>

// Exploratory positive-side input for a provisional physical-left mapping.
// The output is NOT a modeled 500mA response. The caller must use the real
// current/width command, keep ordinary actuator/attitude/STOP protections,
// record the actual result, and exclude the trial from model learning.
namespace positive_left_boost {
struct Input {
  uint32_t now_ms=0, opposite_peak_ms=0;
  float preset_deg=NAN, side_target_deg=NAN, opposite_peak_deg=NAN;
  float wheel_rpm=NAN, battery_mv=NAN;
  int8_t next_side=0;
  int16_t normal_ma=0;
  uint16_t normal_width_ms=0, normal_cap_ms=0, hardware_cap_ms=0;
  bool normal_valid=false, normal_motion_aligned=false;
  bool hardware_healthy=false, speed_fresh=false;
};
struct Choice { bool selected=false; int16_t current_ma=0; uint16_t width_ms=0; };

class Controller {
 public:
  void reset() { *this=Controller{}; }
  // Interrupted trial results are never accepted; ordinary control resumes.
  void interrupt() {pending_=0;baseline_count_=0;baseline_sum_=0.f;baseline_mean_=0.f;
    accepted_=false;proposed_=false;desired_ms_=100;last_good_width_=0;backoff_=false;}
  unsigned baselineCount()const{return baseline_count_;}
  bool accepted()const{return accepted_;}
  bool awaitingPeak()const{return pending_!=0;}

  Choice propose(const Input& in) {
    Choice out;
    proposed_=false;
    if(!valid(in)) {interrupt();return out;}
    // A missing expected result cannot keep the experimental state alive.
    if(pending_ && (in.now_ms<command_ms_ || in.now_ms-command_ms_>1800)) {
      interrupt();cooldown_=4;return out;
    }
    if(pending_ || cooldown_ || baseline_count_<2 ||
       baseline_mean_>=in.side_target_deg-.5f ||
       (accepted_ && last_good_peak_>=in.side_target_deg-.5f))return out;
    // First 500mA trial is 100ms; only after each matched result may the
    // duration increase to 125 and then 150ms. Physical cap always wins.
    const uint16_t max_ms=in.hardware_cap_ms<150?in.hardware_cap_ms:150;
    out.selected=true;out.current_ma=500;
    out.width_ms=desired_ms_<max_ms?desired_ms_:max_ms;
    proposed_width_=out.width_ms;proposed_=true;proposed_ms_=in.now_ms;return out;
  }

  // Called only with the actual normal command and write result. A normal
  // powered command outside the required saturated mode breaks the pair.
  void confirmNormal(const Input& in,int16_t actual_ma,uint16_t actual_width_ms,int8_t actual_sign,bool executed) {
    if(!valid(in) || !executed || actual_ma!=400 ||
       actual_sign!=1 || actual_width_ms!=in.normal_width_ms || pending_) {interrupt();return;}
    if(!baseline_count_) {reference_rpm_=in.wheel_rpm;reference_opposite_=in.opposite_peak_deg;}
    command_ms_=in.now_ms;command_width_=actual_width_ms;opposite_before_=in.opposite_peak_deg;
    pending_=1;proposed_=false;
  }

  // Confirmation must refer to the precise proposal sent, not its intent.
  void confirmTrial(const Input& in,int16_t actual_ma,uint16_t actual_width_ms,int8_t actual_sign,bool executed) {
    if(!valid(in) || !proposed_ || proposed_ms_!=in.now_ms || !executed ||
       actual_ma!=500 || actual_sign!=1 || actual_width_ms!=proposed_width_ ||
       actual_width_ms>in.hardware_cap_ms || actual_width_ms>150 || pending_) {interrupt();cooldown_=4;return;}
    command_ms_=in.now_ms;command_width_=actual_width_ms;opposite_before_=in.opposite_peak_deg;
    trial_width_=actual_width_ms;pending_=2;proposed_=false;accepted_=false;
  }

  // Feed each confirmed physical peak, including the opposite-side peak.
  // Current is acknowledged at send time; this is not measured motor torque.
  void peak(int side,uint32_t ms,float angle,bool fresh) {
    if(!pending_)return;
    if(!fresh || !std::isfinite(angle) || angle<=0.f || angle>=16.f ||
       ms<=command_ms_ || ms-command_ms_>1800 ||
       ((pending_==1 || pending_==2) && ms-command_ms_<command_width_)) {
      interrupt();cooldown_=4;return;
    }
    if(pending_==1) {
      if(side!=1) {interrupt();return;}
      if(angle>=target_snapshot_-.5f) {interrupt();return;}
      if(baseline_count_<2) {
        baseline_sum_+=angle;++baseline_count_;
        baseline_mean_=baseline_sum_/baseline_count_;
      } else {
        baseline_mean_=.5f*(baseline_mean_+angle);
      }
      pending_=0;if(cooldown_)--cooldown_;return;
    }
    if(pending_==2) {
      if(side!=1) {interrupt();cooldown_=4;return;}
      trial_positive_=angle;trial_positive_ms_=ms;pending_=3;return;
    }
    if(pending_==3) {
      if(side!=-1 || ms<=trial_positive_ms_) {interrupt();cooldown_=4;return;}
      const bool positive_improved=trial_positive_>=baseline_mean_+.2f &&
          (!last_good_width_ || trial_positive_>=last_good_peak_-.15f);
      const bool opposite_kept=angle>=opposite_before_-.8f && angle>=.7f*opposite_before_;
      pending_=0;
      if(positive_improved && opposite_kept) {
        accepted_=true;last_good_width_=trial_width_;last_good_peak_=trial_positive_;
        if(!backoff_ && trial_positive_<target_snapshot_-.5f)
          desired_ms_=trial_width_<125 ? 125 : 150;
        else desired_ms_=trial_width_;
      } else if(opposite_kept && last_good_width_ && trial_width_>last_good_width_) {
        // Retain the last observed acceptable width but don't immediately
        // retry the failed longer one. This remains an observational result.
        desired_ms_=last_good_width_;accepted_=true;backoff_=true;
      } else {interrupt();cooldown_=4;}
    }
  }

 private:
  bool valid(const Input& in) {
    if(in.preset_deg!=8.f || !std::isfinite(in.side_target_deg) ||
       in.side_target_deg<9.95f || in.side_target_deg>10.05f || in.next_side!=1 ||
       !in.hardware_healthy || !in.speed_fresh || !std::isfinite(in.wheel_rpm) ||
       !std::isfinite(in.battery_mv) || in.battery_mv<=0.f ||
       !std::isfinite(in.opposite_peak_deg) || in.opposite_peak_deg<=0.f || in.opposite_peak_deg>=16.f ||
       !in.opposite_peak_ms || in.now_ms<=in.opposite_peak_ms ||
       in.now_ms-in.opposite_peak_ms>1500 || !in.normal_valid || !in.normal_motion_aligned ||
       in.normal_ma!=400 || in.normal_cap_ms<80 || in.hardware_cap_ms<100 ||
       in.normal_width_ms>in.normal_cap_ms || in.normal_width_ms>in.hardware_cap_ms ||
       in.normal_width_ms+5<in.normal_cap_ms) return false;
    // This is a comparability gate, never a sensor-range reason to stop
    // normal control. Other body/contact changes can still confound results.
    if(baseline_count_ && (std::fabs(in.wheel_rpm-reference_rpm_)>100.f ||
       std::fabs(in.opposite_peak_deg-reference_opposite_)>.8f))return false;
    target_snapshot_=in.side_target_deg;return true;
  }
  float baseline_sum_=0.f,baseline_mean_=0.f,opposite_before_=0.f;
  float trial_positive_=0.f,target_snapshot_=NAN,reference_rpm_=0.f,reference_opposite_=0.f,last_good_peak_=0.f;
  uint32_t command_ms_=0,trial_positive_ms_=0,proposed_ms_=0;
  uint16_t command_width_=0,proposed_width_=0,trial_width_=0,desired_ms_=100,last_good_width_=0;
  uint8_t baseline_count_=0,pending_=0,cooldown_=0;
  bool accepted_=false,proposed_=false,backoff_=false;
};
}
