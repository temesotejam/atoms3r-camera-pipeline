#pragma once
#include <cmath>
#include <cstdint>

// Positive values move energy from the positive/left-foot swing to the
// negative/right-foot swing. The mapping is experimental, not proof that
// increased side energy causes a particular ground-frame step length.
namespace foot_energy_balance {
class Controller {
 public:
  void reset(uint16_t id) { *this=Controller{};run_id_=id; }
  __attribute__((noinline)) void frame(uint16_t id,uint32_t ms,bool left_valid,bool right_valid,float left_px,float right_px) {
    if(id!=run_id_) return;
    Frame& slot=frames_[frame_next_];slot.ms=ms;slot.left=left_px;slot.right=right_px;
    slot.left_valid=left_valid;slot.right_valid=right_valid;
    frame_next_=(frame_next_+1)%8;if(frame_count_<8)++frame_count_;
    if(pending_ && ms>=pending_ms_) {
      const bool valid=pending_side_>0?left_valid:right_valid;
      const float value=pending_side_>0?left_px:right_px;
      const uint32_t distance=ms-pending_ms_;
      const uint32_t previous=candidate_ms_>pending_ms_?candidate_ms_-pending_ms_:pending_ms_-candidate_ms_;
      if(valid && std::isfinite(value) && distance<=100 && (!candidate_valid_ || distance<previous)) {
        candidate_valid_=true;candidate_ms_=ms;candidate_value_=value;
      }
      if(candidate_valid_) record(pending_side_,candidate_value_,pending_ms_,candidate_ms_);
      pending_=false;
    }
  }
  __attribute__((noinline)) void peak(int side,uint32_t ms) {
    if(side!=1 && side!=-1)return;
    if(pending_ && candidate_valid_)record(pending_side_,candidate_value_,pending_ms_,candidate_ms_);
    pending_=true;pending_side_=side;pending_ms_=ms;candidate_valid_=false;
    bool after=false;
    uint32_t best=101;
    for(unsigned i=0;i<frame_count_;++i) {
      const Frame& f=frames_[i];
      if(f.ms>=ms)after=true;
      const uint32_t distance=f.ms>ms?f.ms-ms:ms-f.ms;
      const bool valid=side>0?f.left_valid:f.right_valid;
      const float value=side>0?f.left:f.right;
      if(valid && std::isfinite(value) && distance<=100 && distance<best) {
        best=distance;candidate_valid_=true;candidate_ms_=f.ms;candidate_value_=value;
      }
    }
    // Peak detection is retrospective: the nearest frame may already have arrived.
    if(after) {
      if(candidate_valid_)record(side,candidate_value_,ms,candidate_ms_);
      pending_=false;
    }
  }
  // At most 2% per second; stale/missing observations gently unwind to zero.
  __attribute__((noinline)) void update(uint32_t ms) {
    if(have_update_ && (ms<=update_ms_ || ms-update_ms_<1000))return;
    have_update_=true;update_ms_=ms;
    float desired=0;
    bool fresh=counts_[0]==3 && counts_[1]==3 && ms>=times_[0] && ms>=times_[1] &&
        ms-times_[0]<=2500 && ms-times_[1]<=2500;
    if(fresh)for(unsigned s=0;s<2;++s)for(unsigned i=0;i<3;++i)
      if(ms<sample_times_[s][i] || ms-sample_times_[s][i]>5000)fresh=false;
    if(fresh) {
      const float l=mean(0),r=mean(1),sum=l+r;
      if(sum>=3.f && std::fabs(l-r)>=1.f)
        desired=std::fmax(-.20f,std::fmin(.20f,.25f*(l-r)/(sum+2.f)));
    }
    bias_+=std::fmax(-.02f,std::fmin(.02f,desired-bias_));
    valid_=fresh;
  }
  float factor(int side) const { return side>0?1.f-bias_:1.f+bias_; }
  float bias() const {return bias_;}
  float leftMean() const {return counts_[0]==3?mean(0):NAN;}
  float rightMean() const {return counts_[1]==3?mean(1):NAN;}
  bool valid() const {return valid_;}
 private:
  float mean(unsigned s) const{return (samples_[s][0]+samples_[s][1]+samples_[s][2])/3.f;}
  void record(int side,float value,uint32_t peak_ms,uint32_t sample_ms) {
    if(!std::isfinite(value)||std::fabs(value)>80.f ||
       (peak_ms>sample_ms?peak_ms-sample_ms:sample_ms-peak_ms)>100 ||
       (have_record_ && peak_ms<=last_peak_ms_)) return;
    const unsigned s=side>0?0u:1u;
    samples_[s][next_[s]]=std::fmax(0.f,value);sample_times_[s][next_[s]]=peak_ms;next_[s]=(next_[s]+1)%3;
    if(counts_[s]<3)++counts_[s];
    times_[s]=peak_ms;last_peak_ms_=peak_ms;have_record_=true;
  }
  uint16_t run_id_=0;float samples_[2][3]={{0,0,0},{0,0,0}};unsigned next_[2]={},counts_[2]={};
  uint32_t times_[2]={},sample_times_[2][3]={{0,0,0},{0,0,0}},last_peak_ms_=0,update_ms_=0;bool have_record_=false,have_update_=false,valid_=false;
  struct Frame { uint32_t ms=0;float left=NAN,right=NAN;bool left_valid=false,right_valid=false; };
  Frame frames_[8];unsigned frame_next_=0,frame_count_=0;
  bool pending_=false,candidate_valid_=false;
  uint32_t pending_ms_=0,candidate_ms_=0;int pending_side_=0;
  float candidate_value_=NAN,bias_=0;
};
}
