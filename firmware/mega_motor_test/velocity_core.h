#pragma once
#include <stdint.h>
#include <math.h>

// v8: signed PWM velocity servo, 1 kHz. Driver order L/M2, R/M1.
// Position references are raw uint32 encoder counts (rollover is intentional).
struct VelocityControl {
  static const uint16_t PERIOD_MS = 1, COUNTS_PER_REV = 2652;
  static const int16_t MAX_RPM_CENTI = 10000, MAX_PWM = 650;
  enum State : uint8_t { DISABLED = 0, TRACK = 1, ZERO_SPEED = 2, HOLD = 3 };
  struct Gains { int32_t kp=0, ki=0, kd=0, slope=0, offset=0; bool configured=false; };
  struct HoldConfig {
    bool enabled=false;
    uint16_t kpCenti=0, speedCenti=0, pwm=0, deadband=2, maxError=0, maxMs=0, saturationMs=0;
  } hold[2];
  Gains gains[2][2], zeroGains[2], holdGains[2];
  uint16_t holdPositionKpQ8[2]={0,0};
  const int8_t encoderSign[2];
  explicit VelocityControl(int8_t left=1, int8_t right=1) : encoderSign{left,right} {}
  bool active=false, seeded=false;
  int16_t target[2]={0,0}, output[2]={0,0}, brakeLimit[2]={150,150};
  uint8_t state[2]={DISABLED,DISABLED}, limited[2]={0,0};
  bool settled[2]={false,false};
  int32_t measuredQ16[2]={0,0}, fastQ16[2]={0,0}, measuredCenti[2]={0,0};
  int32_t integral[2]={0,0}, derivative[2]={0,0}, filterRemainder[2]={0,0};
  int32_t integralRemainder[2]={0,0}, derivativeRemainder[2]={0,0};
  int32_t feedforward[2]={0,0}, positionError[2]={0,0};
  uint32_t lastCount[2]={0,0}, lastInvalid[2]={0,0}, lastEdge[2]={0,0};
  uint32_t holdReference[2]={0,0}, holdStarted[2]={0,0}, stableOrigin[2]={0,0}, stableSince[2]={0,0};
  int16_t stableLow[2]={0,0}, stableHigh[2]={0,0};
  uint32_t lastUpdate=0, lastProgress[2]={0,0}, targetSince[2]={0,0};
  uint16_t badDirectionMs[2]={0,0}, saturatedMs[2]={0,0}, overMs[2]={0,0};

  static int32_t clip(int32_t v,int32_t lo,int32_t hi) { return v<lo?lo:v>hi?hi:v; }
  static int8_t sign(int32_t v) { return v>0?1:v<0?-1:0; }
  template<uint8_t Bits> static int32_t shift(int32_t value) {
    const uint32_t n=value<0?0UL-static_cast<uint32_t>(value):static_cast<uint32_t>(value);
    const int32_t q=static_cast<int32_t>(n>>Bits); return value<0?-q:q;
  }
  template<uint32_t Divisor> static int32_t divide(int32_t value) {
    const uint32_t n=value<0?0UL-static_cast<uint32_t>(value):static_cast<uint32_t>(value);
    const uint32_t reciprocal=static_cast<uint32_t>((static_cast<uint64_t>(1)<<32)/Divisor);
    uint32_t q=(static_cast<uint64_t>(n)*reciprocal)>>32;
    if(n-q*Divisor>=Divisor) ++q;
    return value<0?-static_cast<int32_t>(q):static_cast<int32_t>(q);
  }
  static int32_t delta(uint32_t n,uint32_t p) {
    const uint32_t d=n-p; return d<=0x7fffffffUL?static_cast<int32_t>(d):-1-static_cast<int32_t>(0xffffffffUL-d);
  }
  int16_t rawMeasuredCenti(uint8_t i) const { return measuredCenti[i]*encoderSign[i]; }
  static void scaled(Gains& g,int32_t kp,int32_t ki,int32_t kd,int32_t slope,int32_t offset) {
    g.kp=(kp*512+500)/1000; g.ki=(static_cast<int64_t>(ki)*262144+500000)/1000000;
    g.kd=kd; g.slope=(slope*512+500)/1000; g.offset=(static_cast<int64_t>(offset)*65536+500)/1000;
    g.configured=true;
  }
  bool configureScaled(uint8_t channel,int8_t direction,int32_t kp,int32_t ki,int32_t kd,int32_t slope,int32_t offset) {
    if(active||channel>1||(direction!=-1&&direction!=1)||kp<0||kp>20000||ki<0||ki>50000||
       kd<0||kd>2000||slope<1||slope>20000||offset<0||offset>200000) return false;
    scaled(gains[channel][direction>0],kp,ki,kd,slope,offset); return true;
  }
  bool configure(uint8_t c,int8_t d,float kp,float ki,float kd,float slope,float offset) {
    if(!(kp>=0&&kp<=20)||!(ki>=0&&ki<=50)||!(kd>=0&&kd<=2)||!(slope>0&&slope<=20)||!(offset>=0&&offset<=200)) return false;
    return configureScaled(c,d,kp*1000+.5f,ki*1000+.5f,kd*1000+.5f,slope*1000+.5f,offset*1000+.5f);
  }
  bool configureZero(uint8_t c,int32_t kp,int32_t ki,uint16_t pwm) {
    if(active||c>1||kp<1||kp>20000||ki<0||ki>50000||pwm<1||pwm>150) return false;
    scaled(zeroGains[c],kp,ki,0,0,0); brakeLimit[c]=pwm; return true;
  }
  bool configureHoldPI(uint8_t c,int32_t kp,int32_t ki) {
    if(active||c>1||kp<1||kp>60000||ki<0||ki>50000) return false;
    scaled(holdGains[c],kp,ki,0,0,0); return true;
  }
  bool configureHoldPosition(uint8_t c,int32_t kpMilli) {
    if(active||c>1||kpMilli<0||kpMilli>30000) return false;
    holdPositionKpQ8[c]=(kpMilli*256L+500)/1000; return true;
  }
  bool configureHold(uint8_t c,bool enabled,uint16_t kp,uint16_t speed,uint16_t pwm,uint16_t deadband,
                     uint16_t maxError,uint16_t maxMs,uint16_t saturationMs) {
    if(active||c>1||deadband>20) return false;
    if(enabled&&(!kp||kp>1000||!speed||speed>500||!pwm||pwm>MAX_PWM||maxError<=deadband||maxError>2652||
                 (maxMs&&maxMs<100)||maxMs>60000||saturationMs<100||saturationMs>5000||
                 !holdGains[c].configured)) return false;
    HoldConfig& h=hold[c]; h.enabled=enabled; h.kpCenti=kp; h.speedCenti=speed; h.pwm=pwm;
    h.deadband=deadband; h.maxError=maxError; h.maxMs=maxMs; h.saturationMs=saturationMs; return true;
  }
  bool ready(int16_t l,int16_t r) const {
    if(l < -MAX_RPM_CENTI||l>MAX_RPM_CENTI||r < -MAX_RPM_CENTI||r>MAX_RPM_CENTI) return false;
    for(uint8_t i=0;i<2;++i)
      if(!gains[i][0].configured||!gains[i][1].configured||!zeroGains[i].configured||
         (hold[i].enabled&&!holdGains[i].configured)) return false;
    return true;
  }
  void clearIntegral(uint8_t i) { integral[i]=integralRemainder[i]=0; }
  void stop() {
    active=false;
    for(uint8_t i=0;i<2;++i) {
      target[i]=output[i]=0; state[i]=DISABLED; settled[i]=false; limited[i]=0;
      clearIntegral(i); derivative[i]=derivativeRemainder[i]=0;
      badDirectionMs[i]=saturatedMs[i]=0; positionError[i]=0;
    }
  }
  void resetWindow(uint8_t i,uint32_t now) {
    stableSince[i]=now; stableOrigin[i]=lastCount[i]; stableLow[i]=stableHigh[i]=0; settled[i]=false;
  }
  bool setTargets(int16_t l,int16_t r,uint32_t now) {
    if(!active||!ready(l,r)) return false;
    const int16_t next[2]={l,r};
    for(uint8_t i=0;i<2;++i) {
      if(next[i]==target[i]) continue; // A lease renewal must never re-anchor HOLD.
      if(!next[i]||sign(next[i])!=sign(target[i])) {
        clearIntegral(i); derivative[i]=derivativeRemainder[i]=0;
      }
      if(!target[i]&&next[i]) lastProgress[i]=now;
      targetSince[i]=now; target[i]=next[i]; state[i]=next[i]?TRACK:ZERO_SPEED;
      saturatedMs[i]=badDirectionMs[i]=0; positionError[i]=0; resetWindow(i,now);
      const Gains& g=gains[i][next[i]>0];
      const int32_t rpmQ8=divide<100>(static_cast<int32_t>(next[i])*256);
      feedforward[i]=next[i]?sign(next[i])*g.offset+shift<1>(g.slope*rpmQ8):0;
    }
    return true;
  }
  void start(int16_t l,int16_t r,const uint32_t* counts,const uint32_t* invalid,uint32_t now) {
    stop();
    if(!ready(l,r)) return;
    if(!seeded) {
      seeded=true; lastUpdate=now;
      for(uint8_t i=0;i<2;++i) { lastCount[i]=counts[i]; lastInvalid[i]=invalid[i]; }
    }
    active=true;
    for(uint8_t i=0;i<2;++i) {
      lastProgress[i]=targetSince[i]=now; state[i]=ZERO_SPEED; resetWindow(i,now);
    }
    setTargets(l,r,now);
  }
  bool due(uint32_t now) const { return !seeded||static_cast<uint32_t>(now-lastUpdate)>=PERIOD_MS; }

  // monitorRaw applies encoder/overspeed guards to diagnostic PWM too.
  uint8_t update(const uint32_t* counts,const uint32_t* invalid,uint32_t now,bool monitorRaw=false) {
    if(!seeded) {
      seeded=true; lastUpdate=now;
      for(uint8_t i=0;i<2;++i) { lastCount[i]=counts[i]; lastInvalid[i]=invalid[i]; }
      return 0;
    }
    const uint32_t elapsed=now-lastUpdate;
    if(!elapsed) return 0;
    if((active||monitorRaw)&&elapsed!=PERIOD_MS) return 7;
    lastUpdate=now;
    for(uint8_t i=0;i<2;++i) {
      int32_t steps=delta(counts[i],lastCount[i]);
      const bool invalidChanged=invalid[i]!=lastInvalid[i];
      lastCount[i]=counts[i]; lastInvalid[i]=invalid[i];
      if((active||monitorRaw)&&invalidChanged) return 6;
      if((active||monitorRaw)&&(steps < -16||steps>16)) return 5;
      steps=clip(steps,-16,16)*encoderSign[i];
      if(steps) lastEdge[i]=lastProgress[i]=now;
      const int32_t previous=measuredQ16[i];
      const int32_t numerator=(steps*1482715L-measuredQ16[i])*5+filterRemainder[i];
      const int32_t increment=shift<8>(numerator);
      filterRemainder[i]=numerator-increment*256;
      measuredQ16[i]=clip(measuredQ16[i]+increment,-300*65536L,300*65536L);
      const int32_t fastBefore=fastQ16[i];
      fastQ16[i]+=shift<3>(steps*1482715L-fastQ16[i]); // 7.5 ms observer for braking.
      measuredCenti[i]=shift<16>(measuredQ16[i]*100);
      if((active||monitorRaw)&&(measuredCenti[i]>MAX_RPM_CENTI||measuredCenti[i] < -MAX_RPM_CENTI)) ++overMs[i];
      else overMs[i]=0;
      if(overMs[i]>=100) return 11; // sustained measured overspeed; not an instantaneous physical speed guarantee
      if(!active) continue;
      int32_t desired=divide<100>(static_cast<int32_t>(target[i])*256);
      int32_t measured=measuredQ16[i];
      const bool zero=!target[i];
      if(zero) {
        // Capture this control tick's encoder snapshot as soon as the zero
        // target is applied. Renewing zero must never move the HOLD origin.
        if(state[i]!=HOLD&&hold[i].enabled) {
          holdReference[i]=counts[i]; holdStarted[i]=now; state[i]=HOLD; clearIntegral(i);
        }
        // First reverse edge: discard stale forward velocity instead of
        // continuing to apply the old braking polarity into reverse motion.
        if(state[i]!=HOLD&&steps&&sign(steps)*sign(fastBefore)<0&&sign(output[i])==sign(steps)) {
          fastQ16[i]=0; clearIntegral(i);
        }
        if(static_cast<uint32_t>(now-lastEdge[i])>=8) fastQ16[i]=0;
        measured=fastQ16[i];
        // Standstill is telemetry/cleanup information, not a HOLD entry gate.
        const int32_t d=delta(counts[i],stableOrigin[i]);
        if(d < -32767||d>32767||measured > 65536L||measured < -65536L) resetWindow(i,now);
        else {
          if(d<stableLow[i]) stableLow[i]=d;
          if(d>stableHigh[i]) stableHigh[i]=d;
          if(stableHigh[i]-stableLow[i]>2) resetWindow(i,now);
          else settled[i]=static_cast<uint32_t>(now-stableSince[i])>=100;
        }
        if(state[i]==HOLD) {
          const HoldConfig& h=hold[i];
          const int32_t rawError=delta(holdReference[i],counts[i]);
          if(rawError > h.maxError||rawError < -static_cast<int32_t>(h.maxError)||
             (h.maxMs&&static_cast<uint32_t>(now-holdStarted[i])>=h.maxMs)) return 10;
          positionError[i]=rawError*encoderSign[i];
          const int32_t e=positionError[i];
          const int32_t correction=(e>h.deadband||e < -static_cast<int32_t>(h.deadband))?
              clip(e*h.kpCenti,-static_cast<int32_t>(h.speedCenti),h.speedCenti):0;
          desired=divide<100>(correction*256);
        }
        if(!desired&&measured==0) clearIntegral(i);
      }
      const Gains& g=state[i]==HOLD?holdGains[i]:zero?zeroGains[i]:gains[i][target[i]>0];
      const int32_t error=desired-shift<8>(measured);
      int32_t dTerm=0;
      if(!zero&&g.kd) {
        const int32_t n=(measuredQ16[i]-previous-derivative[i])*5+derivativeRemainder[i];
        const int32_t change=shift<9>(n);
        derivativeRemainder[i]=n-change*512;
        derivative[i]=clip(derivative[i]+change,-200000L,200000L); dTerm=g.kd*derivative[i];
      }
      // High HOLD stiffness must not overflow at a fast externally imposed
      // encoder velocity. 60 PWM/RPM * 100 RPM is already far beyond the
      // actuator cap; clip only this P operand, preserving its sign and the I
      // error. Driving and zero-speed braking arithmetic remain unchanged.
      const int32_t pError=state[i]==HOLD?clip(error,-100*256L,100*256L):error;
      int32_t base=(zero?0:feedforward[i])+shift<1>(g.kp*pError)-dTerm;
      if(state[i]==HOLD) {
        const int32_t e=positionError[i];
        if(e>hold[i].deadband||e < -static_cast<int32_t>(hold[i].deadband)) {
          // Position stiffness is independent of velocity damping. Bound the
          // Q8 product before Q16 conversion (2652 counts * 30 PWM/count).
          const int32_t positionQ8=clip(e*holdPositionKpQ8[i],-MAX_PWM*256L,MAX_PWM*256L);
          base+=positionQ8*256L;
        }
      }
      int16_t limit=state[i]==HOLD?hold[i].pwm:zero?brakeLimit[i]:MAX_PWM;
      // Torque-producing polarity opposing measured motion uses the brake cap,
      // including commanded reversals and overspeed at a nonzero target.
      if(state[i]!=HOLD&&sign(base+integral[i])*sign(fastQ16[i])<0&&limit>brakeLimit[i]) limit=brakeLimit[i];
      const int32_t bound=static_cast<int32_t>(limit)*65536L;
      const int32_t iNumerator=g.ki*error+integralRemainder[i];
      const int32_t iIncrement=shift<10>(iNumerator);
      const int32_t candidate=clip(integral[i]+iIncrement,-200*65536L,200*65536L);
      if(!((base+candidate>bound&&error>0)||(base+candidate < -bound&&error<0))) {
        integral[i]=candidate; integralRemainder[i]=iNumerator-iIncrement*1024;
      } else integralRemainder[i]=0;
      const int32_t demand=base+integral[i], pwm=clip(demand,-bound,bound);
      limited[i]=demand!=pwm;
      output[i]=shift<16>(pwm+(pwm<0?-32768L:32768L));
      if(zero) {
        if(state[i]==HOLD&&limited[i]) ++saturatedMs[i]; else saturatedMs[i]=0;
        if(state[i]==HOLD&&saturatedMs[i]>=hold[i].saturationMs) return 10;
      } else {
        const int32_t actual=sign(target[i])*measuredCenti[i];
        if(static_cast<uint32_t>(now-targetSince[i])>500&&actual < -200) ++badDirectionMs[i]; else badDirectionMs[i]=0;
        if(badDirectionMs[i]>=100) return 5;
        if(static_cast<uint32_t>(now-lastProgress[i])>=500&&(output[i]>150||output[i] < -150)) return 4;
        if(limited[i]&&(error>10*256||error < -10*256)) ++saturatedMs[i]; else saturatedMs[i]=0;
        if(saturatedMs[i]>=2000) return 8;
      }
    }
    return 0;
  }
};
