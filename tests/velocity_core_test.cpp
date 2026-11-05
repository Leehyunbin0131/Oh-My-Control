#include <cassert>
#include <cmath>
#include <initializer_list>
#include <iostream>
#include "../firmware/mega_motor_test/test_core.h"
#include "../firmware/mega_motor_test/velocity_core.h"

void configure(VelocityControl& v) {
  for(int i=0;i<2;++i) {
    for(int s:{-1,1}) assert(v.configure(i,s,1.5f,8.f,.03f,6.3f,35));
    assert(v.configureZero(i,2000,0,150));
  }
}
struct Plant {
  VelocityControl v{1,-1};
  uint32_t counts[2]={0xfffffffcUL,4}, invalid[2]={0,0}, t=0;
  double rpm[2]={0,0}, position[2]={-4,4};
  double friction[2]={45,25}, slope[2]={6.5,6.1}, tau[2]={.12,.18};
  Plant() { configure(v); }
  uint8_t step() {
    for(int i=0;i<2;++i) {
      double driven=std::fmax(0,(std::abs(v.output[i])-friction[i])/slope[i]);
      if(v.output[i]<0) driven=-driven;
      rpm[i]+=.001/tau[i]*(driven-rpm[i]);
      position[i]+=rpm[i]*2652/60000*(i?-1:1);
      counts[i]=static_cast<uint32_t>(static_cast<int32_t>(std::round(position[i])));
    }
    return v.update(counts,invalid,++t);
  }
  void run(unsigned ms) { for(unsigned i=0;i<ms;++i) { const auto f=step(); if(f) std::cerr<<"plant fault "<<int(f)<<" at "<<t<<"\n"; assert(!f); } }
};
int main() {
  uint32_t random=14231;
  for(int n=0;n<10000;++n) {
    random=random*1664525UL+1013904223UL;
    const int32_t v=VelocityControl::delta(random,0);
    assert(VelocityControl::shift<16>(v)==v/65536);
    assert(VelocityControl::divide<100>(v)==v/100);
  }
  assert(VelocityControl::delta(2,0xfffffffeUL)==4);
  assert(VelocityControl::delta(0xfffffffeUL,2)==-4);
  VelocityControl limits;
  assert(!limits.ready(0,0)); configure(limits);
  assert(limits.ready(0,0)); assert(limits.ready(10000,-10000));
  assert(!limits.ready(10001,0)); assert(!limits.ready(0,-10001));
  for(int sign:{-1,1}) {
    Plant p; p.v.start(sign*7200,sign*7200,p.counts,p.invalid,p.t);
    p.run(4000);
    for(int i=0;i<2;++i) assert(std::abs(p.rpm[i]-sign*72)<.8);
    assert(p.v.rawMeasuredCenti(0)*sign>7000&&p.v.rawMeasuredCenti(1)*sign < -7000);
    const auto measured=p.v.measuredQ16[0];
    assert(p.v.setTargets(sign*5000,sign*5000,p.t));
    assert(p.v.measuredQ16[0]==measured); // target change preserves observer
    p.run(2500);
    for(int i=0;i<2;++i) assert(std::abs(p.rpm[i]-sign*50)<.8);
    p.v.setTargets(0,0,p.t);
    p.step();
    for(int i=0;i<2;++i) assert(p.v.output[i]*sign<0&&std::abs(p.v.output[i])<=150);
    for(int n=0;n<1200;++n) { if(n%20==0) assert(p.v.setTargets(0,0,p.t)); p.run(1); }
    assert(p.v.active&&p.v.settled[0]&&p.v.settled[1]);
    for(int i=0;i<2;++i) assert(std::abs(p.rpm[i])<.1&&p.v.output[i]==0);
    assert(p.v.state[0]==VelocityControl::ZERO_SPEED);
    p.v.setTargets(-sign*3000,0,p.t); p.run(2500);
    assert(std::abs(p.rpm[0]+sign*30)<.8&&std::abs(p.rpm[1])<.1);
    p.v.setTargets(sign*3000,-sign*3000,p.t); p.run(2500);
    assert(std::abs(p.rpm[0]-sign*30)<.8&&std::abs(p.rpm[1]+sign*30)<.8);
    p.v.stop(); assert(!p.v.active&&!p.v.output[0]&&!p.v.output[1]);
  }
  // Hold enters after standstill, never at the original stop command position.
  Plant h;
  for(int i=0;i<2;++i) {
    assert(h.v.configureZero(i,8000,10000,150));
    assert(h.v.configureHold(i,true,25,500,120,2,100,10000,1000));
  }
  h.v.start(3000,3000,h.counts,h.invalid,0); h.run(2500);
  const auto stopPosition=h.counts[0]; h.v.setTargets(0,0,h.t); h.run(1500);
  assert(h.v.state[0]==VelocityControl::HOLD&&h.v.state[1]==VelocityControl::HOLD);
  const auto held=h.v.holdReference[0]; assert(held!=stopPosition);
  for(int n=0;n<20;++n) { h.v.setTargets(0,0,h.t); h.run(20); assert(h.v.holdReference[0]==held); }
  for(int n=0;n<20;++n) { h.position[0]+=1; h.run(1); } // finite external displacement
  h.run(4000);
  assert(std::abs(VelocityControl::delta(h.counts[0],held))<=3);
  h.v.setTargets(3000,0,h.t); assert(h.v.state[0]==VelocityControl::TRACK&&h.v.state[1]==VelocityControl::HOLD);
  h.run(400); assert(h.rpm[0]>20);

  // No movement is valid at zero, but invalid feedback, leases and limits fail.
  for(int kind=0;kind<6;++kind) {
    VelocityControl v; configure(v); uint32_t c[2]={0,0}, bad[2]={0,0};
    if(kind==4) assert(v.configureHold(0,true,25,500,120,2,20,500,100));
    v.start(kind==0||kind==4?0:7200,0,c,bad,0);
    uint8_t fault=0;
    if(kind==0) { for(int t=1;t<2000;++t) assert(v.update(c,bad,t)==0); assert(v.settled[0]); }
    if(kind==1) { bad[0]=1; assert(v.update(c,bad,1)==ENCODER_INVALID); }
    if(kind==2) assert(v.update(c,bad,2)==CONTROL_TIMING);
    if(kind==3) { for(int t=1;t<1000&&!fault;++t) fault=v.update(c,bad,t); assert(fault==ENCODER_STALL); }
    if(kind==4) { for(int t=1;t<150;++t) assert(!v.update(c,bad,t)); c[0]+=11; assert(!v.update(c,bad,150)); c[0]+=10; assert(v.update(c,bad,151)==HOLD_LIMIT); }
    if(kind==5) { for(int t=1;t<500&&!fault;++t) { c[0]+=5; fault=v.update(c,bad,t); } assert(fault==OVERSPEED); }
  }
  // Raw PWM uses the same physical overspeed guard, even without a configured PID.
  VelocityControl raw; uint32_t c[2]={0,0}, bad[2]={0,0}; raw.update(c,bad,0);
  uint8_t fault=0;
  for(int t=1;t<500&&!fault;++t) { c[1]+=5; fault=raw.update(c,bad,t,true); }
  assert(fault==OVERSPEED);
  // A renewed lease updates targets without resetting a finite drive or a fault.
  MotionGuard guard;
  assert(!guard.setSpeed(0,0,300,0)); assert(guard.arm(0));
  assert(!guard.setSpeed(10001,0,300,0)); assert(!guard.setSpeed(0,0,301,0));
  assert(guard.setSpeed(10000,-10000,300,0));
  assert(guard.setSpeed(0,0,300,200)); guard.heartbeat=499; assert(!guard.tick(499));
  guard.heartbeat=500; assert(guard.tick(500)&&guard.fault==COMMAND_EXPIRED);
  guard.stop(); assert(!guard.arm(501)); assert(guard.clearFault()); assert(guard.arm(502));
  assert(guard.setSpeed(0,0,100,0xfffffffeUL)); assert(!guard.tick(50)); assert(guard.tick(98));
  std::cout<<"signed velocity, standstill, hold, limits and lease tests passed\n";
}
