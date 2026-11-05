#include <cassert>
#include <iostream>
#include <string>
#include "../firmware/mega_motor_test/mega_motor_test.ino"

void tick(unsigned ms=1) {
  for(unsigned i=0;i<ms;++i) { nativeUs+=1000; TIMER1_COMPA_vect(); loop(); }
}
std::string packet(uint32_t seq,const std::string& operation) {
  const std::string body="OMR1,C,"+std::to_string(seq)+","+operation;
  char crc[8]; snprintf(crc,sizeof(crc),"*%04X\n",frameCrc(body.c_str(),body.size()));
  return body+crc;
}
void command(uint32_t seq,const std::string& op,bool fragmented=false) {
  const auto wire=packet(seq,op);
  for(char c:wire) { Serial.receive(c); if(fragmented) loop(); }
  while(Serial.available()) loop();
}
uint32_t nextSeq() { return lastSequence+1; }
void testHoldProtocol() {
  // Enabling without its independent speed PI must fail before ARM.
  command(nextSeq(),"HOLD,0,1,25,500,120,2,100,0,1000");
  assert(motion.fault==BAD_COMMAND&&!velocity.hold[0].enabled);
  tick(110); command(nextSeq(),"CLEAR_FAULT");
  for(const char* invalid : {"HOLD_PI,2,8000,10000", "HOLD_PI,0,0,10000",
                             "HOLD_PI,0,8000,50001", "HOLD_PI,0,8000,10000,"}) {
    command(nextSeq(),invalid); assert(motion.fault==BAD_COMMAND&&!motion.armed);
    tick(110); command(nextSeq(),"CLEAR_FAULT");
  }
  for(unsigned i=0;i<2;++i) {
    command(nextSeq(),"HOLD_PI,"+std::to_string(i)+",8000,10000");
    command(nextSeq(),"HOLD,"+std::to_string(i)+",1,25,500,120,2,100,0,1000");
    assert(velocity.hold[i].enabled&&velocity.zeroGains[i].ki==0);
  }
  command(nextSeq(),"ARM"); command(nextSeq(),"SET_SPEED,0,0,300"); tick(120);
  assert(velocity.state[0]==VelocityControl::HOLD&&velocity.state[1]==VelocityControl::HOLD);
  const uint32_t left=velocity.holdReference[0],right=velocity.holdReference[1];
  // Continuous 50 Hz zero targets keep the original position past 10 seconds.
  for(unsigned n=0;n<600;++n) {
    command(nextSeq(),"SET_SPEED,0,0,300"); tick(20);
    assert(!motion.fault&&velocity.holdReference[0]==left&&velocity.holdReference[1]==right);
  }
  encL.count+=6; encR.count+=8; tick(10);
  assert(velocity.positionError[0]==-8&&velocity.positionError[1]==6);
  assert(velocity.output[0]<0&&velocity.output[1]>0); // opposite encoder signs
  command(nextSeq(),"SET_SPEED,0,0,300"); tick(20);
  assert(velocity.holdReference[0]==left&&velocity.holdReference[1]==right);
  command(nextSeq(),"SET_SPEED,-3000,0,300"); tick();
  assert(velocity.state[0]==VelocityControl::TRACK&&velocity.state[1]==VelocityControl::HOLD);
  assert(velocity.holdReference[1]==right);
  // A live gain edit is rejected and clears both torque-producing outputs.
  command(nextSeq(),"HOLD_PI,1,8000,10000");
  assert(motion.fault==BAD_COMMAND&&!motion.armed&&!controlEnabled);
  assert(!motion.left&&!motion.right); tick();
  assert(!velocity.output[0]&&!velocity.output[1]);
  tick(110); command(nextSeq(),"CLEAR_FAULT");
  command(nextSeq(),"ARM"); command(nextSeq(),"SET_SPEED,0,0,300"); tick(120);
  assert(velocity.holdReference[0]==encR.count&&velocity.holdReference[1]==encL.count);
  // Connection heartbeats cannot keep an obsolete position command alive.
  for(unsigned n=0;n<5;++n) { tick(70); command(nextSeq(),"PING"); }
  assert(motion.fault==COMMAND_EXPIRED&&!controlEnabled&&!velocity.active);
  assert(!velocity.output[0]&&!velocity.output[1]);
  command(nextSeq(),"STOP"); assert(motion.fault==COMMAND_EXPIRED);
  tick(110); command(nextSeq(),"CLEAR_FAULT");
  assert(!motion.armed&&!controlEnabled);
  command(nextSeq(),"ARM"); command(nextSeq(),"SET_SPEED,0,0,300"); tick(120);
  command(nextSeq(),"STOP"); tick();
  assert(!motion.armed&&!velocity.active&&!velocity.output[0]&&!velocity.output[1]);
  for(unsigned i=0;i<2;++i)
    command(nextSeq(),"HOLD,"+std::to_string(i)+",0,25,500,120,2,100,10000,1000");
  encL.count=encR.count=0; tick(500); // reset idle estimator before baseline tests
}
uint32_t arrivalUs, nextControlUs, streamSequence;
unsigned streamOffset=0;
std::string streamPacket;
void elapsedWork(unsigned us) {
  const uint32_t until=nativeUs+us;
  while(nativeUs<until) {
    ++nativeUs;
    if(nativeUs>=arrivalUs) {
      Serial.receive(streamPacket[streamOffset++]);
      if(streamOffset==streamPacket.size()) {
        arrivalUs+=20000-87*(streamPacket.size()-1);
        streamPacket=packet(++streamSequence,"SET_SPEED,-3000,-3000,300"); streamOffset=0;
      } else arrivalUs+=87; // 115200 baud, 8N1
    }
    if(nativeUs>=nextControlUs) {
      --encR.count; ++encL.count; // synthetic valid encoder movement, ~22.6 RPM
      TIMER1_COMPA_vect(); nextControlUs+=1000;
    }
  }
}
int main() {
  setup(); tick(110);
  // Real parser and timer code, fake UART/I2C only; no motors are attached.
  for(unsigned i=0;i<2;++i) {
    command(nextSeq(),"PID,"+std::to_string(i)+",1,1500,8000,0,6300,35000");
    command(nextSeq(),"PID,"+std::to_string(i)+",-1,1500,8000,0,6300,35000");
    command(nextSeq(),"ZERO,"+std::to_string(i)+",2000,0,150");
    command(nextSeq(),"HOLD,"+std::to_string(i)+",0,25,500,120,2,100,10000,1000");
  }
  assert(!motion.fault);
  testHoldProtocol();
  command(nextSeq(),"ARM"); assert(motion.armed);
  command(nextSeq(),"SET_SPEED,10000,-10000,300",true);
  const auto first=lastSequence; tick();
  assert(appliedSequence==first&&velocity.target[0]==10000&&velocity.target[1]==-10000);
  const auto before=velocity.measuredQ16[0];
  command(nextSeq(),"SET_SPEED,7200,3600,300");
  command(nextSeq(),"SET_SPEED,5000,0,300"); tick();
  assert(velocity.target[0]==5000&&velocity.target[1]==0&&velocity.measuredQ16[0]==before);
  assert(velocity.active);
  // Zero remains ACTIVE; renewals do not create a new start or hold origin.
  command(nextSeq(),"SET_SPEED,0,0,300");
  for(int n=0;n<10;++n) { tick(20); command(nextSeq(),"SET_SPEED,0,0,300"); }
  assert(velocity.active&&velocity.settled[0]&&velocity.settled[1]);
  for(int n=0;n<5;++n) { tick(70); command(nextSeq(),"PING"); }
  assert(!motion.armed&&motion.fault==COMMAND_EXPIRED&&!controlEnabled);
  command(nextSeq(),"STOP"); assert(motion.fault==COMMAND_EXPIRED);
  command(nextSeq(),"ARM"); assert(!motion.armed);
  tick(110); command(nextSeq(),"CLEAR_FAULT"); assert(!motion.fault);
  command(nextSeq(),"ARM"); command(nextSeq(),"SET_SPEED,-10001,0,300");
  assert(!motion.armed&&motion.fault==BAD_COMMAND);
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  command(nextSeq(),"DRIVE,651,650,1000"); assert(!motion.armed&&motion.fault==BAD_COMMAND);
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  command(nextSeq(),"SET_SPEED,0,0,300"); tick();
  command(lastSequence,"STOP"); assert(!motion.armed&&!controlEnabled); // duplicate STOP wins
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  command(nextSeq(),"SET_SPEED,0,0,300"); tick();
  command(lastSequence,"SET_SPEED,7200,7200,300"); assert(!motion.armed&&motion.fault==BAD_COMMAND);
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  command(nextSeq(),"SET_SPEED,0,0,300"); tick();
  std::string broken=packet(nextSeq(),"SET_SPEED,7200,7200,300"); broken[5]='X';
  for(char c:broken) Serial.receive(c);
  loop(); loop();
  assert(!motion.armed&&motion.fault==BAD_COMMAND);
  tick(110); command(nextSeq(),"CLEAR_FAULT");
  lastSequence=0xfffffffeUL; command(0xffffffffUL,"ARM"); command(1,"SET_SPEED,0,0,300"); tick();
  assert(motion.armed&&appliedSequence==1);
  command(1,"STOP"); tick();
  command(2,"BRAKE"); assert(!motion.armed&&motion.fault==BAD_COMMAND);
  for(const char* invalid : {"SET_SPEED,,0,0,300", "SET_SPEED,0,0,300,", "SET_SPEED,0,0,301", "SET_SPEED,+0,0,300"}) {
    tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
    command(nextSeq(),invalid); assert(!motion.armed&&motion.fault==BAD_COMMAND);
  }
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  for(int n=0;n<150;++n) Serial.receive('X');
  Serial.receive('\n'); while(Serial.available()) loop();
  assert(!motion.armed&&motion.fault==BAD_COMMAND);
  tick(110); command(nextSeq(),"CLEAR_FAULT"); command(nextSeq(),"ARM");
  command(nextSeq(),"SET_SPEED,0,0,300"); tick();
  Wire.fail=true; tick();
  assert(!motion.armed&&motion.fault==DRIVER_IO);
  Wire.fail=false; tick(110); assert(!motion.armed); // recovery never resumes a target
  command(nextSeq(),"CLEAR_FAULT"); assert(!motion.armed&&!motion.fault);
  // UART bytes continue arriving during formatting and I2C work. Unlike the
  // old unbounded fake queue, the real-sized ring drops bytes when it fills.
  command(nextSeq(),"ARM"); command(nextSeq(),"SET_SPEED,-3000,-3000,300"); tick();
  rxMaxQueued=0; rxMaxGapUs=0;
  streamSequence=nextSeq(); streamPacket=packet(streamSequence,"SET_SPEED,-3000,-3000,300");
  arrivalUs=nativeUs+100; nextControlUs=nativeUs+1000;
  nativeWork=elapsedWork;
  const uint32_t stressEnd=nativeUs+5000000;
  while(nativeUs<stressEnd) { loop(); nativeConsume(100); }
  while(streamOffset) nativeConsume(1);
  nativeWork=nullptr;
  while(Serial.available()) loop();
  assert(!Serial.dropped&&!motion.fault&&motion.armed);
  assert(lastSequence>=streamSequence-2&&rxMaxQueued<64&&rxMaxGapUs<10000);
  command(nextSeq(),"STOP"); tick();
  // Maximum diagnostics and framing must stay bounded and CRC-valid.
  encL.count=0xffffffffUL; encR.count=0x80000000UL;
  encL.invalid=encR.invalid=0xffffffffUL;
  sendInfo(); sendTelemetry(); sendControlState();
  while(txHead!=txTail) drainSerial();
  assert(Serial.output.find(",0,0,1,15*")!=std::string::npos);
  std::size_t start=0;
  while(start<Serial.output.size()) {
    auto end=Serial.output.find('\n',start); assert(end!=std::string::npos);
    auto line=Serial.output.substr(start,end-start); auto star=line.find('*');
    assert(line.size()<256&&star!=std::string::npos&&line.size()-star==5);
    assert(strtoul(line.c_str()+star+1,nullptr,16)==frameCrc(line.c_str(),star));
    start=end+1;
  }
  std::cout<<"production parser, ISR target application, continuous HOLD, lease, STOP, limits and CRC passed\n";
}
