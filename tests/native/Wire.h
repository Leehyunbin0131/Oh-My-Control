#pragma once
#include <string>
#include <vector>
struct NativeWire {
  bool fail=false;
  std::vector<std::string> writes;
  void begin() {} void setClock(unsigned long) {} void setWireTimeout(unsigned long,bool) {}
  void clearWireTimeoutFlag() {} bool getWireTimeoutFlag() { return false; }
  void beginTransmission(unsigned char) {}
  void write(const char* s) { writes.emplace_back(s); }
  unsigned char endTransmission() { nativeConsume(1000); return fail?2:0; }
} Wire;
