#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <deque>
#include <string>
#define __AVR_ATmega2560__ 1
#define F_CPU 16000000UL
#define SERIAL_RX_BUFFER_SIZE 256
#define ISR(name) void name()
#define _BV(bit) (1U << (bit))
#define INPUT_PULLUP 2
#define ATOMIC_RESTORESTATE 0
#define OCIE1A 0
#define OCIE5A 0
#define WGM12 0
#define WGM52 0
#define CS11 0
#define CS51 0
uint8_t TIMSK1, TIMSK5, PING, PINB, TCCR1A, TCCR1B, TCCR5A, TCCR5B;
uint16_t TCNT1, TCNT5, OCR1A, OCR5A;
uint32_t nativeUs=0;
void (*nativeWork)(unsigned)=nullptr;
void nativeConsume(unsigned us) { if(nativeWork) nativeWork(us); }
uint32_t micros() { return nativeUs; }
uint32_t millis() { return nativeUs/1000; }
void sei() {} void cli() {} void pinMode(uint8_t,uint8_t) {}
uint8_t digitalRead(uint8_t) { return 0; }
char* ltoa(long n,char* p,int) { nativeConsume(150); sprintf(p,"%ld",n); return p; }
char* ultoa(unsigned long n,char* p,int) { nativeConsume(150); sprintf(p,"%lu",n); return p; }
struct NativeSerial {
  std::deque<char> input;
  std::string output;
  unsigned dropped=0;
  void receive(char c) { if(input.size()<SERIAL_RX_BUFFER_SIZE-1) input.push_back(c); else ++dropped; }
  void begin(uint32_t) {}
  int available() { return input.size(); }
  int availableForWrite() { return 64; }
  int read() { char c=input.front(); input.pop_front(); return c; }
  void write(char c) { output+=c; }
  void print(const char* s) { output+=s; }
} Serial,Serial1;
