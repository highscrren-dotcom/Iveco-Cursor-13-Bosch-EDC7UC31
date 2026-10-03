// Host-side mock of the Arduino core: just enough for VCM_Iveco_TSC1_xx.ino to compile and run on a PC.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <string>
#include <deque>
#include <vector>

#define HIGH 1
#define LOW  0
#define INPUT_PULLUP 2
#define A0 14
#define DEC 10
#define HEX 16
#define BIN 2
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

// --- simulated time and I/O (driven by the test) ---
extern unsigned long sim_millis;
extern int sim_pinLevel[32];
extern int sim_adc;
extern uint8_t MCUSR;
unsigned long millis();
void delay(unsigned long ms);
void pinMode(uint8_t pin, uint8_t mode);
int  digitalRead(uint8_t pin);
int  analogRead(uint8_t pin);

// --- Serial: mirrors the overload set of Arduino's Print ---
class SerialMock {
public:
  std::string out;
  std::deque<char> in;
  FILE* tee = nullptr;                      // optional copy of everything printed (e.g. for the recon summarizer)
  void begin(unsigned long) {}
  int  available() { return (int)in.size(); }
  int  read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  size_t print(const __FlashStringHelper* s) { return print((const char*)s); }
  size_t print(const char* s) { emit(s); return strlen(s); }
  size_t print(char c) { char b[2] = {c, 0}; emit(b); return 1; }
  size_t print(unsigned char v, int base = DEC) { return num((unsigned long)v, base); }
  size_t print(int v, int base = DEC) { return snum((long)v, base); }
  size_t print(unsigned int v, int base = DEC) { return num((unsigned long)v, base); }
  size_t print(long v, int base = DEC) { return snum(v, base); }
  size_t print(unsigned long v, int base = DEC) { return num(v, base); }
  size_t print(double v, int = 2) { char b[32]; snprintf(b, sizeof b, "%.2f", v); emit(b); return strlen(b); }
  template <typename T> size_t println(T v) { size_t n = print(v); emit("\n"); return n + 1; }
  template <typename T> size_t println(T v, int base) { size_t n = print(v, base); emit("\n"); return n + 1; }
  size_t println() { emit("\n"); return 1; }
private:
  void emit(const char* s) { out += s; if (tee) fputs(s, tee); }
  size_t num(unsigned long v, int base) {
    char b[40]; int i = 39; b[i] = 0;
    if (v == 0) b[--i] = '0';
    while (v) { int d = v % base; b[--i] = (char)(d < 10 ? '0' + d : 'A' + d - 10); v /= base; }
    emit(b + i); return strlen(b + i);
  }
  size_t snum(long v, int base) {
    if (base == DEC && v < 0) { emit("-"); return 1 + num((unsigned long)(-v), base); }
    return num((unsigned long)v, base);
  }
};
extern SerialMock Serial;
