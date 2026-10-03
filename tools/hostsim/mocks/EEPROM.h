#pragma once
#include <stdint.h>
#include <string.h>
struct EEPROMMock {
  uint8_t mem[1024];
  unsigned writes = 0;
  EEPROMMock() { memset(mem, 0xFF, sizeof mem); }
  uint8_t read(int a) { return mem[a & 1023]; }
  void update(int a, uint8_t v) { if (mem[a & 1023] != v) { mem[a & 1023] = v; writes++; } }
};
extern EEPROMMock EEPROM;
