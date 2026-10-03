// Mock of a 16x2 HD44780 behind LiquidCrystal_I2C. Models the real DDRAM layout (row 0 = 0x00.., row 1 = 0x40..,
// 40 cells per row) so a wrong cursor handling ends up in an invisible cell and the test catches it.
#pragma once
#include <stdint.h>
#include <string.h>
#include <string>
class __FlashStringHelper;
class LiquidCrystal_I2C {
public:
  LiquidCrystal_I2C(uint8_t, uint8_t, uint8_t) { clear(); }
  void init() {}
  void backlight() {}
  void clear() { memset(ddram, ' ', sizeof ddram); addr = 0; }
  void setCursor(uint8_t col, uint8_t row) { addr = (row ? 0x40 : 0x00) + col; setCursorCalls++; }
  size_t write(uint8_t c) { if (addr < sizeof ddram) ddram[addr] = (char)c; addr++; writes++; return 1; }
  size_t print(const char* s) { size_t n = 0; while (*s) { write((uint8_t)*s++); n++; } return n; }
  size_t print(const __FlashStringHelper* s) { return print((const char*)s); }
  std::string row(int r) const { return std::string(ddram + (r ? 0x40 : 0x00), 16); }
  bool hiddenCellsClean() const {                 // nothing written outside the visible 16 columns
    for (int i = 0x10; i < 0x28; i++) if (ddram[i] != ' ') return false;
    for (int i = 0x50; i < 0x68; i++) if (ddram[i] != ' ') return false;
    return true;
  }
  char ddram[0x68]; uint8_t addr = 0; int writes = 0, setCursorCalls = 0;
};
