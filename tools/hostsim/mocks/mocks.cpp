#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "EEPROM.h"
#include "avr/pgmspace.h"

unsigned long sim_millis = 0;
int sim_pinLevel[32];
int sim_adc = 0;
uint8_t MCUSR = 0;
SerialMock Serial;
SPIMock SPI;
WireMock Wire;
EEPROMMock EEPROM;
int __heap_start = 0; int* __brkval = 0;      // referenced by freeRam()

unsigned long millis() { return sim_millis; }
void delay(unsigned long ms) { sim_millis += ms; }
void pinMode(uint8_t, uint8_t) {}
int  digitalRead(uint8_t pin) { return sim_pinLevel[pin & 31]; }
int  analogRead(uint8_t) { return sim_adc; }

static void fixFmt(const char* fmt, char* out, size_t n) {   // "%S" (PROGMEM string on AVR) -> "%s"
  size_t j = 0; bool pct = false;
  for (size_t i = 0; fmt[i] && j + 1 < n; i++) {
    char c = fmt[i];
    if (pct && c == 'S') c = 's';
    pct = (c == '%') ? !pct : (pct && (c == '-' || c == '0' || (c >= '1' && c <= '9') || c == '.' || c == 'l' || c == 'h'));
    out[j++] = c;
  }
  out[j] = 0;
}
int vsnprintf_P(char* buf, size_t n, const char* fmt, va_list ap) { char f[256]; fixFmt(fmt, f, sizeof f); return vsnprintf(buf, n, f, ap); }
int snprintf_P(char* buf, size_t n, const char* fmt, ...) { va_list ap; va_start(ap, fmt); int r = vsnprintf_P(buf, n, fmt, ap); va_end(ap); return r; }

struct PinInit { PinInit() { for (int i = 0; i < 32; i++) sim_pinLevel[i] = HIGH; } } pinInit;
