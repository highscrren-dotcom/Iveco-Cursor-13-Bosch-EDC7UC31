#pragma once
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#define PROGMEM
#define PSTR(s) (s)
typedef const char* PGM_P;
#define pgm_read_byte(a)  (*(const uint8_t*)(a))
#define pgm_read_word(a)  (*(const uint16_t*)(a))
#define pgm_read_dword(a) (*(const uint32_t*)(a))
#define pgm_read_ptr(a)   (*(void* const*)(a))
#define strncpy_P strncpy
#define memcpy_P  memcpy
// On AVR "%S" prints a PROGMEM string; on the host every string is in RAM, so map %S -> %s.
int snprintf_P(char* buf, size_t n, const char* fmt, ...);
int vsnprintf_P(char* buf, size_t n, const char* fmt, va_list ap);
