#pragma once
#include <stdint.h>
#define WIRE_HAS_TIMEOUT 1
struct WireMock {
  uint32_t timeoutUs = 0; bool resetOnTimeout = false;
  void setWireTimeout(uint32_t t, bool r) { timeoutUs = t; resetOnTimeout = r; }
};
extern WireMock Wire;
