// Mock of autowp/arduino-mcp2515: frames the sketch sends land in `tx`, the test feeds frames into `rx`.
#pragma once
#include <stdint.h>
#include <deque>
#include <vector>
#define CAN_EFF_FLAG 0x80000000UL
#define CAN_EFF_MASK 0x1FFFFFFFUL
#define CAN_MAX_DLEN 8
struct can_frame { uint32_t can_id; uint8_t can_dlc; uint8_t data[8]; };
enum CAN_SPEED { CAN_250KBPS };
enum CAN_CLOCK { MCP_8MHZ, MCP_16MHZ };
class MCP2515 {
public:
  enum ERROR { ERROR_OK = 0, ERROR_FAIL, ERROR_ALLTXBUSY, ERROR_FAILINIT, ERROR_FAILTX, ERROR_NOMSG };
  explicit MCP2515(int) {}
  ERROR reset() { resets++; return ERROR_OK; }
  ERROR setBitrate(CAN_SPEED, CAN_CLOCK) { return ERROR_OK; }
  ERROR setNormalMode() { return ERROR_OK; }
  ERROR readMessage(struct can_frame* f) { if (rx.empty()) return ERROR_NOMSG; *f = rx.front(); rx.pop_front(); return ERROR_OK; }
  ERROR sendMessage(const struct can_frame* f) { tx.push_back(*f); return failTx ? ERROR_FAILTX : ERROR_OK; }
  uint8_t getErrorFlags() { return eflg; }
  std::deque<can_frame>  rx;
  std::vector<can_frame> tx;
  int resets = 0; bool failTx = false; uint8_t eflg = 0;
};
