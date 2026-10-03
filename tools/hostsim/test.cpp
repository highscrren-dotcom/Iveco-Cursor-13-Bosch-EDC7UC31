// Scenario tests: the real sketch runs on the PC against a small ECU simulator.
// Build & run: tools/hostsim/run.sh
#include "Arduino.h"
#include "../../VCM_Iveco_TSC1_75.ino"

static int failures = 0, checks = 0;
void settle();
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, msg); } } while (0)
#define CHECK_ROW(r, expect) do { settle(); std::string got = lcd.row(r); checks++; \
  if (got != expect) { failures++; printf("  FAIL line %d: LCD row %d = |%s| expected |%s|\n", __LINE__, r, got.c_str(), expect); } } while (0)

// ============================================================ ECU simulator
struct EcuSim {
  bool ignition = false, running = false;
  uint16_t rpm = 0, idle = 600;
  uint8_t  acceptSa = 0x27;          // TSC1 source address the ECU obeys
  bool     reportCtrl = true;        // fills SPN 1483 in EEC1
  uint8_t  ctrlSa = 0xFF;
  int      target = -1;
  std::string ci, soft;              // "" = not supported -> NACK
  uint8_t  name[8] = {1,2,3,4,5,6,7,8};
  bool     hasName = true;
  std::vector<uint8_t> dm2;          // DM2 payload to answer with (empty = none)
  struct Pending { unsigned long at; can_frame f; };
  std::vector<Pending> queue;
  unsigned long lastEec1 = 0, busyUntil = 0;   // a real ECU runs one BAM session at a time
  int reqCI = 0, reqSOFT = 0, reqACL = 0, reqDM2 = 0, reqDM3 = 0, reqDM11 = 0, reqBadDlc = 0, reqNotGlobal = 0;
  int tsc1Speed[256]; int tsc1Off[256];
  can_frame lastSpeed{};

  EcuSim() { memset(tsc1Speed, 0, sizeof tsc1Speed); memset(tsc1Off, 0, sizeof tsc1Off); }

  static can_frame mk(uint32_t id, const uint8_t* d, uint8_t n) {
    can_frame f; f.can_id = id | CAN_EFF_FLAG; f.can_dlc = 8; memset(f.data, 0xFF, 8); memcpy(f.data, d, n); return f;
  }
  void later(unsigned long dt, const can_frame& f) { queue.push_back({sim_millis + dt, f}); }
  void bam(uint32_t pgn, const std::vector<uint8_t>& payload, unsigned long dt) {
    uint16_t sz = payload.size(); uint8_t np = (sz + 6) / 7;
    unsigned long start = sim_millis + dt; if ((long)(start - busyUntil) < 0) start = busyUntil;
    busyUntil = start + 50UL * np + 10;
    uint8_t cm[8] = {0x20, (uint8_t)(sz & 0xFF), (uint8_t)(sz >> 8), np, 0xFF, (uint8_t)(pgn & 0xFF), (uint8_t)((pgn >> 8) & 0xFF), (uint8_t)(pgn >> 16)};
    queue.push_back({start, mk(0x1CECFF00, cm, 8)});
    for (uint8_t s = 1; s <= np; s++) {
      uint8_t dt8[8]; memset(dt8, 0xFF, 8); dt8[0] = s;
      for (int k = 0; k < 7; k++) { size_t idx = (s - 1) * 7 + k; if (idx < payload.size()) dt8[1 + k] = payload[idx]; }
      queue.push_back({start + 50UL * s, mk(0x1CEBFF00, dt8, 8)});
    }
  }
  void answer(uint32_t pgn, const std::vector<uint8_t>& payload, unsigned long dt) {
    if (payload.size() <= 8) { later(dt, mk(0x18000000UL | (pgn << 8), payload.data(), payload.size())); }
    else bam(pgn, payload, dt);
  }
  void nack(uint32_t pgn, uint8_t requester) {
    uint8_t d[8] = {1, 0xFF, 0xFF, 0xFF, requester, (uint8_t)(pgn & 0xFF), (uint8_t)((pgn >> 8) & 0xFF), (uint8_t)(pgn >> 16)};
    later(30, mk(0x18E8FF00, d, 8));
  }
  can_frame eec1() {
    uint8_t d[8] = {0x00, 0xFF, 0xFF, (uint8_t)((rpm * 8) & 0xFF), (uint8_t)((rpm * 8) >> 8), reportCtrl ? ctrlSa : (uint8_t)0xFF, 0xFF, 0xFF};
    return mk(0x0CF00400, d, 8);
  }
  void handleTx(const can_frame& f) {
    uint32_t id = f.can_id & CAN_EFF_MASK; uint8_t pf = (id >> 16) & 0xFF, ps = (id >> 8) & 0xFF, sa = id & 0xFF;
    if (pf == 0x00) {                                   // TSC1
      if ((f.data[0] & 3) == 1) {
        tsc1Speed[sa]++; lastSpeed = f;
        if (sa == acceptSa && running) { target = (f.data[1] | (f.data[2] << 8)) / 8; ctrlSa = sa; }
      } else { tsc1Off[sa]++; if (sa == ctrlSa) { target = -1; ctrlSa = 0xFF; } }
    } else if (pf == 0xEA) {                            // Request
      uint32_t pgn = f.data[0] | ((uint32_t)f.data[1] << 8) | ((uint32_t)f.data[2] << 16);
      if (f.can_dlc != 3) reqBadDlc++;
      if (ps != 0xFF && (pgn == 65259 || pgn == 65242 || pgn == 60928 || pgn == 65227)) reqNotGlobal++;
      switch (pgn) {
        case 65259: reqCI++;  if (ci.size()) answer(pgn, std::vector<uint8_t>(ci.begin(), ci.end()), 40); else nack(pgn, sa); break;
        case 65242: reqSOFT++; if (soft.size()) answer(pgn, std::vector<uint8_t>(soft.begin(), soft.end()), 40); else nack(pgn, sa); break;
        case 60928: reqACL++; if (hasName) later(40, mk(0x18EEFF00, name, 8)); break;
        case 65227: reqDM2++; if (dm2.size()) answer(pgn, dm2, 40); break;
        case 65228: reqDM3++; break;
        case 65235: reqDM11++; break;
        default: nack(pgn, sa);
      }
    }
  }
  void tick(MCP2515& m) {
    for (auto& f : m.tx) handleTx(f);
    m.tx.clear();
    if (running) { int tgt = (target >= 0) ? target : idle; if (rpm < tgt) rpm = min(rpm + 20, tgt); else if (rpm > tgt) rpm = max(rpm - 20, tgt); }
    else { rpm = 0; target = -1; ctrlSa = 0xFF; }
    if (ignition && sim_millis - lastEec1 >= 10) { lastEec1 = sim_millis; m.rx.push_back(eec1()); }
    for (size_t i = 0; i < queue.size();) {
      if ((long)(sim_millis - queue[i].at) >= 0) { if (ignition) m.rx.push_back(queue[i].f); queue.erase(queue.begin() + i); } else i++;
    }
  }
  void identity(const char* c, const char* s, uint8_t n0) { ci = c; soft = s; for (int i = 0; i < 8; i++) name[i] = n0 + i; }
};

EcuSim ecu;
static int pumpDesyncs = 0;
void run(unsigned long ms) {
  for (unsigned long i = 0; i < ms; i++) {
    sim_millis++; ecu.tick(mcp2515); loop();
    if (fbDirty == 0 && (lcd.row(0) != std::string(fb, 16) || lcd.row(1) != std::string(fb + 16, 16))) pumpDesyncs++;   // LCD must mirror fb once pumped
  }
}
void settle() { for (int i = 0; i < 60 && fbDirty; i++) run(1); }   // let the pump finish before looking at the LCD
void press(int pin)   { sim_pinLevel[pin] = LOW; }
void release(int pin) { sim_pinLevel[pin] = HIGH; }
void tap(int pin)     { press(pin); run(80); release(pin); run(80); }
void hold(int pin, unsigned long ms) { press(pin); run(ms); release(pin); run(80); }
void showLcd(const char* tag) { settle(); printf("  %-28s |%s|\n  %-28s |%s|\n", tag, lcd.row(0).c_str(), "", lcd.row(1).c_str()); }
void serialCmd(const char* s) { for (const char* p = s; *p; p++) Serial.in.push_back(*p); Serial.in.push_back('\n'); run(5); }
bool logHas(const char* s) { return Serial.out.find(s) != std::string::npos; }
void ignitionOff() { ecu.ignition = false; ecu.running = false; run(900); }
void ignitionOn()  { ecu.ignition = true; }
int  eeFpCount(uint16_t fp) { int n = 0; for (uint8_t p = 0; p < PRESET_COUNT; p++) for (uint8_t k = 0; k < FP_PER_PRESET; k++) if (eeFpGet(p, k) == fp) n++; return n; }

int main(int argc, char** argv) {
  if (argc > 1) Serial.tee = fopen(argv[1], "w");          // full device log, e.g. for recon/tools/recon_capture.py --summarize
  // ---------------------------------------------------------- S1 boot with blank EEPROM
  printf("S1 boot, blank EEPROM\n");
  setup();
  CHECK(EEPROM.mem[0] == 0xA5 && EEPROM.mem[1] == EE_VERSION, "EEPROM defaults written");
  CHECK(EEPROM.read(EE_A_CRC) == eeCalcCrc(), "EEPROM CRC valid");
  CHECK(curPreset == 0 && pSa[0] == 0x27 && pMode[0] == MODE_FIXED, "IVECO preset: 0x27 FIXED");
  CHECK(pSa[1] == 0x03 && pMode[1] == MODE_AUTO && pMode[3] == MODE_AUTO, "KAMAZ/spare presets: AUTO");
  CHECK(Wire.timeoutUs == 25000 && Wire.resetOnTimeout, "I2C timeout armed");
  CHECK(logHas("EEPROM DEFAULTS written"), "boot log mentions defaults");
  run(300);
  CHECK(phase == PH_WAITRUN, "phase WAITRUN");
  CHECK_ROW(0, "START ENGINE    ");
  CHECK_ROW(1, "IVECO SA:27 FIX ");
  showLcd("start screen, bus dead");

  // ---------------------------------------------------------- S2 ignition on, unknown ECU
  printf("S2 ignition on, ECU identification (unknown ECU)\n");
  Serial.out.clear();
  ecu.identity("BOSCH*EDC7UC31*IVECO0001*", "\x01" "1037388123*", 0x10);
  ignitionOn();
  run(1600);
  CHECK(ecu.reqCI == 1 && ecu.reqSOFT == 1 && ecu.reqACL == 1, "one request each for CI, SOFT, ACL");
  CHECK(ecu.reqBadDlc == 0, "requests use DLC 3");
  CHECK(ecu.reqNotGlobal == 0, "identification requests are global (0xFF)");
  CHECK(idState == ID_DONE && idSrc == 7, "ID done with all three sources");
  CHECK(idMatch == -1 && idFp != 0, "ECU unknown, fingerprint computed");
  uint16_t fpIveco = idFp;
  CHECK(ecu.tsc1Off[0x27] > 100 && ecu.tsc1Speed[0x27] == 0, "only TSC1 'override off' frames from 0x27 while idle");
  CHECK_ROW(0, "ECU neizvesten  ");
  CHECK_ROW(1, "READ 1.2s=preset");
  showLcd("unknown ECU message");
  run(2500);
  CHECK_ROW(0, "START ENGINE   ?");
  showLcd("start screen, marker ?");
  CHECK(logHas("ID done"), "log has ID summary");

  // ---------------------------------------------------------- S3 manual preset switch binds the ECU
  printf("S3 long READ: next preset + bind\n");
  hold(PIN_BTN_READ, 1300);
  CHECK(curPreset == 1 && activeAddr == 0x03, "preset KAMAZ active");
  CHECK(fpFind(fpIveco) == 1 && eeFpCount(fpIveco) == 1, "ECU fingerprint bound to KAMAZ only");
  CHECK(EEPROM.read(EE_A_CUR) == 1 && EEPROM.read(EE_A_CRC) == eeCalcCrc(), "EEPROM current preset + CRC");
  CHECK_ROW(0, "Preset: KAMAZ   ");
  CHECK_ROW(1, "SA:03 AUTO +ECU ");
  showLcd("after long READ");
  run(1600);
  CHECK_ROW(0, "START ENGINE   A");
  CHECK_ROW(1, "KAMAZ SA:03 AUTO");
  hold(PIN_BTN_READ, 1300); hold(PIN_BTN_READ, 1300); hold(PIN_BTN_READ, 1300);   // -> ECU-3 -> ECU-4 -> IVECO
  run(1600);
  CHECK(curPreset == 0 && fpFind(fpIveco) == 0 && eeFpCount(fpIveco) == 1, "fingerprint follows the last manual choice (IVECO)");
  CHECK_ROW(1, "IVECO SA:27 FIX ");

  // ---------------------------------------------------------- S4 ignition cycle: same ECU recognised
  printf("S4 ignition cycle, same ECU -> recognised\n");
  ignitionOff();
  CHECK(idState == ID_IDLE && phase == PH_WAITRUN, "bus loss resets ID state");
  Serial.out.clear();
  ignitionOn(); run(1500);
  CHECK(idState == ID_DONE && idFp == fpIveco && idMatch == 0, "same fingerprint, matches IVECO");
  CHECK_ROW(0, "ECU opoznan:    ");
  CHECK_ROW(1, "IVECO SA:27 FIX ");
  showLcd("recognised, same preset");
  run(2000);
  CHECK_ROW(0, "START ENGINE   A");

  // ---------------------------------------------------------- S5 another ECU (KamAZ) -> unknown -> bind to KAMAZ
  printf("S5 different ECU -> unknown -> bind KAMAZ\n");
  ignitionOff();
  ecu.identity("BOSCH*EDC7UC31*KAMAZ740*", "\x01" "1037500999*", 0x40);
  ignitionOn(); run(4000);
  CHECK(idState == ID_DONE && idMatch == -1 && idFp != fpIveco, "different ECU is unknown");
  uint16_t fpKamaz = idFp;
  hold(PIN_BTN_READ, 1300);                       // IVECO -> KAMAZ
  CHECK(curPreset == 1 && fpFind(fpKamaz) == 1 && fpFind(fpIveco) == 0, "KAMAZ bound, IVECO binding intact");
  run(1600);
  CHECK_ROW(1, "KAMAZ SA:03 AUTO");

  // ---------------------------------------------------------- S6 automatic switching both ways
  printf("S6 automatic preset switching by ECU identity\n");
  ignitionOff(); ecu.identity("BOSCH*EDC7UC31*IVECO0001*", "\x01" "1037388123*", 0x10);
  Serial.out.clear(); ignitionOn(); run(1500);
  CHECK(curPreset == 0 && activeAddr == 0x27, "auto-switched to IVECO");
  CHECK(logHas("PRESET -> IVECO") && logHas("(auto)"), "log: automatic preset change");
  CHECK_ROW(0, "ECU opoznan:    ");
  CHECK_ROW(1, "IVECO SA:27 FIX ");
  showLcd("auto-switched to IVECO");
  ignitionOff(); ecu.identity("BOSCH*EDC7UC31*KAMAZ740*", "\x01" "1037500999*", 0x40);
  ignitionOn(); run(4000);
  CHECK(curPreset == 1 && activeAddr == 0x03, "auto-switched to KAMAZ");
  CHECK_ROW(0, "START ENGINE   A");
  CHECK_ROW(1, "KAMAZ SA:03 AUTO");

  // ---------------------------------------------------------- S6b recognition finishes while engine runs -> deferred
  printf("S6b recognition while engine running is deferred\n");
  ignitionOff(); ecu.identity("BOSCH*EDC7UC31*IVECO0001*", "\x01" "1037388123*", 0x10);
  ignitionOn(); ecu.running = true; run(3000);
  CHECK(engineRunning && curPreset == 1 && pendingAuto == 0, "switch deferred while running");
  ecu.running = false; run(900);
  CHECK(!engineRunning && curPreset == 0 && pendingAuto == -1, "applied after engine stop");
  ignitionOff(); ecu.identity("BOSCH*EDC7UC31*KAMAZ740*", "\x01" "1037500999*", 0x40);
  ignitionOn(); run(4000);
  CHECK(curPreset == 1, "back on KAMAZ");

  // ---------------------------------------------------------- S7a sweep can be aborted with BACK (cold ECU ignores everything)
  printf("S7a sweep abort with BACK\n");
  ecu.acceptSa = 0xFE; ecu.running = true; run(2500);
  CHECK(phase == PH_SCAN && revActive, "sweep running");
  tap(PIN_BTN_BACK);
  CHECK(phase == PH_RUN && !revActive && activeAddr == 0x03, "sweep aborted, preset address restored");
  CHECK_ROW(0, "Poisk ostanovlen");
  ecu.running = false; run(900);

  // ---------------------------------------------------------- S7 AUTO sweep finds the address, save it
  printf("S7 AUTO sweep: ECU obeys 0x0B, reports SPN 1483\n");
  ecu.acceptSa = 0x0B; ecu.reportCtrl = true;
  ecu.running = true; run(4500);
  CHECK(phase == PH_RUN && foundSa == 0x0B, "address 0x0B found");
  CHECK(ecu.tsc1Speed[0x03] > 0 && ecu.tsc1Speed[0x0B] > 0, "probed 0x03 then 0x0B");
  CHECK(ui == UI_SAVE_SA, "save prompt shown");
  CHECK_ROW(0, "SA 0x0B najden! ");
  CHECK_ROW(1, "CLR=sohranit    ");
  showLcd("sweep result");
  CHECK(logHas("SA FOUND 0xB (SPN1483)"), "found via SPN 1483 before rpm rise");
  tap(PIN_BTN_CLEAR);
  CHECK(pSa[1] == 0x0B && pMode[1] == MODE_FIXED, "saved as FIXED 0x0B");
  CHECK(EEPROM.read(eePresetAddr(1) + EE_P_SA) == 0x0B && EEPROM.read(eePresetAddr(1) + EE_P_MODE) == MODE_FIXED, "EEPROM updated");
  CHECK_ROW(0, "Sohraneno:      ");
  CHECK_ROW(1, "KAMAZ SA:0B FIX ");
  run(1600);
  CHECK_ROW(1, "T:-40C KAMAZ 0B ");
  showLcd("main screen, running");
  // pedal
  sim_adc = 1023; run(3000);
  CHECK(commandedRpm == RPM_MAX && realRpm >= RPM_MAX - 20, "full pedal -> RPM_MAX commanded and reached");
  CHECK_ROW(1, "T:-40C KAMAZ 0B*");
  CHECK((ecu.lastSpeed.can_id & CAN_EFF_MASK) == 0x0C00000BUL && ecu.lastSpeed.data[0] == 0xC1
        && ecu.lastSpeed.data[3] == 0xFF && ecu.lastSpeed.data[4] == 0xFF, "TSC1 frame format");
  CHECK((ecu.lastSpeed.data[1] | (ecu.lastSpeed.data[2] << 8)) == RPM_MAX * 8, "TSC1 speed = rpm*8");
  CHECK_ROW(0, "S:2200 A:2200   ");
  showLcd("full pedal");
  sim_adc = 0; run(600);
  CHECK(!revActive && ecu.target == -1 && realRpm <= 650, "pedal released -> override off, idle");
  // safety: no preset/menu while running
  hold(PIN_BTN_READ, 1300);
  CHECK(curPreset == 1, "preset unchanged while running");
  CHECK_ROW(0, "Zaglushi        ");
  CHECK_ROW(1, "dvigatel!       ");
  hold(PIN_BTN_BACK, 1300); run(1300);
  CHECK(ui == UI_MAIN, "menu refused while running");

  // ---------------------------------------------------------- S8 menu
  printf("S8 menu with engine stopped\n");
  ecu.running = false; run(900);
  CHECK(phase == PH_WAITRUN, "engine stopped");
  hold(PIN_BTN_BACK, 1300); run(300);
  CHECK(ui == UI_MENU, "menu opened");
  CHECK_ROW(0, "KAMAZ: rezhim SA");
  CHECK_ROW(1, "FIXED        CLR");
  showLcd("menu item 0");
  tap(PIN_BTN_READ); run(300);
  CHECK_ROW(0, "KAMAZ: adres SA ");
  CHECK_ROW(1, "0x0B Brakes/ABS ");
  tap(PIN_BTN_CLEAR); run(300);
  CHECK(ui == UI_EDIT_SA, "edit SA");
  CHECK_ROW(0, "KAMAZ: novyi SA ");
  tap(PIN_BTN_READ); run(300);
  CHECK(editSa == 0x10, "quick list -> 0x10");
  CHECK_ROW(1, "0x10 Retarder   ");
  showLcd("edit SA quick list");
  hold(PIN_BTN_READ, 1560);
  CHECK(editSa >= 0x12 && editSa <= 0x15, "hold READ increments by one with auto-repeat");
  tap(PIN_BTN_BACK);
  CHECK(ui == UI_MENU && pSa[1] == 0x0B, "cancel keeps old SA");
  tap(PIN_BTN_CLEAR); tap(PIN_BTN_READ); tap(PIN_BTN_CLEAR);   // edit -> 0x10 -> confirm
  CHECK(pSa[1] == 0x10 && activeAddr == 0x10 && EEPROM.read(eePresetAddr(1) + EE_P_SA) == 0x10, "SA 0x10 saved");
  tap(PIN_BTN_READ); run(300);                                 // item 2: ECU ID
  CHECK_ROW(0, "ECU ID          ");
  CHECK_ROW(1, "BOSCH*EDC7UC31*K");
  showLcd("menu ECU ID");
  tap(PIN_BTN_CLEAR); run(300);
  { char exp[17]; snprintf(exp, 17, "FP:%04X -> KAMAZ", idFp); CHECK_ROW(1, exp); }
  tap(PIN_BTN_READ); run(300);
  CHECK_ROW(0, "Sbros nastroek  ");
  tap(PIN_BTN_BACK); run(300);
  CHECK(ui == UI_MAIN, "back to main");
  CHECK_ROW(1, "KAMAZ SA:10 FIX ");
  // bus loss while in menu keeps the menu open
  hold(PIN_BTN_BACK, 1300);
  ignitionOff();
  CHECK(ui == UI_MENU, "menu survives ignition off");
  tap(PIN_BTN_BACK);
  ignitionOn(); run(4000);
  CHECK(curPreset == 1 && idMatch == 1, "still KAMAZ after ignition cycle");

  // ---------------------------------------------------------- S9 serial commands
  printf("S9 serial commands\n");
  Serial.out.clear();
  serialCmd("e");
  CHECK(logHas("preset=KAMAZ") && logHas("2 KAMAZ SA=0x10 mode=FIX"), "'e' prints presets");
  serialCmd("a 0B");
  CHECK(pSa[1] == 0x0B && activeAddr == 0x0B, "'a 0B' sets SA");
  serialCmd("m"); CHECK(pMode[1] == MODE_AUTO, "'m' toggles to AUTO");
  serialCmd("m"); CHECK(pMode[1] == MODE_FIXED, "'m' toggles back");
  Serial.out.clear();
  serialCmd("q FEEB"); run(400);
  CHECK(logHas("RECON pgn=65259") && logHas("'BOSCH*E'") && logHas("'DC7UC31'"), "'q FEEB' prints raw Component ID chunks");
  serialCmd("q FDC5"); run(200);
  CHECK(logHas("NACK from SA=0x0 pgn=64965"), "NACK logged for unsupported PGN");
  serialCmd("p 1");
  CHECK(curPreset == 0 && fpFind(fpKamaz) == 0, "'p 1' selects IVECO and rebinds this ECU");
  serialCmd("p 2");
  CHECK(curPreset == 1 && fpFind(fpKamaz) == 1 && fpFind(fpIveco) == 0, "'p 2' back to KAMAZ, bindings consistent");
  Serial.out.clear();
  serialCmd("i"); run(2500);
  CHECK(logHas("ID done") && idMatch == 1, "'i' re-runs identification");

  // ---------------------------------------------------------- S10 EEC1 n/a, DM1 BAM, sequence gap, DTC screen, DM2 request
  printf("S10 DTC handling\n");
  ecu.running = true; run(2500);
  CHECK(realRpm >= 580 && realRpm <= 620, "idle rpm from EEC1");
  { uint8_t d[8] = {0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; mcp2515.rx.push_back(EcuSim::mk(0x0CF00400, d, 8)); run(5); }
  CHECK(realRpm >= 580 && realRpm <= 620, "EEC1 0xFFFF (not available) ignored");
  // DM1 via BAM: lamp 2 bytes + SPN100 FMI1 OC3 + SPN157 FMI3 OC1
  ecu.bam(65226, {0x04, 0xFF, 100, 0, 0x01, 0x03, 157, 0, 0x03, 0x01}, 1); run(300);
  CHECK(dtcCount == 2 && dtcList[0].spn == 100 && dtcList[0].fmi == 1 && dtcList[0].oc == 3 && dtcList[1].spn == 157, "DM1 BAM parsed: 2 active codes");
  { char d[44]; buildDtcDesc(d, sizeof d, dtcList[0]); CHECK(strcmp(d, "Davlenie masla: nizhe normy KRIT (x3)") == 0, "DTC description"); }
  // BAM with a lost packet: announce 3 packets, send 1 and 3
  { uint8_t cm[8] = {0x20, 18, 0, 3, 0xFF, 0xCA, 0xFE, 0x00}; mcp2515.rx.push_back(EcuSim::mk(0x1CECFF00, cm, 8));
    uint8_t p1[8] = {1, 0x04, 0xFF, 100, 0, 0x01, 0x03, 157}; mcp2515.rx.push_back(EcuSim::mk(0x1CEBFF00, p1, 8));
    uint8_t p3[8] = {3, 0, 0x03, 0x01, 0xFF, 0xFF, 0xFF, 0xFF}; mcp2515.rx.push_back(EcuSim::mk(0x1CEBFF00, p3, 8)); run(5); }
  CHECK(!tpActive && dtcCount == 2 && logHas("BAM seq gap"), "lost BAM packet -> session dropped, list unchanged");
  // DTC screen: READ x5 (main -> 4 data pages -> DTC) requests DM2 globally
  ecu.dm2 = {0x00, 0xFF, 94, 0, 0x04, 0x02};      // one stored code SPN94 FMI4 OC2 (single frame)
  int dm2Before = ecu.reqDM2;
  tap(PIN_BTN_READ); run(300); CHECK_ROW(0, "Maslo:     ---  "); showLcd("data page 1");
  tap(PIN_BTN_READ); tap(PIN_BTN_READ); tap(PIN_BTN_READ); tap(PIN_BTN_READ); run(300);
  CHECK(ui == UI_DTC && ecu.reqDM2 == dm2Before + 1 && ecu.reqNotGlobal == 0, "DTC screen requested DM2 globally");
  CHECK(dtcCount == 3 && dtcList[2].stored && dtcList[2].spn == 94, "DM2 single frame parsed as stored code");
  CHECK_ROW(0, "1/3 A S100 F1   ");
  showLcd("DTC screen");
  tap(PIN_BTN_READ); tap(PIN_BTN_READ); run(300);
  CHECK_ROW(0, "3/3 S S94 F4    ");
  tap(PIN_BTN_CLEAR); run(300);
  CHECK_ROW(0, "Sbrosit kody?   ");
  tap(PIN_BTN_CLEAR); run(100);
  CHECK(ecu.reqDM11 == 1 && ecu.reqDM3 == 1, "clear sends DM11 + DM3 to the engine");
  CHECK_ROW(0, "Sbros otpravlen ");
  run(1600); tap(PIN_BTN_BACK); run(300);
  CHECK(ui == UI_MAIN, "back to main");

  // ---------------------------------------------------------- S11 buttons: short tap registers, bounce is one press
  printf("S11 buttons\n");
  ecu.running = false; run(900);
  press(PIN_BTN_READ); run(30); release(PIN_BTN_READ); run(100);
  CHECK(ui == UI_DATA && dataPage == 0, "30 ms tap still counts as one short press");
  tap(PIN_BTN_BACK);
  press(PIN_BTN_READ); run(5); release(PIN_BTN_READ); run(5); press(PIN_BTN_READ); run(60); release(PIN_BTN_READ); run(100);
  CHECK(ui == UI_DATA && dataPage == 0, "contact bounce = one press");
  tap(PIN_BTN_BACK);

  // ---------------------------------------------------------- S12 factory reset (last: destroys bindings)
  printf("S12 factory reset\n");
  hold(PIN_BTN_BACK, 1300); tap(PIN_BTN_READ); tap(PIN_BTN_READ); tap(PIN_BTN_READ); tap(PIN_BTN_CLEAR); run(300);
  CHECK_ROW(0, "Sbrosit vse?    ");
  tap(PIN_BTN_BACK);
  CHECK(ui == UI_MENU && curPreset == 1, "reset cancelled");
  tap(PIN_BTN_CLEAR); tap(PIN_BTN_CLEAR); run(100);
  CHECK(curPreset == 0 && pSa[1] == 0x03 && pMode[1] == MODE_AUTO && fpFind(fpKamaz) == -1 && fpFind(fpIveco) == -1, "defaults restored, bindings cleared");
  CHECK_ROW(0, "Nastroyki       ");
  CHECK_ROW(1, "sbrosheny       ");

  // ---------------------------------------------------------- S13 ECU without identification support
  printf("S13 ECU that answers nothing\n");
  ignitionOff(); ecu.ci = ""; ecu.soft = ""; ecu.hasName = false; Serial.out.clear();
  int ciBefore = ecu.reqCI;
  ignitionOn(); run(7000);
  CHECK(idState == ID_NONE && ecu.reqCI == ciBefore + 3, "three attempts then give up");
  CHECK(logHas("net otveta"), "log: no answer");
  run(1000);
  CHECK_ROW(0, "START ENGINE    ");
  showLcd("no identification");

  CHECK(lcd.hiddenCellsClean(), "LCD: nothing ever written outside the visible 16x2");
  CHECK(pumpDesyncs == 0, "LCD always mirrors the frame buffer once the pump is idle");
  printf("\n%d checks, %d failures\n", checks, failures);
  if (Serial.tee) fclose(Serial.tee);
  return failures ? 1 : 0;
}
