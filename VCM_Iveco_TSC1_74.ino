#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

// ============================================================
//  VCM Iveco Cursor 13 / Bosch EDC7UC31 — стендовый пульт.
//  Управление оборотами по J1939 (TSC1, PGN 0) с автоподбором SA
//  + ЧТЕНИЕ КОДОВ НЕИСПРАВНОСТЕЙ (DM1/DM2) с расшифровкой и сбросом.
// ============================================================
//
//  ВНИМАНИЕ ПРО ЭКРАН: стандартный 1602 (HD44780) не имеет кириллицы.
//  Поэтому описания ошибок — ТРАНСЛИТОМ (латиницей). Для настоящей
//  кириллицы нужен OLED SSD1306 + U8g2 (отдельная версия).
//
//  КНОПКИ (каждая между пином и GND, внутр. подтяжка):
//    READ  (D3) — из главного экрана открыть список ошибок;
//                 в списке — следующий код.
//    CLEAR (D4) — в списке: запросить сброс (с подтверждением).
//    BACK  (D5) — вернуться на главный экран / отмена.
//
//  !!! TSC1 реально крутит двигатель. Аварийный стоп под рукой. !!!
//
// ============================================================
//  КОНФИГУРАЦИЯ
// ============================================================

// --- Автоподбор адреса ---
#define ENABLE_AUTO_SWEEP   0       // 0 = жёстко зашитый TSC1_SOURCE_ADDR; код перебора оставлен в runPhases как опция
#define SWEEP_PROBE_MARGIN  450
#define SWEEP_DETECT_MARGIN 250
#define SWEEP_SETTLE_MS     1500
#define SWEEP_TEST_MS       2500
const uint8_t SWEEP_ADDRS[] = {0x21, 0x27, 0x03, 0x07};

// --- Фиксированный адрес (если ENABLE_AUTO_SWEEP = 0) ---
#define TSC1_SOURCE_ADDR    0x27    // подтверждено на стенде: ЭБУ слушает этот адрес
#define TSC1_BYTE5          0xFF

// --- Обороты ---
#define RPM_MIN             650
#define RPM_MAX             2200
#define RPM_RAMP            10
#define RUN_CMD_DEADBAND    100

// --- Детекция работы двигателя ---
#define ENGINE_RUNNING_RPM  350
#define ENGINE_STOPPED_RPM  150

// --- Тестовый режим стенда (проверка педали БЕЗ ЭБУ) ---
#define BENCH_TEST_MODE     0

// --- Прочее ---
#define ENABLE_PERMISSIVES  0
#define ENABLE_WATCHDOG     1       // сторожевой таймер: зависание МК -> сброс, TSC1 пропадает, EDC7 роняет обороты в холостой
#define ENABLE_BUSOFF_REG_CHECK 1   // детект bus-off по флагу TXBO в регистре EFLG (нужен getErrorFlags(); не компилится -> 0)

// --- DTC-ридер ---
#define ENABLE_DTC_READER   1
#define PIN_BTN_READ        3       // читать / следующий код
#define PIN_BTN_CLEAR       4       // сброс (с подтверждением)
#define PIN_BTN_BACK        5       // назад на главный экран
#define BTN_DEBOUNCE_MS     40
#define DTC_MAX             15      // макс. кодов в буфере
#define TP_BUF              48      // буфер сборки многокадровых (BAM, ~11 кодов)
#define RX_MAX_PER_LOOP     8       // макс. CAN-кадров за проход loop (чтобы не голодал TSC1)

// --- Отладочные логи (Serial 115200) ---
#define DEBUG_LEVEL     2   // 0 тихо | 1 +события | 2 +инвентарь CAN и детали DTC | 3 трейс каждого кадра (тормозит loop!)
#define DEBUG_MCP_REGS  0   // 1 = печатать регистр ошибок MCP2515 (нужен getErrorFlags() в вашей версии autowp; не компилится -> 0)
#define DBG(lvl) if (DEBUG_LEVEL >= (lvl))

// --- Живость шины / авто-восстановление CAN ---
#define BUS_TIMEOUT_MS  600   // нет приёма дольше -> шина считается мёртвой (не шлём, чтобы не словить bus-off)
#define CAN_REINIT_MS   1000  // как часто переинициализировать MCP2515, пока шина мертва
#define TX_FAIL_RECOVER 16    // столько неудачных отправок подряд -> переинициализация CAN
#define PGN_SEEN_MAX    16    // размер таблицы инвентаря PGN

// --- Экран живых данных двигателя ---
#define DATA_PAGES   4        // число страниц параметров
#define DV_OILP  0x01
#define DV_OILT  0x02
#define DV_BOOST 0x04
#define DV_AIRT  0x08
#define DV_BATT  0x10
#define DV_FUEL  0x20
#define DV_LOAD  0x40
#define DV_COOL  0x80

// ============================================================
//  ПИНЫ И ОБОРУДОВАНИЕ
// ============================================================
const int PIN_CS  = 10;
const int PIN_POT = A0;

MCP2515 mcp2515(PIN_CS);
struct can_frame canMsgTSC1;
struct can_frame canMsgPerm;
struct can_frame canMsgRead;
struct can_frame canMsgReq;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================================
//  ТАЙМИНГИ
// ============================================================
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10;
unsigned long lastPermTime = 0;
const unsigned long PERM_INTERVAL = 100;
unsigned long lastLcdTime = 0;
const unsigned long LCD_INTERVAL = 200;
unsigned long lastSerialTime = 0;
const unsigned long SERIAL_INTERVAL = 500;

// ============================================================
//  ФИЛЬТР ПЕДАЛИ И ДАННЫЕ
// ============================================================
const int NUM_SAMPLES = 8;
int  adcReadings[NUM_SAMPLES];
int  readIndex = 0;
long adcTotal  = 0;

float realRpm      = 0.0;
int   engineTemp   = -40;
int   commandedRpm = RPM_MIN;
int   targetRpm    = RPM_MIN;
bool  revActive    = false;
bool  engineRunning = false;
int   canRxCount   = 0;
unsigned long txOk = 0, txFail = 0;          // счётчики отправок TSC1
unsigned long lastRxTime = 0;                // время последнего принятого CAN-кадра
unsigned long lastCanReinit = 0;             // время последней переинициализации CAN
int txFailStreak = 0;                        // неудачи отправки подряд
// --- Живые данные двигателя (J1939) ---
int oilP = 0, oilT = 0, boostKpa = 0, airT = 0, battV10 = 0, fuelRate10 = 0, engLoad = 0;
uint8_t dataValid = 0;                        // биты принятых параметров (DV_*)
#if DEBUG_LEVEL >= 2
uint16_t seenPgn[PGN_SEEN_MAX]; uint8_t seenCnt = 0;   // инвентарь шины (первая встреча PGN; PGN влезает в 16 бит)
#endif

// ============================================================
//  АВТОПОДБОР АДРЕСА
// ============================================================
const uint8_t SWEEP_COUNT = sizeof(SWEEP_ADDRS);
uint8_t  sweepIndex = 0;
int      idleBaseline = RPM_MIN;
uint8_t  activeAddr;
uint32_t activeCanId;

// ---- Типы: объявлены ДО первой функции (иначе авто-прототипы Arduino IDE ломают сборку) ----
enum Phase { PH_WAITRUN, PH_SETTLE, PH_SCAN, PH_RUN };
#if ENABLE_DTC_READER
struct Dtc   { uint32_t spn; uint8_t fmi; uint8_t oc; uint8_t cm; bool stored; };
struct Btn   { uint8_t pin; int last; unsigned long t; };
enum   UiMode { UI_MAIN, UI_DATA, UI_DTC, UI_CONFIRM_CLR, UI_MSG };
#endif

void setActiveAddr(uint8_t addr) {
  activeAddr  = addr;
  activeCanId = (0x0C000000UL | (uint32_t)addr) | CAN_EFF_FLAG;
}
int potToRpm(int adc) {
  int r = RPM_MIN + ((long)adc * (RPM_MAX - RPM_MIN)) / 1023;
  if (r > RPM_MAX) r = RPM_MAX;  if (r < RPM_MIN) r = RPM_MIN;  return r;
}
int potToRevRpm(int adc) {
  int top = RPM_MAX; if (top < idleBaseline) top = idleBaseline;
  int r = idleBaseline + ((long)adc * (top - idleBaseline)) / 1023;
  if (r > top) r = top;  if (r < idleBaseline) r = idleBaseline;  return r;
}

Phase phase;
unsigned long phaseTimer = 0;

// ============================================================
//  DTC: ХРАНИЛИЩЕ И ИНТЕРФЕЙС
// ============================================================
#if ENABLE_DTC_READER
Dtc      dtcList[DTC_MAX];
uint8_t  dtcCount = 0;
uint8_t  dtcView  = 0;
uint8_t  lampStatus = 0;
unsigned int scrollPos = 0;

// сборка многокадровых (BAM)
uint8_t  tpBuf[TP_BUF];
bool     tpActive = false, tpStored = false;
uint16_t tpSize = 0;
uint8_t  tpPackets = 0;

UiMode   ui = UI_MAIN;
UiMode   uiAfterMsg = UI_MAIN;
uint8_t  dataPage = 0;
char     msgL1[17] = "", msgL2[17] = "";
unsigned long msgUntil = 0;

Btn bRead, bClr, bBack;

// ---- Таблица FMI (0..31), транслит ----
const char f0[]  PROGMEM = "vyshe normy KRIT";
const char f1[]  PROGMEM = "nizhe normy KRIT";
const char f2[]  PROGMEM = "nestabil/neverno";
const char f3[]  PROGMEM = "napr.vysoko/KZ +";
const char f4[]  PROGMEM = "napr.nizko/KZ massa";
const char f5[]  PROGMEM = "obryv tsepi/tok nizok";
const char f6[]  PROGMEM = "KZ na massu/tok vysok";
const char f7[]  PROGMEM = "mehanika ne reagiruet";
const char f8[]  PROGMEM = "chastota/PWM error";
const char f9[]  PROGMEM = "redkoe obnovlenie";
const char f10[] PROGMEM = "rezkoe izmenenie";
const char f11[] PROGMEM = "prichina neizvestna";
const char f12[] PROGMEM = "neispr. komponent";
const char f13[] PROGMEM = "narush. kalibrovka";
const char f14[] PROGMEM = "osobye ukazaniya";
const char f15[] PROGMEM = "vyshe normy (slabo)";
const char f16[] PROGMEM = "vyshe normy (umer)";
const char f17[] PROGMEM = "nizhe normy (slabo)";
const char f18[] PROGMEM = "nizhe normy (umer)";
const char f19[] PROGMEM = "oshibka CAN dannyh";
const char f20[] PROGMEM = "dreyf vverh";
const char f21[] PROGMEM = "dreyf vniz";
const char frz[] PROGMEM = "rezerv";
const char f31[] PROGMEM = "uslovie prisutstvuet";
const char* const FMI_T[] PROGMEM = {
  f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,
  f16,f17,f18,f19,f20,f21,frz,frz,frz,frz,frz,frz,frz,frz,frz,f31
};

// ---- Таблица SPN: ТОЛЬКО двигатель Cursor 13 / EDC7UC31 (SAE SPN), транслит ----
//   Убраны «машинные» и неприменимые к Cursor 13 коды (скорость, VIN, момент,
//   EGR — у Cursor SCR/нет, дроссельная заслонка — отсутствует на этом CR-моторе).
const char n91[]   PROGMEM = "Pedal/zadanie gaza";
const char n94[]   PROGMEM = "Nizk. davl. topliva";
const char n97[]   PROGMEM = "Voda v toplive";
const char n98[]   PROGMEM = "Uroven masla";
const char n100[]  PROGMEM = "Davlenie masla";
const char n102[]  PROGMEM = "Davlenie nadduva";
const char n105[]  PROGMEM = "Temp.vpusk.kollekt";
const char n107[]  PROGMEM = "Zasor vozd. filtra";
const char n108[]  PROGMEM = "Atm. davlenie";
const char n110[]  PROGMEM = "Temp. OZH";
const char n111[]  PROGMEM = "Uroven OZH";
const char n132[]  PROGMEM = "Rashod vozduha MAF";
const char n157[]  PROGMEM = "Davlenie rampy Rail";
const char n158[]  PROGMEM = "Napr. AKB (klyuch)";
const char n168[]  PROGMEM = "Napr. bortseti";
const char n174[]  PROGMEM = "Temp. topliva";
const char n175[]  PROGMEM = "Temp. masla";
const char n190[]  PROGMEM = "Oboroty dvigatelya";
const char n611[]  PROGMEM = "Provodka forsunok";
const char n636[]  PROGMEM = "Datchik kolenvala";
const char n651[]  PROGMEM = "Forsunka cil.1";
const char n652[]  PROGMEM = "Forsunka cil.2";
const char n653[]  PROGMEM = "Forsunka cil.3";
const char n654[]  PROGMEM = "Forsunka cil.4";
const char n655[]  PROGMEM = "Forsunka cil.5";
const char n656[]  PROGMEM = "Forsunka cil.6";
const char n723[]  PROGMEM = "Datchik raspredval";
const char n1136[] PROGMEM = "Temp. ECU (blok)";
const char n1347[] PROGMEM = "Klapan rampy (ZME)";
const char n1485[] PROGMEM = "Glavnoe rele ECU";

const uint32_t SPN_ID[] PROGMEM = {
  91,94,97,98,100,102,105,107,108,110,111,132,157,158,168,174,175,190,
  611,636,651,652,653,654,655,656,723,1136,1347,1485
};
const char* const SPN_NM[] PROGMEM = {
  n91,n94,n97,n98,n100,n102,n105,n107,n108,n110,n111,n132,n157,n158,n168,n174,n175,n190,
  n611,n636,n651,n652,n653,n654,n655,n656,n723,n1136,n1347,n1485
};
const uint8_t SPN_N = sizeof(SPN_ID) / sizeof(SPN_ID[0]);

void lookupFmi(uint8_t fmi, char* out, size_t n) {
  if (fmi > 31) { snprintf(out, n, "FMI %u", (unsigned)fmi); return; }
  strncpy_P(out, (PGM_P)pgm_read_ptr(&FMI_T[fmi]), n); out[n - 1] = 0;
}
void lookupSpn(uint32_t spn, char* out, size_t n) {
  for (uint8_t i = 0; i < SPN_N; i++) {
    if (pgm_read_dword(&SPN_ID[i]) == spn) {
      strncpy_P(out, (PGM_P)pgm_read_ptr(&SPN_NM[i]), n); out[n - 1] = 0; return;
    }
  }
  snprintf(out, n, "SPN %lu", (unsigned long)spn);
}
void buildDtcDesc(char* out, size_t n, const Dtc &d) {
  char sN[22], fN[24];
  lookupSpn(d.spn, sN, sizeof(sN));
  lookupFmi(d.fmi, fN, sizeof(fN));
  if (d.oc > 0 && d.oc < 0x7F) snprintf(out, n, "%s: %s (x%u)", sN, fN, (unsigned)d.oc);
  else                        snprintf(out, n, "%s: %s", sN, fN);
}

void removeKind(bool stored) {
  uint8_t w = 0;
  for (uint8_t r = 0; r < dtcCount; r++)
    if (dtcList[r].stored != stored) dtcList[w++] = dtcList[r];
  dtcCount = w;
}
void parseDtcPayload(const uint8_t* d, uint16_t len, bool stored) {
  if (len < 2) return;
  lampStatus = d[0];
  removeKind(stored);
  uint16_t i = 2;
  while (i + 4 <= len && dtcCount < DTC_MAX) {
    uint8_t b0 = d[i], b1 = d[i + 1], b2 = d[i + 2], b3 = d[i + 3];
    // Стандарт SAE J1939-73, SPN Conversion Method = 4 (CM=0). FMI и OC от метода не зависят.
    uint32_t spn = (uint32_t)b0 | ((uint32_t)b1 << 8) | (((uint32_t)(b2 >> 5)) << 16);
    uint8_t fmi = b2 & 0x1F;
    uint8_t oc  = b3 & 0x7F;
    uint8_t cm  = b3 >> 7;                     // 0 = станд. v4; 1 = старый метод (SPN может отличаться)
    if (spn == 0 && fmi == 0) break;          // нет (больше) ошибок
    dtcList[dtcCount].spn = spn;
    dtcList[dtcCount].fmi = fmi;
    dtcList[dtcCount].oc  = oc;
    dtcList[dtcCount].cm  = cm;
    dtcList[dtcCount].stored = stored;
    dtcCount++;
    i += 4;
  }
  if (dtcView >= dtcCount) { dtcView = 0; scrollPos = 0; }
}

void sendReqPgn(uint32_t pgn, uint8_t dest) {                 // J1939 Request (PGN 59904)
  uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xEA << 16) | ((uint32_t)dest << 8) | activeAddr;
  canMsgReq.can_id  = id | CAN_EFF_FLAG;
  canMsgReq.can_dlc = 8;
  canMsgReq.data[0] = pgn & 0xFF;
  canMsgReq.data[1] = (pgn >> 8) & 0xFF;
  canMsgReq.data[2] = (pgn >> 16) & 0xFF;
  for (uint8_t i = 3; i < 8; i++) canMsgReq.data[i] = 0xFF;
  mcp2515.sendMessage(&canMsgReq);
}
void requestDM2() { sendReqPgn(65227, 0x00); }               // сохранённые
void clearDtcs()  { sendReqPgn(65235, 0x00); sendReqPgn(65228, 0x00); } // DM11 + DM3

bool btnEdge(Btn &b) {
  int raw = digitalRead(b.pin);
  unsigned long now = millis();
  if (raw != b.last && (now - b.t) > BTN_DEBOUNCE_MS) {
    b.t = now; b.last = raw;
    if (raw == LOW) return true;     // нажатие (кнопка к GND)
  }
  return false;
}
void showMsg(const char* a, const char* b, unsigned long dur, UiMode after) {
  strncpy(msgL1, a, 16); msgL1[16] = 0;
  strncpy(msgL2, b, 16); msgL2[16] = 0;
  msgUntil = millis() + dur; uiAfterMsg = after; ui = UI_MSG;
}

void lcdLine(uint8_t row, const char* s) {
  char b[17]; uint8_t i = 0;
  for (; i < 16 && s[i]; i++) b[i] = s[i];
  for (; i < 16; i++) b[i] = ' ';
  b[16] = 0;
  lcd.setCursor(0, row); lcd.print(b);
}
void lcdScroll(uint8_t row, const char* s, unsigned int pos) {
  unsigned int len = strlen(s);
  if (len <= 16) { lcdLine(row, s); return; }
  char b[17]; unsigned int total = len + 3;     // 3 пробела-разделитель
  for (uint8_t i = 0; i < 16; i++) {
    unsigned int idx = (pos + i) % total;
    b[i] = (idx < len) ? s[idx] : ' ';
  }
  b[16] = 0;
  lcd.setCursor(0, row); lcd.print(b);
}
void renderDtcUi() {
  if (ui == UI_MSG)          { lcdLine(0, msgL1); lcdLine(1, msgL2); return; }
  if (ui == UI_CONFIRM_CLR)  { lcdLine(0, "Sbrosit kody?"); lcdLine(1, "CLR=da  BACK=net"); return; }
  if (dtcCount == 0)         { lcdLine(0, "Oshibok net"); lcdLine(1, "DM1/DM2: chisto"); return; }
  if (dtcView >= dtcCount) dtcView = 0;
  const Dtc &d = dtcList[dtcView];
  char l1[20], l2[44];
  snprintf(l1, sizeof(l1), "%u/%u %c S%lu F%u",
           (unsigned)(dtcView + 1), (unsigned)dtcCount,
           d.stored ? 'S' : 'A', (unsigned long)d.spn, (unsigned)d.fmi);
  lcdLine(0, l1);
  buildDtcDesc(l2, sizeof(l2), d);
  lcdScroll(1, l2, scrollPos);
  scrollPos++;
}

void renderDataUi() {
  char l1[17], l2[17];
  switch (dataPage) {
    case 0:                                   // масло
      if (dataValid & DV_OILP) snprintf(l1, sizeof(l1), "Maslo:  %d.%d bar", oilP / 100, (oilP % 100) / 10);
      else                     snprintf(l1, sizeof(l1), "Maslo:     ---");
      if (dataValid & DV_OILT) snprintf(l2, sizeof(l2), "T masla: %4d C", oilT);
      else                     snprintf(l2, sizeof(l2), "T masla:    ---");
      break;
    case 1:                                   // ОЖ + наддув
      if (dataValid & DV_COOL) snprintf(l1, sizeof(l1), "T OZH:   %4d C", engineTemp);
      else                     snprintf(l1, sizeof(l1), "T OZH:      ---");
      if (dataValid & DV_BOOST)snprintf(l2, sizeof(l2), "Nadduv:%4d kPa", boostKpa);
      else                     snprintf(l2, sizeof(l2), "Nadduv:     ---");
      break;
    case 2:                                   // борт + впуск
      if (dataValid & DV_BATT) snprintf(l1, sizeof(l1), "Bort:   %d.%d V", battV10 / 10, battV10 % 10);
      else                     snprintf(l1, sizeof(l1), "Bort:      ---");
      if (dataValid & DV_AIRT) snprintf(l2, sizeof(l2), "T vpusk: %4d C", airT);
      else                     snprintf(l2, sizeof(l2), "T vpusk:    ---");
      break;
    default:                                  // топливо + нагрузка
      if (dataValid & DV_FUEL) snprintf(l1, sizeof(l1), "Rashod:%d.%d L/h", fuelRate10 / 10, fuelRate10 % 10);
      else                     snprintf(l1, sizeof(l1), "Rashod:    ---");
      if (dataValid & DV_LOAD) snprintf(l2, sizeof(l2), "Nagruz:  %4d %%", engLoad);
      else                     snprintf(l2, sizeof(l2), "Nagruz:     ---");
      break;
  }
  lcdLine(0, l1);
  lcdLine(1, l2);
}
void printDtcSerial() {
  Serial.print(F(">>> DTC: ")); Serial.print(dtcCount); Serial.println(F(" kod(ov)  [SPN FMI OC CM]"));
  for (uint8_t i = 0; i < dtcCount; i++) {
    Dtc &t = dtcList[i];
    char d[44]; buildDtcDesc(d, sizeof(d), t);
    Serial.print(i + 1); Serial.print(t.stored ? F(" [S] SPN") : F(" [A] SPN"));
    Serial.print((unsigned long)t.spn);
    Serial.print(F(" FMI")); Serial.print(t.fmi);
    Serial.print(F(" OC"));  Serial.print(t.oc);
    Serial.print(F(" CM"));  Serial.print(t.cm); if (t.cm) Serial.print(F("(legacy!)"));
    Serial.print(F("  ")); Serial.println(d);
  }
}
#endif  // ENABLE_DTC_READER

// ============================================================
//  ОТЛАДОЧНЫЕ ХЕЛПЕРЫ
// ============================================================
void ts() { Serial.print('['); Serial.print(millis()); Serial.print(F("] ")); }

int freeRam() {                                // свободная SRAM между кучей и стеком
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void canReinit() {                             // полный сброс CAN-контроллера (выход из bus-off)
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void resetRunState() {                         // полный сброс логики при пропаже шины (как при включении питания)
  phase = PH_WAITRUN;
  sweepIndex = 0;
#if BENCH_TEST_MODE
  setActiveAddr(TSC1_SOURCE_ADDR);
#elif ENABLE_AUTO_SWEEP
  setActiveAddr(SWEEP_ADDRS[0]);
#else
  setActiveAddr(TSC1_SOURCE_ADDR);
#endif
  idleBaseline  = RPM_MIN;
  commandedRpm  = RPM_MIN;
  targetRpm     = RPM_MIN;
  revActive     = false;
  engineRunning = false;
  realRpm       = 0;
  txFailStreak  = 0;
#if ENABLE_DTC_READER
  dtcCount = 0; dtcView = 0; scrollPos = 0; lampStatus = 0;
  tpActive = false; tpStored = false;
  ui = UI_MAIN;
#endif
}

const __FlashStringHelper* phaseName(Phase p) {
  switch (p) {
    case PH_WAITRUN: return F("WAITRUN");
    case PH_SETTLE:  return F("SETTLE");
    case PH_SCAN:    return F("SCAN");
    default:         return F("RUN");
  }
}

#if ENABLE_DTC_READER
const __FlashStringHelper* uiName(UiMode u) {
  switch (u) {
    case UI_MAIN:        return F("MAIN");
    case UI_DATA:        return F("DATA");
    case UI_DTC:         return F("DTC");
    case UI_CONFIRM_CLR: return F("CONFIRM");
    default:             return F("MSG");
  }
}
#endif

#if DEBUG_LEVEL >= 2
void noteNewPgn(uint32_t pgn, uint8_t sa) {    // лог первой встречи PGN — инвентарь шины
  for (uint8_t i = 0; i < seenCnt; i++) if (seenPgn[i] == pgn) return;
  if (seenCnt >= PGN_SEEN_MAX) return;          // таблица полна — не засоряем лог повторами
  seenPgn[seenCnt++] = (uint16_t)pgn;
  ts(); Serial.print(F("PGN seen ")); Serial.print(pgn);
  Serial.print(F(" SA=0x")); Serial.println(sa, HEX);
}
#endif

// ============================================================
void setup() {
  MCUSR = 0;
  wdt_disable();
  for (int i = 0; i < NUM_SAMPLES; i++) adcReadings[i] = 0;

  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("VCM Iveco TSC1  ");

#if ENABLE_DTC_READER
  pinMode(PIN_BTN_READ,  INPUT_PULLUP);
  pinMode(PIN_BTN_CLEAR, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK,  INPUT_PULLUP);
  bRead = {PIN_BTN_READ,  HIGH, 0};
  bClr  = {PIN_BTN_CLEAR, HIGH, 0};
  bBack = {PIN_BTN_BACK,  HIGH, 0};
#endif

#if BENCH_TEST_MODE
  setActiveAddr(TSC1_SOURCE_ADDR);
#elif ENABLE_AUTO_SWEEP
  setActiveAddr(SWEEP_ADDRS[0]);
#else
  setActiveAddr(TSC1_SOURCE_ADDR);
#endif

  SPI.begin();
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    lcd.setCursor(0, 1); lcd.print("CAN BUS: OK     ");
    Serial.println(F("VCM READY. TSC1 (PGN 0) + DTC reader."));
    DBG(1) {
      ts(); Serial.println(F("=== START VCM Iveco Cursor13/EDC7 ==="));
      ts(); Serial.print(F("cfg AUTO_SWEEP=")); Serial.print(ENABLE_AUTO_SWEEP);
            Serial.print(F(" BENCH="));        Serial.print(BENCH_TEST_MODE);
            Serial.print(F(" PERM="));         Serial.print(ENABLE_PERMISSIVES);
            Serial.print(F(" WDT="));          Serial.print(ENABLE_WATCHDOG);
            Serial.print(F(" DBG="));          Serial.println(DEBUG_LEVEL);
      ts(); Serial.print(F("cfg RPM "));       Serial.print(RPM_MIN); Serial.print('-'); Serial.print(RPM_MAX);
            Serial.print(F(" ramp="));         Serial.print(RPM_RAMP);
            Serial.print(F(" SA0=0x"));        Serial.println(activeAddr, HEX);
      ts(); Serial.print(F("freeRAM="));       Serial.print(freeRam()); Serial.println(F(" B"));
    }
  } else {
    lcd.setCursor(0, 1); lcd.print("CAN BUS: ERROR! ");
    Serial.println(F("КРИТ. ОШИБКА: модуль CAN не отвечает!"));
    while (1);
  }

  mcp2515.setNormalMode();
  delay(500);
  lcd.clear();

#if BENCH_TEST_MODE
  phase = PH_RUN;
#else
  phase = PH_WAITRUN;
#endif
  phaseTimer = millis();

#if ENABLE_WATCHDOG
  wdt_enable(WDTO_500MS);
#endif
}

void buildTSC1speed(int rpm) {
  uint16_t raw = (uint16_t)((long)rpm * 8);
  canMsgTSC1.can_id  = activeCanId;
  canMsgTSC1.can_dlc = 8;
  canMsgTSC1.data[0] = 0xC1;
  canMsgTSC1.data[1] = raw & 0xFF;
  canMsgTSC1.data[2] = (raw >> 8) & 0xFF;
  canMsgTSC1.data[3] = 0xFF;
  canMsgTSC1.data[4] = TSC1_BYTE5;
  canMsgTSC1.data[5] = 0xFF; canMsgTSC1.data[6] = 0xFF; canMsgTSC1.data[7] = 0xFF;
}
void buildTSC1disable() {
  canMsgTSC1.can_id  = activeCanId;
  canMsgTSC1.can_dlc = 8;
  canMsgTSC1.data[0] = 0xC0;
  canMsgTSC1.data[1] = 0xFF; canMsgTSC1.data[2] = 0xFF; canMsgTSC1.data[3] = 0xFF;
  canMsgTSC1.data[4] = TSC1_BYTE5;
  canMsgTSC1.data[5] = 0xFF; canMsgTSC1.data[6] = 0xFF; canMsgTSC1.data[7] = 0xFF;
}

void runPhases(unsigned long now, int adc) {
  switch (phase) {
    case PH_WAITRUN:
      revActive = false;
      if (engineRunning) { phase = PH_SETTLE; phaseTimer = now; }
      break;
    case PH_SETTLE:
      revActive = false;
      if (!engineRunning) { phase = PH_WAITRUN; break; }
      if (now - phaseTimer >= SWEEP_SETTLE_MS) {
        idleBaseline = (int)realRpm;
#if ENABLE_AUTO_SWEEP
        sweepIndex = 0; setActiveAddr(SWEEP_ADDRS[0]); commandedRpm = idleBaseline;
        phase = PH_SCAN; phaseTimer = now;
        Serial.print(F("Холостой ~")); Serial.print(idleBaseline); Serial.println(F(" rpm. Подбор адреса..."));
#else
        phase = PH_RUN;
#endif
      }
      break;
    case PH_SCAN: {
      if (!engineRunning) { phase = PH_WAITRUN; break; }
      revActive = true;
      int probe  = idleBaseline + SWEEP_PROBE_MARGIN; if (probe > RPM_MAX) probe = RPM_MAX;
      int detect = idleBaseline + SWEEP_DETECT_MARGIN;
      targetRpm = probe;
      if ((int)realRpm >= detect) {
        Serial.print(F(">>> НАЙДЕН АДРЕС! SA = 0x")); Serial.println(activeAddr, HEX);
        phase = PH_RUN;
      } else if (now - phaseTimer >= SWEEP_TEST_MS) {
        sweepIndex = (sweepIndex + 1) % SWEEP_COUNT;
        setActiveAddr(SWEEP_ADDRS[sweepIndex]);
        commandedRpm = idleBaseline;
        phaseTimer = now;
        DBG(1) { ts(); Serial.print(F("SWEEP next SA=0x")); Serial.println(activeAddr, HEX); }
      }
      break;
    }
    case PH_RUN:
      if (engineRunning) {
        targetRpm = potToRevRpm(adc);
        revActive = (targetRpm > idleBaseline + RUN_CMD_DEADBAND);
      } else {
        revActive = false;
      }
      break;
  }
}

#if ENABLE_PERMISSIVES
void sendPermissives() {
  canMsgPerm.can_id  = 0x18FEF121UL | CAN_EFF_FLAG; canMsgPerm.can_dlc = 8;
  canMsgPerm.data[0] = 0xF7; canMsgPerm.data[1] = 0x00; canMsgPerm.data[2] = 0x00;
  canMsgPerm.data[3] = 0xC0; canMsgPerm.data[4] = 0xFF; canMsgPerm.data[5] = 0xFF;
  canMsgPerm.data[6] = 0xFF; canMsgPerm.data[7] = 0xFF; mcp2515.sendMessage(&canMsgPerm);
  canMsgPerm.can_id  = 0x18F00203UL | CAN_EFF_FLAG; canMsgPerm.can_dlc = 8;
  canMsgPerm.data[0] = 0xFC; canMsgPerm.data[1] = 0xFF; canMsgPerm.data[2] = 0xFF;
  canMsgPerm.data[3] = 0xFF; canMsgPerm.data[4] = 0xFF; canMsgPerm.data[5] = 0xFF;
  canMsgPerm.data[6] = 0xFF; canMsgPerm.data[7] = 0xFF; mcp2515.sendMessage(&canMsgPerm);
}
#endif

void loop() {
#if ENABLE_WATCHDOG
  wdt_reset();
#endif
  unsigned long currentMillis = millis();

  // --- 1. ПРИЁМ CAN (вычитываем всё накопленное за цикл, но не больше RX_MAX_PER_LOOP) ---
  uint8_t rxThisLoop = 0;
  while (mcp2515.readMessage(&canMsgRead) == MCP2515::ERROR_OK) {
    canRxCount++;
    lastRxTime = currentMillis;
    if (canMsgRead.can_id & CAN_EFF_FLAG) {
      uint32_t ext = canMsgRead.can_id & CAN_EFF_MASK;
      uint8_t  pf  = (ext >> 16) & 0xFF;
      uint8_t  ps  = (ext >> 8)  & 0xFF;
      uint8_t  sa  = ext & 0xFF;
      uint32_t pgn = (pf < 240) ? ((uint32_t)pf << 8) : (((uint32_t)pf << 8) | ps);
#if DEBUG_LEVEL >= 3
      ts(); Serial.print(F("RX id=0x")); Serial.print(ext, HEX);
      Serial.print(F(" pgn=")); Serial.print(pgn);
      Serial.print(F(" dlc=")); Serial.println(canMsgRead.can_dlc);
#endif
#if DEBUG_LEVEL >= 2
      noteNewPgn(pgn, sa);
#endif

      if (pgn == 61444) {                              // EEC1 — обороты (SPN 190)
        uint16_t rawRpm = (canMsgRead.data[4] << 8) | canMsgRead.data[3];
        realRpm = rawRpm * 0.125;
      } else if (pgn == 65262) {                       // ET1 — темп. ОЖ (SPN110) + масла (SPN175)
        if (canMsgRead.data[0] != 0xFF) { engineTemp = canMsgRead.data[0] - 40; dataValid |= DV_COOL; }
        uint16_t ot = canMsgRead.data[2] | ((uint16_t)canMsgRead.data[3] << 8);
        if (ot != 0xFFFF) { oilT = (int)(ot >> 5) - 273; dataValid |= DV_OILT; }   // 0.03125 C/bit, -273
      } else if (pgn == 65263) {                       // EFL/P1 — давление масла (SPN100, b4, x4 кПа)
        if (canMsgRead.data[3] != 0xFF) { oilP = (int)canMsgRead.data[3] * 4; dataValid |= DV_OILP; }
      } else if (pgn == 65270) {                       // IC1 — наддув (SPN102, b2, x2 кПа) + темп.впуска (SPN105, b3, -40)
        if (canMsgRead.data[1] != 0xFF) { boostKpa = (int)canMsgRead.data[1] * 2; dataValid |= DV_BOOST; }
        if (canMsgRead.data[2] != 0xFF) { airT = (int)canMsgRead.data[2] - 40; dataValid |= DV_AIRT; }
      } else if (pgn == 65271) {                       // VEP1 — напряжение АКБ (SPN168, b5-6, x0.05 В)
        uint16_t bv = canMsgRead.data[4] | ((uint16_t)canMsgRead.data[5] << 8);
        if (bv != 0xFFFF) { battV10 = (int)(bv / 2); dataValid |= DV_BATT; }       // x0.05 В -> десятые
      } else if (pgn == 65266) {                       // LFE — расход топлива (SPN183, b1-2, x0.05 л/ч)
        uint16_t fr = canMsgRead.data[0] | ((uint16_t)canMsgRead.data[1] << 8);
        if (fr != 0xFFFF) { fuelRate10 = (int)(fr / 2); dataValid |= DV_FUEL; }    // x0.05 -> десятые
      } else if (pgn == 61443) {                       // EEC2 — нагрузка двигателя (SPN92, b3, %)
        if (canMsgRead.data[2] != 0xFF) { engLoad = (int)canMsgRead.data[2]; dataValid |= DV_LOAD; }
      }
#if ENABLE_DTC_READER
      else if (pgn == 65226) {                         // DM1 (активные), одиночный кадр
        parseDtcPayload(canMsgRead.data, canMsgRead.can_dlc, false);
        DBG(2) { ts(); Serial.print(F("DM1 1-frame, codes=")); Serial.println(dtcCount); }
      } else if (pgn == 65227) {                       // DM2 (сохранённые), одиночный кадр
        parseDtcPayload(canMsgRead.data, canMsgRead.can_dlc, true);
        DBG(2) { ts(); Serial.print(F("DM2 1-frame, codes=")); Serial.println(dtcCount); }
      } else if (pgn == 60416) {                       // TP.CM (анонс многокадрового)
        if (canMsgRead.data[0] == 0x20) {              // BAM
          uint16_t sz = canMsgRead.data[1] | ((uint16_t)canMsgRead.data[2] << 8);
          uint8_t  np = canMsgRead.data[3];
          uint32_t tg = (uint32_t)canMsgRead.data[5] | ((uint32_t)canMsgRead.data[6] << 8)
                        | ((uint32_t)canMsgRead.data[7] << 16);
          if (tg == 65226 || tg == 65227) {
            tpActive = true; tpStored = (tg == 65227);
            tpSize = (sz > TP_BUF) ? TP_BUF : sz;
            tpPackets = np;
            memset(tpBuf, 0xFF, TP_BUF);
            DBG(2) { ts(); Serial.print(F("BAM start pgn=")); Serial.print(tg); Serial.print(F(" size=")); Serial.print(sz); Serial.print(F(" pkts=")); Serial.println(np); }
          } else tpActive = false;
        }
      } else if (pgn == 60160) {                       // TP.DT (данные)
        if (tpActive) {
          uint8_t seq = canMsgRead.data[0];
          if (seq >= 1) {
            uint16_t off = (uint16_t)(seq - 1) * 7;
            for (uint8_t k = 0; k < 7; k++)
              if (off + k < TP_BUF) tpBuf[off + k] = canMsgRead.data[1 + k];
            if (seq >= tpPackets) {                    // последний пакет — разобрать
              parseDtcPayload(tpBuf, tpSize, tpStored);
              tpActive = false;
              DBG(2) { ts(); Serial.print(F("BAM done, total codes=")); Serial.println(dtcCount); }
            }
          }
        }
      }
#endif
    }
    if (++rxThisLoop >= RX_MAX_PER_LOOP) break;
  }

  // --- 1b. Живость шины + сброс состояния + авто-восстановление CAN ---
  static bool prevBusAlive = false;
  bool busAlive = (lastRxTime != 0) && (currentMillis - lastRxTime < BUS_TIMEOUT_MS);
  if (prevBusAlive && !busAlive) {                 // шина пропала (заглушили + выключили зажигание)
    resetRunState();
    DBG(1) { ts(); Serial.println(F("BUS lost -> full reset, wait for RPM")); }
  }
  prevBusAlive = busAlive;
  if (!busAlive) {
    realRpm = 0;                                   // нет шины -> мотор считаем остановленным
    if (currentMillis - lastCanReinit >= CAN_REINIT_MS) {
      lastCanReinit = currentMillis;
      canReinit();                                 // вдруг мы в bus-off — переинициализируем контроллер
      DBG(1) { ts(); Serial.println(F("CAN re-init (bus down)")); }
    }
  }

  // --- 2. ОПРОС ПЕДАЛИ ---
  adcTotal -= adcReadings[readIndex];
  adcReadings[readIndex] = analogRead(PIN_POT);
  adcTotal += adcReadings[readIndex];
  readIndex = (readIndex + 1) % NUM_SAMPLES;
  int averageADC = adcTotal / NUM_SAMPLES;
  int pedalPercent = ((long)averageADC * 100) / 1023;

#if BENCH_TEST_MODE
  engineRunning = true;
#else
  if (realRpm > ENGINE_RUNNING_RPM)      engineRunning = true;
  else if (realRpm < ENGINE_STOPPED_RPM) engineRunning = false;
#endif
  static bool prevRunning = false;
  if (engineRunning != prevRunning) {
    DBG(1) { ts(); Serial.print(F("ENGINE ")); Serial.print(prevRunning ? F("RUN->STOP") : F("STOP->RUN")); Serial.print(F(" rpm=")); Serial.println((int)realRpm); }
    if (!engineRunning) { phase = PH_WAITRUN; revActive = false; commandedRpm = idleBaseline; }  // заглушили -> переарм цикла
    prevRunning = engineRunning;
  }

#if BENCH_TEST_MODE
  targetRpm = potToRpm(averageADC); revActive = true;
#else
  runPhases(currentMillis, averageADC);
#endif
  static Phase prevPhase = PH_WAITRUN;
  if (phase != prevPhase) {
    DBG(1) { ts(); Serial.print(F("PHASE ")); Serial.print(phaseName(prevPhase)); Serial.print(F("->")); Serial.println(phaseName(phase)); }
    prevPhase = phase;
  }
  static bool prevRev = false;
  if (revActive != prevRev) {
    DBG(2) { ts(); if (revActive) { Serial.print(F("CMD speed target=")); Serial.println(targetRpm); } else { Serial.print(F("CMD disable, idle base=")); Serial.println(idleBaseline); } }
    prevRev = revActive;
  }

  // --- 3. ОТПРАВКА TSC1 (10 мс) ---
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentMillis;
    if (revActive) {
      if (commandedRpm < targetRpm)      commandedRpm = min(commandedRpm + RPM_RAMP, targetRpm);
      else if (commandedRpm > targetRpm) commandedRpm = max(commandedRpm - RPM_RAMP, targetRpm);
      buildTSC1speed(commandedRpm);
    } else {
      commandedRpm = idleBaseline;
      buildTSC1disable();
    }
    if (busAlive) {
      if (mcp2515.sendMessage(&canMsgTSC1) == MCP2515::ERROR_OK) { txOk++; txFailStreak = 0; }
      else {
        txFail++; DBG(1) { ts(); Serial.println(F("TSC1 TX FAIL")); }
        if (++txFailStreak >= TX_FAIL_RECOVER) { canReinit(); txFailStreak = 0; DBG(1) { ts(); Serial.println(F("CAN re-init (tx fail)")); } }
      }
#if ENABLE_BUSOFF_REG_CHECK
      if (mcp2515.getErrorFlags() & 0x20) {          // бит TXBO (Bus-Off) -> мгновенно поднимаем контроллер
        canReinit(); txFailStreak = 0;
        DBG(1) { ts(); Serial.println(F("CAN re-init (TXBO)")); }
      }
#endif
    }
  }

#if ENABLE_PERMISSIVES
  if (busAlive && currentMillis - lastPermTime >= PERM_INTERVAL) { lastPermTime = currentMillis; sendPermissives(); }
#endif

  // --- 4. КНОПКИ И ИНТЕРФЕЙС DTC ---
#if ENABLE_DTC_READER
  if (ui == UI_MSG && (long)(currentMillis - msgUntil) >= 0) ui = uiAfterMsg;

  bool eRead = btnEdge(bRead);
  bool eClr  = btnEdge(bClr);
  bool eBack = btnEdge(bBack);
  DBG(1) {
    if (eRead) { ts(); Serial.println(F("BTN READ")); }
    if (eClr)  { ts(); Serial.println(F("BTN CLEAR")); }
    if (eBack) { ts(); Serial.println(F("BTN BACK")); }
  }

  if (eBack) {
    scrollPos = 0;
    ui = (ui == UI_CONFIRM_CLR) ? UI_DTC : UI_MAIN;
  } else if (eRead) {
    if (ui == UI_MAIN) { ui = UI_DATA; dataPage = 0; }
    else if (ui == UI_DATA) {
      if (dataPage < DATA_PAGES - 1) dataPage++;
      else { ui = UI_DTC; dtcView = 0; scrollPos = 0; requestDM2(); printDtcSerial(); }
    } else if (ui == UI_DTC) {
      if (dtcCount && dtcView < dtcCount - 1) { dtcView++; scrollPos = 0; }
      else ui = UI_MAIN;
    }
  } else if (eClr) {
    if (ui == UI_DTC) ui = UI_CONFIRM_CLR;
    else if (ui == UI_CONFIRM_CLR) { clearDtcs(); requestDM2(); showMsg("Sbros otpravlen", "DM11 + DM3", 1500, UI_DTC); DBG(1) { ts(); Serial.println(F("CLEAR sent: DM11+DM3")); } }
  }
  static UiMode prevUi = UI_MAIN;
  if (ui != prevUi) { DBG(1) { ts(); Serial.print(F("UI ")); Serial.print(uiName(prevUi)); Serial.print(F("->")); Serial.println(uiName(ui)); } prevUi = ui; }
#endif

  // --- 5. LCD ---
  if (currentMillis - lastLcdTime >= LCD_INTERVAL) {
    lastLcdTime = currentMillis;
#if ENABLE_DTC_READER
    if (ui == UI_DATA) { renderDataUi(); }
    else if (ui != UI_MAIN) { renderDtcUi(); }
    else
#endif
    {
      char buf1[17]; char buf2[17];
      switch (phase) {
        case PH_WAITRUN:
          snprintf(buf1, sizeof(buf1), "START ENGINE    ");
#if ENABLE_AUTO_SWEEP
          snprintf(buf2, sizeof(buf2), "(auto address)  ");
#else
          snprintf(buf2, sizeof(buf2), "SA fixed: 0x%02X", activeAddr);
#endif
          break;
        case PH_SETTLE:
          snprintf(buf1, sizeof(buf1), "SETTLE...       ");
          snprintf(buf2, sizeof(buf2), "A:%4d T:%3dC", (int)realRpm, engineTemp);
          break;
        case PH_SCAN:
          snprintf(buf1, sizeof(buf1), "SCAN:%02X A:%4d ", activeAddr, (int)realRpm);
          snprintf(buf2, sizeof(buf2), "A:%4d T:%3dC", (int)realRpm, engineTemp);
          break;
        case PH_RUN:
        default:
          snprintf(buf1, sizeof(buf1), "S:%4d A:%4d   ", commandedRpm, (int)realRpm);
          snprintf(buf2, sizeof(buf2), "T:%3dC  SA:%02X  ", engineTemp, activeAddr);
          break;
      }
      lcd.setCursor(0, 0); lcd.print(buf1);
      lcd.setCursor(0, 1); lcd.print(buf2);
    }
  }

  // --- 6. SERIAL (heartbeat) ---
  if (currentMillis - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = currentMillis;
    int rxPerSec = canRxCount * 2; canRxCount = 0;
    ts();
    Serial.print(F("HB ph="));  Serial.print(phaseName(phase));
#if ENABLE_DTC_READER
    Serial.print(F(" ui="));    Serial.print(uiName(ui));
#endif
    Serial.print(engineRunning ? F(" run=1") : F(" run=0"));
    Serial.print(busAlive ? F(" bus=1") : F(" bus=0"));
    Serial.print(F(" ped="));   Serial.print(pedalPercent); Serial.print('%');
    Serial.print(F(" SA=0x"));  Serial.print(activeAddr, HEX);
    Serial.print(revActive ? F(" cmd") : F(" idle"));
    Serial.print(F(" set="));   Serial.print(revActive ? commandedRpm : idleBaseline);
    Serial.print(F(" act="));   Serial.print((int)realRpm);
    Serial.print(F(" T="));     Serial.print(engineTemp);
    Serial.print(F(" rxps="));  Serial.print(rxPerSec);
    Serial.print(F(" txOk="));  Serial.print(txOk);
    Serial.print(F(" txFail="));Serial.print(txFail);
#if ENABLE_DTC_READER
    Serial.print(F(" dtc="));   Serial.print(dtcCount);
    Serial.print(F(" lamp=0x"));Serial.print(lampStatus, HEX);
#endif
#if DEBUG_MCP_REGS
    Serial.print(F(" eflg=0x"));Serial.print(mcp2515.getErrorFlags(), HEX);
#endif
    Serial.print(F(" ram="));   Serial.print(freeRam());
    Serial.println();
  }
}
