#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdarg.h>

// ============================================================
//  VCM Iveco Cursor 13 / KamAZ (Bosch EDC7UC31 и др.) — стендовый пульт.
//  FW 75.
//  Управление оборотами по J1939 (TSC1, PGN 0) + чтение/сброс кодов
//  неисправностей (DM1/DM2) + живые данные двигателя.
//
//  НОВОЕ В FW 75 — ПРЕСЕТЫ ЭБУ И АВТООПОЗНАНИЕ:
//    * 4 пресета (IVECO, KAMAZ, ECU-3, ECU-4), у каждого свой адрес источника
//      (SA) и режим: FIXED (зашитый адрес) или AUTO (перебор адресов).
//      Хранятся в EEPROM, правятся с пульта, перепрошивка не нужна.
//    * Автоопознание ЭБУ: при включении зажигания пульт запрашивает у ЭБУ
//      идентификацию (Component ID 65259, Software ID 65242, Address Claim
//      60928), считает «отпечаток» и, если он уже привязан к пресету,
//      сам включает нужный пресет. Привязка делается ручным выбором пресета.
//    * Найденный автоподбором адрес сохраняется в пресет одной кнопкой.
//
//  ЭКРАН: стандартный 1602 (HD44780) без кириллицы — надписи транслитом.
//
//  КНОПКИ (каждая между пином и GND, внутр. подтяжка). Короткое нажатие
//  срабатывает при отпускании, длинное — через BTN_LONG_MS удержания:
//    READ  (D3) коротко — главный экран -> страницы данных -> ошибки -> главный;
//               в списке ошибок — следующий код; в меню — следующий пункт.
//          (D3) ДЛИННО на главном экране (двигатель остановлен) — следующий
//               пресет (применяется, пишется в EEPROM, текущий ЭБУ привязывается).
//    CLEAR (D4) коротко — в списке ошибок: сброс (с подтверждением);
//               в меню: изменить/подтвердить; в вопросе «sohranit?» — да.
//    BACK  (D5) коротко — назад / отмена; во время поиска адреса — остановить поиск.
//          (D5) ДЛИННО на главном экране (двигатель остановлен) — меню настроек.
//  Любая кнопка закрывает информационное сообщение и сразу обрабатывается.
//  На главном экране '*' после адреса = ЭБУ подтверждает (SPN 1483), что слушает пульт.
//
//  Разведка нового ЭБУ без перепрошивки — команды в Serial Monitor (serialCmdHelp, README разд. 13).
//
//  !!! TSC1 реально крутит двигатель. Аварийный стоп под рукой. !!!
//
// ============================================================
//  КОНФИГУРАЦИЯ
// ============================================================
#define FW_VERSION          "75"

// --- Адреса источника (SA) ---
#define TSC1_SOURCE_ADDR    0x27    // адрес по умолчанию для пресета IVECO (подтверждён на стенде)
#define TSC1_BYTE5          0xFF    // пятый байт TSC1 (Control Purpose / резерв), 0xFF = не задавать
// Кандидаты для режима AUTO. Первым всегда пробуется адрес, записанный в пресете.
const uint8_t SWEEP_ADDRS[] PROGMEM = {0x03, 0x0B, 0x10, 0x11, 0x21, 0x27, 0x31, 0xF9};
#define SWEEP_PROBE_MARGIN  450     // на сколько выше холостого просим обороты во время пробы адреса
#define SWEEP_DETECT_MARGIN 250     // рост оборотов, который считается «ЭБУ откликнулся»
#define SWEEP_SETTLE_MS     1500    // выдержка после пуска перед замером холостого
#define SWEEP_TEST_MS       2500    // сколько пробовать каждый адрес

// --- Пресеты ЭБУ (EEPROM) ---
#define PRESET_COUNT        4
#define FP_PER_PRESET       6       // сколько «отпечатков» разных ЭБУ помнит один пресет
#define MODE_FIXED          0
#define MODE_AUTO           1
#define EE_MAGIC            0xA5
#define EE_VERSION          1

// --- Автоопознание ЭБУ ---
#define ENABLE_ECU_ID       1       // 0 = не запрашивать идентификацию (только ручной выбор пресета)
#define ID_REQ_DELAY_MS     400     // пауза после оживления шины перед первым запросом
#define ID_SRC_WAIT_MS      1000    // ожидание ответа (или NACK) на один запрос; запросы идут строго по одному
#define ID_ATTEMPTS         3       // полных циклов опроса, если ЭБУ не ответил / ответил не на всё
#define ID_RETRY_GAP_MS     300

// --- Обороты (общие для всех пресетов) ---
#define RPM_MIN             650
#define RPM_MAX             2200    // главный лимит безопасности
#define RPM_RAMP            10      // об/мин за цикл отправки (10 мс)
#define RUN_CMD_DEADBAND    100     // мёртвая зона над холостым

// --- Детекция работы двигателя (гистерезис) ---
#define ENGINE_RUNNING_RPM  350
#define ENGINE_STOPPED_RPM  150

// --- Тестовый режим стенда (проверка педали и меню БЕЗ ЭБУ) ---
#define BENCH_TEST_MODE     0

// --- Прочее ---
#define ENABLE_PERMISSIVES  0
#define ENABLE_WATCHDOG     1       // сторожевой таймер: зависание МК -> сброс, TSC1 пропадает, EDC7 роняет обороты в холостой
#define ENABLE_BUSOFF_REG_CHECK 1   // детект bus-off по флагу TXBO (нужен getErrorFlags(); не компилится -> 0)

// --- DTC-ридер ---
#define ENABLE_DTC_READER   1
#define DTC_MAX             15      // макс. кодов в буфере
#define TP_BUF              48      // буфер сборки многокадровых DM1/DM2 (BAM, ~11 кодов)
#define RX_MAX_PER_LOOP     8       // макс. CAN-кадров за проход loop

// --- Кнопки ---
#define PIN_BTN_READ        3
#define PIN_BTN_CLEAR       4
#define PIN_BTN_BACK        5
#define BTN_DEBOUNCE_MS     40
#define BTN_LONG_MS         1200    // длинное нажатие
#define BTN_REPEAT_MS       120     // автоповтор при удержании (редактор адреса)
#define SAVE_SA_PROMPT_MS   10000   // сколько висит вопрос «SA najden, sohranit?»

// --- Команды в Serial Monitor (разведка ЭБУ без перепрошивки, см. serialCmdHelp) ---
#define ENABLE_SERIAL_CMD   1
#define RECON_MS            3000    // сколько печатать ответы после команды q <PGN>
#define RECON_MAX_FRAMES    24      // и не больше стольких кадров/пакетов

// --- Отладочные логи (Serial 115200) ---
#define DEBUG_LEVEL     2   // 0 тихо | 1 +события | 2 +инвентарь CAN и детали DTC/ID | 3 трейс каждого кадра (тормозит loop!)
#define DEBUG_MCP_REGS  0   // 1 = печатать регистр ошибок MCP2515 (нужен getErrorFlags(); не компилится -> 0)
#define DBG(lvl) if (DEBUG_LEVEL >= (lvl))

// --- Живость шины / авто-восстановление CAN ---
#define BUS_TIMEOUT_MS  600   // нет приёма дольше -> шина мертва (не шлём, чтобы не словить bus-off)
#define CAN_REINIT_MS   1000  // как часто переинициализировать MCP2515, пока шина мертва
#define TX_FAIL_RECOVER 16    // столько неудачных отправок подряд -> переинициализация CAN
#define PGN_SEEN_MAX    16    // размер таблицы инвентаря PGN

// --- Дисплей ---
#define LCD_CHARS_PER_LOOP 1  // сколько символов дописывать на LCD за один проход loop (1 ≈ 1.5 мс блокировки)

// --- Экран живых данных двигателя ---
#define DATA_PAGES   4
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
struct can_frame canMsgRead;
struct can_frame canMsgReq;
#if ENABLE_PERMISSIVES
struct can_frame canMsgPerm;
#endif
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

uint16_t realRpm   = 0;                      // об/мин (целое, SPN 190 / 8)
uint8_t  ctrlSa    = 0xFF;                   // SPN 1483: адрес устройства, которое сейчас управляет двигателем (0xFF = н/д)
int   engineTemp   = -40;
int   commandedRpm = RPM_MIN;
int   targetRpm    = RPM_MIN;
bool  revActive    = false;
bool  engineRunning = false;
bool  busAlive     = false;
int   canRxCount   = 0;
unsigned long txOk = 0, txFail = 0;
unsigned long lastRxTime = 0;
unsigned long lastCanReinit = 0;
int txFailStreak = 0;
// --- Живые данные двигателя (J1939) ---
int oilP = 0, oilT = 0, boostKpa = 0, airT = 0, battV10 = 0, fuelRate10 = 0, engLoad = 0;
uint8_t dataValid = 0;
#if DEBUG_LEVEL >= 2
uint16_t seenPgn[PGN_SEEN_MAX]; uint8_t seenCnt = 0;
#endif

// ============================================================
//  ПРЕСЕТЫ (зеркало EEPROM в RAM) И АВТОПОДБОР АДРЕСА
// ============================================================
uint8_t  curPreset = 0;
uint8_t  pSa[PRESET_COUNT];
uint8_t  pMode[PRESET_COUNT];
uint8_t  sweepIndex = 0;
int      idleBaseline = RPM_MIN;
uint8_t  activeAddr;
uint32_t activeCanId;

const char PN0[] PROGMEM = "IVECO";
const char PN1[] PROGMEM = "KAMAZ";
const char PN2[] PROGMEM = "ECU-3";
const char PN3[] PROGMEM = "ECU-4";
const char* const PRESET_NAMES[] PROGMEM = {PN0, PN1, PN2, PN3};

struct SaName { uint8_t sa; char name[11]; };
const SaName SA_NAMES[] PROGMEM = {
  {0x03, "Transmiss."}, {0x0B, "Brakes/ABS"}, {0x10, "Retarder"},  {0x11, "Cruise ctl"},
  {0x21, "Body ctrl"},  {0x27, "Mgmt comp."}, {0x31, "Cab ctrl"},  {0xF9, "Diag tool"}
};
#define SA_NAMES_N (sizeof(SA_NAMES) / sizeof(SA_NAMES[0]))

// ---- Типы: объявлены ДО первой функции (иначе авто-прототипы Arduino IDE ломают сборку) ----
enum Phase  : uint8_t { PH_WAITRUN, PH_SETTLE, PH_SCAN, PH_RUN };
enum UiMode : uint8_t { UI_MAIN, UI_DATA, UI_DTC, UI_CONFIRM_CLR, UI_MSG, UI_MENU, UI_EDIT_SA, UI_CONFIRM_RESET, UI_SAVE_SA };
enum BtnEv  : uint8_t { BE_NONE, BE_SHORT, BE_LONG, BE_REPEAT };
enum IdState: uint8_t { ID_IDLE, ID_REQ, ID_WAIT, ID_DONE, ID_NONE };
struct Btn  { uint8_t pin; uint8_t flags; uint16_t tChange; uint16_t tPress; uint16_t tRep; };
struct Dtc  { uint32_t spn; uint8_t fmi; uint8_t oc; uint8_t cm; bool stored; };

Phase phase;
unsigned long phaseTimer = 0;

// ============================================================
//  ИНТЕРФЕЙС: КАДРОВЫЙ БУФЕР LCD, СООБЩЕНИЯ, КНОПКИ
// ============================================================
char     fb[32];                 // 2 строки по 16 символов — что должно быть на экране
uint32_t fbDirty = 0;            // биты символов, которые ещё не дописаны на LCD
uint8_t  fbPos   = 0;            // позиция сканирования буфера
uint8_t  lcdCur  = 0xFF;         // где сейчас курсор LCD (0xFF = неизвестно)

UiMode   ui = UI_MAIN;
UiMode   uiAfterMsg = UI_MAIN;
unsigned long msgUntil = 0;
uint8_t  dataPage = 0;
uint8_t  menuItem = 0;           // 0 режим SA, 1 адрес SA, 2 ECU ID, 3 сброс настроек
uint8_t  editSa = 0;
bool     menuShowFp = false;
uint8_t  foundSa = 0xFF;         // адрес, найденный автоподбором (вопрос «sohranit?»)
unsigned long saveSaUntil = 0;
Btn bRead, bClr, bBack;

// ============================================================
//  DTC: ХРАНИЛИЩЕ
// ============================================================
#if ENABLE_DTC_READER
Dtc      dtcList[DTC_MAX];
uint8_t  dtcCount = 0;
uint8_t  dtcView  = 0;
uint8_t  lampStatus = 0;
unsigned int scrollPos = 0;
uint8_t  tpBuf[TP_BUF];
#endif

// сборка многокадровых (BAM): общая для DM1/DM2 и идентификации ЭБУ
#if ENABLE_DTC_READER || ENABLE_ECU_ID
bool     tpActive = false;
uint16_t tpPgn = 0;
uint16_t tpSize = 0;
uint8_t  tpPackets = 0, tpSeq = 0;
#endif

// ============================================================
//  АВТООПОЗНАНИЕ ЭБУ
// ============================================================
#if ENABLE_ECU_ID
uint8_t  idState = ID_IDLE, idStep = 0, idAttempt = 0;
uint8_t  idSrc = 0;              // получено: бит0 = Component ID, бит1 = Software ID, бит2 = Address Claim NAME
uint8_t  idNack = 0;             // ЭБУ ответил NACK (не поддерживает), те же биты
uint16_t crcCI = 0xFFFF, crcSOFT = 0xFFFF, crcACL = 0xFFFF;
uint16_t idFp = 0;               // итоговый отпечаток ЭБУ (0 = нет)
int8_t   idMatch = -1;           // пресет, к которому привязан отпечаток (-1 = не привязан)
int8_t   pendingAuto = -1;       // пресет, который надо включить автоматически, когда это станет безопасно
unsigned long idTimer = 0;
char     ecuName[17];            // первые 16 символов ответа ЭБУ (для экрана)
#endif

#if ENABLE_SERIAL_CMD
char     cmdBuf[8]; uint8_t cmdLen = 0;
uint16_t reconPgn = 0; unsigned long reconUntil = 0; uint8_t reconLeft = 0;
#endif

// ============================================================
//  ТАБЛИЦЫ FMI / SPN (PROGMEM, транслит)
// ============================================================
#if ENABLE_DTC_READER
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

// SPN двигателя (SAE), транслит. Общие для Cursor 13 и KamAZ 740 на EDC7; плюс EGR/SCR для других калибровок.
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
const char n411[]  PROGMEM = "EGR perepad davl.";
const char n412[]  PROGMEM = "EGR temperatura";
const char n611[]  PROGMEM = "Provodka forsunok";
const char n636[]  PROGMEM = "Datchik kolenvala";
const char n651[]  PROGMEM = "Forsunka cil.1";
const char n652[]  PROGMEM = "Forsunka cil.2";
const char n653[]  PROGMEM = "Forsunka cil.3";
const char n654[]  PROGMEM = "Forsunka cil.4";
const char n655[]  PROGMEM = "Forsunka cil.5";
const char n656[]  PROGMEM = "Forsunka cil.6";
const char n657[]  PROGMEM = "Forsunka cil.7";
const char n658[]  PROGMEM = "Forsunka cil.8";
const char n723[]  PROGMEM = "Datchik raspredval";
const char n1136[] PROGMEM = "Temp. ECU (blok)";
const char n1347[] PROGMEM = "Klapan rampy (ZME)";
const char n1485[] PROGMEM = "Glavnoe rele ECU";
const char n1761[] PROGMEM = "Uroven mocheviny";
const char n3031[] PROGMEM = "Temp. mocheviny";
const char n3226[] PROGMEM = "Datchik NOx";
const char n3251[] PROGMEM = "DPF perepad davl.";

const uint16_t SPN_ID[] PROGMEM = {
  91,94,97,98,100,102,105,107,108,110,111,132,157,158,168,174,175,190,411,412,
  611,636,651,652,653,654,655,656,657,658,723,1136,1347,1485,1761,3031,3226,3251
};
const char* const SPN_NM[] PROGMEM = {
  n91,n94,n97,n98,n100,n102,n105,n107,n108,n110,n111,n132,n157,n158,n168,n174,n175,n190,n411,n412,
  n611,n636,n651,n652,n653,n654,n655,n656,n657,n658,n723,n1136,n1347,n1485,n1761,n3031,n3226,n3251
};
const uint8_t SPN_N = sizeof(SPN_ID) / sizeof(SPN_ID[0]);
#endif  // ENABLE_DTC_READER

// ============================================================
//  ОБЩИЕ ХЕЛПЕРЫ
// ============================================================
void ts() { Serial.print('['); Serial.print(millis()); Serial.print(F("] ")); }

int freeRam() {                                // свободная SRAM между кучей и стеком
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

uint16_t crc16Update(uint16_t crc, uint8_t b) {   // CRC-16/CCITT-FALSE, poly 0x1021
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}
uint8_t crc8Update(uint8_t crc, uint8_t b) {      // CRC-8, poly 0x07
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  return crc;
}

PGM_P presetNameP(uint8_t p) { return (PGM_P)pgm_read_ptr(&PRESET_NAMES[p < PRESET_COUNT ? p : 0]); }
PGM_P modeNameP(uint8_t m)   { return m == MODE_AUTO ? PSTR("AUTO") : PSTR("FIX"); }

int8_t saNameIndex(uint8_t sa) {
  for (uint8_t i = 0; i < SA_NAMES_N; i++) if (pgm_read_byte(&SA_NAMES[i].sa) == sa) return i;
  return -1;
}
void saLabel(uint8_t sa, char* out, size_t n) {   // "0x27 Mgmt comp."
  int8_t i = saNameIndex(sa);
  if (i >= 0) { char nm[11]; memcpy_P(nm, SA_NAMES[i].name, 11); nm[10] = 0; snprintf_P(out, n, PSTR("0x%02X %s"), sa, nm); }
  else snprintf_P(out, n, PSTR("0x%02X"), sa);
}

void setActiveAddr(uint8_t addr) {
  activeAddr  = addr;
  activeCanId = (0x0C000000UL | (uint32_t)addr) | CAN_EFF_FLAG;
}
int potToRpm(int adc) {
  int r = RPM_MIN + ((long)adc * (RPM_MAX - RPM_MIN)) / 1023;
  if (r > RPM_MAX) r = RPM_MAX;
  if (r < RPM_MIN) r = RPM_MIN;
  return r;
}
int potToRevRpm(int adc) {
  int top = RPM_MAX; if (top < idleBaseline) top = idleBaseline;
  int r = idleBaseline + ((long)adc * (top - idleBaseline)) / 1023;
  if (r > top) r = top;
  if (r < idleBaseline) r = idleBaseline;
  return r;
}

// ============================================================
//  EEPROM: ПРЕСЕТЫ И ОТПЕЧАТКИ ЭБУ
// ============================================================
//  0: magic  1: version  2: текущий пресет  3: резерв
//  4 + p*16: пресет p -> [0] SA, [1] режим, [2..13] 6 отпечатков (uint16 LE), [14..15] резерв
//  68: CRC-8 байтов 0..67
#define EE_A_MAGIC 0
#define EE_A_VER   1
#define EE_A_CUR   2
#define EE_P_BASE  4
#define EE_P_SIZE  16
#define EE_P_SA    0
#define EE_P_MODE  1
#define EE_P_FP    2
#define EE_LEN     (EE_P_BASE + PRESET_COUNT * EE_P_SIZE)
#define EE_A_CRC   EE_LEN

int eePresetAddr(uint8_t p) { return EE_P_BASE + (int)p * EE_P_SIZE; }
uint8_t eeCalcCrc() { uint8_t c = 0; for (uint8_t i = 0; i < EE_LEN; i++) c = crc8Update(c, EEPROM.read(i)); return c; }
void eeCommit() { EEPROM.update(EE_A_CRC, eeCalcCrc()); }
uint16_t eeFpGet(uint8_t p, uint8_t k) {
  int a = eePresetAddr(p) + EE_P_FP + 2 * k;
  return (uint16_t)EEPROM.read(a) | ((uint16_t)EEPROM.read(a + 1) << 8);
}
void eeFpSet(uint8_t p, uint8_t k, uint16_t v) {
  int a = eePresetAddr(p) + EE_P_FP + 2 * k;
  EEPROM.update(a, v & 0xFF); EEPROM.update(a + 1, v >> 8);
}
void eeWriteDefaults() {
  for (uint8_t p = 0; p < PRESET_COUNT; p++) {
    int a = eePresetAddr(p);
    EEPROM.update(a + EE_P_SA,   p == 0 ? TSC1_SOURCE_ADDR : pgm_read_byte(&SWEEP_ADDRS[0]));
    EEPROM.update(a + EE_P_MODE, p == 0 ? MODE_FIXED : MODE_AUTO);
    for (uint8_t k = 0; k < FP_PER_PRESET; k++) eeFpSet(p, k, 0xFFFF);
    EEPROM.update(a + 14, 0xFF); EEPROM.update(a + 15, 0xFF);
    wdt_reset();
  }
  EEPROM.update(EE_A_CUR, 0);
  EEPROM.update(3, 0xFF);
  EEPROM.update(EE_A_MAGIC, EE_MAGIC);
  EEPROM.update(EE_A_VER, EE_VERSION);
  eeCommit();
}
bool eeLoad() {                                  // true = настройки прочитаны, false = записаны заводские
  bool ok = EEPROM.read(EE_A_MAGIC) == EE_MAGIC && EEPROM.read(EE_A_VER) == EE_VERSION
            && EEPROM.read(EE_A_CRC) == eeCalcCrc();
  if (!ok) eeWriteDefaults();
  curPreset = EEPROM.read(EE_A_CUR); if (curPreset >= PRESET_COUNT) curPreset = 0;
  for (uint8_t p = 0; p < PRESET_COUNT; p++) {
    int a = eePresetAddr(p);
    pSa[p]   = EEPROM.read(a + EE_P_SA); if (pSa[p] > 0xFD) pSa[p] = TSC1_SOURCE_ADDR;
    pMode[p] = EEPROM.read(a + EE_P_MODE) ? MODE_AUTO : MODE_FIXED;
  }
  return ok;
}
void eeSavePreset(uint8_t p) {
  int a = eePresetAddr(p);
  EEPROM.update(a + EE_P_SA, pSa[p]); EEPROM.update(a + EE_P_MODE, pMode[p]);
  eeCommit();
}
void eeSaveCur() { EEPROM.update(EE_A_CUR, curPreset); eeCommit(); }

int8_t fpFind(uint16_t fp) {                     // к какому пресету привязан отпечаток
  if (fp == 0 || fp == 0xFFFF) return -1;
  for (uint8_t p = 0; p < PRESET_COUNT; p++)
    for (uint8_t k = 0; k < FP_PER_PRESET; k++) if (eeFpGet(p, k) == fp) return p;
  return -1;
}
void fpBind(uint16_t fp, uint8_t p) {            // привязать отпечаток к пресету p (и отвязать от остальных)
  if (fp == 0 || fp == 0xFFFF) return;
  for (uint8_t q = 0; q < PRESET_COUNT; q++)
    for (uint8_t k = 0; k < FP_PER_PRESET; k++) if (eeFpGet(q, k) == fp) eeFpSet(q, k, 0xFFFF);
  for (uint8_t k = 0; k < FP_PER_PRESET; k++)
    if (eeFpGet(p, k) == 0xFFFF) { eeFpSet(p, k, fp); eeCommit(); return; }
  for (uint8_t k = 0; k < FP_PER_PRESET - 1; k++) eeFpSet(p, k, eeFpGet(p, k + 1));   // полон -> вытесняем старейший
  eeFpSet(p, FP_PER_PRESET - 1, fp);
  eeCommit();
}

// ============================================================
//  КАДРОВЫЙ БУФЕР LCD (неблокирующая отрисовка)
// ============================================================
void fbPut(uint8_t i, char c) { if (fb[i] != c) { fb[i] = c; fbDirty |= (1UL << i); } }
void fbText(uint8_t row, const char* s) {
  uint8_t base = row ? 16 : 0, i = 0;
  for (; i < 16 && s[i]; i++) fbPut(base + i, s[i]);
  for (; i < 16; i++) fbPut(base + i, ' ');
}
void fbTextP(uint8_t row, PGM_P s) {
  uint8_t base = row ? 16 : 0, i = 0;
  for (; i < 16; i++) { char c = pgm_read_byte(s + i); if (!c) break; fbPut(base + i, c); }
  for (; i < 16; i++) fbPut(base + i, ' ');
}
void fbPrintf(uint8_t row, PGM_P fmt, ...) {
  char b[17]; va_list ap; va_start(ap, fmt); vsnprintf_P(b, sizeof(b), fmt, ap); va_end(ap);
  fbText(row, b);
}
void fbScroll(uint8_t row, const char* s, unsigned int pos) {   // бегущая строка, если длиннее 16
  unsigned int len = strlen(s);
  if (len <= 16) { fbText(row, s); return; }
  uint8_t base = row ? 16 : 0; unsigned int total = len + 3;
  for (uint8_t i = 0; i < 16; i++) { unsigned int idx = (pos + i) % total; fbPut(base + i, (idx < len) ? s[idx] : ' '); }
}
void fbPump() {                                  // дописать на LCD не больше LCD_CHARS_PER_LOOP символов
  uint8_t done = 0;
  for (uint8_t k = 0; k < 32 && fbDirty && done < LCD_CHARS_PER_LOOP; k++) {
    uint8_t i = fbPos; fbPos = (fbPos + 1) & 31;
    if (!(fbDirty & (1UL << i))) continue;
    if (lcdCur != i) lcd.setCursor(i & 15, i >> 4);
    lcd.write((uint8_t)fb[i]);
    fbDirty &= ~(1UL << i);
    lcdCur = ((i & 15) == 15) ? 0xFF : i + 1;    // на границе строк DDRAM не непрерывна
    done++;
  }
}
void showMsgP(PGM_P a, PGM_P b, uint16_t dur, UiMode after) {
  fbTextP(0, a); fbTextP(1, b);
  msgUntil = millis() + dur; uiAfterMsg = after; ui = UI_MSG;
}
void showMsg(const char* a, const char* b, uint16_t dur, UiMode after) {
  fbText(0, a); fbText(1, b);
  msgUntil = millis() + dur; uiAfterMsg = after; ui = UI_MSG;
}

// ============================================================
//  КНОПКИ: короткое (при отпускании), длинное, автоповтор
// ============================================================
#define BF_PRESSED 1
#define BF_LONG    2
BtnEv btnPoll(Btn &b, bool repeat) {
  uint16_t now = (uint16_t)millis();
  bool raw = (digitalRead(b.pin) == LOW);
  bool pressed = b.flags & BF_PRESSED;
  if (raw != pressed) {
    if ((uint16_t)(now - b.tChange) > BTN_DEBOUNCE_MS) {
      b.tChange = now;
      if (raw) { b.flags = BF_PRESSED; b.tPress = now; }
      else { bool wasLong = b.flags & BF_LONG; b.flags = 0; if (!wasLong) return BE_SHORT; }
    }
    return BE_NONE;
  }
  if (pressed) {
    if (!(b.flags & BF_LONG)) {
      if ((uint16_t)(now - b.tPress) >= BTN_LONG_MS) { b.flags |= BF_LONG; b.tRep = now; return BE_LONG; }
    } else if (repeat && (uint16_t)(now - b.tRep) >= BTN_REPEAT_MS) { b.tRep = now; return BE_REPEAT; }
  }
  return BE_NONE;
}

// ============================================================
//  J1939: ЗАПРОСЫ, DTC
// ============================================================
void sendReqPgn(uint32_t pgn, uint8_t dest) {    // J1939 Request (PGN 59904), DLC 3 по J1939-21
  uint32_t id = ((uint32_t)6 << 26) | ((uint32_t)0xEA << 16) | ((uint32_t)dest << 8) | activeAddr;
  canMsgReq.can_id  = id | CAN_EFF_FLAG;
  canMsgReq.can_dlc = 3;
  canMsgReq.data[0] = pgn & 0xFF;
  canMsgReq.data[1] = (pgn >> 8) & 0xFF;
  canMsgReq.data[2] = (pgn >> 16) & 0xFF;
  mcp2515.sendMessage(&canMsgReq);
}

#if ENABLE_DTC_READER
void requestDM2() { sendReqPgn(65227, 0xFF); }   // глобальный запрос -> ЭБУ отвечает широковещательно (BAM)
void clearDtcs()  { sendReqPgn(65235, 0x00); sendReqPgn(65228, 0x00); } // DM11 + DM3

void lookupFmi(uint8_t fmi, char* out, size_t n) {
  if (fmi > 31) { snprintf_P(out, n, PSTR("FMI %u"), (unsigned)fmi); return; }
  strncpy_P(out, (PGM_P)pgm_read_ptr(&FMI_T[fmi]), n); out[n - 1] = 0;
}
void lookupSpn(uint32_t spn, char* out, size_t n) {
  for (uint8_t i = 0; i < SPN_N; i++) {
    if (pgm_read_word(&SPN_ID[i]) == spn) {
      strncpy_P(out, (PGM_P)pgm_read_ptr(&SPN_NM[i]), n); out[n - 1] = 0; return;
    }
  }
  snprintf_P(out, n, PSTR("SPN %lu"), (unsigned long)spn);
}
void buildDtcDesc(char* out, size_t n, const Dtc &d) {
  char sN[22], fN[24];
  lookupSpn(d.spn, sN, sizeof(sN));
  lookupFmi(d.fmi, fN, sizeof(fN));
  if (d.oc > 0 && d.oc < 0x7F) snprintf_P(out, n, PSTR("%s: %s (x%u)"), sN, fN, (unsigned)d.oc);
  else                        snprintf_P(out, n, PSTR("%s: %s"), sN, fN);
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
    // SAE J1939-73, SPN Conversion Method 4 (CM=0). FMI и OC от метода не зависят.
    uint32_t spn = (uint32_t)b0 | ((uint32_t)b1 << 8) | (((uint32_t)(b2 >> 5)) << 16);
    uint8_t fmi = b2 & 0x1F;
    uint8_t oc  = b3 & 0x7F;
    uint8_t cm  = b3 >> 7;
    if (spn == 0 && fmi == 0) break;             // нет (больше) ошибок
    dtcList[dtcCount].spn = spn; dtcList[dtcCount].fmi = fmi; dtcList[dtcCount].oc = oc;
    dtcList[dtcCount].cm = cm; dtcList[dtcCount].stored = stored;
    dtcCount++;
    i += 4;
  }
  if (dtcView >= dtcCount) { dtcView = 0; scrollPos = 0; }
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
//  ПРЕСЕТЫ И АВТОПОДБОР
// ============================================================
uint8_t sweepCount() { return 1 + sizeof(SWEEP_ADDRS); }
uint8_t sweepAddr(uint8_t i) { return i == 0 ? pSa[curPreset] : pgm_read_byte(&SWEEP_ADDRS[i - 1]); }
void sweepNext() {
  uint8_t n = sweepCount();
  do { sweepIndex = (sweepIndex + 1) % n; } while (sweepIndex != 0 && sweepAddr(sweepIndex) == pSa[curPreset]);
  setActiveAddr(sweepAddr(sweepIndex));
}

#if ENABLE_ECU_ID
void idCycleReset() {                           // чистый лист перед (повторным) циклом опроса
  idStep = 0; idSrc = 0; idNack = 0;
  crcCI = crcSOFT = crcACL = 0xFFFF;
  memset(ecuName, 0, sizeof(ecuName));
}
void idReset() {
  idState = ID_IDLE; idAttempt = 0; idFp = 0; idMatch = -1;
  idCycleReset();
}
#endif

void applyPreset(uint8_t p, bool manual) {       // manual = выбор оператора (привязывает текущий ЭБУ к пресету)
  if (p >= PRESET_COUNT) p = 0;
  curPreset = p; eeSaveCur();
  sweepIndex = 0; setActiveAddr(pSa[p]);
  idleBaseline = RPM_MIN; commandedRpm = RPM_MIN; targetRpm = RPM_MIN; revActive = false;
#if ENABLE_ECU_ID
  bool bound = false;
  if (manual && idState == ID_DONE) { fpBind(idFp, p); idMatch = p; bound = true; }
  pendingAuto = -1;
#endif
  DBG(1) {
    ts(); Serial.print(F("PRESET -> ")); Serial.print((const __FlashStringHelper*)presetNameP(p));
    Serial.print(F(" SA=0x")); Serial.print(pSa[p], HEX);
    Serial.print(F(" mode=")); Serial.print((const __FlashStringHelper*)modeNameP(pMode[p]));
    Serial.print(manual ? F(" (manual") : F(" (auto"));
#if ENABLE_ECU_ID
    if (bound) { Serial.print(F(", ECU fp=0x")); Serial.print(idFp, HEX); Serial.print(F(" privyazan")); }
#endif
    Serial.println(')');
  }
}

// ============================================================
//  АВТООПОЗНАНИЕ ЭБУ
// ============================================================
#if ENABLE_ECU_ID
void idStart(unsigned long now) { idReset(); idState = ID_REQ; idTimer = now + ID_REQ_DELAY_MS; }

void idFeed(uint8_t src, const uint8_t* d, uint8_t n, uint16_t off) {   // src 0=CI 1=SOFT 2=NAME; off = смещение в сообщении
  uint16_t* c = (src == 0) ? &crcCI : (src == 1) ? &crcSOFT : &crcACL;
  for (uint8_t k = 0; k < n; k++) {
    *c = crc16Update(*c, d[k]);
    if (src == 0 && off + k < 16) { char ch = (char)d[k]; ecuName[off + k] = (ch >= 32 && ch < 127) ? ch : ' '; }
  }
  DBG(2) {
    ts(); Serial.print(F("ID src")); Serial.print(src); Serial.print(F(" @")); Serial.print(off); Serial.print(F(": "));
    for (uint8_t k = 0; k < n; k++) {
      if (src == 2) { if (d[k] < 16) Serial.print('0'); Serial.print(d[k], HEX); Serial.print(' '); }
      else { char ch = (char)d[k]; Serial.print((ch >= 32 && ch < 127) ? ch : '.'); }
    }
    Serial.println();
  }
}
uint16_t idCombine() {
  uint16_t fp = 0;
  if (idSrc & 1) fp ^= crcCI;
  if (idSrc & 2) fp ^= (uint16_t)((crcSOFT << 5) | (crcSOFT >> 11));
  if (idSrc & 4) fp ^= (uint16_t)((crcACL << 10) | (crcACL >> 6));
  if (fp == 0 || fp == 0xFFFF) fp = 1;
  return fp;
}
void idFinalize() {
  idFp = idCombine(); idMatch = fpFind(idFp); idState = ID_DONE;
  if (idMatch >= 0 && idMatch != (int8_t)curPreset) pendingAuto = idMatch;   // применится, когда безопасно (двигатель стоит, главный экран)
  DBG(1) {
    ts(); Serial.print(F("ID done src=")); Serial.print(idSrc, BIN); Serial.print(F(" fp=0x")); Serial.print(idFp, HEX);
    Serial.print(F(" name='")); Serial.print(ecuName); Serial.print(F("' match="));
    if (idMatch >= 0) Serial.println((const __FlashStringHelper*)presetNameP(idMatch)); else Serial.println(F("none"));
  }
  if (ui == UI_MAIN) {
    if (idMatch == (int8_t)curPreset) {
      char l2[17]; snprintf_P(l2, sizeof(l2), PSTR("%S SA:%02X %S"), presetNameP(curPreset), pSa[curPreset], modeNameP(pMode[curPreset]));
      showMsg("ECU opoznan:", l2, 1500, UI_MAIN);
    } else if (idMatch < 0) {
      showMsgP(PSTR("ECU neizvesten"), PSTR("READ 1.2s=preset"), 2000, UI_MAIN);
    }
  }
}
// Запросы идут по одному: следующий только после ответа, NACK или таймаута на предыдущий,
// чтобы ЭБУ не пришлось отвечать двумя BAM одновременно.
void idTick(unsigned long now) {
  switch (idState) {
    case ID_REQ:
      if (busAlive && (long)(now - idTimer) >= 0) {
        sendReqPgn(idStep == 0 ? 65259UL : idStep == 1 ? 65242UL : 60928UL, 0xFF);
        idState = ID_WAIT; idTimer = now + ID_SRC_WAIT_MS;
        DBG(2) { ts(); Serial.print(F("ID request src")); Serial.print(idStep); Serial.print(F(" attempt ")); Serial.println(idAttempt + 1); }
      }
      break;
    case ID_WAIT: {
      uint8_t bit = (uint8_t)(1 << idStep);
      bool answered = ((idSrc | idNack) & bit) != 0;
      if (!answered && (long)(now - idTimer) < 0) break;
      idStep++;
      if (idStep < 3) { idState = ID_REQ; idTimer = now + 50; break; }
      // цикл из трёх запросов завершён
      bool complete = ((idSrc | idNack) == 7);              // на всё ответил (данными или NACK)
      bool known    = idSrc && (fpFind(idCombine()) >= 0);
      if (idSrc && (complete || known || idAttempt + 1 >= ID_ATTEMPTS)) idFinalize();
      else if (++idAttempt < ID_ATTEMPTS) { idCycleReset(); idState = ID_REQ; idTimer = now + ID_RETRY_GAP_MS; }
      else {
        idState = ID_NONE;
        DBG(1) { ts(); Serial.println(F("ID: net otveta ECU")); }
        if (ui == UI_MAIN) showMsgP(PSTR("ECU ID: net otv."), PSTR("vybor: READ 1.2s"), 1500, UI_MAIN);
      }
      break;
    }
    default: break;
  }
}
#endif  // ENABLE_ECU_ID

// ============================================================
//  ПЕЧАТЬ НАСТРОЕК И КОМАНДЫ SERIAL (разведка ЭБУ)
// ============================================================
void printPresets() {
  ts(); Serial.print(F("preset=")); Serial.println((const __FlashStringHelper*)presetNameP(curPreset));
  for (uint8_t p = 0; p < PRESET_COUNT; p++) {
    ts(); Serial.print(F("  ")); Serial.print(p + 1); Serial.print(' '); Serial.print((const __FlashStringHelper*)presetNameP(p));
    Serial.print(F(" SA=0x")); Serial.print(pSa[p], HEX);
    Serial.print(F(" mode=")); Serial.print((const __FlashStringHelper*)modeNameP(pMode[p]));
    Serial.print(F(" fp:"));
    for (uint8_t k = 0; k < FP_PER_PRESET; k++) { uint16_t v = eeFpGet(p, k); if (v != 0xFFFF) { Serial.print(F(" 0x")); Serial.print(v, HEX); } }
    Serial.println();
  }
}

#if ENABLE_SERIAL_CMD
bool reconActive() { return reconLeft && (long)(millis() - reconUntil) < 0; }
void reconPrint(uint16_t pgn, uint8_t sa, const uint8_t* d, uint8_t n, uint16_t off) {
  if (!reconLeft) return;
  reconLeft--;
  ts(); Serial.print(F("RECON pgn=")); Serial.print(pgn); Serial.print(F(" SA=0x")); Serial.print(sa, HEX);
  Serial.print(F(" @")); Serial.print(off); Serial.print(F(" hex:"));
  for (uint8_t k = 0; k < n; k++) { Serial.print(' '); if (d[k] < 16) Serial.print('0'); Serial.print(d[k], HEX); }
  Serial.print(F("  '"));
  for (uint8_t k = 0; k < n; k++) { char ch = (char)d[k]; Serial.print((ch >= 32 && ch < 127) ? ch : '.'); }
  Serial.println('\'');
}
void serialCmdHelp() {
  Serial.println(F("cmd: h help | i ECU ID | q FEEB request PGN hex | e presets | s status | p 1..4 preset | a 27 SA hex | m AUTO/FIXED"));
}
long parseHex(const char* s) {                  // -1 = не число
  long v = 0; bool any = false;
  for (; *s; s++) {
    char c = *s; uint8_t d;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
    else return -1;
    v = (v << 4) | d; any = true;
    if (v > 0xFFFFF) return -1;
  }
  return any ? v : -1;
}
bool refuseRunning() {
  bool running = engineRunning && !BENCH_TEST_MODE;
  if (running) Serial.println(F("refused: engine running"));
  return running;
}
void execCmd(unsigned long now) {
  char c = cmdBuf[0];
  const char* arg = cmdBuf + 1; while (*arg == ' ') arg++;
  switch (c) {
    case 'h': case '?': serialCmdHelp(); break;
    case 'i':
#if ENABLE_ECU_ID
      if (busAlive) { idStart(now); idTimer = now; Serial.println(F("ECU ID request...")); }
      else Serial.println(F("bus down"));
#else
      Serial.println(F("ECU ID disabled"));
#endif
      break;
    case 'q': {
      long pgn = parseHex(arg);
      if (pgn <= 0 || pgn > 0xFFFF) { Serial.println(F("q <PGN hex>, e.g. q FEEB")); break; }
      reconPgn = (uint16_t)pgn; reconUntil = now + RECON_MS; reconLeft = RECON_MAX_FRAMES;
      if (busAlive) sendReqPgn((uint32_t)pgn, 0xFF);
      Serial.print(F("request PGN ")); Serial.print(pgn); Serial.println(busAlive ? F(" sent (global)") : F(" NOT sent: bus down, listening only"));
      break;
    }
    case 'e': printPresets(); break;
    case 's': lastSerialTime = 0; break;
    case 'p': {
      int p = arg[0] - '0';
      if (p < 1 || p > PRESET_COUNT) { Serial.println(F("p 1..4")); break; }
      if (refuseRunning()) break;
      applyPreset((uint8_t)(p - 1), true);
      break;
    }
    case 'a': {
      long sa = parseHex(arg);
      if (sa < 0 || sa > 0xFD) { Serial.println(F("a <SA hex 00..FD>")); break; }
      if (refuseRunning()) break;
      pSa[curPreset] = (uint8_t)sa; eeSavePreset(curPreset); sweepIndex = 0; setActiveAddr((uint8_t)sa);
      Serial.print(F("SA=0x")); Serial.println((uint8_t)sa, HEX);
      break;
    }
    case 'm':
      if (refuseRunning()) break;
      pMode[curPreset] ^= 1; eeSavePreset(curPreset); sweepIndex = 0; setActiveAddr(pSa[curPreset]);
      Serial.print(F("mode=")); Serial.println((const __FlashStringHelper*)modeNameP(pMode[curPreset]));
      break;
    default: Serial.println(F("? (h = help)"));
  }
}
void handleSerialCmd(unsigned long now) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (cmdLen) { cmdBuf[cmdLen] = 0; execCmd(now); } cmdLen = 0; }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
    else cmdLen = 0;
  }
}
#endif  // ENABLE_SERIAL_CMD

// ============================================================
//  ОТРИСОВКА ЭКРАНОВ (в кадровый буфер)
// ============================================================
void renderMain() {
  switch (phase) {
    case PH_WAITRUN: {
      char m = ' ';
#if ENABLE_ECU_ID
      if (idState == ID_DONE) m = (idMatch == (int8_t)curPreset) ? 'A' : (idMatch >= 0 ? '!' : '?');
      else if (idState == ID_REQ || idState == ID_WAIT) m = '.';
#endif
      fbPrintf(0, PSTR("START ENGINE   %c"), m);
      fbPrintf(1, PSTR("%S SA:%02X %S"), presetNameP(curPreset), pSa[curPreset], modeNameP(pMode[curPreset]));
      break;
    }
    case PH_SETTLE:
      fbTextP(0, PSTR("SETTLE..."));
      fbPrintf(1, PSTR("A:%4u T:%3dC"), realRpm, engineTemp);
      break;
    case PH_SCAN:
      fbPrintf(0, PSTR("SCAN:%02X A:%4u"), activeAddr, realRpm);
      fbPrintf(1, PSTR("%S  poisk SA..."), presetNameP(curPreset));
      break;
    case PH_RUN:
    default:
      fbPrintf(0, PSTR("S:%4d A:%4u"), commandedRpm, realRpm);
      // '*' после адреса = ЭБУ подтверждает (SPN 1483), что сейчас слушает именно нас
      fbPrintf(1, PSTR("T:%3dC %S %02X%c"), engineTemp, presetNameP(curPreset), activeAddr, (revActive && ctrlSa == activeAddr) ? '*' : ' ');
      break;
  }
}
void renderDataUi() {
  switch (dataPage) {
    case 0:                                   // масло
      if (dataValid & DV_OILP) fbPrintf(0, PSTR("Maslo:  %d.%d bar"), oilP / 100, (oilP % 100) / 10);
      else                     fbTextP(0, PSTR("Maslo:     ---"));
      if (dataValid & DV_OILT) fbPrintf(1, PSTR("T masla: %4d C"), oilT);
      else                     fbTextP(1, PSTR("T masla:    ---"));
      break;
    case 1:                                   // ОЖ + наддув
      if (dataValid & DV_COOL) fbPrintf(0, PSTR("T OZH:   %4d C"), engineTemp);
      else                     fbTextP(0, PSTR("T OZH:      ---"));
      if (dataValid & DV_BOOST)fbPrintf(1, PSTR("Nadduv:%4d kPa"), boostKpa);
      else                     fbTextP(1, PSTR("Nadduv:     ---"));
      break;
    case 2:                                   // борт + впуск
      if (dataValid & DV_BATT) fbPrintf(0, PSTR("Bort:   %d.%d V"), battV10 / 10, battV10 % 10);
      else                     fbTextP(0, PSTR("Bort:      ---"));
      if (dataValid & DV_AIRT) fbPrintf(1, PSTR("T vpusk: %4d C"), airT);
      else                     fbTextP(1, PSTR("T vpusk:    ---"));
      break;
    default:                                  // топливо + нагрузка
      if (dataValid & DV_FUEL) fbPrintf(0, PSTR("Rashod:%d.%d L/h"), fuelRate10 / 10, fuelRate10 % 10);
      else                     fbTextP(0, PSTR("Rashod:    ---"));
      if (dataValid & DV_LOAD) fbPrintf(1, PSTR("Nagruz:  %4d %%"), engLoad);
      else                     fbTextP(1, PSTR("Nagruz:     ---"));
      break;
  }
}
#if ENABLE_DTC_READER
void renderDtcUi() {
  if (ui == UI_CONFIRM_CLR)  { fbTextP(0, PSTR("Sbrosit kody?")); fbTextP(1, PSTR("CLR=da  BACK=net")); return; }
  if (dtcCount == 0)         { fbTextP(0, PSTR("Oshibok net")); fbTextP(1, PSTR("DM1/DM2: chisto")); return; }
  if (dtcView >= dtcCount) dtcView = 0;
  const Dtc &d = dtcList[dtcView];
  char l1[20], l2[44];
  snprintf_P(l1, sizeof(l1), PSTR("%u/%u %c S%lu F%u"),
             (unsigned)(dtcView + 1), (unsigned)dtcCount,
             d.stored ? 'S' : 'A', (unsigned long)d.spn, (unsigned)d.fmi);
  fbText(0, l1);
  buildDtcDesc(l2, sizeof(l2), d);
  fbScroll(1, l2, scrollPos);
  scrollPos++;
}
#endif
void renderMenu() {
  char v[17];
  switch (menuItem) {
    case 0:
      fbPrintf(0, PSTR("%S: rezhim SA"), presetNameP(curPreset));
      fbTextP(1, pMode[curPreset] == MODE_AUTO ? PSTR("AUTO (poisk) CLR") : PSTR("FIXED        CLR"));
      break;
    case 1:
      fbPrintf(0, PSTR("%S: adres SA"), presetNameP(curPreset));
      saLabel(pSa[curPreset], v, sizeof(v)); fbText(1, v);
      break;
    case 2:
      fbTextP(0, PSTR("ECU ID"));
#if ENABLE_ECU_ID
      if (idState == ID_DONE) {
        if (menuShowFp) fbPrintf(1, PSTR("FP:%04X -> %S"), idFp, idMatch >= 0 ? presetNameP(idMatch) : PSTR("---"));
        else if (ecuName[0]) fbText(1, ecuName);
        else fbTextP(1, PSTR("(bez teksta)"));
      }
      else if (idState == ID_NONE) fbTextP(1, PSTR("net otveta ECU"));
      else if (idState == ID_IDLE) fbTextP(1, PSTR("shina ne aktivna"));
      else fbTextP(1, PSTR("zapros..."));
#else
      fbTextP(1, PSTR("vyklyucheno"));
#endif
      break;
    default:
      fbTextP(0, PSTR("Sbros nastroek"));
      fbTextP(1, PSTR("CLR=da"));
      break;
  }
}
void renderUi() {
  char v[17];
  switch (ui) {
    case UI_MSG:            break;             // сообщение уже в буфере, не трогаем
    case UI_DATA:           renderDataUi(); break;
    case UI_MENU:           renderMenu(); break;
    case UI_EDIT_SA:
      fbPrintf(0, PSTR("%S: novyi SA"), presetNameP(curPreset));
      saLabel(editSa, v, sizeof(v)); fbText(1, v);
      break;
    case UI_CONFIRM_RESET:  fbTextP(0, PSTR("Sbrosit vse?")); fbTextP(1, PSTR("CLR=da  BACK=net")); break;
    case UI_SAVE_SA:        fbPrintf(0, PSTR("SA 0x%02X najden!"), foundSa); fbTextP(1, PSTR("CLR=sohranit")); break;
#if ENABLE_DTC_READER
    case UI_DTC:
    case UI_CONFIRM_CLR:    renderDtcUi(); break;
#endif
    default:                renderMain(); break;
  }
}

// ============================================================
//  ОТЛАДОЧНЫЕ ХЕЛПЕРЫ
// ============================================================
void canReinit() {                             // полный сброс CAN-контроллера (выход из bus-off)
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

bool uiInMenu() { return ui == UI_MENU || ui == UI_EDIT_SA || ui == UI_CONFIRM_RESET; }

void resetRunState() {                         // шина пропала (зажигание выключено): как при включении питания
  phase = PH_WAITRUN;
  sweepIndex = 0;
  setActiveAddr(pSa[curPreset]);
  idleBaseline  = RPM_MIN;
  commandedRpm  = RPM_MIN;
  targetRpm     = RPM_MIN;
  revActive     = false;
  engineRunning = false;
  realRpm       = 0;
  ctrlSa        = 0xFF;
  txFailStreak  = 0;
#if ENABLE_DTC_READER
  dtcCount = 0; dtcView = 0; scrollPos = 0; lampStatus = 0;
#endif
#if ENABLE_DTC_READER || ENABLE_ECU_ID
  tpActive = false;
#endif
#if ENABLE_ECU_ID
  idReset(); pendingAuto = -1;
#endif
  if (!uiInMenu()) ui = UI_MAIN;               // меню не закрываем: настройки правят как раз при выключенном зажигании
}

const __FlashStringHelper* phaseName(Phase p) {
  switch (p) {
    case PH_WAITRUN: return F("WAITRUN");
    case PH_SETTLE:  return F("SETTLE");
    case PH_SCAN:    return F("SCAN");
    default:         return F("RUN");
  }
}
const __FlashStringHelper* uiName(UiMode u) {
  switch (u) {
    case UI_MAIN:          return F("MAIN");
    case UI_DATA:          return F("DATA");
    case UI_DTC:           return F("DTC");
    case UI_CONFIRM_CLR:   return F("CONFIRM");
    case UI_MENU:          return F("MENU");
    case UI_EDIT_SA:       return F("EDITSA");
    case UI_CONFIRM_RESET: return F("RESET?");
    case UI_SAVE_SA:       return F("SAVESA");
    default:               return F("MSG");
  }
}

#if DEBUG_LEVEL >= 2
void noteNewPgn(uint32_t pgn, uint8_t sa) {    // лог первой встречи PGN — инвентарь шины
  for (uint8_t i = 0; i < seenCnt; i++) if (seenPgn[i] == pgn) return;
  if (seenCnt >= PGN_SEEN_MAX) return;
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
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000, true);            // зависший I2C не должен вешать контроллер
#endif
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(F("VCM Iveco/KamAZ "));

  bool eeOk = eeLoad();

  pinMode(PIN_BTN_READ,  INPUT_PULLUP);
  pinMode(PIN_BTN_CLEAR, INPUT_PULLUP);
  pinMode(PIN_BTN_BACK,  INPUT_PULLUP);
  bRead = {PIN_BTN_READ,  0, 0, 0, 0};
  bClr  = {PIN_BTN_CLEAR, 0, 0, 0, 0};
  bBack = {PIN_BTN_BACK,  0, 0, 0, 0};

  setActiveAddr(pSa[curPreset]);
  sweepIndex = 0;

  SPI.begin();
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    lcd.setCursor(0, 1); lcd.print(F("FW " FW_VERSION "  CAN: OK  "));
    Serial.println(F("VCM READY. FW " FW_VERSION ". TSC1 (PGN 0) + DTC + ECU ID."));
    DBG(1) {
      ts(); Serial.println(F("=== START VCM Iveco/KamAZ ==="));
      ts(); Serial.print(F("cfg BENCH="));      Serial.print(BENCH_TEST_MODE);
            Serial.print(F(" PERM="));          Serial.print(ENABLE_PERMISSIVES);
            Serial.print(F(" WDT="));           Serial.print(ENABLE_WATCHDOG);
            Serial.print(F(" ECUID="));         Serial.print(ENABLE_ECU_ID);
            Serial.print(F(" DBG="));           Serial.println(DEBUG_LEVEL);
      ts(); Serial.print(F("cfg RPM "));        Serial.print(RPM_MIN); Serial.print('-'); Serial.print(RPM_MAX);
            Serial.print(F(" ramp="));          Serial.println(RPM_RAMP);
      ts(); Serial.print(F("EEPROM "));         Serial.println(eeOk ? F("ok") : F("DEFAULTS written"));
      printPresets();
      ts(); Serial.print(F("freeRAM="));        Serial.print(freeRam()); Serial.println(F(" B"));
#if ENABLE_SERIAL_CMD
      serialCmdHelp();
#endif
    }
  } else {
    lcd.setCursor(0, 1); lcd.print(F("CAN BUS: ERROR! "));
    Serial.println(F("FATAL: MCP2515 (CAN) ne otvechaet!"));
    while (1);
  }

  mcp2515.setNormalMode();
  delay(500);
  if (!eeOk) { lcd.setCursor(0, 1); lcd.print(F("EEPROM: defaults")); delay(1000); }
  lcd.clear();
  memset(fb, ' ', sizeof(fb)); fbDirty = 0; fbPos = 0; lcdCur = 0;

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
  canMsgTSC1.data[0] = 0xC1;                   // override on, Speed Control, высший приоритет
  canMsgTSC1.data[1] = raw & 0xFF;
  canMsgTSC1.data[2] = (raw >> 8) & 0xFF;
  canMsgTSC1.data[3] = 0xFF;
  canMsgTSC1.data[4] = TSC1_BYTE5;
  canMsgTSC1.data[5] = 0xFF; canMsgTSC1.data[6] = 0xFF; canMsgTSC1.data[7] = 0xFF;
}
void buildTSC1disable() {
  canMsgTSC1.can_id  = activeCanId;
  canMsgTSC1.can_dlc = 8;
  canMsgTSC1.data[0] = 0xC0;                   // override off
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
        idleBaseline = realRpm;
        if (pMode[curPreset] == MODE_AUTO) {
          sweepIndex = 0; setActiveAddr(sweepAddr(0)); commandedRpm = idleBaseline;
          phase = PH_SCAN; phaseTimer = now;
          DBG(1) { ts(); Serial.print(F("idle ~")); Serial.print(idleBaseline); Serial.println(F(" rpm. SWEEP start")); }
        } else {
          phase = PH_RUN;
        }
      }
      break;
    case PH_SCAN: {
      if (!engineRunning) { phase = PH_WAITRUN; break; }
      revActive = true;
      int probe  = idleBaseline + SWEEP_PROBE_MARGIN; if (probe > RPM_MAX) probe = RPM_MAX;
      int detect = idleBaseline + SWEEP_DETECT_MARGIN;
      targetRpm = probe;
      bool byRpm  = ((int)realRpm >= detect);
      bool byCtrl = (ctrlSa == activeAddr);     // SPN 1483: ЭБУ сам сообщил, что слушает наш адрес
      if (byRpm || byCtrl) {
        foundSa = activeAddr;
        DBG(1) { ts(); Serial.print(F(">>> SA FOUND 0x")); Serial.print(activeAddr, HEX); Serial.println(byCtrl ? F(" (SPN1483)") : F(" (rpm)")); }
        phase = PH_RUN;
        if (ui == UI_MAIN) { ui = UI_SAVE_SA; saveSaUntil = now + SAVE_SA_PROMPT_MS; }
      } else if (now - phaseTimer >= SWEEP_TEST_MS) {
        sweepNext();
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

// ============================================================
//  ОБРАБОТКА КНОПОК И ЛОГИКА ИНТЕРФЕЙСА
// ============================================================
void handleUi(unsigned long now) {
  BtnEv eR = btnPoll(bRead, ui == UI_EDIT_SA);
  BtnEv eC = btnPoll(bClr, false);
  BtnEv eB = btnPoll(bBack, false);
  bool cfgAllowed = !engineRunning || BENCH_TEST_MODE;   // настройки и смена пресета — только при остановленном двигателе
  DBG(1) {
    if (eR != BE_NONE) { ts(); Serial.print(F("BTN READ ")); Serial.println((uint8_t)eR); }
    if (eC != BE_NONE) { ts(); Serial.print(F("BTN CLEAR ")); Serial.println((uint8_t)eC); }
    if (eB != BE_NONE) { ts(); Serial.print(F("BTN BACK ")); Serial.println((uint8_t)eB); }
  }

  if (ui == UI_MSG && ((long)(now - msgUntil) >= 0 || eR != BE_NONE || eC != BE_NONE || eB != BE_NONE))
    ui = uiAfterMsg;                                           // сообщение закрывается по таймеру или любой кнопкой, кнопка обрабатывается дальше
  if (ui == UI_SAVE_SA && (long)(now - saveSaUntil) >= 0) ui = UI_MAIN;

#if ENABLE_ECU_ID
  if (pendingAuto >= 0 && ui == UI_MAIN && cfgAllowed) {       // автопереключение пресета по опознанному ЭБУ
    uint8_t p = pendingAuto;
    applyPreset(p, false);
    char l2[17]; snprintf_P(l2, sizeof(l2), PSTR("%S SA:%02X %S"), presetNameP(p), pSa[p], modeNameP(pMode[p]));
    showMsg("ECU opoznan:", l2, 2000, UI_MAIN);
    return;
  }
#endif

  switch (ui) {
    case UI_MAIN:
      if (eR == BE_SHORT) { ui = UI_DATA; dataPage = 0; }
      else if (eR == BE_LONG) {
        if (!cfgAllowed) { showMsgP(PSTR("Zaglushi"), PSTR("dvigatel!"), 1200, UI_MAIN); break; }
        uint8_t p = (curPreset + 1) % PRESET_COUNT;
        applyPreset(p, true);
        char l1[17], l2[17];
        snprintf_P(l1, sizeof(l1), PSTR("Preset: %S"), presetNameP(p));
#if ENABLE_ECU_ID
        snprintf_P(l2, sizeof(l2), PSTR("SA:%02X %S%S"), pSa[p], modeNameP(pMode[p]), (idState == ID_DONE) ? PSTR(" +ECU") : PSTR(""));
#else
        snprintf_P(l2, sizeof(l2), PSTR("SA:%02X %S"), pSa[p], modeNameP(pMode[p]));
#endif
        showMsg(l1, l2, 1500, UI_MAIN);
      }
      else if (eB == BE_LONG) {
        if (!cfgAllowed) { showMsgP(PSTR("Zaglushi"), PSTR("dvigatel!"), 1200, UI_MAIN); break; }
        ui = UI_MENU; menuItem = 0; menuShowFp = false;
      }
      else if (eB == BE_SHORT && phase == PH_SCAN) {           // остановить перебор адресов (например, мотор ещё холодный)
        phase = PH_RUN; revActive = false; sweepIndex = 0; setActiveAddr(pSa[curPreset]);
        showMsgP(PSTR("Poisk ostanovlen"), PSTR("SA ne najden"), 1500, UI_MAIN);
        DBG(1) { ts(); Serial.println(F("SWEEP aborted by BACK")); }
      }
      break;

    case UI_DATA:
      if (eR == BE_SHORT) {
        if (dataPage < DATA_PAGES - 1) dataPage++;
        else {
#if ENABLE_DTC_READER
          ui = UI_DTC; dtcView = 0; scrollPos = 0; requestDM2(); printDtcSerial();
#else
          ui = UI_MAIN;
#endif
        }
      }
      else if (eB == BE_SHORT) ui = UI_MAIN;
      break;

#if ENABLE_DTC_READER
    case UI_DTC:
      if (eR == BE_SHORT) { if (dtcCount && dtcView < dtcCount - 1) { dtcView++; scrollPos = 0; } else ui = UI_MAIN; }
      else if (eC == BE_SHORT) ui = UI_CONFIRM_CLR;
      else if (eB == BE_SHORT) { scrollPos = 0; ui = UI_MAIN; }
      break;
    case UI_CONFIRM_CLR:
      if (eC == BE_SHORT) {
        clearDtcs(); requestDM2();
        showMsgP(PSTR("Sbros otpravlen"), PSTR("DM11 + DM3"), 1500, UI_DTC);
        DBG(1) { ts(); Serial.println(F("CLEAR sent: DM11+DM3")); }
      }
      else if (eB == BE_SHORT) ui = UI_DTC;
      break;
#endif

    case UI_MENU:
      if (eR == BE_SHORT) { menuItem = (menuItem + 1) % 4; menuShowFp = false; }
      else if (eB == BE_SHORT) ui = UI_MAIN;
      else if (eC == BE_SHORT) {
        switch (menuItem) {
          case 0: pMode[curPreset] ^= 1; eeSavePreset(curPreset); sweepIndex = 0; setActiveAddr(pSa[curPreset]); break;
          case 1: editSa = pSa[curPreset]; ui = UI_EDIT_SA; break;
          case 2: menuShowFp = !menuShowFp; break;
          default: ui = UI_CONFIRM_RESET; break;
        }
      }
      break;

    case UI_EDIT_SA:
      if (eR == BE_SHORT) {                                     // быстрый список известных адресов
        int8_t i = saNameIndex(editSa);
        i = (i < 0) ? 0 : (i + 1) % (int8_t)SA_NAMES_N;
        editSa = pgm_read_byte(&SA_NAMES[i].sa);
      }
      else if (eR == BE_LONG || eR == BE_REPEAT) editSa = (editSa >= 0xFD) ? 0 : editSa + 1;   // удержание: +1
      else if (eC == BE_SHORT) {
        pSa[curPreset] = editSa; eeSavePreset(curPreset);
        sweepIndex = 0; setActiveAddr(editSa);
        ui = UI_MENU; menuItem = 1;
        DBG(1) { ts(); Serial.print(F("SA set 0x")); Serial.println(editSa, HEX); }
      }
      else if (eB == BE_SHORT) ui = UI_MENU;
      break;

    case UI_CONFIRM_RESET:
      if (eC == BE_SHORT) {
        eeWriteDefaults(); eeLoad();
        applyPreset(curPreset, false);
#if ENABLE_ECU_ID
        idMatch = (idState == ID_DONE) ? fpFind(idFp) : -1;
#endif
        showMsgP(PSTR("Nastroyki"), PSTR("sbrosheny"), 1500, UI_MAIN);
        DBG(1) { ts(); Serial.println(F("EEPROM factory reset")); }
      }
      else if (eB == BE_SHORT) ui = UI_MENU;
      break;

    case UI_SAVE_SA:
      if (eC == BE_SHORT) {
        pSa[curPreset] = foundSa; pMode[curPreset] = MODE_FIXED; eeSavePreset(curPreset);
        char l2[17]; snprintf_P(l2, sizeof(l2), PSTR("%S SA:%02X FIX"), presetNameP(curPreset), foundSa);
        showMsg("Sohraneno:", l2, 1500, UI_MAIN);
        DBG(1) { ts(); Serial.print(F("SA saved 0x")); Serial.println(foundSa, HEX); }
      }
      else if (eB == BE_SHORT || eR == BE_SHORT) ui = UI_MAIN;
      break;

    default: break;
  }
}

// ============================================================
void loop() {
#if ENABLE_WATCHDOG
  wdt_reset();
#endif
  unsigned long currentMillis = millis();

  // --- 1. ПРИЁМ CAN (не больше RX_MAX_PER_LOOP кадров за проход) ---
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
      const uint8_t* d = canMsgRead.data;
#if DEBUG_LEVEL >= 3
      ts(); Serial.print(F("RX id=0x")); Serial.print(ext, HEX);
      Serial.print(F(" pgn=")); Serial.print(pgn);
      Serial.print(F(" dlc=")); Serial.println(canMsgRead.can_dlc);
#endif
#if DEBUG_LEVEL >= 2
      noteNewPgn(pgn, sa);
#endif
#if ENABLE_SERIAL_CMD
      if (pgn == reconPgn && reconActive()) reconPrint(pgn, sa, d, canMsgRead.can_dlc, 0);   // разведка: сырой кадр запрошенного PGN
#endif

      if (pgn == 61444) {                              // EEC1 — обороты (SPN 190), управляющее устройство (SPN 1483)
        uint16_t raw = d[3] | ((uint16_t)d[4] << 8);
        if (raw < 0xFE00) realRpm = raw >> 3;          // 0xFExx = ошибка, 0xFFxx = нет данных -> игнорируем
        ctrlSa = d[5];
      } else if (pgn == 65262) {                       // ET1 — темп. ОЖ (SPN110) + масла (SPN175)
        if (d[0] != 0xFF) { engineTemp = d[0] - 40; dataValid |= DV_COOL; }
        uint16_t ot = d[2] | ((uint16_t)d[3] << 8);
        if (ot != 0xFFFF) { oilT = (int)(ot >> 5) - 273; dataValid |= DV_OILT; }
      } else if (pgn == 65263) {                       // EFL/P1 — давление масла (SPN100, x4 кПа)
        if (d[3] != 0xFF) { oilP = (int)d[3] * 4; dataValid |= DV_OILP; }
      } else if (pgn == 65270) {                       // IC1 — наддув (SPN102, x2 кПа) + темп. впуска (SPN105)
        if (d[1] != 0xFF) { boostKpa = (int)d[1] * 2; dataValid |= DV_BOOST; }
        if (d[2] != 0xFF) { airT = (int)d[2] - 40; dataValid |= DV_AIRT; }
      } else if (pgn == 65271) {                       // VEP1 — напряжение борта (SPN168, x0.05 В)
        uint16_t bv = d[4] | ((uint16_t)d[5] << 8);
        if (bv != 0xFFFF) { battV10 = (int)(bv / 2); dataValid |= DV_BATT; }
      } else if (pgn == 65266) {                       // LFE — расход топлива (SPN183, x0.05 л/ч)
        uint16_t fr = d[0] | ((uint16_t)d[1] << 8);
        if (fr != 0xFFFF) { fuelRate10 = (int)(fr / 2); dataValid |= DV_FUEL; }
      } else if (pgn == 61443) {                       // EEC2 — нагрузка (SPN92, %)
        if (d[2] != 0xFF) { engLoad = (int)d[2]; dataValid |= DV_LOAD; }
      } else if (pgn == 59392) {                       // Acknowledgment: ответ ЭБУ на наш запрос (NACK = PGN не поддерживается)
        uint32_t apgn = (uint32_t)d[5] | ((uint32_t)d[6] << 8) | ((uint32_t)d[7] << 16);
        DBG(1) {
          ts(); Serial.print(d[0] == 0 ? F("ACK") : d[0] == 1 ? F("NACK") : d[0] == 2 ? F("ACK-denied") : F("ACK-busy"));
          Serial.print(F(" from SA=0x")); Serial.print(sa, HEX);
          Serial.print(F(" pgn=")); Serial.println(apgn);
        }
#if ENABLE_ECU_ID
        if (sa == 0x00 && idState == ID_WAIT && d[0] != 0) {
          if (apgn == 65259) idNack |= 1; else if (apgn == 65242) idNack |= 2; else if (apgn == 60928) idNack |= 4;
        }
#endif
      }
      // Дальше — только сообщения самого двигателя (SA 0x00): сканер или другой блок на шине не должны мешать.
      else if (sa != 0x00) { }
#if ENABLE_DTC_READER
      else if (pgn == 65226) {                         // DM1 (активные), одиночный кадр
        parseDtcPayload(d, canMsgRead.can_dlc, false);
        DBG(2) { ts(); Serial.print(F("DM1 1-frame, codes=")); Serial.println(dtcCount); }
      } else if (pgn == 65227) {                       // DM2 (сохранённые), одиночный кадр
        parseDtcPayload(d, canMsgRead.can_dlc, true);
        DBG(2) { ts(); Serial.print(F("DM2 1-frame, codes=")); Serial.println(dtcCount); }
      }
#endif
#if ENABLE_ECU_ID
      else if (idState == ID_WAIT && pgn == 65259) { idFeed(0, d, canMsgRead.can_dlc, 0); idSrc |= 1; }   // Component ID в одном кадре
      else if (idState == ID_WAIT && pgn == 65242) { idFeed(1, d, canMsgRead.can_dlc, 0); idSrc |= 2; }   // Software ID в одном кадре
      else if (idState == ID_WAIT && pgn == 60928) { idFeed(2, d, 8, 0); idSrc |= 4; }                   // Address Claimed (NAME)
#endif
#if ENABLE_DTC_READER || ENABLE_ECU_ID
      else if (pgn == 60416) {                         // TP.CM
        if (d[0] == 0x20) {                            // BAM
          uint16_t sz = d[1] | ((uint16_t)d[2] << 8);
          uint8_t  np = d[3];
          uint32_t tg = (uint32_t)d[5] | ((uint32_t)d[6] << 8) | ((uint32_t)d[7] << 16);
          bool take = false;
#if ENABLE_DTC_READER
          if (tg == 65226 || tg == 65227) { take = true; memset(tpBuf, 0xFF, TP_BUF); }
#endif
#if ENABLE_ECU_ID
          if ((tg == 65259 || tg == 65242) && idState == ID_WAIT) take = true;
#endif
#if ENABLE_SERIAL_CMD
          if (tg == reconPgn && reconActive()) { take = true; reconPrint((uint16_t)tg, sa, d, 8, 0xFFFF); }   // 0xFFFF = сам анонс BAM
#endif
          if (take) {
            tpActive = true; tpPgn = (uint16_t)tg; tpSize = sz; tpPackets = np; tpSeq = 1;
            DBG(2) { ts(); Serial.print(F("BAM start pgn=")); Serial.print(tg); Serial.print(F(" size=")); Serial.print(sz); Serial.print(F(" pkts=")); Serial.println(np); }
          } else tpActive = false;
        }
      } else if (pgn == 60160) {                       // TP.DT
        if (tpActive) {
          uint8_t seq = d[0];
          if (seq != tpSeq) {                          // потерян пакет -> сессию бросаем, мусор не разбираем
            tpActive = false;
            DBG(2) { ts(); Serial.print(F("BAM seq gap, got ")); Serial.print(seq); Serial.print(F(" exp ")); Serial.println(tpSeq); }
          } else {
            uint16_t off = (uint16_t)(seq - 1) * 7;
            uint8_t  n   = 7;
            if (off >= tpSize) n = 0; else if (off + n > tpSize) n = tpSize - off;
#if ENABLE_SERIAL_CMD
            if (tpPgn == reconPgn && reconActive()) reconPrint(tpPgn, sa, &d[1], n, off);
#endif
#if ENABLE_ECU_ID
            if (tpPgn == 65259 || tpPgn == 65242) idFeed(tpPgn == 65259 ? 0 : 1, &d[1], n, off);
            else
#endif
            {
#if ENABLE_DTC_READER
              for (uint8_t k = 0; k < n; k++) if (off + k < TP_BUF) tpBuf[off + k] = d[1 + k];
#endif
            }
            tpSeq++;
            if (seq >= tpPackets) {                    // последний пакет
              tpActive = false;
#if ENABLE_ECU_ID
              if (tpPgn == 65259) idSrc |= 1;
              else if (tpPgn == 65242) idSrc |= 2;
              else
#endif
              {
#if ENABLE_DTC_READER
                parseDtcPayload(tpBuf, (tpSize > TP_BUF) ? TP_BUF : tpSize, tpPgn == 65227);
                DBG(2) { ts(); Serial.print(F("BAM done, total codes=")); Serial.println(dtcCount); }
#endif
              }
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
  busAlive = (lastRxTime != 0) && (currentMillis - lastRxTime < BUS_TIMEOUT_MS);
  if (prevBusAlive && !busAlive) {                 // шина пропала (заглушили + выключили зажигание)
    resetRunState();
    DBG(1) { ts(); Serial.println(F("BUS lost -> full reset, wait for RPM")); }
  }
#if ENABLE_ECU_ID
  if (!prevBusAlive && busAlive) {                 // шина ожила -> запросить идентификацию ЭБУ
    idStart(currentMillis);
    DBG(1) { ts(); Serial.println(F("BUS alive -> ECU ID request scheduled")); }
  }
#endif
  prevBusAlive = busAlive;
  if (!busAlive) {
    realRpm = 0; ctrlSa = 0xFF;                    // нет шины -> мотор считаем остановленным
    if (currentMillis - lastCanReinit >= CAN_REINIT_MS) {
      lastCanReinit = currentMillis;
      canReinit();                                 // вдруг мы в bus-off — переинициализируем контроллер
      DBG(1) { ts(); Serial.println(F("CAN re-init (bus down)")); }
    }
  }
#if ENABLE_ECU_ID
  idTick(currentMillis);
#endif

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
    DBG(1) { ts(); Serial.print(F("ENGINE ")); Serial.print(prevRunning ? F("RUN->STOP") : F("STOP->RUN")); Serial.print(F(" rpm=")); Serial.println(realRpm); }
    if (!engineRunning) { phase = PH_WAITRUN; revActive = false; commandedRpm = idleBaseline; }
    else if (uiInMenu()) { showMsgP(PSTR("Dvigatel"), PSTR("zapushen"), 1200, UI_MAIN); }   // завели с открытым меню -> меню закрываем
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

  // --- 4. КНОПКИ, ИНТЕРФЕЙС, КОМАНДЫ SERIAL ---
  handleUi(currentMillis);
  static UiMode prevUi = UI_MAIN;
  if (ui != prevUi) { DBG(1) { ts(); Serial.print(F("UI ")); Serial.print(uiName(prevUi)); Serial.print(F("->")); Serial.println(uiName(ui)); } prevUi = ui; }
#if ENABLE_SERIAL_CMD
  handleSerialCmd(currentMillis);
#endif

  // --- 5. LCD: отрисовка в буфер раз в LCD_INTERVAL, вывод на дисплей по чуть-чуть каждый проход ---
  if (currentMillis - lastLcdTime >= LCD_INTERVAL) {
    lastLcdTime = currentMillis;
    renderUi();
  }
  fbPump();

  // --- 6. SERIAL (heartbeat) ---
  if (currentMillis - lastSerialTime >= SERIAL_INTERVAL) {
    lastSerialTime = currentMillis;
    int rxPerSec = canRxCount * 2; canRxCount = 0;
    ts();
    Serial.print(F("HB ph="));  Serial.print(phaseName(phase));
    Serial.print(F(" ui="));    Serial.print(uiName(ui));
    Serial.print(engineRunning ? F(" run=1") : F(" run=0"));
    Serial.print(busAlive ? F(" bus=1") : F(" bus=0"));
    Serial.print(F(" ped="));   Serial.print(pedalPercent); Serial.print('%');
    Serial.print(F(" P="));     Serial.print((const __FlashStringHelper*)presetNameP(curPreset));
    Serial.print(F(" SA=0x"));  Serial.print(activeAddr, HEX);
    Serial.print(revActive ? F(" cmd") : F(" idle"));
    Serial.print(F(" set="));   Serial.print(revActive ? commandedRpm : idleBaseline);
    Serial.print(F(" act="));   Serial.print(realRpm);
    Serial.print(F(" ctrl=0x"));Serial.print(ctrlSa, HEX);
    Serial.print(F(" T="));     Serial.print(engineTemp);
    Serial.print(F(" rxps="));  Serial.print(rxPerSec);
    Serial.print(F(" txOk="));  Serial.print(txOk);
    Serial.print(F(" txFail="));Serial.print(txFail);
#if ENABLE_DTC_READER
    Serial.print(F(" dtc="));   Serial.print(dtcCount);
    Serial.print(F(" lamp=0x"));Serial.print(lampStatus, HEX);
#endif
#if ENABLE_ECU_ID
    Serial.print(F(" id="));    Serial.print(idState);
    if (idState == ID_DONE) { Serial.print(F("/0x")); Serial.print(idFp, HEX); }
#endif
#if DEBUG_MCP_REGS
    Serial.print(F(" eflg=0x"));Serial.print(mcp2515.getErrorFlags(), HEX);
#endif
    Serial.print(F(" ram="));   Serial.print(freeRam());
    Serial.println();
  }
}
