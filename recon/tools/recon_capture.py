#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
recon_capture.py — запись лога пульта VCM (Serial 115200), команды разведки и автосводка.

  python recon_capture.py --list
  python recon_capture.py --port COM5 --ecu kamaz-740-edc7-01
  python recon_capture.py --summarize recon/logs/2026-10-03_kamaz-740-edc7-01/serial.log

Консоль во время записи:
  <текст>        отправить команду пульту (h, e, s, i, q FEEB, p 2 ...)
  /mark <текст>  пометка оператора в лог (зажигание, пуск, педаль...)
  /ident         серия запросов идентификации с паузами (e, i, q FEEB, q FEDA, q EE00, q FDC5, q FEEC, q FECB, e)
  /quit          завершить: пишутся summary.md, summary.json и заготовка report.md

Нужен пакет pyserial: pip install pyserial
"""
import argparse
import collections
import datetime
import json
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
LOGS = os.path.join(REPO, 'recon', 'logs')
TEMPLATE = os.path.join(REPO, 'recon', 'TEMPLATE_report.md')

IDENT_MACRO = [('e', 2), ('i', 8), ('q FEEB', 4), ('q FEDA', 4), ('q EE00', 3), ('q FDC5', 3), ('q FEEC', 4), ('q FECB', 4), ('e', 2)]

PGN_NAMES = {
    0: 'TSC1', 59392: 'ACK/NACK', 59904: 'Request', 60160: 'TP.DT', 60416: 'TP.CM', 60928: 'Address Claimed',
    61443: 'EEC2 (нагрузка, педаль)', 61444: 'EEC1 (обороты, SPN1483)', 61445: 'ETC2', 61442: 'ETC1',
    65226: 'DM1 активные коды', 65227: 'DM2 сохранённые коды', 65228: 'DM3', 65235: 'DM11',
    65242: 'Software ID', 65259: 'Component ID', 65260: 'VIN', 64965: 'ECU ID',
    65262: 'ET1 (температуры)', 65263: 'EFL/P1 (масло, топливо)', 65265: 'CCVS (скорость, круиз)',
    65266: 'LFE (расход топлива)', 65269: 'AMB (атмосфера)', 65270: 'IC1 (наддув, впуск)', 65271: 'VEP1 (напряжение)',
    65247: 'EEC3', 65248: 'VD (пробег)', 65253: 'HOURS (моточасы)', 65257: 'LFC (топливо всего)',
    65272: 'TRF1', 65276: 'DD (панель)', 65279: 'OI', 65217: 'VDHR', 65244: 'IO (простой)', 65252: 'SHUTDN',
    65237: 'AT1T1I', 61454: 'AT1IG1 (NOx)', 64892: 'DPFC1',
}

PFX = re.compile(r'^\[[^\]]*\]\s*')
HOST_TS = re.compile(r'^\[([^\]]*)\]')

R = {
    'fw':           re.compile(r'VCM READY\. FW (\S+)\.'),
    'boot':         re.compile(r'=== START VCM'),
    'cfg':          re.compile(r'^cfg (.*)'),
    'eeprom':       re.compile(r'^EEPROM (ok|DEFAULTS written)'),
    'preset_cur':   re.compile(r'^preset=(\S+)'),
    'preset_line':  re.compile(r'^\s*(\d) (\S+) SA=0x([0-9A-Fa-f]+) mode=(\S+) fp:(.*)$'),
    'bus_alive':    re.compile(r'^BUS alive'),
    'bus_lost':     re.compile(r'^BUS lost'),
    'reinit':       re.compile(r'^CAN re-init \((.*)\)'),
    'txfail':       re.compile(r'^TSC1 TX FAIL'),
    'pgn_seen':     re.compile(r'^PGN seen (\d+) SA=0x([0-9A-Fa-f]+)'),
    'id_req':       re.compile(r'^ID request src(\d) attempt (\d+)'),
    'id_chunk':     re.compile(r'^ID src(\d) @(\d+): (.*)$'),
    'ack':          re.compile(r'^(ACK|NACK|ACK-denied|ACK-busy) from SA=0x([0-9A-Fa-f]+) pgn=(\d+)'),
    'id_done':      re.compile(r"^ID done src=(\d+) fp=0x([0-9A-Fa-f]+) name='(.*)' match=(\S+)"),
    'id_none':      re.compile(r'^ID: net otveta ECU'),
    'preset_change': re.compile(r'^PRESET -> (\S+) SA=0x([0-9A-Fa-f]+) mode=(\S+) \((manual|auto)(?:, ECU fp=0x([0-9A-Fa-f]+) privyazan)?\)'),
    'sweep_start':  re.compile(r'^idle ~(\d+) rpm\. SWEEP start'),
    'sweep_next':   re.compile(r'^SWEEP next SA=0x([0-9A-Fa-f]+)'),
    'sa_found':     re.compile(r'^>>> SA FOUND 0x([0-9A-Fa-f]+) \((SPN1483|rpm)\)'),
    'sweep_abort':  re.compile(r'^SWEEP aborted'),
    'sa_saved':     re.compile(r'^SA saved 0x([0-9A-Fa-f]+)'),
    'sa_set':       re.compile(r'^SA set 0x([0-9A-Fa-f]+)'),
    'engine':       re.compile(r'^ENGINE (STOP->RUN|RUN->STOP) rpm=(\d+)'),
    'hb':           re.compile(r'^HB ph=(\S+) ui=(\S+) run=(\d) bus=(\d) ped=(\d+)% P=(\S+) SA=0x([0-9A-Fa-f]+) (cmd|idle) '
                               r'set=(-?\d+) act=(\d+) ctrl=0x([0-9A-Fa-f]+) T=(-?\d+) rxps=(\d+) txOk=(\d+) txFail=(\d+)'
                               r'(?: dtc=(\d+) lamp=0x([0-9A-Fa-f]+))?(?: id=(\d)(?:/0x([0-9A-Fa-f]+))?)?(?: eflg=0x[0-9A-Fa-f]+)? ram=(-?\d+)'),
    'recon':        re.compile(r"^RECON pgn=(\d+) SA=0x([0-9A-Fa-f]+) @(\d+) hex:((?: [0-9A-F]{2})*)  '(.*)'$"),
    'dtc_head':     re.compile(r'^>>> DTC: (\d+) kod'),
    'dtc_line':     re.compile(r'^(\d+) \[(A|S)\] SPN(\d+) FMI(\d+) OC(\d+) CM(\d+)(\(legacy!\))?\s+(.*)$'),
    'dm':           re.compile(r'^(DM1 1-frame|DM2 1-frame|BAM start pgn=\d+|BAM done|BAM seq gap)'),
    'mark':         re.compile(r'^### MARK (.*)'),
    'cmd':          re.compile(r'^### CMD (.*)'),
    'session':      re.compile(r'^### SESSION START'),
    'factory':      re.compile(r'^EEPROM factory reset'),
}


def now():
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def clean(line):
    line = PFX.sub('', line, 1)          # метка времени ноутбука
    line = PFX.sub('', line, 1)          # millis пульта
    return line.rstrip('\r\n')


# ============================================================ запись
class Capture:
    def __init__(self, port, baud, folder):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.path = os.path.join(folder, 'serial.log')
        self.f = open(self.path, 'a', encoding='utf-8')
        self.lock = threading.Lock()
        self.stop = False
        self.write_line('### SESSION START %s port=%s baud=%d' % (now(), port, baud))

    def write_line(self, text):
        line = '[%s] %s' % (now(), text)
        with self.lock:
            self.f.write(line + '\n')
            self.f.flush()
        print(line)

    def reader(self):
        buf = b''
        while not self.stop:
            try:
                data = self.ser.read(256)
            except Exception as e:  # noqa
                self.write_line('### SERIAL ERROR %s' % e)
                time.sleep(1)
                continue
            if not data:
                continue
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.write_line(line.decode('utf-8', 'replace').rstrip('\r'))

    def send(self, cmd):
        self.write_line('### CMD %s' % cmd)
        self.ser.write((cmd + '\n').encode('ascii', 'replace'))

    def mark(self, text):
        self.write_line('### MARK %s' % text)

    def close(self):
        self.stop = True
        time.sleep(0.5)
        try:
            self.ser.close()
        except Exception:
            pass
        self.write_line('### SESSION END %s' % now())
        self.f.close()


def run_capture(port, baud, ecu):
    folder = os.path.join(LOGS, '%s_%s' % (datetime.date.today().isoformat(), ecu))
    os.makedirs(folder, exist_ok=True)
    cap = Capture(port, baud, folder)
    t = threading.Thread(target=cap.reader, daemon=True)
    t.start()
    print('Пишу в %s. Команды пульту вводить как есть; /mark <текст>, /ident, /quit' % cap.path)
    try:
        while True:
            try:
                line = input()
            except EOFError:
                break
            line = line.strip()
            if not line:
                continue
            if line == '/quit':
                break
            if line.startswith('/mark'):
                cap.mark(line[5:].strip())
            elif line == '/ident':
                cap.mark('ident macro start')
                for cmd, pause in IDENT_MACRO:
                    cap.send(cmd)
                    time.sleep(pause)
                cap.mark('ident macro end')
            elif line.startswith('/'):
                print('неизвестная команда скрипта: %s' % line)
            else:
                cap.send(line)
    except KeyboardInterrupt:
        pass
    cap.close()
    write_outputs(folder, ecu)


# ============================================================ сводка
def parse(lines):
    s = {
        'lines': 0, 'sessions': 0, 'boots': 0, 'fw': None, 'cfg': [], 'eeprom': [], 'presets': {}, 'preset_cur': None,
        'bus_alive': 0, 'bus_lost': 0, 'reinit': collections.Counter(), 'txfail_lines': 0, 'pgn_seen': {},
        'id_req': collections.Counter(), 'id_texts': {0: [], 1: [], 2: []}, 'acks': [], 'id_done': [], 'id_none': 0,
        'preset_changes': [], 'sweep': [], 'engine': [], 'factory_reset': 0,
        'hb': {'n': 0, 'bus1': 0, 'run1': 0, 'act_max': 0, 'act_max_cmd': 0, 'set_max': 0, 'T_min': None, 'T_max': None,
               'ctrl_cmd': collections.Counter(), 'ctrl_idle': collections.Counter(), 'sa': collections.Counter(),
               'presets': collections.Counter(), 'txfail_max': 0, 'ram_min': None, 'dtc_max': 0, 'lamp': collections.Counter(),
               'id_states': collections.Counter(), 'fps': collections.Counter()},
        'recon': collections.OrderedDict(), 'dtc_dumps': [], 'dm': collections.Counter(), 'timeline': [],
    }
    cur = {0: None, 1: None, 2: None}
    dtc_cur = None

    def flush_text(src):
        if cur[src] is not None:
            s['id_texts'][src].append(cur[src])
            cur[src] = None

    for raw in lines:
        s['lines'] += 1
        ts = HOST_TS.match(raw)
        ts = ts.group(1) if ts else ''
        line = clean(raw)
        if not line:
            continue
        m = R['session'].match(line)
        if m:
            s['sessions'] += 1
            s['timeline'].append((ts, 'SESSION', ''))
            continue
        m = R['mark'].match(line)
        if m:
            s['timeline'].append((ts, 'MARK', m.group(1)))
            continue
        m = R['cmd'].match(line)
        if m:
            s['timeline'].append((ts, 'CMD', m.group(1)))
            continue
        if R['boot'].search(line):
            s['boots'] += 1
            s['timeline'].append((ts, 'BOOT', ''))
            continue
        m = R['fw'].search(line)
        if m:
            s['fw'] = m.group(1)
            continue
        m = R['cfg'].match(line)
        if m:
            if m.group(1) not in s['cfg']:
                s['cfg'].append(m.group(1))
            continue
        m = R['eeprom'].match(line)
        if m:
            s['eeprom'].append(m.group(1))
            continue
        m = R['preset_cur'].match(line)
        if m:
            s['preset_cur'] = m.group(1)
            continue
        m = R['preset_line'].match(line)
        if m:
            s['presets'][m.group(1)] = {'name': m.group(2), 'sa': '0x' + m.group(3).upper().zfill(2), 'mode': m.group(4),
                                       'fp': m.group(5).split()}
            continue
        if R['bus_alive'].match(line):
            s['bus_alive'] += 1
            s['timeline'].append((ts, 'BUS', 'alive'))
            continue
        if R['bus_lost'].match(line):
            s['bus_lost'] += 1
            s['timeline'].append((ts, 'BUS', 'lost'))
            continue
        m = R['reinit'].match(line)
        if m:
            s['reinit'][m.group(1)] += 1
            continue
        if R['txfail'].match(line):
            s['txfail_lines'] += 1
            continue
        m = R['pgn_seen'].match(line)
        if m:
            s['pgn_seen'][int(m.group(1))] = '0x' + m.group(2).upper().zfill(2)
            continue
        m = R['id_req'].match(line)
        if m:
            s['id_req'][int(m.group(1))] += 1
            continue
        m = R['id_chunk'].match(line)
        if m:
            src, off, text = int(m.group(1)), int(m.group(2)), m.group(3)
            if off == 0:
                flush_text(src)
                cur[src] = ''
            if cur[src] is None:
                cur[src] = ''
            if src == 2:
                cur[src] += text.strip() + ' '
            else:
                cur[src] = cur[src].ljust(off) + text
            continue
        m = R['ack'].match(line)
        if m:
            s['acks'].append({'kind': m.group(1), 'sa': '0x' + m.group(2).upper().zfill(2), 'pgn': int(m.group(3))})
            continue
        m = R['id_done'].match(line)
        if m:
            for k in (0, 1, 2):
                flush_text(k)
            s['id_done'].append({'ts': ts, 'src_bits': m.group(1), 'fp': '0x' + m.group(2).upper().zfill(4),
                                 'name': m.group(3), 'match': m.group(4)})
            s['timeline'].append((ts, 'ID', 'fp=0x%s match=%s' % (m.group(2).upper().zfill(4), m.group(4))))
            continue
        if R['id_none'].match(line):
            s['id_none'] += 1
            s['timeline'].append((ts, 'ID', 'net otveta'))
            continue
        m = R['preset_change'].match(line)
        if m:
            s['preset_changes'].append({'ts': ts, 'preset': m.group(1), 'sa': '0x' + m.group(2).upper().zfill(2),
                                        'mode': m.group(3), 'how': m.group(4), 'bound_fp': ('0x' + m.group(5).upper().zfill(4)) if m.group(5) else None})
            s['timeline'].append((ts, 'PRESET', '%s %s' % (m.group(1), m.group(4))))
            continue
        for key, label in (('sweep_start', 'start idle=%s'), ('sweep_next', 'next SA=0x%s'), ('sa_found', 'FOUND 0x%s (%s)'),
                           ('sa_saved', 'saved 0x%s'), ('sa_set', 'set 0x%s')):
            m = R[key].match(line)
            if m:
                s['sweep'].append({'ts': ts, 'event': label % tuple(g.upper() if key != 'sweep_start' else g for g in m.groups())})
                break
        else:
            m = None
        if m:
            continue
        if R['sweep_abort'].match(line):
            s['sweep'].append({'ts': ts, 'event': 'aborted by BACK'})
            continue
        m = R['engine'].match(line)
        if m:
            s['engine'].append({'ts': ts, 'event': m.group(1), 'rpm': int(m.group(2))})
            s['timeline'].append((ts, 'ENGINE', m.group(1)))
            continue
        if R['factory'].match(line):
            s['factory_reset'] += 1
            continue
        m = R['hb'].match(line)
        if m:
            h = s['hb']
            h['n'] += 1
            run, bus, preset, sa, mode = m.group(3), m.group(4), m.group(6), m.group(7), m.group(8)
            setv, act, ctrl, T = int(m.group(9)), int(m.group(10)), m.group(11).upper().zfill(2), int(m.group(12))
            txfail, ram = int(m.group(15)), int(m.group(20))
            h['bus1'] += bus == '1'
            h['run1'] += run == '1'
            h['act_max'] = max(h['act_max'], act)
            if mode == 'cmd':
                h['act_max_cmd'] = max(h['act_max_cmd'], act)
                h['set_max'] = max(h['set_max'], setv)
                h['ctrl_cmd']['0x' + ctrl] += 1
            else:
                h['ctrl_idle']['0x' + ctrl] += 1
            if run == '1':
                h['T_min'] = T if h['T_min'] is None else min(h['T_min'], T)
                h['T_max'] = T if h['T_max'] is None else max(h['T_max'], T)
            h['sa']['0x' + sa.upper().zfill(2)] += 1
            h['presets'][preset] += 1
            h['txfail_max'] = max(h['txfail_max'], txfail)
            h['ram_min'] = ram if h['ram_min'] is None else min(h['ram_min'], ram)
            if m.group(16):
                h['dtc_max'] = max(h['dtc_max'], int(m.group(16)))
                h['lamp']['0x' + m.group(17).upper()] += 1
            if m.group(18):
                h['id_states'][m.group(18)] += 1
            if m.group(19):
                h['fps']['0x' + m.group(19).upper().zfill(4)] += 1
            continue
        m = R['recon'].match(line)
        if m:
            pgn = int(m.group(1))
            s['recon'].setdefault(pgn, []).append({'sa': '0x' + m.group(2).upper().zfill(2), 'off': int(m.group(3)),
                                                   'hex': m.group(4).strip(), 'ascii': m.group(5)})
            continue
        m = R['dtc_head'].match(line)
        if m:
            dtc_cur = {'ts': ts, 'count': int(m.group(1)), 'codes': []}
            s['dtc_dumps'].append(dtc_cur)
            continue
        m = R['dtc_line'].match(line)
        if m and dtc_cur is not None:
            dtc_cur['codes'].append({'kind': m.group(2), 'spn': int(m.group(3)), 'fmi': int(m.group(4)), 'oc': int(m.group(5)),
                                     'cm': int(m.group(6)), 'legacy': bool(m.group(7)), 'text': m.group(8)})
            continue
        m = R['dm'].match(line)
        if m:
            s['dm'][m.group(1)] += 1
            continue
    for k in (0, 1, 2):
        flush_text(k)
    return s


def recon_payload(entries):
    """Собрать полезную нагрузку BAM/одиночных кадров из строк RECON (без анонса @65535)."""
    data = bytearray()
    for e in sorted((e for e in entries if e['off'] != 65535), key=lambda e: e['off']):
        b = bytes.fromhex(e['hex'].replace(' ', '')) if e['hex'] else b''
        if len(data) < e['off']:
            data.extend(b'\x00' * (e['off'] - len(data)))
        data[e['off']:e['off'] + len(b)] = b
    return bytes(data)


def printable(b):
    return ''.join(chr(c) if 32 <= c < 127 else '.' for c in b)


def render_md(s, title):
    L = []
    a = L.append
    a('# Автосводка: %s' % title)
    a('')
    a('Строк в логе: %d · сессий записи: %d · баннеров пульта (включений/перезагрузок): %d · FW: %s' % (s['lines'], s['sessions'], s['boots'], s['fw'] or '—'))
    a('')
    a('## Пульт при старте')
    for c in s['cfg']:
        a('- `cfg %s`' % c)
    a('- EEPROM: %s' % (', '.join(s['eeprom']) or '—'))
    if s['presets']:
        a('- Пресеты (последний дамп, активный: %s):' % (s['preset_cur'] or '—'))
        for k in sorted(s['presets']):
            p = s['presets'][k]
            a('  - %s %s SA=%s %s fp: %s' % (k, p['name'], p['sa'], p['mode'], ' '.join(p['fp']) or '—'))
    a('')
    a('## Шина')
    h = s['hb']
    a('- BUS alive: %d · BUS lost: %d · CAN re-init: %s · строк `TSC1 TX FAIL`: %d · txFail max в heartbeat: %d' % (
        s['bus_alive'], s['bus_lost'], dict(s['reinit']) or 0, s['txfail_lines'], h['txfail_max']))
    a('- Heartbeat: %d записей, шина жива в %d, двигатель работал в %d' % (h['n'], h['bus1'], h['run1']))
    if s['pgn_seen']:
        a('- Инвентарь PGN (`PGN seen`):')
        a('')
        a('| PGN | hex | SA | Имя |')
        a('| --- | --- | --- | --- |')
        for pgn in sorted(s['pgn_seen']):
            a('| %d | 0x%04X | %s | %s |' % (pgn, pgn, s['pgn_seen'][pgn], PGN_NAMES.get(pgn, '')))
    a('')
    a('## Идентификация ЭБУ')
    a('- Запросов отправлено: CI %d · SOFT %d · NAME %d' % (s['id_req'][0], s['id_req'][1], s['id_req'][2]))
    nacks = collections.Counter(x['pgn'] for x in s['acks'] if x['kind'] != 'ACK')
    acks = collections.Counter(x['pgn'] for x in s['acks'] if x['kind'] == 'ACK')
    a('- NACK/отказы по PGN: %s · ACK по PGN: %s' % (dict(nacks) or '—', dict(acks) or '—'))
    for src, label in ((0, 'Component ID 65259'), (1, 'Software ID 65242'), (2, 'Address Claim NAME 60928')):
        texts = []
        for t in s['id_texts'][src]:
            if t not in texts:
                texts.append(t)
        a('- %s: %s' % (label, ('%d ответ(ов), уникальных %d' % (len(s['id_texts'][src]), len(texts))) if texts else 'ответов нет'))
        for t in texts:
            a('  - `%s`' % t.strip())
    if s['id_done']:
        a('')
        a('| № | когда | источники (bits CI,SOFT,NAME) | fp | name | match |')
        a('| --- | --- | --- | --- | --- | --- |')
        for i, d in enumerate(s['id_done'], 1):
            a('| %d | %s | %s | %s | `%s` | %s |' % (i, d['ts'], d['src_bits'], d['fp'], d['name'], d['match']))
        fps = sorted(set(d['fp'] for d in s['id_done']))
        if len(fps) == 1 and len(s['id_done']) >= 2:
            verdict = 'стабилен (%d опросов, один отпечаток)' % len(s['id_done'])
        elif len(fps) > 1:
            verdict = '%d разных отпечатка за %d опросов — если в логе один ЭБУ, отпечаток НЕСТАБИЛЕН' % (len(fps), len(s['id_done']))
        else:
            verdict = 'один опрос, сравнить не с чем'
        a('')
        a('- Отпечатки: %s → **%s**' % (', '.join(fps), verdict))
    a('- `ID: net otveta ECU`: %d раз' % s['id_none'])
    a('')
    a('## Пресеты и привязки')
    if s['preset_changes']:
        for p in s['preset_changes']:
            a('- %s PRESET -> %s SA=%s %s (%s%s)' % (p['ts'], p['preset'], p['sa'], p['mode'], p['how'], (', привязан fp ' + p['bound_fp']) if p['bound_fp'] else ''))
    else:
        a('- смен пресета не было')
    a('- Пресеты в heartbeat: %s · адреса SA: %s' % (dict(h['presets']), dict(h['sa'])))
    a('')
    a('## Двигатель и управление')
    for e in s['engine']:
        a('- %s ENGINE %s rpm=%d' % (e['ts'], e['event'], e['rpm']))
    a('- Макс. обороты: %d (при активной команде %d, макс. задание %d) · T: %s…%s °C при работающем двигателе' % (
        h['act_max'], h['act_max_cmd'], h['set_max'], h['T_min'] if h['T_min'] is not None else '—', h['T_max'] if h['T_max'] is not None else '—'))
    a('- SPN 1483 `ctrl=` при активной команде: %s · на холостом: %s' % (dict(h['ctrl_cmd']) or '— (команда не подавалась)', dict(h['ctrl_idle']) or '—'))
    a('')
    a('## Автоподбор адреса')
    if s['sweep']:
        for e in s['sweep']:
            a('- %s %s' % (e['ts'], e['event']))
    else:
        a('- перебора не было')
    a('')
    a('## Коды неисправностей')
    a('- Транспорт: %s · максимум кодов в heartbeat: %d · лампы: %s' % (dict(s['dm']) or '—', h['dtc_max'], dict(h['lamp']) or '—'))
    if s['dtc_dumps']:
        d = s['dtc_dumps'][-1]
        a('- Последний дамп (%s): %d код(ов)' % (d['ts'], d['count']))
        for c in d['codes']:
            a('  - [%s] SPN %d FMI %d OC %d CM %d%s — %s' % (c['kind'], c['spn'], c['fmi'], c['oc'], c['cm'], ' legacy!' if c['legacy'] else '', c['text']))
    a('')
    a('## Ответы на `q <PGN>` (RECON)')
    if s['recon']:
        for pgn, entries in s['recon'].items():
            payload = recon_payload(entries)
            a('- PGN %d (0x%04X, %s) от %s: %d кадр(ов)/пакет(ов), %d байт' % (pgn, pgn, PGN_NAMES.get(pgn, '?'), entries[0]['sa'], len(entries), len(payload)))
            if payload:
                a('  - hex: `%s`' % payload.hex(' ').upper())
                a('  - ascii: `%s`' % printable(payload))
    else:
        a('- команд `q` не было')
    a('')
    a('## Хронология (пометки, команды, события)')
    for ts, kind, text in s['timeline']:
        a('- %s **%s** %s' % (ts, kind, text))
    a('')
    a('## Аномалии')
    anomalies = []
    if s['sessions'] and s['boots'] > s['sessions']:
        anomalies.append('баннер пульта появлялся %d раз при %d сессиях записи — возможны перезагрузки (сверить с пометками о питании)' % (s['boots'], s['sessions']))
    elif not s['sessions'] and s['boots'] > 1:
        anomalies.append('баннер пульта появлялся %d раз — включения питания или перезагрузки' % s['boots'])
    if s['txfail_lines'] or h['txfail_max']:
        anomalies.append('были неудачные отправки TSC1')
    if s['reinit']:
        anomalies.append('переинициализации CAN: %s' % dict(s['reinit']))
    if h['ram_min'] is not None and h['ram_min'] < 300:
        anomalies.append('минимум свободной SRAM %d байт' % h['ram_min'])
    if s['dm'].get('BAM seq gap'):
        anomalies.append('потери пакетов BAM: %d' % s['dm']['BAM seq gap'])
    if 'DEFAULTS written' in s['eeprom']:
        anomalies.append('EEPROM переписан заводскими значениями')
    if s['factory_reset']:
        anomalies.append('выполнялся сброс настроек: %d' % s['factory_reset'])
    for x in anomalies or ['не замечено']:
        a('- %s' % x)
    a('- минимум `ram=`: %s' % (h['ram_min'] if h['ram_min'] is not None else '—'))
    a('')
    return '\n'.join(L)


def to_jsonable(o):
    if isinstance(o, dict):
        return {str(k): to_jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [to_jsonable(v) for v in o]
    return o


def write_outputs(folder, ecu, log_path=None):
    log_path = log_path or os.path.join(folder, 'serial.log')
    with open(log_path, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    s = parse(lines)
    title = '%s (%s)' % (ecu, os.path.basename(folder))
    with open(os.path.join(folder, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(render_md(s, title))
    with open(os.path.join(folder, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(to_jsonable(s), f, ensure_ascii=False, indent=1)
    report = os.path.join(folder, 'report.md')
    if not os.path.exists(report) and os.path.exists(TEMPLATE):
        with open(TEMPLATE, encoding='utf-8') as f:
            t = f.read()
        t = t.replace('`<ecu-id>`', '`%s`' % ecu).replace('ГГГГ-ММ-ДД', datetime.date.today().isoformat())
        t = t.replace('`recon/logs/<дата>_<ecu-id>/`', '`recon/logs/%s/`' % os.path.basename(folder))
        with open(report, 'w', encoding='utf-8') as f:
            f.write(t)
    print('Готово: %s, summary.md, summary.json%s' % (log_path, '' if os.path.exists(report) else ''))


def main():
    ap = argparse.ArgumentParser(description='Запись лога пульта VCM и автосводка для разведки ЭБУ')
    ap.add_argument('--list', action='store_true', help='показать последовательные порты')
    ap.add_argument('--port', help='порт пульта, например COM5 или /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--ecu', help='идентификатор ЭБУ, например kamaz-740-edc7-01')
    ap.add_argument('--summarize', metavar='SERIAL_LOG', help='только построить сводку по готовому логу')
    args = ap.parse_args()
    if args.list:
        try:
            from serial.tools import list_ports
        except ImportError:
            sys.exit('нужен pyserial: pip install pyserial')
        for p in list_ports.comports():
            print('%s\t%s' % (p.device, p.description))
        return
    if args.summarize:
        folder = os.path.dirname(os.path.abspath(args.summarize))
        ecu = args.ecu or re.sub(r'^\d{4}-\d{2}-\d{2}_', '', os.path.basename(folder))
        write_outputs(folder, ecu, args.summarize)
        return
    if not args.port or not args.ecu:
        ap.error('нужны --port и --ecu (или --summarize / --list)')
    if not re.match(r'^[a-z0-9]+(-[a-z0-9]+)+$', args.ecu):
        ap.error('ecu-id: строчные латинские буквы, цифры и дефисы, например kamaz-740-edc7-01')
    try:
        import serial  # noqa: F401
    except ImportError:
        sys.exit('нужен pyserial: pip install pyserial')
    run_capture(args.port, args.baud, args.ecu)


if __name__ == '__main__':
    main()
