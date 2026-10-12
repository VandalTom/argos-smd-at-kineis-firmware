#!/usr/bin/env node
/**
 * Headless test for Tools/naiad_tool.html.
 *
 * Extracts the DOM-free protocol code between the "// ==== PROTOCOL BEGIN ====" and
 * "// ==== PROTOCOL END ====" markers, evaluates it, and drives it against the tool's
 * built-in simulator (app console + DFU bootloader). No hardware needed.
 *
 *   node Tools/naiad_tool_selftest.mjs
 *   docker run --rm -v "$PWD":/w -w /w node:20 node Tools/naiad_tool_selftest.mjs
 *
 * NAIAD_TEST_SCALE (default 0.05) compresses every simulated and protocol delay/timeout.
 * Exit code 0 = all tests passed.
 */

import { readFileSync, existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { deflateRawSync } from 'node:zlib';

const here = path.dirname(fileURLToPath(import.meta.url));
const html = readFileSync(path.join(here, 'naiad_tool.html'), 'utf8');
const m = /\/\/ ==== PROTOCOL BEGIN ====\r?\n([\s\S]*?)\/\/ ==== PROTOCOL END ====/.exec(html);
if (!m) { console.error('PROTOCOL markers not found in naiad_tool.html'); process.exit(2); }
const P = new Function('"use strict";\n' + m[1] + '\nreturn NaiadProtocol;')();
P.Clock.scale = Number(process.env.NAIAD_TEST_SCALE || 0.05);

const watchdog = setTimeout(() => { console.error('Global timeout (10 min): aborting'); process.exit(3); }, 600000);

/* ---------- tiny harness ---------- */

const results = [];
function assert(cond, msg) { if (!cond) throw new Error('assertion failed: ' + msg); }
function eq(a, b, msg) { if (a !== b) throw new Error(`${msg}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`); }
async function rejects(promise, check, msg) {
  try { await promise; } catch (e) { if (check && !check(e)) throw new Error(`${msg}: unexpected error ${e.name}: ${e.message}`); return e; }
  throw new Error(`${msg}: expected a rejection`);
}
async function test(name, fn) {
  const t0 = Date.now();
  try {
    const info = await fn();
    results.push({ name, ok: true });
    console.log(`PASS  ${name}  (${Date.now() - t0} ms)${info ? '  ' + info : ''}`);
  } catch (e) {
    results.push({ name, ok: false });
    console.log(`FAIL  ${name}  (${Date.now() - t0} ms)\n      ${(e && e.stack) || e}`);
  }
}

const ascii = s => P.asciiBytes(s);
function randomHex(r, n) { let s = ''; for (let i = 0; i < n; i++) s += '0123456789ABCDEF'[Math.floor(r() * 16)]; return s; }
function fakeImage(size, seed, fw) {
  const r = P.mulberry32(seed);
  const img = new Uint8Array(size);
  for (let i = 0; i < size; i++) img[i] = Math.floor(r() * 256);
  P.putU32le(img, 0, 0x2000C000);              // initial SP in RAM
  P.putU32le(img, 4, P.APP_BASE + 0x1235);      // odd reset vector inside the app region
  img.set(ascii(`NAIADFW=${fw}\0`), 0x300);
  return img;
}
/** Wait until the fake transport has written a line starting with prefix (commands go out after the wake check). */
async function sentCmd(t, prefix, ms = 3000) {
  const t0 = Date.now();
  while (!t.sent.some(s => s.startsWith(prefix))) { if (Date.now() - t0 > ms) throw new Error(`${prefix} was never sent`); await new Promise(r => setTimeout(r, 1)); }
}
async function until(pred, ms, what) {
  const t0 = Date.now();
  while (!pred()) { if (Date.now() - t0 > ms) throw new Error(`timed out waiting for ${what}`); await new Promise(r => setTimeout(r, 5)); }
}
function sameBytes(a, b) { if (a.length !== b.length) return false; for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false; return true; }
function allFF(a) { for (let i = 0; i < a.length; i++) if (a[i] !== 0xFF) return false; return true; }

/** Simulator + link at the given host baud, with the console transcript (as the UI would show it) recorded. */
async function rig(simOpts = {}, { baud = 115200, waitApp = true } = {}) {
  const sim = new P.NaiadSimulator({ seed: 1234, ...simOpts });
  const link = new P.Link(new P.SimTransport(sim));
  const log = [];
  link.on('rx', l => log.push('< ' + l));
  link.on('tx', l => log.push('> ' + l));
  link.on('note', l => log.push('# ' + l));
  await link.open(baud);
  let app = null;
  if (waitApp) app = await P.waitForApp(link, { bauds: [baud, 115200, 9600], settle: 0, timeout: 20000 });
  return { sim, link, log, app, done: () => sim.destroy() };
}

/* ---------- pure functions ---------- */

await test('CRC test vectors (MPEG-2 0x0376E6E7, IEEE 0xCBF43926) and reference cross-check', () => {
  const v = ascii('123456789');
  eq(P.crc32Mpeg2(v), 0x0376E6E7, 'CRC-32/MPEG-2("123456789")');
  eq(P.crc32Ieee(v), 0xCBF43926, 'CRC-32/IEEE("123456789")');
  // Bitwise reference, transcribed from Bootloader/Src/bl_crc.c
  const ref = data => { let c = 0xFFFFFFFF; for (const b of data) { c = (c ^ (b << 24)) >>> 0; for (let i = 0; i < 8; i++) c = (c & 0x80000000) ? (((c << 1) ^ 0x04C11DB7) >>> 0) : ((c << 1) >>> 0); } return c >>> 0; };
  const r = P.mulberry32(42), data = new Uint8Array(4099);
  for (let i = 0; i < data.length; i++) data[i] = Math.floor(r() * 256);
  eq(P.crc32Mpeg2(data), ref(data), 'table MPEG-2 equals bl_crc.c bitwise reference');
  eq(P.crc32Mpeg2(data.subarray(1000), P.crc32Mpeg2(data.subarray(0, 1000))), P.crc32Mpeg2(data), 'MPEG-2 continuation');
  eq(P.seckeyCrc('F'.repeat(32)), '3FB3C61A', 'CRC-32 of an all-FF key');
  eq(P.seckeyCrc('00112233445566778899AABBCCDDEEFF'), P.hex8(P.crc32Ieee(P.hexToBytes('00112233445566778899aabbccddeeff'))), 'SECKEY CRC uses raw key bytes');
});

await test('Write order: vector-table chunk last; VERIFY CRC over send order', () => {
  const len = 51200, offs = P.writeOrder(len);
  eq(offs.length, Math.ceil(len / 240), 'chunk count');
  eq(offs[0], 240, 'first chunk');
  eq(offs[offs.length - 1], 0, 'vector table chunk is sent last');
  eq(new Set(offs).size, offs.length, 'offsets unique');
  assert(offs.every(o => o % 8 === 0), 'all offsets 8-byte aligned');
  eq(JSON.stringify(P.writeOrder(100)), '[0]', 'tiny image');
  eq(JSON.stringify(P.writeOrder(240)), '[0]', 'one full chunk');
  eq(JSON.stringify(P.writeOrder(241)), '[240,0]', 'one byte over a chunk');
  const img = fakeImage(len, 5, 'x');
  const sent = new Uint8Array(len);
  sent.set(img.subarray(240), 0); sent.set(img.subarray(0, 240), len - 240);
  eq(P.sendOrderCrc(img), P.crc32Mpeg2(sent), 'sendOrderCrc == CRC(img[240:] + img[0:240])');
  assert(P.sendOrderCrc(img) !== P.crc32Mpeg2(img), 'send-order CRC differs from file-order CRC');
});

await test('Line reader, prefix matching, interleaved logs, redaction', async () => {
  const sp = new P.LineSplitter();
  const a = [...sp.push(ascii('+FW=a')), ...sp.push(ascii('bc\r')), ...sp.push(ascii('\n[I] x\r\n+OK\r\n'))];
  eq(JSON.stringify(a), JSON.stringify(['+FW=abc', '[I] x', '+OK']), 'split across chunks');
  // Fake transport: feed replies by hand.
  const t = { isOpen: false, baud: 0, sent: [], async open(b) { this.isOpen = true; this.baud = b; }, async close() { this.isOpen = false; }, async write(b) { this.sent.push(P.latin1(b)); } };
  const link = new P.Link(t);
  const rx = [], tx = [];
  link.on('rx', l => rx.push(l)); link.on('tx', l => tx.push(l));
  await link.open(115200);
  const p = P.atCmd(link, 'AT+FW=?', { expect: '+FW=' });
  await sentCmd(t, 'AT+FW=?');
  eq(t.sent[0], '\r\n', 'wake preamble before the first command');
  t.onBytes(ascii('\xF0\xFF\x00+FW=CC v1.2\r\n[D] log between reply and OK\r\n'));
  t.onBytes(ascii('+OK\r\n'));
  eq(await p, 'CC v1.2', 'value after garbage prefix and interleaved log');
  const q = P.atCmd(link, 'AT+ID=?', { expect: '+ID=' });
  await sentCmd(t, 'AT+ID=?');
  eq(t.sent.filter(s => s === '\r\n').length, 1, 'no second preamble while the unit is awake');
  t.onBytes(ascii('[CC] state -> SAMPLE_ENV\r\n+ERROR=1203\r\n'));
  // +ERROR=1203 without a preceding wake may be a command that lost its first byte: woken and retried once.
  await until(() => t.sent.filter(x => x.startsWith('AT+ID=?')).length === 2, 3000, 'the single retry');
  t.onBytes(ascii('+ERROR=1203\r\n'));
  const e = await rejects(q, e => e.code === '1203', 'error code');
  assert(/unknown or refused/.test(e.message), 'error text');
  link.secrets.add('00112233445566778899AABBCCDDEEFF');
  const w = P.atCmd(link, 'AT+SECKEY=00112233445566778899AABBCCDDEEFF', { what: 'AT+SECKEY=<redacted>' });
  await sentCmd(t, 'AT+SECKEY=');
  t.onBytes(ascii('[D] echo 00112233445566778899aabbccddeeff\r\n+SECKEY=00112233445566778899AABBCCDDEEFF\r\n+OK\r\n'));
  await w;
  const all = rx.concat(tx).join('\n');
  assert(!/00112233445566778899/i.test(all), 'key redacted from rx and tx');
  assert(tx.includes('AT+SECKEY=<redacted>'), 'tx shows the redacted command');
  // Race: a run finishes just as AT+SELFTEST=? goes out, so its live lines precede the replay.
  const q1 = P.querySelftest(link);
  await sentCmd(t, 'AT+SELFTEST=?');
  t.onBytes(ascii('+ST=TX,PASS,ms=3298\r\n+SELFTEST=FAIL,WS85\r\n+ST=GPS,PASS,bytes=907,nmea=10\r\n+ST=WS85,FAIL,silent\r\n'));
  t.onBytes(ascii('+ST=ENV,PASS,t=21.8\r\n[CC] tick\r\n+ST=ADC,PASS,nimh=2654\r\n+ST=TX,PASS,ms=3298\r\n+SELFTEST=FAIL,WS85\r\n+OK\r\n'));
  const r1 = await q1;
  eq(r1.status, 'FAIL', 'replay status'); eq(Object.keys(r1.items).length, 5, 'all five items from the replay'); eq(r1.lines.length, 6, 'replay lines');
  // Answer RUNNING, then a live summary before the +OK: stays RUNNING (the next poll gets the replay).
  t.sent.length = 0;
  const q2 = P.querySelftest(link);
  await sentCmd(t, 'AT+SELFTEST=?');
  t.onBytes(ascii('+SELFTEST=RUNNING\r\n+ST=TX,PASS,ms=1\r\n+SELFTEST=PASS\r\n+OK\r\n'));
  eq((await q2).status, 'RUNNING', 'first RUNNING answer kept');
  eq(P.redactLine('AT+SECKEY=?'), 'AT+SECKEY=?', 'query not redacted');
  eq(P.normalizeIdReadback('-1'), '4294967295', '%d readback of 0xFFFFFFFF');
  eq(P.normalizeIdReadback('-2147483648'), '2147483648', '%d readback of 2^31');
});

await test('Image sanity checks (vectors, size, _full.bin)', () => {
  const good = fakeImage(4096, 1, 'x');
  assert(P.analyzeImage(good, 'argos-smd-at-kineis-firmware.bin').ok, 'good image accepted');
  const bad = (mut, name = 'a.bin') => { const b = good.slice(); mut(b); return P.analyzeImage(b, name); };
  assert(!bad(b => P.putU32le(b, 0, 0x10000000)).ok, 'SP outside RAM refused');
  assert(!bad(b => P.putU32le(b, 4, 0x08001234)).ok, 'even reset vector refused');
  assert(!bad(b => P.putU32le(b, 4, 0x08040001)).ok, 'reset vector outside app refused');
  assert(!P.analyzeImage(new Uint8Array(0x32001).fill(0), 'big.bin').ok, 'oversize refused');
  assert(!P.analyzeImage(good, 'argos-smd-at-kineis-firmware_full.bin').ok, '_full.bin refused');
  const ok115 = P.analyzeImage(good.slice(0, 8), 't.bin');
  assert(ok115.ok && ok115.warnings.length === 1, 'reset vector past EOF is only a warning');
  const ib = new Uint8Array(15); ib[0] = 1; P.putU32le(ib, 3, P.APP_BASE); P.putU32le(ib, 7, 0x32000); P.putU32le(ib, 11, 0x800);
  const info = P.parseDfuInfo(P.bytesToHex(ib));
  eq(info.version, '1.0.0', 'INFO version'); eq(info.appStart, P.APP_BASE, 'INFO app start'); eq(info.maxSize, 0x32000, 'INFO max'); eq(info.pageSize, 0x800, 'INFO page');
});

await test('Self-test parsing and required-item verdict', () => {
  const g = P.parseStLine('+ST=GPS,PASS,bytes=812,nmea=9');
  eq(g.item, 'GPS', 'item'); eq(g.verdict, 'PASS', 'verdict'); eq(g.detail, 'bytes=812,nmea=9', 'detail with commas');
  eq(P.parseStLine('+ST=WS85,FAIL,silent').detail, 'silent', 'ws85 detail');
  eq(P.parseStLine('+ST=ADC,MAYBE'), null, 'bad verdict rejected');
  const s = P.parseSelftestStatus('+SELFTEST=FAIL,WS85,TX');
  eq(s.status, 'FAIL', 'summary'); eq(s.failed.join(','), 'WS85,TX', 'failed list');
  const items = {};
  for (const l of ['+ST=GPS,PASS,bytes=812,nmea=9', '+ST=WS85,FAIL,silent', '+ST=ENV,PASS,t=21.53,p=1013.2,rh=45', '+ST=ADC,PASS,nimh=2650,scap=4800', '+ST=TX,PASS,ms=1234']) { const x = P.parseStLine(l); items[x.item] = x; }
  const ev = P.evaluateSelftest(items, ['GPS', 'ENV', 'ADC', 'TX']);
  eq(ev.verdict, 'PASS', 'WS85 failure ignored when not required'); eq(ev.warnings.length, 1, 'one warning for WS85');
  eq(P.evaluateSelftest(items, P.ST_ITEMS).verdict, 'FAIL', 'WS85 required -> FAIL');
  eq(P.evaluateSelftest({ GPS: { verdict: 'SKIP' } }, ['GPS']).verdict, 'INCOMPLETE', 'required SKIP -> INCOMPLETE');
  eq(P.evaluateSelftest({}, ['GPS']).verdict, 'INCOMPLETE', 'required missing -> INCOMPLETE');
  const st = P.parseState('CYCLE_WAIT,up=123,tx=4,mac=1,comm=DONE,st=PASS');
  eq(st.state, 'CYCLE_WAIT', 'state'); eq(st.up, '123', 'up'); eq(st.comm, 'DONE', 'comm'); eq(st.st, 'PASS', 'st');
  const c = P.parseCommission('BURST,3/30,next=42');
  eq(c.state, 'BURST', 'comm state'); eq(c.sent, 3, 'sent'); eq(c.target, 30, 'target'); eq(c.extra.next, '42', 'extra');
});

await test('Credential validation and CLS paste parsing', () => {
  const key = '00112233445566778899AABBCCDDEEFF', rc = 'A1B2C3D4E5F60718293A4B5C6D7E8F90';
  const ok = P.validateCredentials({ id: ' 1234567 ', addr: '0a1b2c3d', seckey: key.toLowerCase(), rconf: '' });
  assert(ok.ok, 'valid set'); eq(ok.values.addr, '0A1B2C3D', 'addr upper'); eq(ok.values.seckey, key, 'key upper');
  const bad = P.validateCredentials({ id: '0', addr: '0A1B2C3', seckey: key + 'F', rconf: 'XYZ' });
  assert(!bad.ok && bad.errors.id && bad.errors.addr && bad.errors.seckey && bad.errors.rconf, 'all four fields rejected');
  assert(!P.validateCredentials({ id: '2147483649', addr: '0A1B2C3D', seckey: key }).ok, 'ID > 2^31 rejected');
  assert(P.validateCredentials({ id: '2147483648', addr: '0A1B2C3D', seckey: key }).ok, 'ID = 2^31 accepted');
  assert(P.validateCredentials({ id: '12345678', addr: '0A1B2C3D', seckey: key }).warnings.length === 1, '8-digit ID warns');
  const kv = P.parseCredentialText(`ID=1234567\nADDR: 0A1B2C3D\nSecret key = ${key}\nRadio conf = ${rc}`).values;
  eq(kv.id, '1234567', 'kv id'); eq(kv.addr, '0A1B2C3D', 'kv addr'); eq(kv.seckey, key, 'kv key'); eq(kv.rconf, rc, 'kv rconf');
  const csv = P.parseCredentialText(`device_id;address;seckey;rconf\n1234567;0A1B2C3D;${key};${rc}`);
  eq(csv.method, 'csv-header', 'csv method'); eq(csv.values.rconf, rc, 'csv rconf'); eq(csv.values.seckey, key, 'csv key');
  const pos = P.parseCredentialText(`1234567, 0A1B2C3D, ${key}, ${rc}`).values;
  eq(pos.id, '1234567', 'pos id'); eq(pos.addr, '0A1B2C3D', 'pos addr'); eq(pos.seckey, key, 'pos key'); eq(pos.rconf, rc, 'pos rconf');
});

await test('Report and CSV: no key material, escaping, column count', () => {
  const key = 'DEADBEEF00112233445566778899AABB';
  const checks = P.compareCredentials(P.validateCredentials({ id: '1234567', addr: '0A1B2C3D', seckey: key }).values, null,
    { id: '1234567', addr: '0A1B2C3D', seckeyCrc: P.seckeyCrc(key), rconf: '401620000,401680000,27,LDA2' });
  assert(checks.filter(c => c.ok === true).length === 3, 'three verified fields');
  const run = P.newRun('boot'); run.status = 'FAIL'; run.summary = '+SELFTEST=FAIL,WS85';
  for (const l of ['+ST=GPS,PASS,bytes=1,nmea=2', '+ST=WS85,FAIL,silent', '+ST=ENV,PASS,t=1', '+ST=ADC,PASS,nimh=2650', '+ST=TX,PASS,ms=9']) { const x = P.parseStLine(l); run.items[x.item] = x; run.lines.push(l); }
  run.lines.push(run.summary);
  const rep = P.buildReport({ unit: { fw: 'CC "x", v1', id: '1234567', addr: '0A1B2C3D', sn: 'S1', seckeyCrc: P.seckeyCrc(key), rconf: { error: '+ERROR=1203' } },
    selftest: run, required: ['GPS', 'ENV', 'ADC', 'TX'], commission: P.parseCommission('DONE'), creds: { ok: true, at: 't', checks }, operator: 'op', notes: 'n' });
  eq(rep.verdict, 'PASS', 'unit verdict'); eq(rep.selftest.verdict, 'PASS', 'self-test verdict'); eq(rep.unit.rconf, null, 'error object dropped');
  const json = JSON.stringify(rep);
  assert(!json.toUpperCase().includes(key), 'report JSON has no key');
  const row = P.reportToCsvRow(rep);
  eq(row.length, P.CSV_COLUMNS.length, 'column count');
  const csv = P.buildCsv([row]);
  assert(csv.includes('"CC ""x"", v1"'), 'CSV quoting');
  assert(!csv.toUpperCase().includes(key), 'CSV has no key');
  eq(P.csvEscape('a,b'), '"a,b"', 'escape comma');
  const old = P.csvRecord(row); delete old.awake_duty;                 // a stored row from a version without that column
  const lines = P.buildCsv([old, P.csvRecord(row)]).trim().split('\r\n');
  eq(lines.length, 3, 'header + two rows');
  eq(lines[1].replace(/"[^"]*"/g, 'Q').split(',').length, P.CSV_COLUMNS.length, 'old keyed row still has every column');
  eq(lines[2], csv.trim().split('\r\n')[1], 'keyed row == array row');
});

/* ---------- simulator-driven flows ---------- */

await test('Detect: unit still in its 2 s boot silence when Connect starts', async () => {
  for (const scale of [0.05, 0.01]) {
    const old = P.Clock.scale; P.Clock.scale = scale;
    const r = await rig({ pendingChance: 0 }, { waitApp: false });
    try {
      const d = await P.detectUnit(r.link, { bauds: [115200, 9600] });
      eq(d.mode, 'app', `app found at scale ${scale}`); eq(d.baud, 115200, 'baud');
    } finally { r.done(); P.Clock.scale = old; }
  }
});

await test('Detect: app at 9600 (older build), app at 115200, bootloader', async () => {
  const a = await rig({ appBaud: 9600, pendingChance: 0 }, { baud: 115200, waitApp: false });
  try {
    await P.sleep(4000);
    const d = await P.detectUnit(a.link, { bauds: [115200, 9600] });
    eq(d.mode, 'app', 'mode'); eq(d.baud, 9600, 'baud'); assert(/factory/.test(d.fw), 'factory FW string');
  } finally { a.done(); }
  const b = await rig({ appBaud: 115200, pendingChance: 0 }, { baud: 9600, waitApp: false });
  try {
    await P.sleep(4000);
    const d = await P.detectUnit(b.link, { bauds: [115200, 9600] });
    eq(d.mode, 'app', 'mode'); eq(d.baud, 115200, 'baud');
    await P.requestBoot(b.link);
    await P.sleep(500);
    const d2 = await P.detectUnit(b.link, { bauds: [115200, 9600] });
    eq(d2.mode, 'bl', 'bootloader detected'); eq(d2.baud, 9600, 'bootloader baud'); eq(d2.bl, 'BL_V1.0.0', 'BL version');
  } finally { b.done(); }
});

let flashedReport = '';
await test('Flash a random 50 KB image (app 115200, bootloader 9600, PENDING allowed) + boot self-test', async () => {
  const r = await rig({ appBaud: 115200, blBaud: 9600, pendingChance: 0.5, seed: 99, provisioned: true });
  try {
    const img = fakeImage(50 * 1024, 2024, 'CC_TRACKER v9.9.9-test');
    const phases = [];
    let last = null;
    const res = await P.flashFirmware(r.link, img, { name: 'argos-smd-at-kineis-firmware.bin', expectFw: 'v9.9.9-test', onPhase: p => phases.push(p), onProgress: p => { last = p; } });
    assert(res.ok, 'result ok'); eq(res.fwCheck, true, 'expected FW string present');
    assert(/factory/.test(res.oldFw), 'old FW reported'); eq(res.newFw, 'CC_TRACKER v9.9.9-test', 'new FW');
    eq(res.bootloader.baud, 9600, 'bootloader found at 9600'); eq(res.bootloader.version, 'BL_V1.0.0', 'BL version');
    eq(res.info.appStart, P.APP_BASE, 'INFO app start'); eq(res.info.maxSize, 0x32000, 'INFO max size'); eq(res.info.pageSize, 0x800, 'INFO page size');
    eq(res.validated, '+DFU=OK,VALIDATED', 'VALIDATED seen');
    eq(res.verifyCrc, P.hex8(P.sendOrderCrc(img)), 'VERIFY CRC is the send-order CRC');
    eq(last.done, img.length, 'progress reached 100%');
    assert(sameBytes(r.sim.flash.subarray(0, img.length), img), 'simulated flash == image');
    assert(allFF(r.sim.flash.subarray(img.length)), 'rest of the app region erased');
    eq(r.sim.writeLog[0], P.APP_BASE + 240, 'first WRITE at +240');
    eq(r.sim.writeLog[r.sim.writeLog.length - 1], P.APP_BASE, 'vector table WRITE last');
    assert(!r.log.some(l => l.startsWith('> AT+DFU=JUMP')), 'no AT+DFU=JUMP sent');
    assert(r.log.some(l => l.startsWith(`> AT+DFU=VERIFY,${res.verifyCrc}`)), 'VERIFY sent with the send-order CRC');
    const st = await P.pollBootSelftest(r.link, {});
    eq(st.status, 'FAIL', 'boot self-test summary (WS85 not connected)');
    eq(st.items.WS85.detail, 'silent', 'WS85 silent');
    eq(P.evaluateSelftest(st.items, ['GPS', 'ENV', 'ADC', 'TX']).verdict, 'PASS', 'verdict without WS85 required');
    eq(P.evaluateSelftest(st.items, P.ST_ITEMS).verdict, 'FAIL', 'verdict with WS85 required');
    const pend = r.log.filter(l => /\+(BOOT|RESET|SELFTEST)=PENDING/.test(l)).length;
    flashedReport = `${img.length} B, ${res.bootloader.baud} baud BL, ${phases.length} phase events, ${pend} PENDING replies`;
    return flashedReport;
  } finally { r.done(); }
});

await test('Mid-flash abort leaves flash word 0 erased; recovery from the bootloader; silent ROM boot after power-cycle', async () => {
  const r = await rig({ appBaud: 115200, blBaud: 9600, pendingChance: 0, seed: 5 });
  try {
    const img = fakeImage(30 * 1024, 77, 'CC_TRACKER v2.0.0-abort');
    const ac = new AbortController();
    const err = await rejects(P.flashFirmware(r.link, img, { signal: ac.signal, onProgress: p => { if (p.done >= 20 * 240) ac.abort(); } }),
      e => e.name === 'CancelledError', 'abort mid-write');
    assert(err.inBootloader, 'error says the unit is in the bootloader');
    eq(P.u32le(r.sim.flash, 0), 0xFFFFFFFF, 'flash word 0 (initial SP) still erased');
    eq(P.u32le(r.sim.flash, 4), 0xFFFFFFFF, 'flash word 1 (reset vector) still erased');
    assert(!allFF(r.sim.flash.subarray(240, 240 * 10)), 'later chunks were written');
    eq(r.sim.mode, 'bl', 'unit stays in the bootloader');
    // Recover: the tool finds the bootloader and finishes the update from there.
    const d = await P.detectUnit(r.link, { bauds: [115200, 9600] });
    eq(d.mode, 'bl', 'detect finds the bootloader'); eq(d.baud, 9600, 'at 9600');
    const res = await P.flashFirmware(r.link, img, { startMode: 'bl' });
    assert(res.ok, 'recovery flash ok');
    assert(sameBytes(r.sim.flash.subarray(0, img.length), img), 'flash == image after recovery');
    // Interrupt again, then power-cycle: the empty first word makes the chip start ST's (silent) ROM bootloader.
    const ac2 = new AbortController();
    await rejects(P.flashFirmware(r.link, img, { signal: ac2.signal, onProgress: p => { if (p.done >= 5 * 240) ac2.abort(); } }), e => e.name === 'CancelledError', 'second abort');
    eq(P.u32le(r.sim.flash, 0), 0xFFFFFFFF, 'word 0 erased again');
    r.sim.powerCycle();
    await P.sleep(1000);
    eq(r.sim.mode, 'rom', 'power-up with an empty first word -> ROM bootloader');
    const d2 = await P.detectUnit(r.link, { bauds: [115200, 9600] });
    eq(d2.mode, null, 'app and DFU bootloader both silent (the UI then shows the rom_uart_flash.py hint)');
  } finally { r.done(); }
});

await test('Bootloader WRITE truncation beyond 244 bytes, alignment, NOT_READY, CRC_ERROR, VALIDATION_FAILED', async () => {
  assert((() => { try { P.buildWriteLine(P.APP_BASE, new Uint8Array(245)); return false; } catch (e) { return true; } })(), 'flasher refuses 245-byte chunks');
  assert((() => { try { P.buildWriteLine(P.APP_BASE + 4, new Uint8Array(16)); return false; } catch (e) { return true; } })(), 'flasher refuses unaligned addresses');
  const r = await rig({ appBaud: 115200, blBaud: 115200, pendingChance: 0 });
  try {
    await P.requestBoot(r.link);
    await P.blPing(r.link, { bauds: [115200, 9600] });
    const data = new Uint8Array(250); for (let i = 0; i < data.length; i++) data[i] = i & 0xFF;
    const line = a => `AT+DFU=WRITE,${P.hex8(a)},${P.bytesToHex(data)}`;
    eq(await P.dfuCmd(r.link, line(P.APP_BASE + 0x1000)), '+DFU=ERR,NOT_READY', 'WRITE before ERASE');
    eq(await P.dfuCmd(r.link, 'AT+DFU=ERASE', { timeout: 10000 }), '+DFU=OK', 'ERASE');
    eq(await P.dfuCmd(r.link, line(P.APP_BASE + 0x1004)), '+DFU=ERR,ADDR_ERROR', 'unaligned WRITE');
    eq(await P.dfuCmd(r.link, line(P.APP_BASE + 0x1000)), '+DFU=OK', '250-byte WRITE is accepted (silently truncated)');
    assert(sameBytes(r.sim.flash.subarray(0x1000, 0x1000 + 244), data.subarray(0, 244)), 'first 244 bytes written');
    assert(allFF(r.sim.flash.subarray(0x1000 + 244, 0x1000 + 250)), 'bytes 244..249 were dropped');
    eq(await P.dfuCmd(r.link, `AT+DFU=VERIFY,${P.hex8(P.crc32Mpeg2(data))}`), '+DFU=ERR,CRC_ERROR', 'VERIFY over the full 250 bytes fails');
    eq(await P.dfuCmd(r.link, `AT+DFU=VERIFY,${P.hex8(P.crc32Mpeg2(data.subarray(0, 244)))}`), '+DFU=OK', 'VERIFY over what arrived passes');
    eq(await r.link.waitLine(l => l.startsWith('+DFU='), { timeout: 3000 }), '+DFU=ERR,VALIDATION_FAILED', 'no vector table -> VALIDATION_FAILED');
    eq(await P.dfuCmd(r.link, 'AT+DFU=JUMP'), '+DFU=ERR,NOT_READY', 'JUMP refused after a failed validation');
    eq(await P.dfuCmd(r.link, 'AT+DFU=BOGUS'), '+DFU=ERR,INVALID_CMD', 'unknown command');
  } finally { r.done(); }
});

await test('Enter bootloader & return (read-only), AT+BOOT refused on old firmware', async () => {
  const r = await rig({ appBaud: 115200, blBaud: 9600, pendingChance: 0 });
  try {
    const before = r.sim.flash.slice();
    const res = await P.bootloaderCheck(r.link, {});
    eq(res.bootloader.version, 'BL_V1.0.0', 'PING'); eq(res.info.appStart, P.APP_BASE, 'INFO'); assert(/^[0-9A-F]{22}$/.test(res.status), 'STATUS hex');
    eq(res.newFw, res.oldFw, 'same firmware afterwards'); eq(res.appBaud, 115200, 'app baud');
    assert(sameBytes(before, r.sim.flash), 'flash untouched');
    eq(r.sim.mode, 'app', 'back in the app');
  } finally { r.done(); }
  const o = await rig({ noDfu: true, pendingChance: 0 });
  try {
    const before = o.sim.flash.slice();
    const e = await rejects(P.flashFirmware(o.link, fakeImage(2048, 3, 'x')), e => e.noDfu, 'AT+BOOT refused');
    assert(/SWD|BOOT0/.test(e.message), 'message explains the one-time SWD/BOOT0 flash');
    assert(!e.inBootloader, 'not in the bootloader');
    assert(sameBytes(before, o.sim.flash), 'flash untouched');
  } finally { o.done(); }
});

await test('PENDING handling: AT+RESET= and AT+BOOT= wait for the TX to end', async () => {
  const r = await rig({ pendingChance: 0 });
  try {
    r.sim._startTx('test', 3000);
    let pending = 0;
    const back = await P.resetApp(r.link, { onPending: () => pending++ });
    eq(pending, 1, '+RESET=PENDING seen'); assert(back.fw, 'app back after the reset');
    await P.sleep(6000);
    r.sim._startTx('test', 3000);
    let bootPending = 0;
    await P.requestBoot(r.link, { onPending: () => bootPending++ });
    eq(bootPending, 1, '+BOOT=PENDING seen');
    const bl = await P.blPing(r.link, { bauds: [115200, 9600] });
    eq(bl.baud, 9600, 'bootloader after the pending boot');
    await P.exitBootloader(r.link);
  } finally { r.done(); }
});

await test('Erase credentials (AT+PROV=ERASE): BUSY retry, reboot into NO_CREDS, re-provision', async () => {
  const r = await rig({ pendingChance: 0, provisioned: true, seed: 21 });
  try {
    eq((await P.getState(r.link)).prov, '1', 'starts provisioned');
    const rconfBefore = r.sim.nvm.rconfSaved;
    r.sim._startTx('test', 4000);                      // erase lands on a busy radio first
    let busy = 0;
    const back = await P.eraseCredentials(r.link, { onBusy: () => busy++, retryMs: 500 });
    assert(busy >= 1, 'BUSY retry exercised');
    assert(back && back.fw, 'app came back after the reboot');
    eq(r.sim.nvm.id, 0xFFFFFFFF, 'ID blank'); eq(r.sim.nvm.addr, 'ffffffff', 'ADDR blank');
    assert(r.sim.nvm.seckey.every(b => b === 0xFF), 'SECKEY blank');
    eq(r.sim.nvm.rconfSaved, rconfBefore, 'RCONF kept');
    eq((await P.getState(r.link)).prov, '0', 'NO_CREDS after the reboot');
    assert(r.log.some(l => l === '< +PROV=ERASED') && r.log.some(l => l === '< +RESET=OK'), 'ERASED then RESET=OK seen');
    const rnd = P.mulberry32(4242);
    const creds = { id: String(1000000 + Math.floor(rnd() * 8999999)), addr: randomHex(rnd, 8), seckey: randomHex(rnd, 32) };
    const res = await P.provisionCredentials(r.link, creds, {});
    assert(res.ok, 're-provisioned after the erase');
    // Wrong forms are refused like any unknown command
    await rejects(P.atCmd(r.link, 'AT+PROV=erase'), e => e.code === '1203', 'lower-case form refused');
  } finally { r.done(); }
});

await test('Provision credentials (BUSY retries, SAVE_RCONF, reset, readbacks, SECKEYCRC)', async () => {
  const r = await rig({ pendingChance: 0.3, seed: 7 });
  try {
    const rnd = P.mulberry32(31337);
    const creds = { id: String(1000000 + Math.floor(rnd() * 8999999)), addr: randomHex(rnd, 8), seckey: randomHex(rnd, 32), rconf: randomHex(rnd, 32) };
    r.sim._startTx('test', 4000);                      // first write lands on a busy radio
    const steps = [];
    const res = await P.provisionCredentials(r.link, creds, { onStep: (k, s, l, d) => steps.push(`${k}:${s}:${d || ''}`) });
    assert(res.ok, 'provisioning verified: ' + JSON.stringify(res.checks));
    eq(r.sim.nvm.id, Number(creds.id), 'sim stored ID'); eq(r.sim.nvm.addr, creds.addr.toLowerCase(), 'sim stored ADDR');
    eq(P.bytesToHex(r.sim.nvm.seckey), creds.seckey, 'sim stored key'); eq(r.sim.nvm.rconfSaved, creds.rconf, 'sim saved RCONF');
    eq(res.post.seckeyCrc, P.seckeyCrc(creds.seckey), 'SECKEYCRC readback matches the local CRC');
    eq(res.post.id, creds.id, 'ID after reset'); eq(res.post.addr, creds.addr, 'ADDR after reset');
    eq(res.post.rconf, res.pre.rconf, 'RCONF persisted across the reset');
    const busy = steps.filter(s => s.includes('BUSY')).length;
    assert(busy >= 1, 'BUSY retry exercised');
    assert(steps.some(s => s.startsWith('reset:ok')) && steps.some(s => s.startsWith('readback2:ok')), 'reset and second readback ran');
    const transcript = r.log.join('\n');
    assert(!transcript.toUpperCase().includes(creds.seckey), 'key never in the console transcript');
    assert(transcript.includes('AT+SECKEY=<redacted>'), 'redacted SECKEY write visible');
    const rep = P.buildReport({ unit: { id: res.post.id, addr: res.post.addr, seckeyCrc: res.post.seckeyCrc }, creds: res });
    assert(!JSON.stringify(rep).toUpperCase().includes(creds.seckey), 'key never in the report');
    assert(!P.buildCsv([P.reportToCsvRow(rep)]).toUpperCase().includes(creds.seckey), 'key never in the CSV');
    // RCONF written but not saved -> reverts after reset, and the comparison catches it.
    await P.atCmd(r.link, `AT+RCONF=${randomHex(rnd, 32)}`);
    const pre = await P.readCredentials(r.link);
    await P.resetApp(r.link);
    const post = await P.readCredentials(r.link);
    const rc = P.compareCredentials({ rconf: 'X'.repeat(32) }, pre, post).find(c => c.field === 'RCONF');
    eq(rc.ok, false, 'unsaved RCONF flagged');
    const bad = await rejects(P.atCmd(r.link, 'AT+ADDR=XYZ'), e => e.code === '1200', 'bad ADDR format');
    assert(/bad parameter format/.test(bad.message), 'format error text');
    await rejects(P.provisionCredentials(r.link, { id: '0', addr: creds.addr, seckey: creds.seckey }), e => /Invalid input/.test(e.message), 'invalid input refused before sending');
    return `${busy} BUSY retries`;
  } finally { r.done(); }
});

await test('Self-test: boot poll, explicit subset run, PENDING start, unknown item, forced failure', async () => {
  const r = await rig({ pendingChance: 0, stFail: [], provisioned: true });
  try {
    const boot = await P.pollBootSelftest(r.link, {});
    eq(boot.status, 'FAIL', 'boot run done (WS85 silent)');
    const updates = [];
    const run = await P.runSelftest(r.link, ['GPS', 'ENV', 'ADC', 'TX'], { onUpdate: x => updates.push(x.status) });
    eq(run.status, 'PASS', 'subset passes'); eq(Object.keys(run.items).sort().join(','), 'ADC,ENV,GPS,TX', 'four items'); eq(run.summary, '+SELFTEST=PASS', 'summary');
    assert(updates.includes('RUNNING'), 'RUNNING reported');
    const q = await P.querySelftest(r.link);
    eq(q.status, 'PASS', 'query replays the last run'); eq(q.lines.length, 5, 'four +ST lines and the summary');
    r.sim._startTx('test', 3000);
    const statuses = [];
    const run2 = await P.runSelftest(r.link, ['ENV', 'ADC'], { onUpdate: x => statuses.push(x.status) });
    eq(statuses[0], 'PENDING', 'PENDING while the radio is busy'); eq(run2.status, 'PASS', 'then runs by itself');
    await rejects(P.runSelftest(r.link, ['GPS', 'FOO']), e => e.code === '1203', 'unknown item -> +ERROR=1203');
    r.sim.o.stFail = ['GPS'];
    const run3 = await P.runSelftest(r.link, ['GPS', 'ADC']);
    eq(run3.summary, '+SELFTEST=FAIL,GPS', 'forced GPS failure');
    eq(P.evaluateSelftest(run3.items, ['GPS', 'ADC']).verdict, 'FAIL', 'overall FAIL');
  } finally { r.done(); }
});

await test('Commissioning: query, START burst (TX every minute), SKIP, ARM; STATE line', async () => {
  const r = await rig({ pendingChance: 0, provisioned: true });
  try {
    await P.pollBootSelftest(r.link, {});
    eq((await P.commissionCmd(r.link, null)).state, 'WAIT_FIX', 'fresh unit waits for a fix');
    const s = await P.commissionCmd(r.link, 'START');
    eq(s.state, 'BURST', 'burst started'); eq(s.sent, 0, 'sent'); eq(s.target, 30, 'target');
    await P.sleep(75000);
    const s2 = await P.commissionCmd(r.link, null);
    assert(s2.state === 'BURST' && s2.sent >= 1, `burst progressing (${s2.raw})`);
    eq((await P.commissionCmd(r.link, 'SKIP')).state, 'DONE', 'skip -> DONE');
    eq((await P.commissionCmd(r.link, 'ARM')).state, 'WAIT_FIX', 'arm -> WAIT_FIX');
    await rejects(P.commissionCmd(r.link, 'BOGUS'), e => e.code === '1203', 'unknown action');
    const st = await P.getState(r.link);
    assert(['SAMPLE_WS85', 'SAMPLE_ENV', 'SAMPLE_GPS', 'TX_SEND', 'WAIT_TX_DONE', 'CYCLE_WAIT', 'SELFTEST'].includes(st.state), 'valid STATE');
    eq(st.mac, '1', 'MAC ready'); eq(st.comm, 'WAIT_FIX', 'comm field'); assert(['PASS', 'FAIL'].includes(st.st), 'st field');
    eq(st.prov, '1', 'prov field'); assert(/^\d+$/.test(st.slp), 'slp field');
    const info = await P.readUnitInfo(r.link);
    eq(info.id, '1234567', 'provisioned ID'); assert(/^[0-9A-F]{8}$/.test(info.seckeyCrc) && info.seckeyCrc !== '3FB3C61A', 'provisioned key CRC');
    assert(info.sn && info.atVersion && info.rconf, 'SN / VERSION / RCONF read');
  } finally { r.done(); }
});

await test('STATE parsing is generic (slp, prov, future fields); awake duty; rescue/provisioning monitor', () => {
  const s = P.parseState('CYCLE_WAIT,up=200,tx=4,mac=1,comm=DONE,st=PASS,slp=150,prov=1,future=x');
  eq(s.state, 'CYCLE_WAIT', 'state'); eq(s.slp, '150', 'slp'); eq(s.prov, '1', 'prov'); eq(s.future, 'x', 'unknown trailing field kept');
  eq(P.awakeDuty(s), 0.25, 'awake duty (up - slp) / up');
  eq(P.awakeDuty(P.parseState('CYCLE_WAIT,up=200,tx=4')), null, 'no slp -> null');
  eq(P.awakeDuty(P.parseState('BOOT,up=0,slp=0')), null, 'up=0 -> null');
  const m = new P.UnitMonitor();
  eq(m.feedLine('[I] boot'), false, 'log lines change nothing');
  eq(m.feedLine('+PROV=NONE'), true, '+PROV=NONE'); eq(m.prov, false, 'not provisioned');
  eq(m.feedLine('+RESCUE=failed_boots=3,window_s=600'), true, '+RESCUE line'); eq(m.rescue.failedBoots, 3, 'failed boots'); eq(m.rescue.windowS, 600, 'window');
  eq(m.feedLine('+STATE=RESCUE,up=5,tx=0,mac=0,comm=WAIT_FIX,st=NONE,slp=0,prov=1'), true, 'STATE RESCUE with prov=1'); assert(m.rescue, 'still rescue'); eq(m.prov, true, 'prov from STATE');
  eq(m.feedLine('+STATE=CYCLE_WAIT,up=50,tx=1,mac=1,comm=DONE,st=PASS,slp=10,prov=1'), true, 'normal STATE clears rescue'); eq(m.rescue, null, 'rescue cleared');
  m.feedLine('+STATE=NO_CREDS,up=5,tx=0,mac=0,comm=WAIT_FIX,st=FAIL'); eq(m.prov, false, 'NO_CREDS without a prov field -> not provisioned');
  m.feedLine('+PROV=OK'); eq(m.prov, true, '+PROV=OK');
  const items = { GPS: P.parseStLine('+ST=GPS,PASS,x'), ENV: P.parseStLine('+ST=ENV,PASS,x'), ADC: P.parseStLine('+ST=ADC,PASS,x'), TX: P.parseStLine('+ST=TX,FAIL,unprovisioned') };
  assert(P.isTxDeferred(items.TX), 'TX unprovisioned is deferred');
  const ev = P.evaluateSelftest(items, ['GPS', 'ENV', 'ADC', 'TX']);
  eq(ev.verdict, 'PENDING_TX', 'hardware OK pending TX'); assert(/provision, then re-run/.test(ev.reasons[0]), 'tells the user to provision and re-run');
  eq(P.evaluateSelftest({ ...items, TX: P.parseStLine('+ST=TX,FAIL,timeout') }, ['TX']).verdict, 'FAIL', 'a real TX failure stays FAIL');
});

await test('Wake preamble: STOP2 drops the first byte; the tool wakes the unit; one retry after a slow wake', async () => {
  const r = await rig({ provisioned: true, pendingChance: 0 });
  try {
    await P.pollBootSelftest(r.link, {});
    await P.sleep(40000);                                   // > 30 s without console RX: the unit is in STOP2
    let before = r.sim.droppedWakeBytes, logAt = r.log.length;
    assert(/v0\.9\.0/.test(await P.atCmd(r.link, 'AT+FW=?', { expect: '+FW=' })), 'command after a long idle works');
    eq(r.sim.droppedWakeBytes, before + 1, 'exactly one byte (of the preamble) was lost');
    assert(r.log.slice(logAt).some(l => l.startsWith('> \\r\\n (wake)')), 'wake preamble sent');
    // Without the preamble the first byte of the command itself is lost: proves the emulation bites ...
    r.link.wakeEnabled = false;
    await P.sleep(40000);
    before = r.sim.droppedWakeBytes;
    await rejects(P.atCmd(r.link, 'AT+FW=?', { expect: '+FW=', retry: false }), e => e.code === '1203', 'command without preamble (and no retry) is mangled');
    eq(r.sim.droppedWakeBytes, before + 1, 'the "A" of AT+FW=? was lost');
    // ... and an unnoticed sleep (e.g. after a power-cycle) is recovered by the wake-and-retry on +ERROR=1203.
    await P.sleep(40000);
    const notes0 = [];
    const off0 = r.link.on('note', n => notes0.push(n));
    assert(/v0\.9\.0/.test(await P.atCmd(r.link, 'AT+FW=?', { expect: '+FW=' })), 'recovered by the retry');
    off0();
    assert(notes0.some(n => /\+ERROR=1203 to AT\+FW=\?: waking it and retrying once/.test(n)), notes0.join(' | '));
    r.link.wakeEnabled = true;
    // Slow wake (bytes lost for 1.5 s: longer than the 150 ms preamble wait, shorter than the 2.5 s
    // reply timeout): the first command is lost and times out, the single retry gets through.
    r.sim.o.wakeLatencyMs = 1500;
    await P.sleep(40000);
    const notes = [];
    const off = r.link.on('note', n => notes.push(n));
    assert(await P.atCmd(r.link, 'AT+SN=?', { expect: '+SN=' }), 'answered after the retry');
    off();
    assert(notes.some(n => /retrying once/.test(n)), 'retry-once note');
  } finally { r.done(); }
});

await test('Unprovisioned unit: +PROV=NONE, NO_CREDS, TX unprovisioned; +PROV=OK once ID/ADDR/SECKEY are written', async () => {
  const r = await rig({ pendingChance: 0 }, { waitApp: false });
  const mon = new P.UnitMonitor();
  r.link.on('rx', l => mon.feedLine(l));
  try {
    await P.waitForApp(r.link, { settle: 0, timeout: 20000 });
    const boot = await P.pollBootSelftest(r.link, {});
    eq(boot.items.TX.detail, 'unprovisioned', 'boot self-test TX unprovisioned');
    eq(P.evaluateSelftest(boot.items, ['GPS', 'ENV', 'ADC', 'TX']).verdict, 'PENDING_TX', 'verdict pending TX');
    const s = await P.getState(r.link);
    eq(s.state, 'NO_CREDS', 'state'); eq(s.prov, '0', 'prov=0'); eq(s.mac, '0', 'no MAC');
    eq(mon.prov, false, 'monitor: not provisioned');
    const run = await P.runSelftest(r.link, ['ENV', 'TX']);
    eq(run.items.TX.detail, 'unprovisioned', 'explicit run starts at once, TX unprovisioned');
    await P.atCmd(r.link, 'AT+ID=7654321');
    await P.atCmd(r.link, 'AT+ADDR=0A1B2C3D');
    await P.sleep(2500);
    eq(mon.prov, false, 'two of three credentials: still unprovisioned');
    await P.atCmd(r.link, 'AT+SECKEY=000102030405060708090A0B0C0D0E0F');
    await until(() => mon.prov === true, 5000, '+PROV=OK');
    assert(r.log.includes('< +PROV=OK'), '+PROV=OK printed');
    await P.sleep(4000);
    const s2 = await P.getState(r.link);
    eq(s2.prov, '1', 'prov=1'); eq(s2.mac, '1', 'radio started without a reset');
    await P.resetApp(r.link);
    const boot2 = await P.pollBootSelftest(r.link, {});
    eq(boot2.items.TX.verdict, 'PASS', 'TX tested after provisioning');
  } finally { r.done(); }
});

await test('Rescue mode after 3 failed boots: only BOOT/RESET/queries/credential writes; reflash recovers', async () => {
  const r = await rig({ crashOnBoot: true, provisioned: true, pendingChance: 0 }, { waitApp: false });
  const mon = new P.UnitMonitor();
  r.link.on('rx', l => mon.feedLine(l));
  try {
    await until(() => mon.rescue, 15000, '+RESCUE line');
    eq(mon.rescue.failedBoots, 3, 'failed boots'); eq(mon.rescue.windowS, 600, 'window');
    await P.sleep(500);
    const s = await P.getState(r.link);
    eq(s.state, 'RESCUE', 'STATE reports RESCUE'); assert(mon.rescue, 'banner stays on');
    await rejects(P.runSelftest(r.link, ['GPS']), e => e.code === '1203', 'AT+SELFTEST= refused');
    await rejects(P.querySelftest(r.link), e => e.code === '1203', 'AT+SELFTEST=? refused');
    await rejects(P.commissionCmd(r.link, 'START'), e => e.code === '1203', 'COMMISSION=START refused');
    eq((await P.commissionCmd(r.link, null)).state, 'WAIT_FIX', 'COMMISSION=? still answers');
    eq(await P.atCmd(r.link, 'AT+ID=7654321'), 'OK', 'credential write works');
    await rejects(P.commissionUnit(r.link, { mode: 'app' }), e => e.step === 'connect' && /rescue/.test(e.message), 'wizard without an image stops at once');
    const res = await P.flashFirmware(r.link, fakeImage(8000, 9, 'CC_TRACKER v3.0.0-fixed'), { expectFw: 'v3.0.0' });
    assert(res.ok, 'reflash from rescue mode');
    await P.sleep(8000);
    const s2 = await P.getState(r.link);
    assert(s2.state !== 'RESCUE', `normal boot after the reflash (${s2.state})`); eq(mon.rescue, null, 'banner cleared');
  } finally { r.done(); }
});

await test('Guided commissioning of a factory unit: flash -> sensors -> provision -> +PROV=OK -> reset -> self-test incl. TX -> PASS', async () => {
  const r = await rig({ appBaud: 115200, blBaud: 9600, pendingChance: 0.2, seed: 11 });
  try {
    const steps = [];
    const onStep = (k, s, d) => steps.push(`${k}:${s}${d ? ':' + d : ''}`);
    // Without credentials an unprovisioned unit cannot be commissioned.
    const e = await rejects(P.commissionUnit(r.link, { mode: 'app' }, { onStep }), e => e.step === 'prov', 'stops at provisioning');
    assert(/not provisioned/.test(e.message), e.message);
    assert(steps.some(s => s.startsWith('st1:ok') && /TX pending/.test(s)), 'sensor self-test passed with TX pending');
    steps.length = 0;
    const rnd = P.mulberry32(4242);
    const creds = { id: '2345678', addr: randomHex(rnd, 8), seckey: randomHex(rnd, 32), rconf: randomHex(rnd, 32) };
    const img = fakeImage(40 * 1024, 77, 'CC_TRACKER v1.4.0-prod');
    const required = ['GPS', 'ENV', 'ADC', 'TX'];
    const res = await P.commissionUnit(r.link, { mode: 'app', image: img, imageName: 'argos-smd-at-kineis-firmware.bin', expectFw: 'v1.4.0', creds, required }, { onStep });
    assert(res.ok, 'wizard ok');
    for (const k of ['connect', 'flash', 'st1', 'prov', 'st2', 'comm']) assert(steps.includes(steps.find(s => s.startsWith(`${k}:ok`))), `step ${k} ok: ${steps.join(' | ')}`);
    eq(res.flash.newFw, 'CC_TRACKER v1.4.0-prod', 'flashed'); eq(res.creds.provEvent, '+PROV=OK', '+PROV=OK seen');
    eq(res.selftest.items.TX.verdict, 'PASS', 'TX tested after provisioning'); eq(res.verdict, 'PASS', 'self-test verdict');
    eq(res.state.prov, '1', 'provisioned');
    const rep = P.buildReport({ unit: { fw: res.flash.newFw, id: res.creds.post.id, addr: res.creds.post.addr, seckeyCrc: res.creds.post.seckeyCrc, state: res.state },
      selftest: res.selftest, required, flash: res.flash, creds: res.creds, commission: res.commission });
    eq(rep.verdict, 'PASS', 'report verdict'); eq(rep.unit.provisioned, true, 'report: provisioned');
    assert(!JSON.stringify(rep).toUpperCase().includes(creds.seckey), 'no key in the report');
    assert(!r.log.join('\n').toUpperCase().includes(creds.seckey), 'no key in the console transcript');
    // A second pass on the finished unit skips everything that is already done.
    steps.length = 0;
    const again = await P.commissionUnit(r.link, { mode: 'app', image: img, expectFw: 'v1.4.0', required }, { onStep });
    assert(again.ok, 'second pass ok');
    assert(steps.some(s => s.startsWith('flash:skip')) && steps.some(s => s.startsWith('prov:skip')) && steps.some(s => s.startsWith('st2:skip')), steps.join(' | '));
  } finally { r.done(); }
});

await test('Battery: bat/tier STATE fields, CRIT unit skips the boot TX test (SKIP batt_crit is not a failure)', async () => {
  const s = P.parseState('CYCLE_WAIT,up=100,tx=1,mac=1,comm=DONE,st=PASS,slp=60,prov=1,bat=2650,tier=HIGH');
  eq(s.bat, '2650', 'bat'); eq(s.tier, 'HIGH', 'tier');
  const old = P.parseState('CYCLE_WAIT,up=100,tx=1,mac=1,comm=DONE,st=PASS');
  eq(old.bat, undefined, 'older firmware: no bat field, still parses'); eq(old.state, 'CYCLE_WAIT', 'state');
  const skip = P.parseStLine('+ST=TX,SKIP,batt_crit');
  assert(P.isTxDeferred(skip), 'SKIP batt_crit deferred');
  const items = { GPS: P.parseStLine('+ST=GPS,PASS,x'), ENV: P.parseStLine('+ST=ENV,PASS,x'), ADC: P.parseStLine('+ST=ADC,PASS,x'), TX: skip };
  const ev = P.evaluateSelftest(items, ['GPS', 'ENV', 'ADC', 'TX']);
  eq(ev.verdict, 'PENDING_TX', 'not a failure'); assert(/battery critical/.test(ev.reasons[0]), ev.reasons[0]);
  eq(P.evaluateSelftest({ ADC: P.parseStLine('+ST=ADC,SKIP,x') }, ['ADC']).verdict, 'INCOMPLETE', 'other SKIPs stay INCOMPLETE');
  const r = await rig({ provisioned: true, pendingChance: 0, batMv: 2050 });
  try {
    const boot = await P.pollBootSelftest(r.link, {});
    eq(boot.items.TX.verdict, 'SKIP', 'boot TX skipped'); eq(boot.items.TX.detail, 'batt_crit', 'batt_crit');
    eq(boot.status, 'FAIL', 'firmware summary FAIL (WS85 silent), not because of TX');
    assert(!boot.failed.includes('TX'), 'TX not in the failed list');
    const st = await P.getState(r.link);
    eq(st.tier, 'CRIT', 'tier CRIT'); assert(Math.abs(Number(st.bat) - 2050) <= 10, `bat ~2050 (${st.bat})`);
    const rep = P.buildReport({ unit: { fw: 'x', state: st }, selftest: boot, required: ['GPS', 'ENV', 'ADC', 'TX'] });
    eq(rep.unit.batTier, 'CRIT', 'report tier'); eq(rep.selftest.verdict, 'PENDING_TX', 'report self-test verdict'); eq(rep.verdict, 'INCOMPLETE', 'unit verdict INCOMPLETE, not FAIL');
    const row = P.reportToCsvRow(rep);
    eq(row[P.CSV_COLUMNS.indexOf('bat_tier')], 'CRIT', 'CSV bat_tier'); eq(row[P.CSV_COLUMNS.indexOf('st_tx')], 'SKIP batt_crit', 'CSV st_tx');
  } finally { r.done(); }
  const n = await rig({ provisioned: true, pendingChance: 0, batMv: 2650 });
  try {
    eq((await P.getState(n.link)).bat, '0', 'bat=0 before the first reading');
    await P.pollBootSelftest(n.link, {});
    const st = await P.getState(n.link);
    eq(st.tier, 'HIGH', 'normal pack'); assert(Number(st.bat) > 2600, 'bat measured');
  } finally { n.done(); }
});

await test('Device clock: utc field (0 = no GPS time yet), offset vs PC, survives resets, cleared by power-up', async () => {
  const s = P.parseState('CYCLE_WAIT,up=1,tx=0,mac=1,comm=DONE,st=PASS,slp=0,prov=1,bat=2650,tier=HIGH,utc=1791700000');
  eq(s.utc, '1791700000', 'utc parsed');
  const c = P.deviceClock(s, 1791699998500);
  assert(c.known, 'known'); eq(c.offsetS, 2, 'device 2 s ahead (rounded)');
  eq(P.deviceClock(P.parseState('X,utc=0'), Date.now()).known, false, 'utc=0 -> no GPS time yet');
  eq(P.deviceClock(P.parseState('X,up=1'), Date.now()), null, 'older firmware without utc');
  const rep = P.buildReport({ unit: { state: s, stateAt: 1791699998500 } });
  eq(rep.unit.deviceUtc, new Date(1791700000 * 1000).toISOString(), 'report deviceUtc'); eq(rep.unit.clockOffsetS, 2, 'report offset');
  eq(P.reportToCsvRow(rep)[P.CSV_COLUMNS.indexOf('clock_offset_s')], '2', 'CSV clock_offset_s');
  const r = await rig({ provisioned: true, pendingChance: 0, seed: 1235 });   // seed % 3 = 2 s skew
  try {
    eq((await P.getState(r.link)).utc, '0', 'no GPS time right after power-up');
    await P.pollBootSelftest(r.link, {});
    const t1 = Date.now(), st = await P.getState(r.link);
    const off = P.deviceClock(st, t1).offsetS;
    assert(Math.abs(off - 2) <= 1, `device clock ~2 s ahead after the GPS self-test (${off})`);
    await P.resetApp(r.link);
    assert(Number((await P.getState(r.link)).utc) > 0, 'GPS time survives a reset');
    r.sim.powerCycle();
    await P.sleep(4000);
    eq((await P.getState(r.link)).utc, '0', 'cleared by a power-cycle');
  } finally { r.done(); }
});

/* ---------- credentials vault ---------- */

/** Minimal .xlsx writer for the tests: sharedStrings + sheet1, deflated or stored. */
function makeXlsx(rows, { deflate = true, numericCols = [] } = {}) {
  const shared = [], idx = new Map();
  const si = s => { if (!idx.has(s)) { idx.set(s, shared.length); shared.push(s); } return idx.get(s); };
  const col = i => String.fromCharCode(65 + i);
  const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  const sheet = '<?xml version="1.0" encoding="UTF-8"?><worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>' +
    rows.map((r, ri) => `<row r="${ri + 1}">` + r.map((v, ci) => {
      const ref = `${col(ci)}${ri + 1}`;
      if (v === '') return '';
      return ri > 0 && numericCols.includes(ci) ? `<c r="${ref}"><v>${v}</v></c>` : `<c r="${ref}" t="s"><v>${si(String(v))}</v></c>`;
    }).join('') + '</row>').join('') + '</sheetData></worksheet>';
  const sst = `<?xml version="1.0" encoding="UTF-8"?><sst xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">${shared.map(s => `<si><t>${esc(s)}</t></si>`).join('')}</sst>`;
  const files = [['[Content_Types].xml', '<Types/>'], ['xl/workbook.xml', '<workbook/>'], ['xl/sharedStrings.xml', sst], ['xl/worksheets/sheet1.xml', sheet]];
  const local = [], central = [];
  let off = 0;
  for (const [name, text] of files) {
    const raw = Buffer.from(text, 'utf8'), data = deflate ? deflateRawSync(raw) : raw, nm = Buffer.from(name);
    const lh = Buffer.alloc(30);
    lh.writeUInt32LE(0x04034b50, 0); lh.writeUInt16LE(20, 4); lh.writeUInt16LE(deflate ? 8 : 0, 8);
    lh.writeUInt32LE(data.length, 18); lh.writeUInt32LE(raw.length, 22); lh.writeUInt16LE(nm.length, 26);
    const ch = Buffer.alloc(46);
    ch.writeUInt32LE(0x02014b50, 0); ch.writeUInt16LE(20, 4); ch.writeUInt16LE(20, 6); ch.writeUInt16LE(deflate ? 8 : 0, 10);
    ch.writeUInt32LE(data.length, 20); ch.writeUInt32LE(raw.length, 24); ch.writeUInt16LE(nm.length, 28); ch.writeUInt32LE(off, 42);
    local.push(lh, nm, data); central.push(ch, nm);
    off += 30 + nm.length + data.length;
  }
  const cd = Buffer.concat(central), eocd = Buffer.alloc(22);
  eocd.writeUInt32LE(0x06054b50, 0); eocd.writeUInt16LE(files.length, 8); eocd.writeUInt16LE(files.length, 10);
  eocd.writeUInt32LE(cd.length, 12); eocd.writeUInt32LE(off, 16);
  return new Uint8Array(Buffer.concat([...local, cd, eocd]));
}

function fakeSets(rnd, n, base = 294938) {
  return Array.from({ length: n }, (_, i) => ({ id: String(base + i), addr: randomHex(rnd, 8), key: randomHex(rnd, 32) }));
}

await test('Vault: CLS CSV dialects (header names, semicolons, quotes, BOM, no header) and invalid rows', async () => {
  const rnd = P.mulberry32(77);
  const s = fakeSets(rnd, 3);
  const enc = t => new TextEncoder().encode(t);
  let r = await P.readCredentialFile('cls.csv', enc(`ID,ADDR,KEY\r\n${s.map(x => `${x.id},${x.addr},${x.key}`).join('\r\n')}\r\n`));
  eq(r.records.length, 3, 'comma CSV with header'); eq(r.invalid.length, 0, 'no invalid');
  eq(r.records[1].addr, s[1].addr.toUpperCase(), 'ADDR upper-cased');
  r = await P.readCredentialFile('cls.csv', enc(`﻿Identifiant;Adresse;Clé;Notes\n${s.map(x => `${x.id};"${x.addr.toLowerCase()}";${x.key};"lot A; box 2"`).join('\n')}`));
  eq(r.records.length, 3, 'semicolon, French headers, BOM, quoted values');
  eq(r.records[0].note, 'lot A; box 2', 'quoted delimiter kept inside a field');
  r = await P.readCredentialFile('cls.txt', enc(s.map(x => `${x.id}\t${x.addr}\t${x.key}`).join('\n')));
  eq(r.records.length, 3, 'tab separated without a header (positional)');
  assert(r.notes.some(n => /No header/.test(n)), 'positional mode noted');
  r = await P.readCredentialFile('bad.csv', enc(`ID,ADDR,KEY\n12x,${s[0].addr},${s[0].key}\n${s[1].id},123,${s[1].key}\n${s[2].id},${s[2].addr},ABC\n`));
  eq(r.records.length, 0, 'all three rows rejected');
  eq(r.invalid.map(x => Object.keys(x.errors)[0]).join(','), 'id,addr,key', 'field-level errors');
});

await test('Vault: .xlsx import (deflated and stored entries, ADDR stored as a number)', async () => {
  const rnd = P.mulberry32(78);
  const s = fakeSets(rnd, 4);
  s[2].addr = '00123456';                                  // all-digit ADDR: a spreadsheet keeps it as 123456
  const rows = [['ID', 'ADDR', 'KEY'], ...s.map(x => [x.id, x.addr === '00123456' ? '123456' : x.addr, x.key])];
  for (const deflate of [true, false]) {
    const r = await P.readCredentialFile('8631_ids.xlsx', makeXlsx(rows, { deflate, numericCols: [0, 1] }));
    eq(r.records.length, 4, `4 sets read (${deflate ? 'deflate' : 'stored'})`);
    eq(r.records.map(x => x.id).join(), s.map(x => x.id).join(), 'IDs in order');
    eq(r.records[2].addr, '00123456', 'numeric ADDR padded back to 8 hex');
    assert(r.notes.some(n => /padded/.test(n)), 'padding reported');
    eq(r.records[3].key, s[3].key.toUpperCase(), 'key intact');
  }
  await rejects(P.readCredentialFile('x.xlsx', new Uint8Array([0x50, 0x4b, 3, 4, 0, 0])), e => /zip|xlsx/i.test(e.message), 'damaged xlsx refused');
});

await test('Vault: merge (new / identical / conflicting), tracking, next free, CSV with and without keys', async () => {
  const rnd = P.mulberry32(79);
  const s = fakeSets(rnd, 5).map(x => P.vaultNormalize(x).rec);
  const data = P.vaultNewData();
  let m = P.vaultMerge(data, s.slice(0, 3), { commit: false });
  eq(m.added.length, 3, 'preview counts'); eq(data.records.length, 0, 'preview does not change the vault');
  m = P.vaultMerge(data, s.slice(0, 3), { commit: true, source: 'a.csv', by: 'tom' });
  eq(data.records.length, 3, 'committed'); eq(data.records[0].history[0].ev, 'imported', 'import logged');
  const clash = { ...s[0], key: randomHex(rnd, 32).toUpperCase() };
  const stolenAddr = { ...s[3], addr: s[1].addr };
  m = P.vaultMerge(data, [s[1], clash, stolenAddr, s[4]], { commit: true });
  eq(m.identical.join(), s[1].id, 'identical set skipped');
  eq(m.conflicts.length, 2, 'different key and reused ADDR both refused');
  eq(m.added.join(), s[4].id, 'only the genuinely new set added');
  eq(P.vaultNextFree(data).id, s[0].id, 'next free = first imported');
  const used = P.vaultMarkInUse(data, s[0].id, { sn: 'SMD7130029002F', fw: '5a54854', by: 'tom' });
  eq(used.status, 'in use', 'marked in use'); eq(used.sn, 'SMD7130029002F', 'unit SN recorded');
  eq(P.vaultNextFree(data).id, s[1].id, 'next free moves on');
  eq(P.vaultMarkInUse(data, '999', {}), null, 'unknown ID not tracked');
  const freed = P.vaultMarkFreed(data, s[0].id, { sn: 'SMD7130029002F' });
  eq(freed.status, 'free', 'freed'); eq(freed.sn, '', 'SN cleared');
  assert(freed.history.some(h => h.ev === 'erased' && /SMD7130029002F/.test(h.note)), 'erase keeps the SN in the history');
  data.records.push({ ...P.vaultNormalize({ id: '', addr: '', key: '' }).rec, uid: 'rx', history: [] });
  eq(P.vaultCounts(data).free, 5, 'counts by status');
  P.vaultMarkInUse(data, s[0].id, { sn: 'A' }); P.vaultMarkInUse(data, s[1].id, { sn: 'B' }); P.vaultMarkInUse(data, s[2].id, { sn: 'C' }); P.vaultMarkInUse(data, s[4].id, { sn: 'D' });
  eq(P.vaultNextFree(data), null, 'an incomplete row is never offered as the next free set');
  const plain = P.vaultCsv(data), withKeys = P.vaultCsv(data, { keys: true });
  assert(!s.some(x => plain.includes(x.key)), 'default CSV carries no key');
  assert(s.slice(0, 3).every(x => withKeys.includes(x.key)), 'keys only in the explicit export');
});

await test('Vault: AES-GCM encryption (round trip, wrong passphrase, tampering, fresh IV, re-key)', async () => {
  const rnd = P.mulberry32(80);
  const data = P.vaultNewData();
  P.vaultMerge(data, fakeSets(rnd, 6).map(x => P.vaultNormalize(x).rec), { commit: true });
  const c = await P.vaultCreate(data, 'correct horse battery staple');
  const text = JSON.stringify(c.blob);
  assert(!data.records.some(r => text.includes(r.key) || text.includes(r.addr) || text.includes(r.id)), 'no ID, ADDR or key in the stored blob');
  eq(c.blob.kdf.iter, 600000, 'PBKDF2 iterations'); eq(c.blob.cipher, 'AES-256-GCM', 'cipher');
  const o = await P.vaultOpen(JSON.parse(text), 'correct horse battery staple');
  eq(JSON.stringify(o.data), JSON.stringify(data), 'round trip');
  await rejects(P.vaultOpen(JSON.parse(text), 'correct horse battery stapler'), e => /passphrase/.test(e.message), 'wrong passphrase refused');
  const t = JSON.parse(text); const ct = P.b64dec(t.ct); ct[5] ^= 1; t.ct = P.b64enc(ct);
  await rejects(P.vaultOpen(t, 'correct horse battery staple'), e => /altered|passphrase/.test(e.message), 'tampered ciphertext refused');
  const b2 = await P.vaultSeal(data, o.key, o.salt, o.iter);
  assert(b2.iv !== c.blob.iv && b2.ct !== c.blob.ct, 'every save uses a fresh IV');
  const re = await P.vaultCreate(o.data, 'a different long passphrase');
  assert(re.blob.kdf.salt !== c.blob.kdf.salt, 'new passphrase -> new salt');
  await rejects(P.vaultOpen(re.blob, 'correct horse battery staple'), () => true, 'old passphrase no longer opens it');
  await rejects(P.vaultOpen({ format: 'nope' }, 'x'), e => /Not a Naiad vault/.test(e.message), 'foreign JSON refused');
});

await test('Vault: merging another vault keeps status and history', async () => {
  const rnd = P.mulberry32(81);
  const a = P.vaultNewData(), b = P.vaultNewData();
  const s = fakeSets(rnd, 3).map(x => P.vaultNormalize(x).rec);
  P.vaultMerge(a, s.slice(0, 2), { commit: true, source: 'station-a' });
  P.vaultMarkInUse(a, s[0].id, { sn: 'SN-A' });
  P.vaultMerge(b, [s[1], s[2]], { commit: true, source: 'station-b' });
  const m = P.vaultMerge(b, a.records, { commit: true, source: 'naiad-vault_a.json' });
  eq(m.added.join(), s[0].id, 'only the set station B lacked'); eq(m.identical.join(), s[1].id, 'shared set skipped');
  const got = P.vaultFind(b, s[0].id);
  eq(got.status, 'in use', 'status kept'); eq(got.sn, 'SN-A', 'SN kept');
  assert(got.history.some(h => h.ev === 'provisioned') && got.history.some(h => h.ev === 'merged'), 'history kept and the merge logged');
});

await test('Team vault: import preview by key CRC (new / identical / conflicts, duplicates within the file)', async () => {
  const k1 = '00112233445566778899AABBCCDDEEFF', k2 = 'FFEEDDCCBBAA99887766554433221100';
  const store = [{ id: '100001', addr: '0A1B2C3D', key_crc: P.seckeyCrc(k1), status: 'in use' }];
  const m = P.teamImportPreview(store, [
    { id: '100001', addr: '0A1B2C3D', key: k1 },                 // same set
    { id: '100001', addr: '0A1B2C3D', key: k2 },                 // same ID, other key
    { id: '100002', addr: '0A1B2C3D', key: k2 },                 // ADDR taken
    { id: '100003', addr: '0A1B2C3E', key: k2 },                 // new
    { id: '100004', addr: '0A1B2C3E', key: k1 },                 // ADDR of the new one above
    { id: '100003', addr: '0A1B2C3E', key: k2 },                 // repeated row
  ]);
  eq(m.identical.join(), '100001,100003', 'identical');
  eq(m.added.join(), '100003', 'added'); eq(m.send.length, 1, 'sent once'); eq(m.send[0].key, k2, 'the key goes with it');
  eq(m.conflicts.map(c => `${c.id}: ${c.why}`).join(' | '), '100001: KEY differs from the team vault | 100002: ADDR 0A1B2C3D already belongs to ID 100001 | 100004: ADDR 0A1B2C3E already belongs to ID 100003', 'conflicts');
});

await test('Team vault: passphrase crypto (create, unlock, wrong passphrase, keys bound to their ID, server formats)', async () => {
  const n = await P.teamVaultNew('correct horse battery staple', 1000);
  // the formats the Worker accepts (web/naiad-tool/src/worker.js validKdf / isSealed)
  assert(/^[A-Za-z0-9+/]{22}==$/.test(n.kdf.salt) && n.kdf.alg === 'PBKDF2-SHA256' && n.kdf.iter === 1000, 'kdf format');
  const sealedRe = /^v1:[A-Za-z0-9+/]{16}:[A-Za-z0-9+/]{64}$/;
  assert(sealedRe.test(n.kdf.check), 'check format');
  const o = await P.teamVaultUnlock(n.kdf, 'correct horse battery staple');
  await rejects(P.teamVaultUnlock(n.kdf, 'correct horse battery stapler'), e => /Wrong team passphrase/.test(e.message), 'wrong passphrase');
  const key = '00112233445566778899AABBCCDDEEFF';
  const [rec] = await P.teamSealRecords(n.key, [{ id: '100001', addr: '0A1B2C3D', key, status: 'in use', sn: 'SMD1' }]);
  assert(sealedRe.test(rec.key_enc) && !('key' in rec), 'sealed, no clear key');
  eq(rec.key_crc, P.seckeyCrc(key), 'CRC'); eq(rec.status, 'in use', 'tracking fields kept');
  eq(await P.teamOpenKey(o.key, '100001', rec.key_enc), key, 'opens with the unlocked key');
  await rejects(P.teamOpenKey(o.key, '100002', rec.key_enc), e => /does not open/.test(e.message), 'bound to its ID');
  const other = await P.teamVaultNew('another passphrase', 1000);
  await rejects(P.teamOpenKey(other.key, '100001', rec.key_enc), e => /does not open/.test(e.message), 'other passphrase');
  assert(rec.key_enc !== (await P.teamSealKey(n.key, '100001', key)), 'fresh IV each time');
  // an offline copy sealed with the team key opens as an offline vault with the same passphrase
  const data = P.vaultNewData(); P.vaultMerge(data, [{ id: '100001', addr: '0A1B2C3D', key, rconf: '', status: 'free' }], { commit: true });
  const blob = await P.vaultSeal(data, o.key, o.salt, o.iter);
  eq((await P.vaultOpen(blob, 'correct horse battery staple')).data.records[0].key, key, 'offline copy opens with the team passphrase');
});

/* ---------- SWD flashing (CMSIS-DAP, simulated probe + STM32WL55) ---------- */
function swdImage({ appSize = 120 * 1024, blSize = 20 * 1024, seed = 7 } = {}) {
  const rnd = P.mulberry32(seed);
  const img = new Uint8Array(0x33000 + blSize).fill(0xFF);
  for (let i = 8; i < appSize; i++) img[i] = Math.floor(rnd() * 256);
  P.putU32le(img, 0, 0x20008000); P.putU32le(img, 4, 0x080001C1);
  for (let i = 0x33008; i < 0x33000 + blSize; i++) img[i] = Math.floor(rnd() * 256);
  P.putU32le(img, 0x33000, 0x20008000); P.putU32le(img, 0x33004, 0x08033201);
  return img;
}
function swdUnitFlash(seed = 3) {
  // a module that already ran firmware: old app, the credential mirror page, FLASH_USER credentials
  const f = new Uint8Array(P.WL.FLASH_SIZE).fill(0xFF), rnd = P.mulberry32(seed);
  for (let i = 0; i < 0x20000; i++) f[i] = Math.floor(rnd() * 256);
  for (let i = 0x32000; i < 0x33000; i++) f[i] = Math.floor(rnd() * 256);   // power log + credential mirror (pages 100-101)
  for (let i = 0x3B000; i < 0x40000; i++) f[i] = Math.floor(rnd() * 256);   // FLASH_USER (pages 118-127)
  return f;
}
const PROTECTED = [[0x32000, 0x33000], [0x3B000, 0x40000]];
const sameRange = (a, b, s, e) => a.subarray(s, e).every((x, i) => x === b[s + i]);

await test('SWD: image plan erases app + bootloader pages only, refuses images over the credential pages', async () => {
  const plan = P.analyzeSwdImage(swdImage());
  assert(plan.ok, plan.errors.join(' '));
  assert(!plan.erase.some(p => p === 100 || p === 101 || p >= 118), 'never the mirror or FLASH_USER pages');
  eq(plan.erase.filter(p => p <= 99).length, 100, 'the whole app region (pages 0-99), like make flash-full');
  eq(plan.erase.filter(p => p >= 102).length, 16, 'the bootloader region (pages 102-117)');
  assert(plan.hasBootloader, 'bootloader detected');
  const bad = swdImage(); bad[0x32900] = 0x00;
  assert(/page 101/.test(P.analyzeSwdImage(bad).errors.join()), 'data in the mirror page refused');
  const big = new Uint8Array(0x3B010).fill(0xFF); big.set(swdImage().subarray(0, 0x3B000)); big[0x3B004] = 1;
  assert(/page 118/.test(P.analyzeSwdImage(big).errors.join()), 'data in FLASH_USER refused');
  const noVec = swdImage(); P.putU32le(noVec, 0, 0xFFFFFFFF);
  assert(!P.analyzeSwdImage(noVec).ok, 'no vector table refused');
});

await test('SWD: flash a unit (connect under reset): image written, credentials + mirror kept, vector table last, verified, running', async () => {
  const img = swdImage(), before = swdUnitFlash();
  const sim = new P.SimDapProbe({ flash: before.slice() });
  sim.acr |= P.WL.ACR_EMPTY;                                   // as after a boot with an erased page 0
  const dap = new P.CmsisDap(sim);
  const info = await dap.init();
  eq(info.product, 'Simulated CMSIS-DAP', 'probe info');
  const phases = [];
  let frozen = null;
  const res = await P.swdFlashStm32wl(dap, img, { onPhase: p => { if (phases[phases.length - 1] !== p) phases.push(p); if (p === 'write') frozen = sim.fz; } });
  eq(frozen & P.WL.FZ_IWDG_WWDG, P.WL.FZ_IWDG_WWDG, 'watchdogs frozen while halted');
  assert(sim.fz === 0 && sim.dbgmcuCr === 0, 'debug freeze / low-power debug released afterwards');
  eq(sim.acr & P.WL.ACR_EMPTY, 0, 'empty-flash latch cleared, so the next boot runs the app');
  assert(res.ok && res.verified && res.underReset, JSON.stringify(res));
  eq(phases.join(','), 'connect,unlock,erase,write,vectors,verify,run', 'phases');
  assert(sameRange(sim.flash, img, 0, 120 * 1024), 'app written');
  assert(sameRange(sim.flash, img, 0x33000, 0x33000 + 20 * 1024), 'bootloader written');
  for (const [s, e] of PROTECTED) assert(sameRange(sim.flash, before, s, e), `0x${(0x08000000 + s).toString(16)} untouched`);
  assert(!sim.log.erased.some(p => p === 100 || p === 101 || p >= 118), 'protected pages never erased');
  eq(sim.log.firstVectorWriteIndex, sim.log.programmed - 1, 'first double word programmed last');
  assert(sim.locked && sim.running && !sim.halted, 'flash relocked, module running');
  return `${res.written} B written in ${sim.log.programmed} double words, ${res.pages} pages erased`;
});

await test('SWD: a unit asleep in STOP2 needs nRESET; RDP, wrong chip and cancel are refused safely', async () => {
  const img = swdImage({ appSize: 16 * 1024, blSize: 4096 });
  // asleep, nRESET wired: connect under reset wakes it
  let sim = new P.SimDapProbe({ stop2: true });
  let dap = new P.CmsisDap(sim); await dap.init();
  assert((await P.swdFlashStm32wl(dap, img, { verify: false })).ok, 'flashed through connect-under-reset');
  // asleep, nRESET not wired: a clear error, nothing erased
  sim = new P.SimDapProbe({ stop2: true, nresetWired: false });
  dap = new P.CmsisDap(sim); await dap.init();
  await rejects(P.swdFlashStm32wl(dap, img), e => /does not answer/.test(e.message), 'asleep without nRESET');
  eq(sim.log.erased.length, 0, 'nothing erased');
  // awake, nRESET not wired: plain attach + vector-catch reset
  sim = new P.SimDapProbe({ nresetWired: false });
  dap = new P.CmsisDap(sim); await dap.init();
  const r = await P.swdFlashStm32wl(dap, img);
  assert(r.ok && !r.underReset, 'plain attach path');
  // read protection on: stop before erasing anything
  sim = new P.SimDapProbe({ rdp: 0xBB, flash: swdUnitFlash() });
  dap = new P.CmsisDap(sim); await dap.init();
  await rejects(P.swdFlashStm32wl(dap, img), e => /read protection is on/.test(e.message), 'RDP 1 refused');
  eq(sim.log.erased.length, 0, 'nothing erased under RDP');
  // write protection over the app, or a CPU2 secure area: refused before any erase
  for (const opt of [{ wrp1a: (5 << 16) | 2 }, { ese: true, sfr: 0xFFFFEF60 }]) {
    sim = new P.SimDapProbe(opt);
    dap = new P.CmsisDap(sim); await dap.init();
    await rejects(P.swdFlashStm32wl(dap, img), e => /Stopped before writing/.test(e.message), JSON.stringify(opt));
    eq(sim.log.erased.length, 0, 'nothing erased');
  }
  // another chip
  sim = new P.SimDapProbe({ devId: 0x495 });
  dap = new P.CmsisDap(sim); await dap.init();
  await rejects(P.swdFlashStm32wl(dap, img), e => /Not an STM32WL55/.test(e.message), 'wrong device');
  // cancel during the write: flash relocked, vector table not written (no half image boots)
  sim = new P.SimDapProbe({ packet: 512 });
  dap = new P.CmsisDap(sim); await dap.init();
  const ac = new AbortController();
  await rejects(P.swdFlashStm32wl(dap, swdImage(), { signal: ac.signal, onProgress: p => { if (p.phase === 'write' && p.done > 2000) ac.abort(); } }), e => e instanceof P.CancelledError, 'cancelled');
  assert(sim.locked, 'flash relocked after cancel');
  eq(P.u32le(sim.flash, 0), 0xFFFFFFFF, 'vector table still blank: the ROM bootloader starts next boot');
});

/* ---------- SWD through an ST-Link (simulated ST-Link V2 on the same simulated STM32WL55) ---------- */
await test('ST-Link: flash a unit under reset (stick starts in DFU mode): image written, credentials + mirror kept, 480 kHz while programming', async () => {
  const img = swdImage(), before = swdUnitFlash();
  const io = new P.SimStLinkIo({ flash: before.slice() });
  const stl = new P.StLinkProbe(io);
  const info = await stl.init();
  eq(`${info.product} ${info.version} ${info.targetV} V`, 'ST-Link V2 V2J37S7 3.3 V', 'probe info');
  const res = await P.swdFlashStm32wl(stl, img);
  assert(res.ok && res.verified && res.underReset, JSON.stringify(res));
  assert(sameRange(io.flash, img, 0, 120 * 1024) && sameRange(io.flash, img, 0x33000, 0x33000 + 20 * 1024), 'app and bootloader written');
  for (const [s, e] of PROTECTED) assert(sameRange(io.flash, before, s, e), `0x${(0x08000000 + s).toString(16)} untouched`);
  assert(!io.wl.log.erased.some(p => p === 100 || p === 101 || p >= 118), 'protected pages never erased');
  eq(io.wl.log.firstVectorWriteIndex, io.wl.log.programmed - 1, 'vector table programmed last');
  assert(io.wl.locked && io.wl.running && !io.wl.halted, 'flash relocked, module running');
  eq(io.log.freqs.join(), '1,7,1', 'SWD clock 1.8 MHz, 480 kHz while programming, 1.8 MHz again');
  return `${res.written} B in ${io.log.commands} ST-Link commands`;
});

await test('ST-Link: a sleeping unit needs the RST pin, a dead clone RST falls back to a plain attach, RDP is refused', async () => {
  const img = swdImage({ appSize: 16 * 1024, blSize: 4096 });
  const run = async (opts, fopts = {}) => { const io = new P.SimStLinkIo(opts); const stl = new P.StLinkProbe(io); await stl.init(); return { io, r: P.swdFlashStm32wl(stl, img, fopts) }; };
  let { io, r } = await run({ stop2: true });
  const a = await r;
  assert(a.ok && a.underReset, 'asleep: connect under reset through RST');
  ({ io, r } = await run({ stop2: true, nresetWired: false }));
  await rejects(r, e => /no target found/.test(e.message), 'asleep and RST not driving NRST');
  eq(io.wl.log.erased.length, 0, 'nothing erased');
  ({ io, r } = await run({ nresetWired: false }));
  io.wl.dfsr = P.WL.DFSR_VCATCH;                    // stale flag from an earlier session must not fake a reset
  const b = await r;
  assert(b.ok && !b.underReset, 'awake, RST not wired: plain attach + system reset');
  ({ io, r } = await run({ rdp: 0xBB }));
  await rejects(r, e => /read protection is on/.test(e.message), 'RDP 1');
  eq(io.wl.log.erased.length, 0, 'nothing erased under RDP');
});

await test('Releases: the image offered first is the newest field build without a warning', async () => {
  const a = v => ({ name: `naiad-carrier-${v}__argos-smd-at-kineis-firmware.bin`, variant: v });
  const list = [
    { tag: 'v1.2.0', prerelease: true, warn: '', assets: [a('release')] },
    { tag: 'v1.1.0', warn: '', date: '2026-11-02T10:00:00Z', assets: [a('bench'), a('release')] },
    { tag: 'v1.0.0', warn: 'Do not deploy', assets: [a('release')] },
  ];
  const d = P.defaultRelease(list);
  eq(`${d.release.tag} ${d.asset.variant}`, 'v1.1.0 release', 'default');
  eq(P.defaultRelease([list[2]]), null, 'nothing safe to offer');
  eq(P.releaseLabel(list[1], list[1].assets[1]), 'v1.1.0 · field build · 2026-11-02', 'label');
  assert(/bench build.*DO NOT DEPLOY/.test(P.releaseLabel({ ...list[2], date: '' }, a('bench'))), 'warning in the label');
});

const realBin = path.join(here, '..', 'build', 'argos-smd-at-kineis-firmware.bin');
if (existsSync(realBin)) {
  await test('Flash the real build/argos-smd-at-kineis-firmware.bin into the simulator', async () => {
    const img = new Uint8Array(readFileSync(realBin));
    const a = P.analyzeImage(img, path.basename(realBin));
    assert(a.ok, 'real image passes the vector check: ' + a.errors.join(' '));
    const r = await rig({ appBaud: 115200, blBaud: 115200, pendingChance: 0 });
    try {
      const res = await P.flashFirmware(r.link, img, { name: path.basename(realBin) });
      assert(res.ok, 'flash ok');
      assert(sameBytes(r.sim.flash.subarray(0, img.length), img), 'simulated flash == real image');
      return `${img.length} B, SP 0x${P.hex8(a.sp)}, reset 0x${P.hex8(a.pc)}, VERIFY CRC 0x${res.verifyCrc}`;
    } finally { r.done(); }
  });
} else {
  console.log('SKIP  real build image (build/argos-smd-at-kineis-firmware.bin not found)');
}

clearTimeout(watchdog);
const failed = results.filter(r => !r.ok);
console.log(`\n${results.length - failed.length}/${results.length} tests passed (timing scale ${P.Clock.scale}).`);
process.exit(failed.length ? 1 : 0);
