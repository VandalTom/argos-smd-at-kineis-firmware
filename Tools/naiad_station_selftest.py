#!/usr/bin/env python3
"""
Offline test for naiad_station.py: drives it against a simulated Naiad unit
(console + DFU bootloader) through a fake serial port. No hardware needed.

    python Tools/naiad_station_selftest.py

The simulator reproduces the behaviours the station has to cope with: the
byte that wakes a sleeping (STOP2) unit is lost, credential writes are
refused once with +ERROR=BUSY, the bootloader CRCs WRITE data in arrival
order and jumps to the app after VERIFY, and AT+RESET reboots the unit.
"""

import json
import os
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import uart_dfu_check as dfu  # noqa: E402
import naiad_station as ns    # noqa: E402

KEY = "00112233445566778899AABBCCDDEEFF"


class Unit:
    def __init__(self, mode="app", busy_once=True):
        self.mode, self.asleep, self.busy_once = mode, True, busy_once
        self.rx, self.out = b"", bytearray()
        self.fw = "naiad-old,v11.1.0,CC_TRACKER"
        self.id, self.addr, self.key = None, None, None
        self.flash_img, self.sent, self.boots = {}, bytearray(), 0
        self.erased = False

    def say(self, *lines):
        for ln in lines:
            self.out += ln.encode() + b"\r\n"

    def feed(self, data):
        for b in data:
            if self.asleep and self.mode == "app":
                self.asleep = False              # STOP2: the waking byte is lost
                continue
            if b == 0x0A:
                line = self.rx.decode().strip("\r")
                self.rx = b""
                if line:
                    (self.app if self.mode == "app" else self.bl)(line)
            else:
                self.rx += bytes([b])

    def state(self):
        prov = int(bool(self.id and self.addr and self.key))
        return (f"+STATE={'CYCLE_WAIT' if prov else 'NO_CREDS'},up=42,tx=3,mac={prov},comm=WAIT_FIX,"
                f"st=NONE,slp=30,prov={prov},bat=2410,tier=MID,utc=0")

    def app(self, line):
        q = {"AT+PING=?": None, "AT+FW=?": f"+FW={self.fw}", "AT+VERSION=?": "+VERSION=v1.0",
             "AT+SN=?": "+SN=SIM-0042", "AT+ID=?": f"+ID={self.id or 0}",
             "AT+ADDR=?": f"+ADDR={(self.addr or '00000000').lower()}", "AT+STATE=?": self.state(),
             "AT+COMMISSION=?": "+COMMISSION=WAIT_FIX,fastgps=1,gps_tries=0"}
        if line in q:
            if q[line]:
                self.say(q[line])
            self.say("+OK")
        elif line.startswith(("AT+ID=", "AT+ADDR=", "AT+SECKEY=", "AT+RCONF=", "AT+SAVE_RCONF=")) \
                and not line.endswith("?"):
            if self.busy_once:
                self.busy_once = False
                self.say("+ERROR=BUSY")
                return
            k, v = line[3:].split("=", 1)
            if k == "ID":
                self.id = v
            elif k == "ADDR":
                self.addr = v
            elif k == "SECKEY":
                self.key = v
            self.say("+OK")
        elif line == "AT+SECKEY=?":
            self.say(f"+SECKEYCRC={zlib.crc32(bytes.fromhex(self.key or '00' * 16)):08X}", "+OK")
        elif line == "AT+RESET=":
            self.say("+RESET=OK")
            self.boots += 1
            self.asleep = False
        elif line.startswith("AT+SELFTEST="):
            items = line.split("=", 1)[1]
            if items not in ("ALL", "GPS,TX"):
                self.say("+ERROR=1203")
                return
            self.say("+SELFTEST=RUNNING", "+ST=GPS,FAIL,no_nmea", "+ST=WS85,FAIL,timeout",
                     "+ST=ENV,PASS,t=24.3,p=1016.1,rh=53", "+ST=ADC,PASS,nimh=2410,scap=4900",
                     "+ST=TX,PASS,ms=841" if self.key else "+ST=TX,FAIL,unprovisioned",
                     "+SELFTEST=FAIL,GPS,WS85" + ("" if self.key else ",TX"))
            self.asleep = True                   # back to STOP2 between cycles
        elif line == "AT+COMMISSION=SKIP":
            self.say("+COMMISSION=DONE", "+OK")
        elif line == "AT+BOOT=":
            self.say("+BOOT=OK")
            self.mode = "bl"
        else:
            self.say("+ERROR=1203")

    def bl(self, line):
        if line == "AT+DFU=PING":
            self.say("+DFU=OK,PONG")
        elif line == "AT+DFU=INFO":
            self.say("+DFU=OK," + bytes([1, 2, 0]).hex())
        elif line == "AT+DFU=ERASE":
            self.erased, self.sent, self.flash_img = True, bytearray(), {}
            self.say("+DFU=OK")
        elif line.startswith("AT+DFU=WRITE,"):
            _, addr, data = line.split(",")
            assert self.erased
            raw = bytes.fromhex(data)
            assert len(raw) <= 244, "BL parser caps WRITE payload at 244 bytes"
            self.flash_img[int(addr, 16)] = raw
            self.sent += raw
            self.say("+DFU=OK")
        elif line.startswith("AT+DFU=VERIFY,"):
            ok = int(line.split(",")[1], 16) == dfu.crc32_mpeg2(bytes(self.sent))
            assert min(self.flash_img) != 0x08000000 or list(self.flash_img)[-1] == 0x08000000, \
                "vector table must be written last"
            if ok:
                self.say("+DFU=OK", "+DFU=OK,VALIDATED")
                self.mode, self.fw, self.asleep = "app", "naiad-new,v11.1.0,CC_TRACKER", False
            else:
                self.say("+DFU=ERR,CRC")
        else:
            self.say("+DFU=ERR,UNKNOWN")


UNIT = None


class FakeSerial:
    def __init__(self, *a, **k):
        self.timeout = k.get("timeout")

    def write(self, d):
        UNIT.feed(d)

    def read(self, n):
        r = bytes(UNIT.out[:n])
        del UNIT.out[:n]
        return r

    def reset_input_buffer(self):
        UNIT.out.clear()

    def close(self):
        pass


def run(argv, unit):
    global UNIT
    UNIT = unit
    with tempfile.TemporaryDirectory() as d:
        rep = os.path.join(d, "r.json")
        sys.argv = ["naiad_station.py", "--port", "SIM"] + argv + ["--report", rep]
        rc = ns.main()
        text = open(rep).read()
    return rc, json.loads(text), text


def main():
    dfu.serial.Serial = FakeSerial
    ns.time.sleep = lambda s: None
    fails = 0

    def check(name, cond):
        nonlocal fails
        print(f"  {'ok  ' if cond else 'FAIL'} {name}")
        fails += not cond

    print("case 1: flash + provision + self-test (GPS/WS85 ignored) + commission skip")
    img = bytearray(os.urandom(5000))
    img[0:8] = (0x20008000).to_bytes(4, "little") + (0x080087C5).to_bytes(4, "little")
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(img)
    try:
        u = Unit()
        rc, rep, text = run(["--flash", f.name, "--expect-fw", "naiad-new", "--id", "279665",
                             "--addr", "1A2B3C4D", "--seckey", KEY, "--ignore", "GPS,WS85",
                             "--commission", "skip"], u)
    finally:
        os.remove(f.name)
    check("exit 0 / PASS", rc == 0 and rep["result"] == "PASS")
    check("image landed byte-exact", b"".join(u.flash_img[a] for a in sorted(u.flash_img)) == bytes(img))
    check("credentials stored", (u.id, u.addr, u.key) == ("279665", "1A2B3C4D", KEY))
    check("provision checks all true", all(rep["provision"].values()))
    check("unit rebooted for the new credentials", u.boots == 1)
    check("GPS/WS85 FAIL are warnings", len(rep["warnings"]) == 2 and not rep["failures"])
    check("commission reported", rep["commission"].startswith("+COMMISSION="))
    check("secret key never in the report", KEY not in text and KEY.lower() not in text)

    print("case 2: self-test failures not ignored -> FAIL, exit 1")
    u = Unit(busy_once=False)
    u.id, u.addr, u.key = "1", "01020304", KEY
    rc, rep, _ = run([], u)
    check("exit 1 / FAIL", rc == 1 and rep["result"] == "FAIL")
    check("both sensor failures listed", sum("self-test" in x for x in rep["failures"]) == 2)

    print("case 3: unit stuck in the DFU bootloader, no --flash -> FAIL with advice")
    rc, rep, _ = run(["--selftest", "none"], Unit(mode="bl"))
    check("exit 1", rc == 1 and any("--flash" in x for x in rep["failures"]))

    print("case 4: --expect-fw mismatch, unprovisioned warning")
    rc, rep, _ = run(["--expect-fw", "v99", "--selftest", "none"], Unit())
    check("FW mismatch fails", rc == 1 and any("does not contain" in x for x in rep["failures"]))
    check("unprovisioned warned", any("NOT provisioned" in w for w in rep["warnings"]))

    print("case 5: interrupted update recovered (bootloader waiting, --flash given)")
    img2 = bytearray(os.urandom(700))
    img2[0:8] = (0x20008000).to_bytes(4, "little") + (0x08000101).to_bytes(4, "little")
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(img2)
    try:
        u = Unit(mode="bl")
        rc, rep, _ = run(["--flash", f.name, "--selftest", "none"], u)
    finally:
        os.remove(f.name)
    check("recovered and PASS", rc == 0 and u.fw.startswith("naiad-new"))

    print("ALL OK" if not fails else f"{fails} CHECK(S) FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
