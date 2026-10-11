#!/usr/bin/env python3
"""
Naiad (APP=CC_TRACKER) production-line station: one command per unit.

Runs, in order, whatever the arguments ask for and writes a JSON report:
  1. connect    find the app (115200, wake preamble) - or a DFU bootloader left
                waiting by an interrupted update
  2. flash      (--flash FILE) AT+BOOT -> DFU bootloader -> erase/write/verify
                (same code as uart_dfu_check.py) -> app restarts
  3. identify   AT+FW / VERSION / SN / ID / ADDR / STATE; --expect-fw checks
                the FW string contains the given text
  4. provision  (--id --addr --seckey [--rconf]) write, read back, check the
                key by CRC (+SECKEYCRC, the key is never echoed), AT+RESET
  5. self-test  (--selftest ITEMS, default ALL) AT+SELFTEST, collect +ST= lines
                and the +SELFTEST= summary; --ignore lists items whose FAIL is
                only a warning (e.g. GPS,WS85 on a jig without sensors)
  6. commission (--commission skip|arm) mark the deployment burst done (bench
                units) or re-arm it; the status is reported either way

Exit code 0 = PASS. The report (--report FILE, default naiad_<sn>_<time>.json)
records every reply, so a failed unit can be diagnosed later.

Usage:
    python naiad_station.py --port COM5
    python naiad_station.py --port COM5 --flash build/argos-smd-at-kineis-firmware.bin --expect-fw CC_TRACKER
    python naiad_station.py --port COM5 --id 279665 --addr 1A2B3C4D --seckey <32 hex> --ignore GPS,WS85
    python naiad_station.py --port /dev/ttyUSB0 --selftest none --commission skip

The unit must already carry firmware with the DFU bootloader (first flash:
Tools/rom_uart_flash.py with BOOT0 held, or SWD).
"""

import argparse
import json
import os
import platform
import sys
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import uart_dfu_check as dfu  # noqa: E402  (Link, flash, bl_ping, check_vectors)

BAUD = 115200
ITEMS = ("GPS", "WS85", "ENV", "ADC", "TX")
SELFTEST_TIMEOUT_S = 300     # MAC ready + GPS 5 s + WS85 35 s + TX 60 s, plenty of margin
BUSY_RETRY_S = 240           # a TX in flight can hold credential writes / AT+BOOT this long


class Station:
    def __init__(self, port, verbose):
        self.link = dfu.Link(port, verbose)
        self.log = []                          # every (command, reply) for the report

    # -- console helpers ---------------------------------------------------
    def wake(self):
        """Between cycles the unit sleeps in STOP2 and the byte that wakes it
        is lost; a bare line wakes it and keeps it awake 30 s."""
        self.link.ser.write(b"\r\n")
        self.link.drain(0.08)

    def cmd(self, line, prefixes, timeout=3.0, wake=True):
        if wake:
            self.wake()
        r = self.link.cmd(line, prefixes, timeout)
        self.log.append({"t": round(time.time(), 2), "tx": _mask(line), "rx": r})
        return r

    def wait(self, prefixes, timeout):
        r = self.link.wait(prefixes, timeout)
        self.log.append({"t": round(time.time(), 2), "tx": None, "rx": r})
        return r

    def query(self, name, timeout=2.0):
        """AT+<name>=? -> value after '+<name>=' (None if no answer)."""
        r = self.cmd(f"AT+{name}=?", (f"+{name}=", "+ERROR"), timeout)
        if r and r.startswith(f"+{name}="):
            return r[len(name) + 2:]
        return None

    def write_retry_busy(self, line):
        """Credential writes are refused (+ERROR=BUSY) while a frame is in
        flight; retry until the radio is idle."""
        end = time.time() + BUSY_RETRY_S
        while True:
            r = self.cmd(line, ("+OK", "+ERROR"), 3.0)
            if r != "+ERROR=BUSY" or time.time() > end:
                return r
            time.sleep(2.0)

    # -- stages --------------------------------------------------------------
    def connect(self, tries=4):
        """Return 'app', 'bl' or None."""
        self.link.open(BAUD)
        for _ in range(tries):
            self.wake()
            if self.cmd("AT+PING=?", ("+PING=", "+OK", "+ERROR"), 1.5, wake=False):
                return "app"
            r = self.cmd("AT+DFU=PING", ("+DFU=",), 1.5, wake=False)
            if r and r.startswith("+DFU=OK"):
                return "bl"
            time.sleep(0.5)
        return None

    def wait_app(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            self.wake()
            if self.cmd("AT+FW=?", ("+FW=",), 1.5, wake=False):
                return True
            time.sleep(0.5)
        return False

    def flash(self, path, mode):
        img = open(path, "rb").read()
        ok, sp, pc = dfu.check_vectors(img)
        if not ok:
            raise RuntimeError(f"{path}: vector table looks wrong (SP=0x{sp:08X} PC=0x{pc:08X})")
        if len(img) > dfu.APP_SIZE:
            raise RuntimeError(f"{path}: {len(img)} B exceeds the {dfu.APP_SIZE} B app region")
        if mode == "app":
            r = self.cmd("AT+BOOT=", ("+BOOT=", "+ERROR"), 3.0)
            if r == "+BOOT=PENDING":
                r = self.wait(("+BOOT=OK", "+ERROR"), BUSY_RETRY_S)
            if r != "+BOOT=OK":
                raise RuntimeError(f"AT+BOOT refused: {r}")
            self.link.drain(0.5)
        if not dfu.bl_ping(self.link):
            raise RuntimeError("DFU bootloader did not answer AT+DFU=PING")
        t0 = time.time()
        dfu.flash(self.link, img)
        self.link.open(BAUD)
        time.sleep(1.5)
        if not self.wait_app(30.0):
            raise RuntimeError("application did not come back after the update")
        return {"file": os.path.basename(path), "bytes": len(img),
                "crc32_mpeg2": f"{dfu.crc32_mpeg2(img):08X}", "seconds": round(time.time() - t0, 1)}

    def identify(self):
        out = {k.lower(): self.query(k) for k in ("FW", "VERSION", "SN", "ID", "ADDR")}
        out["state"] = parse_state(self.cmd("AT+STATE=?", ("+STATE=",), 3.0))
        return out

    def provision(self, dev_id, addr, seckey, rconf):
        steps = [f"AT+ID={dev_id}", f"AT+ADDR={addr}", f"AT+SECKEY={seckey}"]
        if rconf:
            steps += [f"AT+RCONF={rconf}", "AT+SAVE_RCONF="]
        for line in steps:
            r = self.write_retry_busy(line)
            if r != "+OK":
                raise RuntimeError(f"{_mask(line)} -> {r}")
        got_id, got_addr = self.query("ID"), self.query("ADDR")
        crc = self.cmd("AT+SECKEY=?", ("+SECKEYCRC=", "+ERROR"), 3.0)
        want_crc = f"{zlib.crc32(bytes.fromhex(seckey)) & 0xFFFFFFFF:08X}"
        res = {
            "id_ok": got_id is not None and got_id.strip() == str(int(dev_id)),
            "addr_ok": got_addr is not None and got_addr.strip().lower() == addr.lower(),
            "key_ok": crc == f"+SECKEYCRC={want_crc}",
        }
        # New credentials take effect at the next MAC init
        r = self.cmd("AT+RESET=", ("+RESET=", "+ERROR"), 3.0)
        if r == "+RESET=PENDING":
            self.wait(("+RESET=OK",), BUSY_RETRY_S)
        time.sleep(2.0)
        res["rebooted"] = self.wait_app(30.0)
        st = parse_state(self.cmd("AT+STATE=?", ("+STATE=",), 3.0))
        res["prov"] = st.get("prov") == "1"
        return res

    def selftest(self, items):
        r = self.cmd(f"AT+SELFTEST={items}", ("+SELFTEST=", "+ERROR"), 5.0)
        if r and r.startswith("+ERROR"):
            raise RuntimeError(f"AT+SELFTEST={items} refused: {r} (rescue boot, or unknown item)")
        # RUNNING arrives at the start (or immediately, if a boot run is already
        # going: its items are then the boot set); PENDING = waiting for the MAC
        results, end = {}, time.time() + SELFTEST_TIMEOUT_S
        while time.time() < end:
            line = self.wait(("+ST=", "+SELFTEST="), end - time.time())
            if line is None:
                break
            if line.startswith("+ST="):
                f = line[4:].split(",", 2)
                results[f[0]] = {"verdict": f[1] if len(f) > 1 else "?", "detail": f[2] if len(f) > 2 else ""}
            elif line.startswith(("+SELFTEST=PASS", "+SELFTEST=FAIL")):
                return {"summary": line[len("+SELFTEST="):], "items": results}
        return {"summary": "TIMEOUT", "items": results}

    def commission(self, action):
        if action in ("skip", "arm"):
            r = self.cmd(f"AT+COMMISSION={action.upper()}", ("+OK", "+ERROR", "+COMMISSION="), 3.0)
            if r and r.startswith("+ERROR"):
                raise RuntimeError(f"AT+COMMISSION={action.upper()} -> {r}")
        return self.cmd("AT+COMMISSION=?", ("+COMMISSION=",), 3.0)


def parse_state(line):
    """'+STATE=CYCLE_WAIT,up=12,tx=1,...' -> {'state': 'CYCLE_WAIT', 'up': '12', ...}"""
    if not line or not line.startswith("+STATE="):
        return {}
    parts = line[len("+STATE="):].split(",")
    out = {"state": parts[0]}
    for p in parts[1:]:
        k, _, v = p.partition("=")
        out[k] = v
    return out


def _mask(line):
    """Never write the device key into logs or reports."""
    return "AT+SECKEY=<hidden>" if line.upper().startswith("AT+SECKEY=") and not line.endswith("?") else line


def check_hex(name, value, nchars):
    if value is None:
        return
    try:
        bytes.fromhex(value)
    except ValueError:
        sys.exit(f"--{name}: not hex")
    if len(value) != nchars:
        sys.exit(f"--{name}: need {nchars} hex characters, got {len(value)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", "-p", required=True)
    ap.add_argument("--flash", metavar="FILE", help="app image (.bin, not the _full image)")
    ap.add_argument("--expect-fw", metavar="TEXT", help="FAIL unless AT+FW=? contains TEXT")
    ap.add_argument("--id", help="Kineis device ID (decimal, from CLS)")
    ap.add_argument("--addr", help="device address, 8 hex")
    ap.add_argument("--seckey", help="device secret key, 32 hex")
    ap.add_argument("--rconf", help="radio configuration, 32 hex (optional)")
    ap.add_argument("--selftest", default="ALL", help="items, e.g. GPS,TX; ALL (default) or none")
    ap.add_argument("--ignore", default="", help="items whose FAIL is only a warning, e.g. GPS,WS85")
    ap.add_argument("--commission", choices=("keep", "skip", "arm"), default="keep")
    ap.add_argument("--station", default=platform.node() or "station", help="name recorded in the report")
    ap.add_argument("--report", metavar="FILE")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    cred_args = [args.id, args.addr, args.seckey]
    if any(cred_args) and not all(cred_args):
        sys.exit("--id, --addr and --seckey go together")
    check_hex("addr", args.addr, 8)
    check_hex("seckey", args.seckey, 32)
    check_hex("rconf", args.rconf, 32)
    if args.id is not None and not args.id.isdigit():
        sys.exit("--id: decimal device ID")
    ignore = {x.strip().upper() for x in args.ignore.split(",") if x.strip()}
    if ignore - set(ITEMS):
        sys.exit(f"--ignore: unknown item(s) {sorted(ignore - set(ITEMS))}")

    st = Station(args.port, args.verbose)
    rep = {"station": args.station, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
           "port": args.port, "failures": [], "warnings": []}
    fail = rep["failures"].append

    def stage(name):
        print(f"[{name}]", flush=True)

    try:
        stage("connect")
        mode = st.connect()
        rep["found"] = mode
        if mode is None:
            fail("nothing answered at 115200 (wiring? powered? pre-bootloader factory firmware?)")
            raise StopIteration
        if mode == "bl":
            print("    a DFU bootloader is waiting (interrupted update?)")
            if not args.flash:
                fail("module is in the DFU bootloader - rerun with --flash")
                raise StopIteration

        if args.flash:
            stage("flash")
            rep["flash"] = st.flash(args.flash, mode)
            print(f"    {rep['flash']}")

        stage("identify")
        ident = st.identify()
        rep["identity"] = ident
        print(f"    FW {ident['fw']}  SN {ident['sn']}  ID {ident['id']}  ADDR {ident['addr']}")
        print(f"    state {ident['state']}")
        if ident["fw"] is None:
            fail("no answer to AT+FW=?")
        if args.expect_fw and (ident["fw"] is None or args.expect_fw not in ident["fw"]):
            fail(f"firmware '{ident['fw']}' does not contain '{args.expect_fw}'")
        if ident["state"].get("state") == "RESCUE":
            fail("unit is in a RESCUE boot (3 failed boots) - reflash it")

        if args.id:
            stage("provision")
            rep["provision"] = st.provision(args.id, args.addr, args.seckey, args.rconf)
            print(f"    {rep['provision']}")
            for k, v in rep["provision"].items():
                if not v:
                    fail(f"provisioning check failed: {k}")
        elif ident["state"].get("prov") == "0":
            rep["warnings"].append("unit is NOT provisioned (prov=0): it will never transmit")

        if args.selftest.lower() != "none":
            stage("self-test")
            stv = st.selftest(args.selftest.upper())
            rep["selftest"] = stv
            for item, r in stv["items"].items():
                print(f"    {item:5} {r['verdict']:5} {r['detail']}")
                if r["verdict"] == "FAIL":
                    (rep["warnings"].append if item in ignore else fail)(f"self-test {item} FAIL {r['detail']}")
            print(f"    summary {stv['summary']}")
            if stv["summary"] == "TIMEOUT":
                fail("self-test did not finish")

        stage("commission")
        rep["commission"] = st.commission(args.commission)
        print(f"    {rep['commission']}")
        rep["state_end"] = parse_state(st.cmd("AT+STATE=?", ("+STATE=",), 3.0))
    except StopIteration:
        pass
    except (RuntimeError, OSError) as e:
        fail(str(e))
    finally:
        st.link.close()

    rep["result"] = "FAIL" if rep["failures"] else "PASS"
    rep["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    rep["console"] = st.log
    sn = ((rep.get("identity") or {}).get("sn") or "unknown").strip().replace("/", "_")
    path = args.report or f"naiad_{sn}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(path, "w") as f:
        json.dump(rep, f, indent=2)
    for w in rep["warnings"]:
        print(f"WARN  {w}")
    for x in rep["failures"]:
        print(f"FAIL  {x}")
    print(f"{rep['result']}  (report: {path})")
    return 0 if rep["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
