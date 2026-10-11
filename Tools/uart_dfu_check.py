#!/usr/bin/env python3
"""
UART flash-path validation for argos-smd (STM32WL55) modules.

Talks to the module over LPUART1 (module P5=PA2 TX, P4=PA3 RX) through a
3.3 V USB-UART adapter, using the repo's DFU bootloader (Bootloader/, AT+DFU=).

Stages (each one only runs if the previous passed):
  1. probe     find the running app (or a bootloader already in DFU) at 9600/115200
  2. enter     AT+BOOT= -> bootloader, then AT+DFU=PING / INFO / STATUS
  3. backup    (--backup FILE) read the 200 KB app region back, twice, compare
  4. flash     (--flash FILE) ERASE -> WRITE (240 B chunks) -> VERIFY -> auto-jump
     or exit   AT+DFU=RESET -> bootloader sees no DFU flag -> boots the app
  5. confirm   the app answers AT+FW=? again

Without --flash nothing is written: the module goes in and out of the
bootloader and comes back running its original firmware.

Needs firmware that has the DFU bootloader + AT+BOOT (Feb 2026 onwards).
Modules still on the pre-bootloader factory build (e511146, AT v0.6) reply
+ERROR=1203 to AT+BOOT and must be flashed once, either over SWD (make flash-full)
or over this same UART through ST's ROM bootloader (Tools/rom_uart_flash.py,
BOOT0 test pad held high at power-up).

Usage:
    python uart_dfu_check.py --port COM3
    python uart_dfu_check.py --port COM3 --backup app_backup.bin
    python uart_dfu_check.py --port COM3 --flash build/argos-smd-at-kineis-firmware.bin

PL2303TA/clone adapters are blocked by the Windows 11 driver; pass them to
WSL with usbipd-win and run this from Linux (--port /dev/ttyUSB0).
"""

import argparse
import sys
import time

import serial
import serial.tools.list_ports

BAUDS = (9600, 115200)
APP_BASE = 0x08000000
APP_SIZE = 0x32000          # bl_config.h APP_FLASH_SIZE (200 KB)
WRITE_CHUNK = 240           # BL UART parser caps WRITE payload at BL_CHUNK_SIZE-4 = 244; keep 8-aligned
READ_CHUNK = 240            # hex reply must fit the BL's 512-byte TX buffer


def crc32_mpeg2(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else (crc << 1)
            crc &= 0xFFFFFFFF
    return crc


class Link:
    def __init__(self, port, verbose):
        self.port = port
        self.verbose = verbose
        self.ser = None
        self.pending = b""

    def open(self, baud):
        if self.ser:
            self.ser.close()
        self.ser = serial.Serial(self.port, baud, timeout=0.05)
        self.baud = baud

    def close(self):
        if self.ser:
            self.ser.close()

    def drain(self, secs=0.0):
        """Discard (but optionally show) whatever arrives for `secs`."""
        end = time.time() + secs
        buf = b""
        while True:
            buf += self.ser.read(4096)
            if time.time() >= end:
                break
        if self.verbose and buf.strip():
            for ln in buf.decode("latin-1").splitlines():
                if ln.strip():
                    print(f"      .. {ln}")
        self.ser.reset_input_buffer()
        self.pending = b""

    def cmd(self, line, prefixes, timeout=3.0):
        """Send one line, return the first reply line starting with a prefix."""
        self.ser.reset_input_buffer()
        self.pending = b""
        self.ser.write(line.encode("ascii") + b"\r\n")
        return self.wait(prefixes, timeout)

    def wait(self, prefixes, timeout):
        """Return the next line starting with a prefix (keeps bytes after it)."""
        end = time.time() + timeout
        while True:
            while b"\n" in self.pending:
                raw, self.pending = self.pending.split(b"\n", 1)
                txt = raw.decode("latin-1").rstrip("\r")
                if txt.startswith(prefixes):
                    return txt
                if self.verbose and txt.strip():
                    print(f"      .. {txt}")
            if time.time() >= end:
                return None
            self.pending += self.ser.read(4096)


def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No COM ports found - is the USB-UART adapter plugged in (and its driver installed)?")
    for p in ports:
        print(f"  {p.device:8} {p.description}  [{p.hwid}]")
    if len(ports) > 1:
        sys.exit("More than one COM port - pass --port COMx")
    return ports[0].device


def probe(link):
    """Return ('app'|'bl', baud) or (None, None)."""
    for baud in BAUDS:
        link.open(baud)
        link.ser.write(b"\r\n")         # wake a STOP-mode console, flush any partial line
        link.drain(0.3)
        for _ in range(2):
            r = link.cmd("AT+FW=?", ("+FW=", "+ERROR"), 1.5)
            if r:
                return "app", baud
            r = link.cmd("AT+DFU=PING", ("+DFU=",), 1.5)
            if r and r.startswith("+DFU=OK"):
                return "bl", baud
    return None, None


def bl_ping(link):
    for baud in [link.baud] + [b for b in BAUDS if b != link.baud]:
        if baud != link.baud:
            link.open(baud)
        for _ in range(3):
            r = link.cmd("AT+DFU=PING", ("+DFU=",), 1.5)
            if r and r.startswith("+DFU=OK"):
                return r
    return None


def payload(reply):
    return reply.split(",", 1)[1] if "," in reply else ""


def bl_read(link, addr, n):
    for _ in range(3):
        r = link.cmd(f"AT+DFU=READ,{addr:08X},{n}", ("+DFU=",), 3.0)
        if r and r.startswith("+DFU=OK,"):
            break
    else:
        raise RuntimeError(f"READ 0x{addr:08X} failed: {r}")
    data = payload(r)
    # BL replies raw ASCII when every byte is printable, hex otherwise
    if len(data) == 2 * n:
        return bytes.fromhex(data)
    if len(data) == n:
        return data.encode("latin-1")
    raise RuntimeError(f"READ 0x{addr:08X}: unexpected reply length {len(data)} for {n} bytes")


def read_app(link):
    out = bytearray()
    t0 = time.time()
    for addr in range(APP_BASE, APP_BASE + APP_SIZE, READ_CHUNK):
        n = min(READ_CHUNK, APP_BASE + APP_SIZE - addr)
        out += bl_read(link, addr, n)
        done = len(out) * 100 // APP_SIZE
        print(f"\r    reading {done:3d}%  ({time.time() - t0:5.1f}s)", end="", flush=True)
    print()
    return bytes(out)


def trim_ff(img):
    end = len(img.rstrip(b"\xff"))
    end = (end + 7) & ~7
    return img[:end]


def check_vectors(img):
    sp = int.from_bytes(img[0:4], "little")
    pc = int.from_bytes(img[4:8], "little")
    ok = 0x20000000 <= sp <= 0x20010000 and APP_BASE <= (pc & ~1) < APP_BASE + APP_SIZE and pc & 1
    return ok, sp, pc


def flash(link, img):
    print(f"    image {len(img)} bytes, CRC32/MPEG-2 0x{crc32_mpeg2(img):08X}")
    r = link.cmd("AT+DFU=ERASE", ("+DFU=",), 10.0)
    if r != "+DFU=OK":
        raise RuntimeError(f"ERASE failed: {r}")
    print("    erase OK")
    # Vector table (chunk 0) goes LAST: if power is lost mid-update the first
    # flash word stays erased, and at the next power-up the STM32WL "empty
    # flash" rule boots ST's ROM bootloader on these same pads — recoverable
    # with Tools/rom_uart_flash.py, no BOOT0. The bootloader CRCs bytes in
    # arrival order, so VERIFY gets the CRC of the same order.
    offsets = list(range(WRITE_CHUNK, len(img), WRITE_CHUNK)) + [0]
    sent = bytearray()
    t0 = time.time()
    for i, off in enumerate(offsets, 1):
        chunk = img[off:off + WRITE_CHUNK]
        line = f"AT+DFU=WRITE,{APP_BASE + off:08X},{chunk.hex().upper()}"
        r = link.cmd(line, ("+DFU=",), 5.0)
        if r != "+DFU=OK":
            raise RuntimeError(f"WRITE @0x{APP_BASE + off:08X} failed: {r}")
        sent += chunk
        print(f"\r    writing {i * 100 // len(offsets):3d}%  ({time.time() - t0:5.1f}s)", end="", flush=True)
    print()
    r = link.cmd(f"AT+DFU=VERIFY,{crc32_mpeg2(bytes(sent)):08X}", ("+DFU=",), 5.0)
    if r != "+DFU=OK":
        raise RuntimeError(f"VERIFY failed: {r}")
    print("    verify OK (bootloader CRC matches)")
    # Successful VERIFY completes the session: BL validates and jumps on its own
    r = link.wait(("+DFU=OK,VALIDATED", "+DFU=ERR"), 5.0)
    print(f"    bootloader: {r or '(no VALIDATED line seen)'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", "-p", help="COM port (auto if only one)")
    ap.add_argument("--backup", metavar="FILE", help="read the app region back to FILE (trailing 0xFF trimmed)")
    ap.add_argument("--flash", metavar="FILE", help="WRITE this .bin to the app region (destructive)")
    ap.add_argument("--verbose", "-v", action="store_true", help="show console chatter")
    args = ap.parse_args()

    print("[0] port")
    port = args.port or find_port()
    link = Link(port, args.verbose)

    try:
        print(f"[1] probing {port} at {BAUDS} ...")
        mode, baud = probe(link)
        if not mode:
            sys.exit("    nothing answered. Check: adapter TX->module P4 (PA3/RX), adapter RX<-module P5 (PA2/TX), GND, module powered.")
        print(f"    found {'application' if mode == 'app' else 'bootloader (already in DFU)'} at {baud} baud")
        app_baud = baud if mode == "app" else None
        if mode == "app":
            print(f"    {link.cmd('AT+FW=?', ('+FW=',), 2.0)}")
            print(f"    {link.cmd('AT+VERSION=?', ('+VERSION=',), 2.0)}")
            print("[2] AT+BOOT= -> bootloader")
            r = link.cmd("AT+BOOT=", ("+BOOT=", "+ERROR"), 2.0)
            if r == "+BOOT=PENDING":
                # CC_TRACKER defers the jump while the radio is transmitting
                print("    +BOOT=PENDING (radio busy) - waiting for the TX to finish")
                r = link.wait(("+BOOT=OK", "+ERROR"), 240.0)
            if r != "+BOOT=OK":
                sys.exit(f"    AT+BOOT refused ({r}) - this firmware has no DFU bootloader entry;"
                         " flash it once with Tools/rom_uart_flash.py (BOOT0) or SWD (make flash-full)")
            link.drain(0.5)

        r = bl_ping(link)
        if not r:
            sys.exit("    bootloader did not answer AT+DFU=PING at either baud")
        print(f"    PING   {r}  (bootloader UART at {link.baud} baud)")
        r = link.cmd("AT+DFU=INFO", ("+DFU=",), 2.0)
        if not r or not r.startswith("+DFU=OK,"):
            sys.exit(f"    INFO failed: {r}")
        info = bytes.fromhex(payload(r))
        print(f"    INFO   BL v{info[0]}.{info[1]}.{info[2]}  app @0x{int.from_bytes(info[3:7], 'little'):08X}"
              f"  max {int.from_bytes(info[7:11], 'little')} B  page {int.from_bytes(info[11:15], 'little')} B")
        print(f"    STATUS {link.cmd('AT+DFU=STATUS', ('+DFU=',), 2.0)}")

        if args.backup:
            print("[3] backup: reading app region twice")
            a = read_app(link)
            b = read_app(link)
            if a != b:
                raise RuntimeError("two reads of the app region differ - not saving")
            img = trim_ff(a)
            ok, sp, pc = check_vectors(img)
            with open(args.backup, "wb") as f:
                f.write(img)
            print(f"    saved {len(img)} bytes -> {args.backup}  CRC 0x{crc32_mpeg2(img):08X}"
                  f"  SP=0x{sp:08X} PC=0x{pc:08X} vectors {'OK' if ok else 'SUSPECT'}")

        if args.flash:
            img = open(args.flash, "rb").read()
            ok, sp, pc = check_vectors(img)
            if not ok:
                raise RuntimeError(f"{args.flash}: vector table looks wrong (SP=0x{sp:08X} PC=0x{pc:08X}) - refusing")
            if len(img) > APP_SIZE:
                raise RuntimeError(f"{args.flash}: {len(img)} B exceeds the {APP_SIZE} B app region")
            print(f"[4] flashing {args.flash}")
            flash(link, img)
        else:
            print("[4] AT+DFU=RESET -> back to application (nothing written)")
            link.cmd("AT+DFU=RESET", ("+DFU=",), 1.0)

        print("[5] waiting for the application")
        time.sleep(1.5)
        for b in ([app_baud] if app_baud else []) + list(BAUDS):
            link.open(b)
            link.ser.write(b"\r\n")
            link.drain(0.2)
            r = link.cmd("AT+FW=?", ("+FW=",), 2.0) or link.cmd("AT+FW=?", ("+FW=",), 2.0)
            if r:
                print(f"    {r}  (at {b} baud)")
                print("PASS")
                return 0
        print("    application did not answer AT+FW=? (it may be booting slowly - check the console)")
        return 1
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
