#!/usr/bin/env python3
"""
One-time UART flashing of argos-smd (STM32WL55) modules through ST's built-in
ROM bootloader (AN2606 / AN3155), for modules that do not yet carry this
repo's DFU bootloader (e.g. the factory e511146 build, which has no AT+BOOT).

Entering the ROM bootloader: hold the module's BOOT0 test pad (top side, next
to R3, ~3.4 mm from L6) at VDD through ~1 kOhm (e.g. from edge pad P32) and
power-cycle. BOOT0 can be released once this tool reports "sync".
The ROM bootloader listens on USART2 = PA2/PA3 = module P5/P4 (same pads as
the AT console), 8 data bits, EVEN parity, 1 stop, auto-baud on 0x7F.

The port is opened first and 0x7F is repeated until the bootloader answers:
any other byte reaching the bootloader first (a port-open glitch, AT text)
mis-locks its auto-baud and it stays silent until the next reset.

Usage:
    python rom_uart_flash.py --port /dev/ttyUSB0 --wait 120
    python rom_uart_flash.py --port /dev/ttyUSB0 --backup module_backup.bin
    python rom_uart_flash.py --port /dev/ttyUSB0 --wait 240 --flash build/argos-smd-at-kineis-firmware_full.bin

After flashing, release BOOT0 and power-cycle. Avoid --go: it starts the app
with the ROM bootloader's clocks still running (HSI/PLL), which this firmware's
SystemClock_Config does not expect, and the app can hang.

--flash erases only the 2 KB pages the image spans and never touches
FLASH_USER (0x0803B000+, Kineis credentials/NVM). After it, the module runs
the repo's DFU bootloader and later updates go over plain UART
(AT+BOOT -> Tools/uart_dfu_check.py --flash).
"""

import argparse
import sys
import time

import serial

ACK, NACK = 0x79, 0x1F
FLASH_BASE, FLASH_SIZE, PAGE = 0x08000000, 0x40000, 0x800
FLASH_USER = 0x0803B000                  # mcu_flash.h FLASH_USER_START_ADDR
# FLASH_PMLOG (0x08032000-0x08032FFF): page 100 = persistent event log, page
# 101 = MGR_CRED credential mirror. The merged image only pads 0xFF there, so
# they are never erased (a wiped mirror would cost the brownout safety net).
PRESERVE_PAGES = (100, 101)
PID_WL55 = 0x497
CHUNK = 256                              # AN3155 max per Read/Write Memory


class RomBL:
    def __init__(self, port, baud):
        self.s = serial.Serial(port, baud, parity=serial.PARITY_EVEN, timeout=0.1)
        time.sleep(0.5)                  # let the adapter's line settle before the first 0x7F
        self.s.reset_input_buffer()

    def rd(self, n, timeout=1.0):
        self.s.timeout = timeout
        b = self.s.read(n)
        if len(b) != n:
            raise RuntimeError(f"timeout ({len(b)}/{n} bytes)")
        return b

    def ack(self, what, timeout=1.0):
        b = self.rd(1, timeout)[0]
        if b != ACK:
            raise RuntimeError(f"{what}: {'NACK' if b == NACK else hex(b)}")

    def cmd(self, c, what):
        self.s.write(bytes([c, c ^ 0xFF]))
        self.ack(what)

    def addr(self, a, what):
        b = a.to_bytes(4, "big")
        self.s.write(b + bytes([b[0] ^ b[1] ^ b[2] ^ b[3]]))
        self.ack(what)

    def sync(self, wait_s):
        end = time.time() + wait_s
        while True:
            self.s.write(bytes([0x7F]))
            t = time.time() + 0.3
            while time.time() < t:
                self.s.timeout = 0.1
                b = self.s.read(1)
                if b and b[0] in (ACK, NACK):       # NACK = already synced earlier
                    time.sleep(0.05)
                    self.s.reset_input_buffer()
                    return
            if time.time() > end:
                raise RuntimeError("no answer to 0x7F - is BOOT0 held high at power-up?")

    def get_id(self):
        self.cmd(0x02, "GET_ID")
        n = self.rd(1)[0]
        pid = self.rd(n + 1)
        self.ack("GET_ID end")
        return int.from_bytes(pid, "big")

    def read(self, a, n):
        self.s.write(bytes([0x11, 0xEE]))
        r = self.rd(1)[0]
        if r != ACK:
            raise PermissionError("READ refused - readout protection is active")
        self.addr(a, "READ addr")
        self.s.write(bytes([n - 1, (n - 1) ^ 0xFF]))
        self.ack("READ len")
        return self.rd(n)

    def erase_pages(self, pages):
        for i in range(0, len(pages), 16):
            batch = pages[i:i + 16]
            self.cmd(0x44, "EXT_ERASE")
            body = (len(batch) - 1).to_bytes(2, "big") + b"".join(p.to_bytes(2, "big") for p in batch)
            cs = 0
            for x in body:
                cs ^= x
            self.s.write(body + bytes([cs]))
            self.ack(f"erase pages {batch[0]}-{batch[-1]}", timeout=15.0)

    def write(self, a, data):
        self.cmd(0x31, "WRITE")
        self.addr(a, "WRITE addr")
        cs = len(data) - 1
        for x in data:
            cs ^= x
        self.s.write(bytes([len(data) - 1]) + data + bytes([cs]))
        self.ack(f"WRITE @0x{a:08X}", timeout=2.0)

    def go(self, a):
        self.cmd(0x21, "GO")
        self.addr(a, "GO addr")


def progress(tag, done, total, t0):
    print(f"\r    {tag} {done * 100 // total:3d}%  ({time.time() - t0:5.1f}s)", end="", flush=True)


def dump(bl):
    img, t0 = bytearray(), time.time()
    for a in range(FLASH_BASE, FLASH_BASE + FLASH_SIZE, CHUNK):
        img += bl.read(a, CHUNK)
        progress("reading", len(img), FLASH_SIZE, t0)
    print()
    return bytes(img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", "-p", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wait", type=float, default=10, help="seconds to keep sending 0x7F (default 10)")
    ap.add_argument("--backup", metavar="FILE", help="dump the whole 256 KB flash (read twice, compared)")
    ap.add_argument("--flash", metavar="FILE", help="write this .bin at --addr (erases only the pages it spans)")
    ap.add_argument("--addr", type=lambda v: int(v, 0), default=FLASH_BASE)
    ap.add_argument("--go", action="store_true", help="jump to the new image (not recommended: app inherits ROM-bootloader clocks; power-cycle instead)")
    args = ap.parse_args()

    bl = RomBL(args.port, args.baud)
    print(f"[1] waiting for the ROM bootloader on {args.port} ({args.baud} 8E1, up to {args.wait:.0f} s)")
    bl.sync(args.wait)
    pid = bl.get_id()
    print(f"    sync OK, product ID 0x{pid:03X}")
    if pid != PID_WL55:
        sys.exit(f"    not an STM32WL55 (expected 0x{PID_WL55:03X}) - stopping")
    try:
        vec = bl.read(FLASH_BASE, 8)
    except PermissionError as e:
        sys.exit(f"    {e}. Unprotecting mass-erases the chip incl. credentials - not doing that.")
    print(f"    readout protection off; current vectors SP=0x{int.from_bytes(vec[:4], 'little'):08X}"
          f" PC=0x{int.from_bytes(vec[4:8], 'little'):08X}")

    if args.backup:
        print(f"[2] backup -> {args.backup}")
        a, b = dump(bl), dump(bl)
        if a != b:
            sys.exit("    the two reads differ - not saving")
        open(args.backup, "wb").write(a)
        print(f"    saved {len(a)} bytes (reads identical)")

    if args.flash:
        img = open(args.flash, "rb").read()
        img += b"\xff" * (-len(img) % 8)                 # STM32WL programs 64-bit double words
        start, end = args.addr, args.addr + len(img)
        if start % PAGE or start < FLASH_BASE:
            sys.exit("    --addr must be page-aligned inside flash")
        if end > FLASH_USER:
            sys.exit(f"    image ends at 0x{end:08X}, past 0x{FLASH_USER:08X} (credentials/NVM) - refusing")
        sp, pc = int.from_bytes(img[:4], "little"), int.from_bytes(img[4:8], "little")
        if start == FLASH_BASE and not (0x20000000 <= sp <= 0x20010000 and FLASH_BASE <= (pc & ~1) < end and pc & 1):
            sys.exit(f"    {args.flash}: vector table looks wrong (SP=0x{sp:08X} PC=0x{pc:08X}) - refusing")
        pages = list(range((start - FLASH_BASE) // PAGE, (end - FLASH_BASE + PAGE - 1) // PAGE))
        for pg in PRESERVE_PAGES:
            o = FLASH_BASE + pg * PAGE - start
            if 0 <= o < len(img) and img[o:o + PAGE] != b"\xff" * len(img[o:o + PAGE]):
                sys.exit(f"    image has data in preserved page {pg} (PMLOG / credential mirror) - refusing")
        pages = [pg for pg in pages if pg not in PRESERVE_PAGES]
        print(f"[3] flashing {args.flash}: {len(img)} bytes at 0x{start:08X}, erasing pages {pages[0]}-{pages[-1]}"
              f" (keeping {', '.join(str(p) for p in PRESERVE_PAGES)})")
        bl.erase_pages(pages)
        print("    erase OK")
        chunks = [(start + o, img[o:o + CHUNK]) for o in range(0, len(img), CHUNK)]
        chunks = [(a, c) for a, c in chunks if c != b"\xff" * len(c)]   # erased flash is already 0xFF
        # Vector table last: an interrupted flash leaves the first word erased,
        # so the next power-up boots this ROM bootloader again without BOOT0.
        chunks = chunks[1:] + chunks[:1]
        t0 = time.time()
        for i, (a, c) in enumerate(chunks, 1):
            bl.write(a, c)
            progress("writing  ", i, len(chunks), t0)
        print()
        t0 = time.time()
        for i, (a, c) in enumerate(chunks, 1):
            if bl.read(a, len(c)) != c:
                sys.exit(f"\n    VERIFY FAILED at 0x{a:08X}")
            progress("verifying", i, len(chunks), t0)
        print("\n    verify OK (every written byte read back)")
        if args.go:
            bl.go(start)
            print(f"    started image at 0x{start:08X}")
        else:
            print("    done - remove the BOOT0 lead and power-cycle to boot the new image")
    return 0


if __name__ == "__main__":
    sys.exit(main())
