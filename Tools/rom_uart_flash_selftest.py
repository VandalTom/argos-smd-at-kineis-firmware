#!/usr/bin/env python3
"""
Offline test for rom_uart_flash.py: drives --flash against a byte-level
simulation of ST's AN3155 ROM bootloader (STM32WL55) over a fake serial port.

    python Tools/rom_uart_flash_selftest.py

Checks: sync survives a glitch byte before the first 0x7F, the image lands
byte-exact, pages 100-101 (PMLOG + MGR_CRED credential mirror) and
FLASH_USER (0x0803B000+, credentials) are never erased or written, all-0xFF
chunks are skipped, writes are 8-byte aligned onto erased flash, and the
vector table is written last.
"""

import os
import random
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rom_uart_flash as t  # noqa: E402

random.seed(2)
FLASH = bytearray(random.randbytes(0x40000))          # pre-existing junk everywhere
ORIG = bytes(FLASH)
ACK, NACK = 0x79, 0x1F


class Rom:
    """Byte-level AN3155 state machine."""
    def __init__(self):
        self.synced, self.buf, self.out, self.state = False, bytearray(), bytearray(), None
        self.erases, self.writes, self.garbage_first = 0, 0, True
        self.write_order = []

    def feed(self, data):
        for b in data:
            self.step(b)

    def step(self, b):
        if not self.synced:
            if self.garbage_first:                    # a line glitch before the first 0x7F
                self.garbage_first = False
                return
            if b == 0x7F:
                self.synced = True
                self.out.append(ACK)
            return
        self.buf.append(b)
        st, buf = self.state, self.buf
        if st is None:
            if len(buf) < 2:
                return
            c, x = buf
            self.buf = bytearray()
            if c == 0x7F and x != (0x7F ^ 0xFF):      # re-sync byte while synced -> NACK, keep second byte
                self.out.append(NACK)
                self.buf = bytearray([x])
                return
            if c ^ x != 0xFF:
                self.out.append(NACK)
                return
            self.out.append(ACK)
            if c == 0x02:
                self.out += bytes([1, 0x04, 0x97, ACK])
            elif c in (0x11, 0x31, 0x21):
                self.state = ("addr", c)
            elif c == 0x44:
                self.state = ("erase_n",)
            return
        if st[0] == "addr" and len(buf) == 5:
            a = int.from_bytes(buf[:4], "big")
            assert buf[4] == buf[0] ^ buf[1] ^ buf[2] ^ buf[3], "addr checksum"
            self.buf = bytearray()
            self.out.append(ACK)
            self.state = {0x11: ("rlen", a), 0x31: ("wlen", a), 0x21: None}[st[1]]
        elif st[0] == "rlen" and len(buf) == 2:
            n = buf[0] + 1
            assert buf[1] == buf[0] ^ 0xFF
            o = st[1] - 0x08000000
            self.out.append(ACK)
            self.out += FLASH[o:o + n]
            self.buf = bytearray()
            self.state = None
        elif st[0] == "wlen" and len(buf) >= 1 and len(buf) == buf[0] + 3:
            n = buf[0] + 1
            data = bytes(buf[1:1 + n])
            cs = buf[0]
            for x in data:
                cs ^= x
            assert cs == buf[-1], "write checksum"
            o = st[1] - 0x08000000
            assert o % 8 == 0 and n % 8 == 0, "WL needs 8-byte alignment"
            assert FLASH[o:o + n] == b"\xff" * n, "writing non-erased flash"
            assert o // 0x800 not in (100, 101) and o < 0x3B000, "write into preserved flash"
            FLASH[o:o + n] = data
            self.writes += 1
            self.write_order.append(st[1])
            self.out.append(ACK)
            self.buf = bytearray()
            self.state = None
        elif st[0] == "erase_n" and len(buf) >= 2:
            cnt = int.from_bytes(buf[:2], "big") + 1
            if len(buf) == 2 + 2 * cnt + 1:
                cs = 0
                for x in buf[:-1]:
                    cs ^= x
                assert cs == buf[-1], "erase checksum"
                for k in range(cnt):
                    p = int.from_bytes(buf[2 + 2 * k:4 + 2 * k], "big")
                    assert p < 118, f"erase touched credential page {p}"
                    assert p not in (100, 101), f"erase touched preserved page {p}"
                    FLASH[p * 0x800:(p + 1) * 0x800] = b"\xff" * 0x800
                self.erases += cnt
                self.out.append(ACK)
                self.buf = bytearray()
                self.state = None


ROM = Rom()


class FakeSerial:
    def __init__(self, *a, **k):
        self.timeout = k.get("timeout")

    def reset_input_buffer(self):
        ROM.out.clear()

    def write(self, d):
        ROM.feed(d)

    def read(self, n):
        r = bytes(ROM.out[:n])
        del ROM.out[:n]
        return r


def main():
    t.serial.Serial = FakeSerial
    t.time.sleep = lambda s: None
    img = bytearray(random.randbytes(70000))
    img[0:8] = (0x20008000).to_bytes(4, "little") + (0x080087C5).to_bytes(4, "little")
    img += b"\xff" * (0x33000 - len(img))             # gap like the merged full image
    bl = bytearray(random.randbytes(20000))
    bl += b"\xff" * (0x8000 - len(bl))
    full = bytes(img + bl)                            # 0x08000000..0x0803AFFF
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
        f.write(full)
    try:
        sys.argv = ["rom_uart_flash.py", "--port", "SIM", "--flash", f.name]
        assert t.main() == 0
    finally:
        os.remove(f.name)
    pm = slice(100 * 0x800, 102 * 0x800)
    chk = bytearray(full)
    chk[pm] = ORIG[pm]
    assert bytes(FLASH[:len(full)]) == bytes(chk), "flash content mismatch"
    assert bytes(FLASH[pm]) == ORIG[pm], "PMLOG / credential mirror pages were touched"
    assert bytes(FLASH[0x3B000:]) == ORIG[0x3B000:], "FLASH_USER (credentials) changed"
    assert ROM.write_order[-1] == 0x08000000, "vector table must be written last"
    print(f"OK: erased {ROM.erases} pages, {ROM.writes} writes (0xFF chunks skipped), "
          "vector table last, credentials untouched")
    return 0


if __name__ == "__main__":
    sys.exit(main())
