# Naiad bench validation

Hardware checks for the `APP=CC_TRACKER` behaviour that unit tests and
simulators cannot cover. Run them in order on one module and note each
result. Console: module P5 (PA2, TX) / P4 (PA3, RX) / GND, 115200 8N1.

Build both images first:

```bash
make APP=CC_TRACKER BOARD=CC_CARRIER COMM=UART MAC_PRFL=BASIC DEBUG=1 -j8 full
```

`build/argos-smd-at-kineis-firmware.bin` is the app image (UART DFU);
`build/argos-smd-at-kineis-firmware_full.bin` is app + bootloader (ROM
bootloader / SWD).

## 1. Get the current build on

- [ ] Power-cycle. If the unit answers `AT+FW=?`, update with
      `Tools/uart_dfu_check.py --port <p> --flash build/argos-smd-at-kineis-firmware.bin`.
- [ ] If it stays silent (builds from before the PRIMASK fix can hang after a
      DFU jump), hold BOOT0 high, power-cycle, then run
      `Tools/rom_uart_flash.py --port <p> --wait 120 --flash build/argos-smd-at-kineis-firmware_full.bin`.
      Release BOOT0 and power-cycle.

## 2. First boot

- [ ] Expect one extra reset on the very first boot: the IWDG-frozen-in-STOP
      option byte is programmed once (`MGR_WDG_ensureIwdgStopOptionByte`).
      Later boots do not reset.
- [ ] The banner shows `[CC] CounterCurrent tracker init`. On an unprovisioned
      unit expect `+PROV=NONE`, then the boot self-test with `+ST=TX,FAIL,unprovisioned`.
- [ ] `AT+STATE=?` shows `NO_CREDS ... prov=0 ... tier=... utc=0`.

## 3. DFU round trip without a power cycle (PRIMASK fix)

- [ ] `Tools/uart_dfu_check.py --flash` with the same image. After
      `+DFU=OK,VALIDATED` the app must answer `AT+FW=?` **without** a power
      cycle. This is the fix that was never bench-validated.

## 4. Provisioning

- [ ] Write credentials: real CLS ones, or build with `CC_ALLOW_TEST_CREDS=1`
      for a bench-only transmit test. Use either
      `Tools/naiad_station.py --port <p> --id .. --addr .. --seckey .. --ignore GPS,WS85`
      or the web tool's Credentials panel.
- [ ] Expect `+PROV=OK` without a reset. `AT+SECKEY=?` returns a CRC only.
- [ ] Power-cycle: the credentials are still there.

## 5. Self-test and TX

- [ ] `AT+SELFTEST=` gives ENV PASS (BME280) and ADC PASS when the NiMH sense
      is fitted, and `+ST=TX,PASS,ms=...` with the antenna connected.
- [ ] `AT+SELFTEST=TX` sent mid-cycle (during the WS85 wait) is followed by
      that cycle's own TX (the review fix for dropped transmissions).

## 6. STOP2 and power

- [ ] After the first cycle `AT+STATE=?` shows `CYCLE_WAIT`, and `slp=` grows
      between queries spaced more than 30 s apart.
- [ ] 3.3 V rail current between cycles: µA level, with the GPS off and the PA
      enable low. Record the value in `Tools/naiad_power_model.py`.
- [ ] Console wake: while asleep, send a bare CR/LF, then `AT+PING=?` within
      5 s. Expect `+OK`.
- [ ] Storm guard: leave the console adapter unplugged for an hour and check
      the rail current. No wake storm should hold the unit awake.
- [ ] Unplug the debugger (SWD): under a debugger the firmware never sleeps.

## 7. Persistence across resets (TAMP / RTC)

- [ ] `AT+COMMISSION=SKIP`, then `AT+RESET=`. `AT+COMMISSION=?` is still
      `DONE`. After a power cycle it shows `WAIT_FIX`.
- [ ] With a GPS fix (outdoors): `utc=` is non-zero, survives `AT+RESET=`,
      and is 0 after a power cycle.
- [ ] The first fix after power-up starts the burst: `+COMMISSION=BURST,...`
      with a TX every ~60 s for 30 min.

## 8. Battery tiers

- [ ] With the NiMH sense on a bench supply, `AT+STATE=?` `bat=` agrees with a
      meter to within ±50 mV.
- [ ] At 2.05 V the tier is `CRIT`: no GPS, the TX is held after a reset, then
      one frame per 2 h. At 2.20 V it comes back to `LOW`.

## 9. Tools against real hardware

- [ ] `Tools/naiad_station.py` full run: flash, provision, self-test with
      `--ignore GPS,WS85`, and a PASS report.
- [ ] `Tools/naiad_tool.html` in Chrome or Edge with a CP2102, CH340 or FTDI
      adapter (Windows blocks PL2303 clones). Connect, flash, run the guided
      commissioning.

Not covered on the bench: a dead LSE crystal mid-mission (RTC liveness
gate), rescue boots (needs a build that crashes before 90 s), and a MAC
that wedges (stuck-MAC reset).
