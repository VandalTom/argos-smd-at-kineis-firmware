# Naiad CLS IQ capture

CLS has to validate IQ recordings of a Naiad transmission before it assigns
Kinéis IDs to the argos-smd module on our carrier. The Wing board and the full
kit are already certified; the bare SMD module is only pre-certified, because
the RF PA supply is on our PCB. Procedure from Susanna Polino (CLS), 1 Sept
2026; the LDA2 RCONF was confirmed by the CLS user office on 2 Sept 2026.

Console: module P5 (PA2, TX) / P4 (PA3, RX) / GND, 115200 8N1, CP2102, CH340
or FTDI adapter.

## Kit

- Naiad on its production power chain: NiMH pack → TPS61094 boost →
  TPS62842 3.3 V buck → module VDD/VDD_EXTPA. Not a bench supply: the
  recording is how CLS judges that supply under PA load.
  `bsp_cc_carrier.h` calls this chain "rev-next", so record on the board
  revision that will go to production.
- ADALM-Pluto on the micro-USB port marked with the USB symbol. Drivers:
  `PlutoSDR-M2k-USB-Drivers.exe` (analogdevicesinc/plutosdr-m2k-drivers-win,
  v0.9).
- EmulSat v1.2 (`release_v1.2.zip`) and its user manual
  CLS-ENV-MU-24-0315, both from the CLS resource centre. The manual is
  written for v0.9; v1.1 added IQ file management.
- 40 dB of attenuation: the Nooelec SMA attenuator kit (inline pads, 2 W
  each) stacked 20 + 10 + 6 + 3 + 1 dB, plus an SMA cable and an adapter
  from the Naiad RF connector. CLS's minimum is 30 dB; the EmulSat manual
  uses 40 dB. Naiad transmits at 27 dBm (0.5 W). The stack is long: lay it
  on the bench with a cable at each end rather than hanging it off the
  Pluto's Rx port.

## 1. Firmware and radio configuration

- [ ] Flash the current Naiad release (web tool, or
      `Tools/uart_dfu_check.py --port <p> --flash <image>`). `AT+FW=?`
      shows `Naiad v<NAIAD_VERSION> (...)`.
- [ ] `AT+RCONF=?` returns `+RCONF=401620000,401680000,27,LDA2`. If not:
      `AT+RCONF=3d678af16b5a572078f3dbc95a1104e7`, `AT+SAVE_RCONF=`,
      `AT+RESET=`, and check again.

## 2. Test identity

CLS wants the recording made under the Kinéis test identity (ID 123456,
address `11223344`). CLS's email gives the address as `0x1122344`, which is a
typo: the firmware and EmulSat both use `11223344`.

- [ ] If the unit holds a real CLS set, `AT+PROV=ERASE` first. This frees the
      set in the team vault.
- [ ] Type the test identity on the console. Do not use the web tool's "Use next
      free set", which hands out a real CLS set:
      `AT+ID=123456`, `AT+ADDR=11223344`,
      `AT+SECKEY=<test_device_secret_key from Kineis/Extdep/Mcu/Src/mcu_aes.c>`,
      then `AT+RESET=`. Expect `+PROV=OK`.
- [ ] Alternatively, an erased unit built with `CC_ALLOW_TEST_CREDS=1`
      (`cc_conf.h`) transmits under the same identity.

## 3. RF connection

- [ ] Remove the antenna. Connect the Naiad RF output → 40 dB stack (20 dB pad
      nearest the Naiad) → SMA cable → Pluto **Rx**. Make every RF connection before the Naiad powers up,
      so the PA never transmits into an open port.
- [ ] Connect the Pluto to the PC. When EmulSat starts, the first line of its
      Logs panel confirms it found the Pluto.

## 4. EmulSat

- [ ] Unzip `release_v1.2` and run `bin/EmulSat.exe`.
- [ ] Load the credentials file `creds_krd_example.txt`. It already holds
      `11223344` with the same key as the firmware's test key.
- [ ] Uplink: select **LDA2 only**, set the frequency to **401650000 Hz** (the
      middle of 401.62-401.68 MHz), and enable RX.

## 5. Dry run

- [ ] `AT+SELFTEST=TX`. Expect `+ST=TX,PASS,ms=...` on the console.
- [ ] EmulSat logs an LDA2 frame from `11223344` with no authentication
      error. If nothing arrives, check the frequency, the modulation selection
      and the RF chain before you record.

## 6. Recording

- [ ] Start the IQ recording in EmulSat, send `AT+SELFTEST=TX`, and stop the
      recording once `+ST=TX,PASS` is back. Start and stop as close to the burst as
      you can, because the files are large.
- [ ] Repeat until you have three good files. CLS needs at least one.
- [ ] For a periodic transmitter instead: `AT+COMMISSION=START` sends a frame
      every ~60 s for 30 min. Stop it with `AT+COMMISSION=SKIP`, and before
      deployment re-arm it with `AT+COMMISSION=ARM`.
- [ ] Note the unit serial (`AT+SN=?`), the firmware ID (`AT+FW=?`), the board
      revision and the file names.

## 7. Clean-up

- [ ] `AT+PROV=ERASE`. No unit leaves the bench with the test identity.

## 8. Submission

Upload the IQ files to a shared Drive folder and send the link to Susanna
Polino (spolino@groupcls.com), cc useroffice@groupcls.com, with:

| Item | Value |
|------|-------|
| Transmitter name | Our name for this module + PA-supply integration. A later device built on the same integration reuses it without a new certification. |
| Device name | Naiad |
| Main application | Drifting weather station: WS85 wind/rain, BME280, GPS. One 24-byte LDA2 frame every 7-30 min, depending on battery. |
| RF PA supply | TPS61094 boost → TPS62842 3.3 V buck → VDD_EXTPA |
| RCONF | `3d678af16b5a572078f3dbc95a1104e7` (ESS4 401.62-401.68 MHz, 27 dBm, LDA2) |
| Pictures / schematic | Module integration and its power lines |
