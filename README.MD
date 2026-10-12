![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Argos SMD AT Kineis Firmware

This repository contains the STM32WL firmware for the Argos SMD module using the Kineis stack. It provides three application modes for STM32WL55xx-based hardware reference designs (KRD_MP):

- **GUI**: AT command interface (UART or SPI) compatible with KimGUI
- **Standalone**: Single periodic SAT message transmission at startup
- **UW_DOPPLER**: Autonomous underwater/surface tracker with satellite TX scheduling

The project is based on `krd_fw_package_v8.5.1` from CLS and has been adapted for the [argos-smd](https://github.com/arribada/argos-smd-hw) module.

To start, look at the [Arduino sample](https://github.com/arribada/argos-smd-test-arduino) code used for certification or the [Zephyr UART driver](https://github.com/arribada/argos-smd-driver-zephyr) tested with Feather nRF52840.

> **Warning**: Since version v8.5.1, the MAC profile is not loaded at startup — reload it thanks to KMAC commands (`AT+KMAC=0` and `AT+KMAC=1`).

## Features

- **GUI mode** (`APP=GUI`): AT command interface compatible with KimGUI (UART or SPI)
- **Standalone mode** (`APP=STDLN`): Single periodic SAT message transmission at startup
- **UW_DOPPLER tracker mode** (`APP=UW_DOPPLER`): Autonomous underwater/surface detection with satellite TX scheduling, battery monitoring, NVM persistence, event logging, and reed switch shutdown
- Configurable MAC profiles: BASIC (single TX) or BLIND (repeated TX with Doppler)
- Low Power Mode management: NONE, SLEEP, STOP, STANDBY, SHUTDOWN
- DFU bootloader (over UART or SPI) for field firmware updates
- Multiple board variants: SMD_PA, SMD_NOPA, SMD_STDALONE, SMD_OP

## Application Modes

### GUI Mode (`APP=GUI`) — Default

AT command interface for host-controlled operation. The module responds to AT commands over UART (9600 baud) or SPI (115200 baud) and is compatible with the KimGUI tool from CLS.

### Standalone Mode (`APP=STDLN`)

Minimal mode that transmits a single satellite message at startup. Useful for quick tests and certification.

### UW_DOPPLER Tracker Mode (`APP=UW_DOPPLER`)

Autonomous underwater/surface tracker designed for wildlife tracking. Requires `BOARD=SMD_STDALONE`.

#### Principle of Operation

1. **Power on**: Reed switch magnet powers the board via PWR_LATCH (PB7). White LED blinks 10x to confirm boot.
2. **Surface/water detection**: Salt Water Switch (SWS) on PA11/PA12 continuously measures conductivity via ADC. Configurable thresholds determine surface vs. underwater state.
3. **TX scheduling**: When the device surfaces, satellite messages are transmitted using the configured MAC profile. TX intervals grow exponentially (configurable initial interval, growth percent, max interval, max count).
4. **Dive tracking**: The device tracks dive duration and only triggers TX after a configurable minimum surface time.
5. **Battery monitoring**: Internal ADC reads VBAT/3 channel with VREFINT calibration (3300mV factory reference). Battery voltage (mV) is included in the TX payload.
6. **NVM persistence**: TX configuration, SWS thresholds, deploy mode, and LED mode are saved to flash via `AT+SAVE` and survive power cycles.
7. **Event logging**: A circular buffer of 256 events in SRAM2 retention RAM survives all resets except power-off. View with `AT+LOG=?`.
8. **Shutdown**: Hold the reed switch magnet for >10 seconds. White LED blinks 10x. NVM save. PWR_LATCH pulled LOW. Board powers off completely.
9. **Low power**: Between SWS measurements, the MCU enters STOP mode (SysTick suspended, RTC tick compensation on wake).

#### State Machine

```
BOOT ──> BOOT_DEPLOY_LED ──> INIT_MAC ──> WAIT_MAC_READY ──> MONITORING
                                                                  │
                                    ┌─────────────────────────────┤
                                    │                             │
                               SURFACE_TX ──> WAIT_TX_DONE ──────┘
                                    │
                           SHUTDOWN_BLINK (reed hold >10s)
```

| State | Description | Timeout |
|-------|-------------|---------|
| `BOOT` | White LED blink 10x to confirm power-on | 10s |
| `BOOT_DEPLOY_LED` | Deploy mode LED feedback (green=deployed, red=not) | 5s |
| `INIT_MAC` | Initialize Kineis MAC profile (BLIND or BASIC) | — |
| `WAIT_MAC_READY` | Wait for MAC profile init confirmation | 30s |
| `MONITORING` | SWS measurements, reed switch polling, LPM STOP between cycles | — |
| `SURFACE_TX` | Build TX payload and submit to MAC | — |
| `WAIT_TX_DONE` | Wait for MAC TX_DONE or TX_TIMEOUT event | 60s |
| `SHUTDOWN_BLINK` | White LED blink 10x, then NVM save + power off | 10s |

All waiting states have hardware watchdog backup (IWDG, 16s timeout). If a state timeout fires, an `EVT_TIMEOUT` is logged, error is recorded to TAMP registers, and the system resets.

#### TX Scheduling Algorithm

TX intervals follow an exponential growth pattern:

```
T(n) = T_initial * (1 + growth/100)^n
```

Where:
- `T_initial` = initial interval (default 10s, configurable via `AT+TXCFG`)
- `growth` = growth percentage per TX (default 10%)
- The interval is capped at `T_max` (default 600s)
- TX count is limited to `max_count` (default 0 = unlimited)

The first TX after surface detection is **immediate**. Subsequent TXs use the growing interval. TX scheduling resets when the device goes back underwater.

Both MAC profiles are supported:
- **BLIND** (`MAC_PRFL=BLIND`): retx_nb=0, retx_period_s=60, nb_parallel_msg=1. The application handles all TX scheduling; the MAC profile sends each message once.
- **BASIC** (`MAC_PRFL=BASIC`): immediate single TX per request. Same application-level scheduling.

#### Salt Water Switch (SWS) Detection Algorithm

The SWS uses an adaptive biofouling-resistant detection algorithm:

1. **ADC measurement**: PA12 drives the sensor, PA11 reads conductivity via ADC_IN7 (12-bit, 0-4095). Higher values = more conductive = underwater.
2. **Moving average filter**: 2-sample history smooths noise.
3. **Threshold detection**: Dynamic threshold computed from adaptive air/water baselines using a configurable ratio (default 35%). Hysteresis band (14%) prevents oscillation.
4. **Biofouling compensation**: Three-tier rapid transition detection catches surface events even when biofouling shifts the baseline upward:
   - **Tier 1**: Large instant drop (>25% and >75 ADC units)
   - **Tier 2**: Medium drop + trend + below water baseline (>12% and >38 ADC units)
   - **Tier 3**: Small drop + strong trend (>8% and >25 ADC units, 2+ consecutive decreases)
5. **Trend + variance analysis**: Tracks consecutive decreasing readings and variance in a sliding window (8 samples). High variance combined with cumulative drop triggers surface override.
6. **EMA water baseline**: Exponential moving average (alpha=19%) adapts the water baseline during dives.
7. **Extended dive recalibration**: After 60s underwater, recalibrates water baseline every 30s.
8. **Surface adaptation**: After stable surface time (>10s), recalibrates air baseline from recent readings.
9. **Max dive timeout**: Forced surface state after configurable duration (default 7200s).
10. **Min surface lockout**: Prevents re-submersion detection for configurable duration (default 10s).

#### TX Payload Format

9 bytes, sent as user data in each satellite message:

| Byte | Content | Description |
|------|---------|-------------|
| 0 | State | 0=init, 1=surface, 2=underwater |
| 1-2 | SWS ADC | Raw 12-bit ADC value (big-endian) |
| 3 | TX count | Number of TXs in current surface event |
| 4-5 | Battery | Voltage in mV (big-endian) |
| 6-8 | Reserved | Zeros (available for future use) |

#### Robustness Features

| Feature | Implementation | Persistence |
|---------|---------------|-------------|
| **Watchdog** | IWDG 16s timeout, refreshed every loop iteration | Hardware (cannot be stopped) |
| **State timeouts** | All waiting states have configurable timeouts | — |
| **Error tracker** | TAMP backup registers BKP2R-BKP7R | Survives all resets (not VBAT removal) |
| **Event log** | 256-entry circular buffer in SRAM2 (2KB) | Survives all resets (not power-off) |
| **NVM config** | CRC32-protected flash page (last 2KB page) | Survives power cycles |
| **Fault handlers** | HardFault/BusFault log error to TAMP, then reset | TAMP registers |
| **LPM tick compensation** | RTC-based tick correction after STOP mode wake | — |

#### Typical Build Command

```bash
make full APP=UW_DOPPLER BOARD=SMD_STDALONE LPM=STOP MAC_PRFL=BLIND COMM=UART DEBUG=1 -j20
```

### Naiad Weather Tracker (`APP=CC_TRACKER`, `BOARD=CC_CARRIER`)

Drifting weather station: the STM32WL55 on the argos-smd module is the only
processor. Each cycle it samples an Ecowitt WS85 (wind/rain, UART), a BME280
(I2C), the NiMH pack and supercap (ADC) and, on schedule, a GPS (NMEA), packs
the 24-byte payload v3 and transmits it as one LDA2 frame. Cadence is
battery-tiered (7/15/30 min + 0..30 % jitter). Configuration is compile-time
(`Kineis/App/Managers/MGR_CC/Inc/cc_conf.h`).

**Cycle order (shortest wake)** — battery reading and tier first; if a fix
is due, the GPS is powered straight away and acquires while the WS85 (up to
~35 s for its broadcast) and BME280 are read, then its NMEA is checked (USART1
serves one sensor at a time; the receiver needs no listener to acquire), then
the frame goes out. Sensor readings are taken every cycle (1 min freshness
limit), so each packet carries readings from right before it. Once a recent
fix has shown the receiver hot-starts (fix within 10 s), it is powered after
the sensors instead: a head start would only burn GPS current through the
WS85 wait. GPS cadence by battery: a fix every 3rd transmission at
HIGH/MID; every 6th and no every-cycle commissioning probes at LOW; none at
CRIT.

**Critical battery** — a NiMH pack measured below 2.10 V (1.05 V/cell) puts
the unit in the `CRIT` tier: no GPS, no deployment burst, one transmission
every 2 h (still carrying the voltage) until the pack recovers past
2.15 V. A boot counts as a transmission (the boot self-test skips its TX and
the first frame waits the full 2 h), so brownout resets under PA load can't
turn into a transmit loop. A reading below 1.0 V counts as "no
sense" (unfitted or failed divider) and keeps the normal `LOW` tier, so a
broken sense line never silences a unit.

```bash
make APP=CC_TRACKER BOARD=CC_CARRIER COMM=UART MAC_PRFL=BASIC DEBUG=0 -j8 full
```

**Console** — debug UART (module P5 = PA2 TX, P4 = PA3 RX), 115200 8N1. Only
the commands below are accepted (`+ERROR=1203` otherwise), so nothing on the
console can drive the radio behind the tracker's back:

| Command | Reply |
|---------|-------|
| `AT+FW=?` `AT+VERSION=?` `AT+PING=?` `AT+ID=?` `AT+ADDR=?` `AT+SN=?` `AT+RCONF=?` | standard `+X=...` then `+OK` |
| `AT+ID=<dec>` `AT+ADDR=<8hex>` `AT+SECKEY=<32hex>` `AT+RCONF=<32hex>` `AT+SAVE_RCONF=` | `+OK`; `+ERROR=BUSY` while transmitting; effective after `AT+RESET` |
| `AT+SECKEY=?` | `+SECKEYCRC=<8hex>` — CRC-32 of the key; the key is never echoed |
| `AT+PROV=ERASE` | `+PROV=ERASED` then `+RESET=OK`: ID, ADDR, SECKEY and the credential mirror blanked (RCONF kept), unit reboots into `NO_CREDS`; `+ERROR=BUSY` while transmitting |
| `AT+STATE=?` | `+STATE=<state>,up=<s>,tx=<n>,mac=<0/1>,comm=<phase>,st=<NONE/RUNNING/PASS/FAIL>,slp=<s in STOP2>,prov=<0/1>,bat=<mV>,tier=<HIGH/MID/LOW/CRIT>,utc=<unix, 0 = no fix yet>` |
| `AT+SELFTEST=[GPS,WS85,ENV,ADC,TX]` / `AT+SELFTEST=?` | run (all items by default) / replay the last report |
| `AT+COMMISSION=?` / `=START` / `=SKIP` / `=ARM` | deployment-burst status / force / mark done / re-arm |
| `AT+RESET=` | `+RESET=OK` (or `+RESET=PENDING` until a TX finishes), reboot |
| `AT+BOOT=` | `+BOOT=OK` (or `PENDING`), hands over to the UART DFU bootloader |

**Boot self-test** — runs once per boot right after the MAC is ready, and on
demand. Line-test fast: GPS passes on a checksum-valid NMEA sentence within
5 s (no fix needed), WS85 on one CRC-valid broadcast (≤ 35 s), ENV on a
plausible BME280 reading, ADC on the NiMH voltage being in range, TX on the
MAC's TX_DONE for one real frame (no satellite pass needed). Report:
`+ST=<ITEM>,<PASS|FAIL|SKIP>,<detail>` per item, then
`+SELFTEST=PASS` or `+SELFTEST=FAIL,<items>`.

The GPS item also diagnoses itself (about 7 s, up to 15 s with a baud scan):
`+ST=GPS,<verdict>,bytes=<n>,nmea=<n>,cs_err=<n>,fe=<n>,line=<off>/<on>[,off=bytes|nmea][,baud=<b>|none]`.

- `line` is the data line (P11/PB6) probed against the module's own pull-down
  and pull-up, first with the receiver switched off, then on:
  - `fl`: nothing drives it.
  - `hi`: held high (an idle receiver output).
  - `lo`: held low (an unpowered receiver clamps it, or a short).
  - `act`: data is moving.
- Before switching the receiver on, the module listens for 1.2 s. If it hears
  NMEA (`off=nmea`), the enable (P3/PA1) does not switch the receiver off,
  which would drain the battery: FAIL.
- A driven line with no NMEA at 9600 baud triggers a scan of 38400, 115200,
  4800, 19200 and 57600 baud. A hit (`baud=<b>`) passes. The module then uses
  that baud for its GPS windows until the next boot self-test re-learns it.
- `fe` counts UART framing/noise errors.

The web tool turns these into plain-words advice.

**Deployment (commissioning) burst** — units are switched on indoors and
deployed later with no button. The first GPS fix since power-up is the
"outdoors now" signal: the unit transmits every 60 s (+0..5 s jitter) for
30 min so it is heard quickly, then runs normally. Until that fix, GPS is
tried every cycle for the first 48 h after power-up (one window, no backoff;
not at LOW or CRIT battery) and the window ends 45 s after GPS power-up if
the receiver tracks fewer than two satellites above 20 dBHz (indoors),
judged over at least 3 complete GSV groups. The "done" flag lives in a TAMP backup register: it
survives watchdog/soft resets and updates and is cleared only by real power
loss.

**Clock** — GPS time is anchored to the RTC calendar (anchor in TAMP
BKP8R/BKP9R). The calendar and the anchor survive every reset — watchdog,
`AT+RESET`, an update, the 14-day reboot — so packet timestamps stay valid
until the next fix instead of dropping to 0; power loss clears both. GPS
dates before 2025 (week-rollover receivers) are ignored.

**Provisioning** — credentials (ID, ADDR, SECKEY from CLS; RCONF optional)
are written over the console. A CRC-protected mirror (`MGR_CRED`, page
`0x08032800`) restores them if a brownout wipes page 0. A unit with no
credentials never starts the radio (it would otherwise transmit under the
Kineis test identity): it reports `+PROV=NONE`, sits in state `NO_CREDS`
(`AT+STATE=?` shows `prov=0`), still runs the sensor self-test, and resumes
by itself (`+PROV=OK`) once ID, ADDR and SECKEY are present. Build with
`CC_ALLOW_TEST_CREDS=1` (cc_conf.h) to transmit under the test identity on a
bench.

**Commissioning web tool** — `Tools/naiad_tool.html`, live at
**https://arribada.siliconvandals.com** (fallback https://naiad-tool.seb-f80.workers.dev;
the file also works opened locally, without the release list and the team vault).
Chrome or Edge (Web Serial), no install: connect, unit info (FW, ID, state,
battery tier, device UTC), flash with progress and the vector-table-last
order, self-test, credential entry (key masked, checked by CRC),
commissioning controls, console, JSON/CSV reports, and a guided "Commission
unit" wizard. A built-in simulator runs everything without hardware;
`node Tools/naiad_tool_selftest.mjs` tests the protocol layer against it.
Windows blocks PL2303 clones: use a CP2102/CH340/FTDI adapter.

*Voyage view* (default for new users; "Pro" in the header switches to the
full panel view, and the choice is remembered): a step-by-step route through
seven "seas", one task each, in production order: hail the module (or
practise on the simulator) → fresh firmware (installs the latest release when
the module runs anything older; says when a module needs a one-time SWD
flash) → sensor check → ship's papers (next free set from the team vault,
written and verified) → first signal (TX test) → deployment orders (arm the
burst or not) → home port (verdict, production-log line, report). Each sea
shows one plain-language status with what to do next, and how to fix it
when something fails; seas unlock in order and the chart tracks progress.
The traditional shanty "Drunken Sailor" plays once when a voyage starts,
synthesised in the page (Sound on/off; "Music…" switches to a short jingle or
an audio file of your own, kept in that browser only).

*Debug probe (SWD)*: for modules the serial update cannot reach (factory
firmware without the update bootloader, or a silent module), the Flash panel
and the Voyage view flash the release's full image (app + bootloader) through
a debug probe on the module's SWD pads (SWDIO P26, SWCLK P25, NRST P35, GND;
on the Naiad carrier the debug JST), from the browser: ST-Link V2/V2-1/V3
(including V2 mini clones) via WebUSB, which on Windows needs the WinUSB
driver (ST's ST-LINK driver from STM32CubeProgrammer, or Zadig); CMSIS-DAP v2
via WebUSB (no driver on Windows), v1 via WebHID. It connects under reset (NRST
held low: reaches a module asleep in STOP2), freezes the watchdogs while the
CPU is halted, refuses read-protected chips, CPU2 secure areas and
write-protected pages, erases only the app (pages 0-99) and bootloader (102-117)
regions (never the power log + credential mirror at 0x08032000-0x08032FFF or
FLASH_USER at 0x0803B000), writes the vector table last, verifies by reading
back, and restarts the module. Probes: a Raspberry Pi Pico running Raspberry
Pi's debugprobe firmware (GP2 SWCLK, GP3 SWDIO, GP1 reset), DAPLink, or a
J-Link BASE/PLUS V12+ switched to CMSIS-DAPv2 in J-Link Configurator (Boot
mode; switch back for JLinkExe). With an ST-Link the SWD clock drops to 480 kHz
while flash programs (no WAIT retries needed). On some ST-Link V2 clones the RST
pin does not reach an STM32: power-cycle a sleeping module just before
flashing. Simulated probes (ST-Link and CMSIS-DAP) are available for practice;
the browser flasher is tested against them (not yet on real hardware).

*Released firmware*: the Flash panel lists the Naiad images attached to the
GitHub Releases (field and bench builds) and loads one with a click; the
flash is then verified against the release's firmware ID (`Naiad vX.Y.Z (`).
Releases with a known problem carry a warning (v1.0.0: do not deploy).

*Team vault* (Team credentials panel): one shared table of the CLS
credential sets for the whole team. Sign in with a @siliconvandals.com
Google account (Cloudflare Access), then unlock with the team passphrase.
Keys are encrypted and decrypted in the browser with that passphrase
(PBKDF2-SHA256, 600,000 iterations → AES-256-GCM, the set's ID bound in), so
the server only ever holds ciphertext and each key's CRC-32. Import the CLS
file (CSV with comma/semicolon/tab, or .xlsx; header row optional), edit in
a spreadsheet-style table (concurrent edits are detected, nothing is
overwritten), track use: free / in use (with the unit's serial number,
firmware and date) / retired, with a per-set history of who did what.
"Use next free set" fills the credentials panel; a verified provisioning
marks the set in use and `AT+PROV=ERASE` frees it (tracking works while the
vault is locked; changes made while signed out are sent at the next
sign-in). Every key fetch is logged with the user's email. Locks after 15 min
idle. Exports: CSV without keys, encrypted backup file. The passphrase can be
changed (every key re-encrypted in the browser). A lost passphrase cannot be
recovered: re-import from CLS.

*Offline vault* (this browser): an encrypted copy of the team vault ("Save
offline copy here", or a downloaded backup file) for when the server cannot
be reached; it opens with the team passphrase.

Hosting: Cloudflare Workers (`web/naiad-tool/`): the page as static assets
plus `src/worker.js` (release list and image proxy, vault API on D1, Access
JWT check). Workers Builds deploys every push to `naiad` (root
`web/naiad-tool`, build `npm run build`, deploy `npx wrangler deploy`);
`npm test` there runs the Worker against mocked GitHub/Access and SQLite.
The page's CSP allows network requests to its own origin only.
Vault setup (done 2026-10-12): D1 database `naiad-vault` (binding `DB`);
Zero Trust Free, team `siliconvandals` (siliconvandals.cloudflareaccess.com);
Access application "Naiad team vault" on `arribada.siliconvandals.com/api/vault`
with policy "siliconvandals staff" (Allow: emails ending in
@siliconvandals.com), accepting every login method; `ACCESS_TEAM_DOMAIN` and
`ACCESS_AUD` in `wrangler.jsonc`. Login methods: email one-time PIN now;
Google once its OAuth client is added: Google Cloud Console → APIs & Services
→ Credentials → OAuth client ID (Web application), authorised JavaScript
origin `https://siliconvandals.cloudflareaccess.com`, redirect URI
`https://siliconvandals.cloudflareaccess.com/cdn-cgi/access/callback`; then
Zero Trust → Integrations → Identity providers → Add → Google (or Google
Workspace) with that client ID and secret.

**Production line** — `Tools/naiad_station.py` does one unit per command:
flash (optional), firmware check, provisioning with read-back and key CRC,
self-test, commissioning flag, and a JSON report; exit code 0 = PASS. Sensor
items a jig can't exercise are downgraded with `--ignore GPS,WS85`.
`Tools/naiad_station_selftest.py` runs it against a simulated unit (no
hardware).

```bash
python Tools/naiad_station.py --port COM5 --flash build/argos-smd-at-kineis-firmware.bin --id 279665 --addr 1A2B3C4D --seckey <32 hex> --ignore GPS,WS85 --commission skip
```

**Update and recovery path**

1. Normal update: `Tools/uart_dfu_check.py --port <port> --flash build/argos-smd-at-kineis-firmware.bin`
   (`AT+BOOT` → DFU bootloader → erase/write/CRC → app restarts, ~30 s at 115200).
2. Interrupted update (power loss): the flashers write the vector table last,
   so the first flash word is still erased and the next power-up boots ST's
   ROM bootloader on the same pads — recover with `Tools/rom_uart_flash.py`,
   no BOOT0 needed.
3. Build that crashes early: after 3 failed boots in a row (a boot is "good"
   once it stays up 90 s) the next boot is a **rescue boot** — no sensors, no
   radio, console only for 10 min (`+RESCUE=failed_boots=N,window_s=600`),
   enough to `AT+BOOT` and reflash. Fault handlers always reset (never hang).
4. Last resort: BOOT0 test pad high at power-up + `Tools/rom_uart_flash.py`,
   or SWD (`make flash-full`).

**Self-healing** — 3 cycles in a row whose frame never completes (MAC
timeout, abort, error, queue refused) or 3 MAC init failures reset the unit
instead of idling at full power; before every STOP2 the RTC must prove it is
ticking (a dead 32 kHz crystal resets onto the LSI fallback rather than
sleeping forever).

**Power** — `Tools/naiad_power_model.py` ranks the consumers from a tagged
assumption table (replace with rail measurements). GPS acquisition and the
27 dBm TX dominate once the MCU sleeps in STOP2 between phases. A console
RX edge wakes a sleeping unit for 5 s, a console line keeps it awake 30 s;
6 edge wakes without a line (noise on an unconnected RX) disarm the RX wake
for 30 min, doubling to 4 h, so a floating console can't hold it out of
STOP2.

## Board Variants

| Feature | SMD_PA | SMD_NOPA | SMD_STDALONE | SMD_OP |
|---------|--------|----------|--------------|--------|
| External PA | Yes | No | Yes | Yes |
| RGB LED (R/G/B) | — | — | PA1/PB4/PB5 | — |
| Reed switch | — | — | PB6 | — |
| Power latch | — | — | PB7 | — |
| VBAT monitoring | — | — | PB9 (enable) + internal ADC | — |
| Salt Water Switch | — | — | PA11 (ADC) / PA12 (drive) | — |
| Use case | Standard AT module | No-PA AT module | Autonomous tracker | Operational (same as PA) |

### SMD_PA (`BOARD=SMD_PA`) — Default
Standard board with external power amplifier. PA control via VPA_EN (PC0) and VSEL (PC1). Suitable for GUI and Standalone modes.

### SMD_NOPA (`BOARD=SMD_NOPA`)
Same as SMD_PA but without external PA. Uses the STM32WL internal RF. Lower TX power, lower cost.

### SMD_STDALONE (`BOARD=SMD_STDALONE`)
Full-featured board for autonomous tracker operation. Includes:
- **RGB LED** (active LOW): Red (PA1), Green (PB4), Blue (PB5)
- **Reed switch** (PB6, active HIGH): Magnetic on/off control with debounce
- **PWR_LATCH** (PB7): External pull-up keeps board powered; pull LOW to shutdown
- **VBAT_EN** (PB9): Enables battery voltage measurement circuit
- **SWS** (PA11/PA12): Salt Water Switch for underwater detection
- **PA** (PC0/PC1): External power amplifier control

Required for `APP=UW_DOPPLER`. The MGR_LED and MGR_REED drivers are automatically included when this board is selected.

### SMD_OP (`BOARD=SMD_OP`)
Operational variant. Currently identical to SMD_PA (placeholder for future differentiation).

## Getting Started

### Configuration

#### Credentials Management

> **Warning**: This firmware requires proper configuration of **credentials** and **radio settings**. Misconfigured settings may lead to improper operation or device malfunction.

- Contact CLS group (user office) to obtain the required credentials (ID, address, secret key) for the stack.
- Securely store the credentials. The provided package stores them in RAM/FLASH without protection.
- Update `device_id` and `address` in the `mcu_nvm.c` file.

Note: Currently, the ID must be changed at build time. A command to modify it will be added soon.

#### Radio Configuration Management

- Obtain radio configuration binary files from CLS group (user office) or use one defined in `mcu_nvm.c`.
- Store configuration data securely. In the provided package, the configuration is hardcoded as a byte table in `mcu_nvm.c`.
- Alternatively:
  - Use the `AT+RCONF=` command.
  - Modify the configuration directly by commenting/uncommenting the corresponding settings in `mcu_nvm.c`.

### MAC Profile Configuration Management

- Choose between **BASIC** and **BLIND** MAC profiles by setting `MAC_PRFL=` in the build command.
- For UW_DOPPLER, either profile works. The application handles TX scheduling directly; the MAC profile is only responsible for the single-message transmission.
- BLIND config: `retx_nb=0`, `retx_period_s=60`, `nb_parallel_msg=1`.

## Building the Project

### From STM32CubeIDE

1. Install STM32CubeIDE.
2. Open the project from the Kineis package archive.
3. Clean the project before rebuilding.
4. Build the project to generate the binary file.

### From Terminal

1. Ensure GNU Make and the ARM toolchain (`arm-none-eabi-gcc`) are available.
2. Clean the current build before rebuilding: `make clean`
3. Run `make` from the root directory.

### Build Variables

| Variable | Values | Default | Description |
|----------|--------|---------|-------------|
| `APP` | `GUI`, `STDLN`, `UW_DOPPLER` | `GUI` | Application mode |
| `BOARD` | `SMD_PA`, `SMD_NOPA`, `SMD_STDALONE`, `SMD_OP` | `SMD_PA` | Hardware board variant |
| `COMM` | `UART`, `SPI` | `UART` | Communication interface |
| `MAC_PRFL` | `BASIC`, `BLIND` | `BASIC` | Kineis MAC profile |
| `LPM` | `NONE`, `SLEEP`, `STOP`, `STANDBY`, `SHUTDOWN` | `NONE` | Deepest low power mode allowed |
| `DEBUG` | `0`, `1` | `1` | Enable debug symbols and -Og optimization |
| `VERBOSE` | `0`, `1` | `1` | Enable verbose debug log output |
| `SMPS_BYPASS_TX` | `0`, `1` | `0` | Use LDO mode (bypass SMPS) during TX for TCXO stability |

### Build Options

#### Application Only (Default)
```bash
# GUI mode — Debug UART
make DEBUG=1 VERBOSE=1 -j20

# GUI mode — Release UART
make DEBUG=0 VERBOSE=0 -j20

# GUI mode — Debug SPI
make DEBUG=1 VERBOSE=1 COMM=SPI -j20

# GUI mode — Release SPI
make DEBUG=0 VERBOSE=0 COMM=SPI -j20

# UW_DOPPLER tracker — Debug UART
make APP=UW_DOPPLER BOARD=SMD_STDALONE MAC_PRFL=BLIND LPM=STOP DEBUG=1 VERBOSE=1 -j20
```

#### Bootloader Only
```bash
# Build bootloader for SPI mode (default)
make bootloader COMM=SPI

# Build bootloader for UART mode
make bootloader COMM=UART
```

#### Combined Firmware (App + Bootloader)
Build both application and bootloader, then merge them into a single flashable image:
```bash
# GUI — Debug SPI
make full COMM=SPI DEBUG=1 VERBOSE=1 -j20

# GUI — Release SPI
make full COMM=SPI DEBUG=0 VERBOSE=0 -j20

# UW_DOPPLER tracker — Release UART
make full APP=UW_DOPPLER BOARD=SMD_STDALONE MAC_PRFL=BLIND LPM=STOP COMM=UART DEBUG=0 VERBOSE=0 -j20
```

Output files:
- `build/argos-smd-at-kineis-firmware_full.bin` - Binary for flashing
- `build/argos-smd-at-kineis-firmware_full.hex` - Intel HEX format

#### DFU Binary (For OTA Updates)
Generate a binary file for Device Firmware Update via the bootloader:
```bash
make dfu COMM=SPI -j20
```

Output: `build/argos-smd-at-kineis-firmware_dfu.bin`

### Flashing

#### Using Make (JLink)
```bash
# Flash application only
make flash-app

# Flash bootloader only
make flash-bl

# Flash combined firmware (app + bootloader)
make flash-full
```

#### Over UART (console pads P5/P4 = PA2/PA3)
Updates go through the DFU bootloader: `AT+BOOT` hands over to it, then the
app region is rewritten and CRC-checked. Needs `pip install pyserial`.
```bash
python Tools/uart_dfu_check.py --port COM3                      # read-only: enter DFU and come back
python Tools/uart_dfu_check.py --port COM3 --backup app_old.bin  # also save the current app
python Tools/uart_dfu_check.py --port COM3 --flash build/argos-smd-at-kineis-firmware.bin
```
- Works with GUI builds (9600 baud) and CC_TRACKER builds (115200; `AT+BOOT`
  waits for any TX in progress and replies `+BOOT=PENDING` meanwhile).
- UART GUI builds must use `LPM=STOP` (or lighter): with the default
  `LPM=SHUTDOWN` the module idles into shutdown, where the UART cannot wake it.
- First time only, a module without the DFU bootloader (e.g. factory firmware
  without `AT+BOOT`) needs the full image (`make ... full`) once, either with
  `make flash-full` (J-Link) or over the same UART through ST's ROM bootloader:
  hold the module's BOOT0 test pad (top side, next to R3) at VDD through ~1 kΩ
  while powering up, run
  `python Tools/rom_uart_flash.py --port COM3 --wait 240 --flash build/argos-smd-at-kineis-firmware_full.bin`,
  then release BOOT0 and power-cycle. FLASH_USER (credentials) is never erased.
- Both flashers write the vector table (first 240 B) **last**. If an update is
  interrupted (power loss), the first flash word is still erased, so at the next
  power-up the STM32WL boots ST's ROM bootloader by itself (empty-flash rule) —
  recover with the `rom_uart_flash.py` command above, no BOOT0 needed.

#### Using VS Code Tasks
- **Build Full (App + Bootloader)** - Compile everything
- **Flash Full (App + Bootloader)** - Build and flash combined image
- **Generate DFU Binary** - Create DFU update file

### Memory Layout

| Region | Address Range | Size | Description |
|--------|---------------|------|-------------|
| Application | `0x08000000` - `0x08032FFF` | 204 KB | Main firmware |
| Bootloader | `0x08033000` - `0x0803AFFF` | 32 KB | DFU bootloader |
| Flash User Page 0 | `0x0803B000` - `0x0803B7FF` | 2 KB | Kineis config (ID, ADDR, key, radio, counters overflow) |
| Flash User Pages 1-4 | `0x0803B800` - `0x0803D7FF` | 8 KB | MSG counter wear-leveling |
| Flash User Pages 5-8 | `0x0803D800` - `0x0803F7FF` | 8 KB | WKU counter wear-leveling |
| NVM Config | `0x0803F800` - `0x0803FFFF` | 2 KB | App config persistence (UW_DOPPLER) |

#### RAM Layout (UW_DOPPLER)

| Section | Memory | Size | Description |
|---------|--------|------|-------------|
| `.retentionRamData` | SRAM2 | Variable | State machine state, LPM context (survives STANDBY) |
| `.retentionRamBss` | SRAM2 | ~2 KB | Event log buffer (survives resets, not power-off) |
| TAMP BKP2R-BKP7R | Backup domain | 24 bytes | Error tracker (survives all resets, not VBAT removal) |

### Clean

```bash
# Clean application only
make clean

# Clean bootloader only
make bootloader-clean

# Clean everything
make clean-all
```

### Unit Tests

Unit tests are in the `Tests/` directory and run on the host (x86) using GCC:

```bash
cd Tests
make        # Build all tests
make run    # Run all tests
```

Test suites:
- `test_crc32` — CRC32 (MPEG-2) implementation correctness
- `test_dfu_protocol` — DFU AT command parsing, address validation, SPI commands
- `test_app_header` — Application header structure validation
- `test_spi_protocol` — SPI protocol framing, timing, state machine
- `test_evtlog` — Event log circular buffer, wrap-around, warm boot preservation
- `test_nvm` — NVM save/load/reset, CRC integrity, struct alignment
- `test_tx_interval` — TX interval exponential growth computation
- `test_memory_safety` — Memory bounds, canary checks, fault injection

## AT Commands

### General Commands (All Modes)

| Command | Mode | Description |
|---------|------|-------------|
| `AT+PING` | Action | Ping (returns `+OK`) |
| `AT+VERSION=?` | Status | AT command protocol version |
| `AT+FW=?` | Status | Firmware build string (app mode, MAC profile, board, comm) |
| `AT+ADDR=?` | Status | Device address (hex) |
| `AT+ADDR=<hex>` | Action | Set device address |
| `AT+ID=?` | Status | Device ID (hex) |
| `AT+ID=<hex>` | Action | Set device ID |
| `AT+SN=?` | Status | Device serial number |
| `AT+SECKEY=?` | Status | Secret key (hex) |
| `AT+SECKEY=<hex>` | Action | Set secret key |
| `AT+RCONF=?` | Status | Radio configuration |
| `AT+RCONF=<hex>` | Action | Set radio configuration |
| `AT+RCONFRAW=?` | Status | Raw radio config (16 bytes hex) |
| `AT+SAVE_RCONF` | Action | Save radio config to flash |
| `AT+LPM=?` | Status | Current low power mode |
| `AT+LPM=<mode>` | Action | Set low power mode |
| `AT+MC=?` | Status | Message counter for next TX |
| `AT+TCXO_WU=?` | Status | TCXO warm-up time (ms) |
| `AT+TCXO_WU=<ms>` | Action | Set TCXO warm-up time |
| `AT+TX=<hex>` | Action | Transmit user data |
| `AT+CW=<params>` | Action | Continuous wave / modulated wave test |
| `AT+UDATE=<epoch>` | Action | Set UTC date/time |
| `AT+KMAC=?` | Status | Current MAC profile |
| `AT+KMAC=<id>` | Action | Set MAC profile |
| `AT+PREPASS_EN=?` | Status | PREVIPASS enable state |
| `AT+BOOT` | Action | Enter DFU bootloader mode |

### UW_DOPPLER Commands (`APP=UW_DOPPLER` only)

These commands are available in addition to the general commands when the firmware is built with `APP=UW_DOPPLER`.

#### AT+SWS — Salt Water Switch Status / Enable

```
AT+SWS=?
+SWS=<state>,<adc>,<air_baseline>,<water_baseline>,<threshold_min>,<enabled>
+OK

AT+SWS=<0|1>
+OK
```

| Field | Type | Description |
|-------|------|-------------|
| `state` | uint | 0=unknown, 1=surface, 2=underwater |
| `adc` | uint | Last raw ADC reading (0-4095) |
| `air_baseline` | uint | Current air baseline |
| `water_baseline` | uint | Current water baseline |
| `threshold_min` | uint | Minimum detection threshold |
| `enabled` | uint | 1=enabled, 0=disabled |

#### AT+SWSCFG — SWS Configuration

```
AT+SWSCFG=?
+SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
+OK

AT+SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
+OK
```

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `thr_min` | uint16 | 0-4095 | 13 | Min valid ADC value (noise floor) |
| `thr_max` | uint16 | 0-4095 | 2000 | Max valid ADC value (saturation) |
| `air_init` | uint16 | 0-4095 | 50 | Initial air baseline |
| `water_init` | uint16 | 0-4095 | 750 | Initial water baseline |
| `interval_ms` | uint32 | >0 | 1000 | Measurement interval (ms) |
| `max_dive_s` | uint32 | — | 7200 | Max dive time before forced surface (s) |
| `min_surf_s` | uint32 | — | 10 | Min surface time before re-submersion (s) |

Validation: `thr_min < thr_max`, `water_init > air_init`, `interval_ms > 0`.

#### AT+SWSFORCE — Force SWS Measurement

```
AT+SWSFORCE
+SWS=<state>,<adc>
+OK
```

Triggers an immediate SWS measurement regardless of the interval timer. Returns the current state and raw ADC value.

#### AT+TXCFG — TX Scheduling Configuration

```
AT+TXCFG=?
+TXCFG=<interval_s>,<growth_pct>,<max_interval_s>,<max_count>
+OK

AT+TXCFG=<interval_s>,<growth_pct>,<max_interval_s>,<max_count>
+OK
```

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `interval_s` | uint16 | 1-65535 | 10 | Initial TX interval (seconds) |
| `growth_pct` | uint8 | 0-255 | 10 | Interval growth percentage per TX |
| `max_interval_s` | uint16 | 1-65535 | 600 | Maximum TX interval cap (seconds) |
| `max_count` | uint8 | 0-255 | 0 | Max TX count per surface (0=unlimited) |

#### AT+DEPLOY — Deploy Mode

```
AT+DEPLOY=?
+DEPLOY=<mode>
+OK

AT+DEPLOY=<0|1>
+OK
```

- `0` = Not deployed: SWS runs but no satellite TX.
- `1` = Deployed: Full operation with satellite TX on surface events.

#### AT+LED — LED Mode

```
AT+LED=?
+LED=<mode>
+OK

AT+LED=<0|1|2>
+OK
```

| Mode | Description |
|------|-------------|
| 0 | LED always off |
| 1 | LED enabled (state-dependent colors) |
| 2 | LED enabled for 24h then auto-off |

Only available on boards with `BSP_HAS_LED_RGB` (SMD_STDALONE). Returns `+LED=N/A` on other boards.

#### AT+LOG — Event Log

```
AT+LOG=?
+LOG=<count>
#000 t=00012345 e=00 s=04 d=0000
#001 t=00012400 e=05 s=04 d=0512
...
+OK

AT+LOG
+OK
```

**Status mode** (`AT+LOG=?`): Dumps all events (oldest first). Each line:
- `#NNN` — entry index
- `t=` — HAL tick (ms since boot)
- `e=` — event type (see table below)
- `s=` — UW_DOPPLER state at time of event
- `d=` — context data

**Action mode** (`AT+LOG`): Clears the event log.

**Event types:**

| Code | Name | Data Field |
|------|------|------------|
| 00 | `EVT_BOOT` | Reset cause (RCC_CSR bits) |
| 01 | `EVT_STATE_CHANGE` | New state |
| 02 | `EVT_REED_ON` | 0 |
| 03 | `EVT_REED_OFF` | Hold duration / 100 (centiseconds) |
| 04 | `EVT_SWS_SURFACE` | ADC value |
| 05 | `EVT_SWS_UNDERWATER` | ADC value |
| 06 | `EVT_TX_START` | TX count |
| 07 | `EVT_TX_DONE` | 0 |
| 08 | `EVT_TX_TIMEOUT` | 0 |
| 09 | `EVT_MAC_READY` | 0 |
| 10 | `EVT_MAC_ERROR` | Error code |
| 11 | `EVT_BAT` | Voltage in mV |
| 12 | `EVT_SHUTDOWN` | 0 |
| 13 | `EVT_TIMEOUT` | Timed-out state |
| 14 | `EVT_ERROR` | Error code |

#### AT+SAVE — Save Configuration to Flash

```
AT+SAVE
+OK
```

Saves all current configuration (TX scheduling, SWS thresholds, deploy mode, LED mode) to flash with CRC32 integrity protection. Configuration is automatically loaded on next boot.

> **Note**: Configuration is also auto-saved during shutdown (reed switch hold). Use `AT+SAVE` after changing settings via `AT+TXCFG`, `AT+SWSCFG`, `AT+DEPLOY`, or `AT+LED` to persist changes immediately.

## Architecture

### Module Overview (UW_DOPPLER)

```
┌──────────────────────────────────────────────────┐
│                  kns_app_uw_doppler.c             │
│             (State machine + TX scheduling)       │
├──────────────────────────────────────────────────┤
│  MGR_SWS   │  MGR_BAT  │  MGR_REED │  MGR_LED   │
│  (Water    │  (Battery │  (Reed    │  (RGB LED  │
│  detection)│  voltage) │  switch)  │  feedback) │
├────────────┼───────────┼───────────┼────────────┤
│  MGR_NVM   │  MGR_ERR  │  MGR_WDG  │ MGR_EVTLOG │
│  (Flash    │  (TAMP    │  (IWDG    │ (SRAM2     │
│  config)   │  errors)  │  16s WDG) │ event log) │
├──────────────────────────────────────────────────┤
│  MGR_AT_CMD (AT command interface over LPUART1)  │
├──────────────────────────────────────────────────┤
│  Kineis Stack (KNS_MAC + KNS_Q + KNS_CFG)       │
├──────────────────────────────────────────────────┤
│  LPM (Low Power Manager: SLEEP / STOP / STANDBY)│
├──────────────────────────────────────────────────┤
│  STM32WL55 HAL + CMSIS                          │
└──────────────────────────────────────────────────┘
```

### Source Tree (UW_DOPPLER-specific)

```
Kineis/App/
├── kns_app_uw_doppler.c          # Main state machine and TX scheduling
├── kns_app_uw_doppler.h          # Public API
└── Managers/
    ├── MGR_AT_CMD/
    │   ├── Inc/mgr_at_cmd_list_uw_doppler.h   # UW_DOPPLER AT command declarations
    │   └── Src/mgr_at_cmd_list_uw_doppler.c   # AT command handlers (SWS, TXCFG, LED, etc.)
    ├── MGR_SWS/
    │   ├── Inc/mgr_sws.h          # SWS API and config struct
    │   └── Src/mgr_sws.c          # Biofouling-adaptive detection algorithm
    ├── MGR_BAT/
    │   ├── Inc/mgr_bat.h          # Battery monitoring API
    │   └── Src/mgr_bat.c          # VBAT/3 + VREFINT ADC measurement
    ├── MGR_NVM/
    │   ├── Inc/mgr_nvm.h          # NVM config struct and API
    │   └── Src/mgr_nvm.c          # Flash read/write with CRC32
    ├── MGR_ERR/
    │   ├── Inc/mgr_err.h          # Error codes and TAMP register layout
    │   └── Src/mgr_err.c          # Reset tracking and error persistence
    ├── MGR_WDG/
    │   ├── Inc/mgr_wdg.h          # Watchdog API
    │   └── Src/mgr_wdg.c          # IWDG direct register configuration
    ├── MGR_EVTLOG/
    │   ├── Inc/mgr_evtlog.h       # Event types and log API
    │   └── Src/mgr_evtlog.c       # SRAM2 circular buffer implementation
    ├── MGR_LED/
    │   ├── Inc/mgr_led.h          # RGB LED API (colors, modes, blink)
    │   └── Src/mgr_led.c          # GPIO driver (active LOW, non-blocking blink)
    └── MGR_REED/
        ├── Inc/mgr_reed.h         # Reed switch API (events, hold duration)
        └── Src/mgr_reed.c         # EXTI interrupt + power latch control
```

### NVM Configuration Structure

The NVM stores all user-configurable parameters in a single flash page with CRC32 protection:

```c
typedef struct {
    uint32_t magic;                     // 0x434F4E46 ("CONF")
    uint8_t  version;                   // Schema version (currently 1)
    uint8_t  deploy_mode;               // 0=not deployed, 1=deployed
    uint8_t  led_mode;                  // MGR_LED_Mode_t
    // TX scheduling
    uint16_t tx_initial_interval_s;     // Default: 10
    uint8_t  tx_growth_percent;         // Default: 10
    uint8_t  tx_max_count;              // Default: 0 (unlimited)
    uint16_t tx_max_interval_s;         // Default: 600
    // SWS detection
    uint16_t sws_threshold_min;         // Default: 13
    uint16_t sws_threshold_max;         // Default: 2000
    uint16_t sws_initial_air_baseline;  // Default: 50
    uint16_t sws_initial_water_baseline;// Default: 750
    uint32_t sws_test_interval_ms;      // Default: 1000
    uint32_t sws_max_dive_time_s;       // Default: 7200
    uint32_t sws_min_surface_time_s;    // Default: 10
    uint8_t  sws_enabled;               // Default: true
    uint32_t crc32;                     // CRC-32/MPEG-2 of all bytes before this field
} NVM_Config_t;
```

### Initialization Order

```
main() →
  HAL_Init()
  SystemClock_Config()
  enable_backup_access()     // Enable TAMP registers
  MX_GPIO_Init()
  MX_LPUART1_UART_Init()    // 9600 baud
  MX_RTC_Init()
  MX_ADC_Init()
  KNS_OS_init()
  LPM_init()
  MGR_ERR_init()             // Log previous reset cause
  MGR_REED_init()            // Reed switch + power latch
  MGR_LED_init()             // RGB LED GPIOs
  KNS_APP_uw_doppler_init()  // NVM load + EVTLOG init + MAC config
  MGR_SWS_init()             // SWS GPIO + ADC + initial calibration (uses NVM config)
  KNS_OS_main()              // Start scheduler
```

## Documentation

### Doxygen

Auto-generated Doxygen documentation can be built with:

```bash
make doc
```

Output is in `Kineis/Doc/krd_fw/html/index.html`.

All public APIs have Doxygen `@file`, `@brief`, `@param`, and `@return` annotations in their header files.

## License

This project is licensed under the GNU General Public License v3. See the `LICENSE` file for details.

## Todo / In Progress

- ~~SPI driver instead of UART~~ Done
- ~~Programming firmware by SPI~~ Done (DFU bootloader)
- ~~UW_DOPPLER autonomous tracker~~ Done (SWS, TX scheduling, NVM, battery, reed shutdown, LPM)
- ~~Event logging~~ Done (SRAM2 retention, AT+LOG)
- ~~Watchdog + error tracking~~ Done (IWDG 16s, TAMP backup registers)
- ~~Unit tests~~ Done (179 tests: CRC32, DFU, SPI, NVM, EVTLOG, TX interval, memory safety)
- Add and manage CLS ID at runtime
