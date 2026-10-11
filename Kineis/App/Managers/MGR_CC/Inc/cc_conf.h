/**
 * @file    cc_conf.h
 * @brief   CC_TRACKER mission configuration — compile-time, by design.
 *
 * Ported from CounterCurrent WS85-Tracker board_config.h (XIAO nRF52840 host).
 * Pin assignments live in the BSP (bsp_cc_carrier.h), not here. Everything in
 * this header is pure constants: it is included by the host-side unit tests,
 * so it must not pull in any HAL or MCU header.
 *
 * Staged bring-up: as on the Arduino original, each sensor compiles out
 * entirely when its CC_ENABLE_* switch is 0 (no pins driven, no session run),
 * so power/bench characterisation can add one module at a time.
 */

#ifndef CC_CONF_H
#define CC_CONF_H

/* ---- Staged bring-up switches ------------------------------------------ */
#define CC_ENABLE_WS85      1   /**< WS85 wind/rain sampling (USART1 session B) */
#define CC_ENABLE_GPS       1   /**< GPS fix acquisition (USART1 session A) */
#define CC_ENABLE_ENV_I2C   1   /**< env sensors on I2C2 (always-powered rail) */
#define CC_ENABLE_ADC_SENSE 1   /**< Supercap + NiMH via internal ADC */

/* Per-chip env gates. The CC carrier fits ONLY a BME280 (temp + pressure +
 * humidity, ~0.1 uA sleeping on the always-on rail); the SHT45/AHT20 paths
 * from the XIAO design are compiled out but kept portable. */
#define CC_ENABLE_BMP280    1   /**< BMP280/BME280 (auto-detects which) */
#define CC_ENABLE_SHT45     0
#define CC_ENABLE_AHT20     0

/* ---- Timing (mission cadence) ------------------------------------------ */
/* Base publish period per battery tier. One full cycle = WS85 sample ->
 * env sample -> GPS fix (every Nth cycle) -> pack -> TX, then wait out the
 * remainder of the period. Each cycle's actual period gets +0..CC_TX_JITTER_PCT%
 * of hardware-RNG jitter so TX times don't phase-lock against the Kineis
 * satellite pass schedule. */
#define CC_TX_PERIOD_MS         420000UL  /**< 7 min base (bench: 60000UL) */
#define CC_TX_PERIOD_MID_MS     900000UL  /**< 15 min base (mid battery tier) */
#define CC_TX_PERIOD_LOW_MS     1800000UL /**< 30 min base (low battery tier) */
#define CC_TX_JITTER_PCT        30        /**< max random period stretch, % of tier base */

/* ---- Battery tiers (two threshold sets, picked by measurement source) ---
 * NiMH set: the 2S NiMH pack read via the gated divider (MGR_BAT / ADC_IN0).
 * VBAT set: fallback when the NiMH read fails — module supply, LiPo-ish curve.
 * Demotion is immediate at the raw threshold; promotion needs the hysteresis
 * margin so a straddling reading can't flap the cadence. A failed read (~0 mV)
 * lands LOW — conservative. */
#define CC_BATT_HIGH_MV         4000      /**< VBAT set: above = full rate */
#define CC_BATT_MID_MV          3600      /**< VBAT set: above = mid; below = low */
#define CC_BATT_HYST_MV         100       /**< VBAT set: margin to move UP a tier */
#define CC_BATT_NIMH_HIGH_MV    2450      /**< NiMH set: above = full rate (plateau) */
#define CC_BATT_NIMH_MID_MV     2250      /**< NiMH set: above = mid; below = low (knee) */
#define CC_BATT_NIMH_HYST_MV    50        /**< NiMH set: smaller margin, flat plateau */
/* CRIT tier (below LOW): no GPS, no deployment burst, one TX per
 * CC_TX_PERIOD_CRIT_MS so the pack can recover (solar) instead of browning
 * out under PA/GPS load — the packets keep reporting its voltage. Entered only
 * on a plausible reading: below CC_BATT_SENSE_MIN_MV the sense is treated as
 * failed/unfitted and the unit stays LOW, never quieter. Leaving needs the
 * set's hysteresis margin. */
#define CC_BATT_CRIT_MV         3400      /**< VBAT set: below = critical */
#define CC_BATT_NIMH_CRIT_MV    2100      /**< NiMH set: 1.05 V/cell */
#define CC_BATT_SENSE_MIN_MV    1000      /**< below = no reading, not a flat pack */
#define CC_TX_PERIOD_CRIT_MS    (2UL * 3600UL * 1000UL) /**< 2 h base (CRIT tier) */

/* ---- WS85 --------------------------------------------------------------- */
#define CC_WS85_BAUD            115200UL  /**< sensor's stored link baud */
#define CC_WS85_WAIT_MS         40000UL   /**< max wait for one CRC-valid broadcast
                                           *   (~30 s period on battery + margin) */
#define CC_WS85_MAX_AGE_MS      60000UL   /**< cache max age before re-sampling
                                           *   (TESTING value; field: 30 min) */
#define CC_WS85_ADDR            0x85      /**< Modbus slave address */

/* ---- GPS ---------------------------------------------------------------- */
/* The carrier routes the receiver's NMEA output to PB6 — USART1's TX-capable
 * pad — so the GPS session enables the WL55 UART's CR2.SWAP feature to make
 * PB6 the receive pin. The session is RX-only (no wire toward the GPS). */
#define CC_GPS_UART_SWAP        1
#define CC_GPS_BAUD             9600UL
#define CC_GPS_TTFF_BUDGET_MS   90000UL   /**< one search window */
#define CC_GPS_MAX_DURATIONS    6         /**< adaptive ramp: max chained windows
                                           *   (6 x 90 s = 9 min ephemeris download) */
#define CC_GPS_FIX_EVERY_N_TX   3         /**< scheduled fix every Nth transmit */
/* Sky check (power): GPS is the dominant consumer, and indoors a window can
 * never succeed. After CC_GPS_SKY_CHECK_MS, if the receiver's GSV reports
 * fewer than CC_GPS_SKY_MIN_SATS satellites at >= CC_GPS_SKY_MIN_CNO dBHz, the
 * window ends early (counted as a miss). Receivers that send no GSV are never
 * cut short. Outdoors several satellites exceed 20 dBHz within seconds. */
#define CC_GPS_SKY_CHECK_MS     45000UL
#define CC_GPS_SKY_MIN_SATS     2
#define CC_GPS_SKY_MIN_CNO      20        /**< dBHz */

/* ---- Env sensors (I2C2) -------------------------------------------------- */
#define CC_ENV_MAX_AGE_MS       60000UL   /**< shared cache max age (TESTING: 1 min) */
#define CC_BMP280_ADDR          0         /**< 0 = auto-probe 0x76 then 0x77 */
#define CC_SHT45_ADDR           0x44      /**< SHT45-AD1B variants use 0x45/0x46 */

/* ---- ADC sense ----------------------------------------------------------- */
/* Supercap on PB4 = ADC_IN3 through a 1M:1M divider (actual = 2x pin voltage).
 * NiMH on PB13 = ADC_IN0 through the MGR_BAT gated divider (ratio set by the
 * MGR_BAT_DIV_R_*_KOHM overrides in the BSP). */
#define CC_SCAP_ADC_CHANNEL     3         /**< ADC_IN3 (PB4) */
#define CC_SCAP_DIV_NUM         2         /**< 1M:1M divider -> x2 */
#define CC_SCAP_DIV_DEN         1

/* ---- TX ------------------------------------------------------------------ */
#define CC_TX_RESULT_TIMEOUT_MS 180000UL  /**< wait for MAC TX_DONE (3 min) */
#define CC_MAC_READY_TIMEOUT_MS 30000UL
#define CC_MAC_INIT_MAX_RETRIES 3

/* ---- Self-test (boot + AT+SELFTEST; see cc_selftest.h) ---------------------
 * Production-line fast: GPS proves NMEA arrives (no fix), TX passes on the
 * MAC's TX_DONE (no satellite pass). Runs once per boot, after MAC ready; a
 * boot run then continues straight into the first normal cycle. */
#define CC_SELFTEST_AT_BOOT      1
#define CC_SELFTEST_BOOT_ITEMS   0x1Fu     /**< GPS|WS85|ENV|ADC|TX (cc_selftest.h bits) */
#define CC_SELFTEST_GPS_MS       5000UL    /**< first checksum-valid NMEA sentence */
#define CC_SELFTEST_WS85_MS      35000UL   /**< one broadcast (~30 s period) */
#define CC_SELFTEST_TX_MS        60000UL   /**< MAC TX_DONE */
#define CC_SELFTEST_NIMH_MIN_MV  1800      /**< 2S NiMH: 0.9 V/cell floor */
#define CC_SELFTEST_NIMH_MAX_MV  3300      /**< 2S NiMH: 1.65 V/cell (charging) ceiling */

/* ---- Commissioning (deployment burst; see cc_commission.h) -----------------
 * First GPS fix since power-up -> TX every minute for 30 min, then normal. */
#define CC_COMM_BURST_MS         (30UL * 60UL * 1000UL)
#define CC_COMM_BURST_PERIOD_MS  60000UL
#define CC_COMM_BURST_JITTER_MS  5000UL    /**< 0..5 s added per burst cycle */
#define CC_COMM_FAST_GPS_MS      (48UL * 3600UL * 1000UL) /**< GPS every cycle until then */
#define CC_COMM_GPS_WINDOW_MS    120000UL  /**< fixed window per fast attempt */
#define CC_COMM_DONE_MAGIC       0xC0331551UL /**< TAMP BKP12R value = commissioned */

/* ---- Credentials --------------------------------------------------------------
 * MGR_CRED keeps a CRC-protected mirror of ID/ADDR/SECKEY (page 0x08032800)
 * and restores page 0 from it at boot. With blank credentials and no mirror
 * the libkineis getters would silently use a TEST identity, so a unit would
 * transmit under an ID nobody receives: instead the tracker never starts the
 * MAC, sits in NO_CREDS with the console up (sensor self-test still runs) and
 * resumes as soon as ID, ADDR and SECKEY have been written over the console.
 * 1 = engineering benches may transmit under the test identity. */
#define CC_ALLOW_TEST_CREDS    0

/* ---- Deep sleep (cc_sleep.c) ----------------------------------------------
 * Between cycles the MCU sleeps in STOP2 until the next cycle's deadline
 * (RTC wake-up timer, HAL tick RTC-compensated). Every other wait (sensor
 * sessions, TX) idles with WFI + SysTick running. Needs the IWDG frozen in
 * STOP (option byte, programmed once on first boot). */
#define CC_USE_STOP2           1
#define CC_STOP2_MIN_MS        2000UL      /**< shorter waits just WFI */
#define CC_STOP2_MAX_MS        3600000UL   /**< per call; WUT range and the
                                                <24 h tick compensation */
#define CC_CONSOLE_AWAKE_MS    30000UL     /**< no STOP2 this long after console
                                                activity or a console-RX wake */

/* ---- Rescue mode (keeps the UART update path alive through a bad build) ----
 * Every boot bumps a counter in TAMP BKP13R; a boot that stays up
 * CC_BOOT_STABLE_MS clears it, and so do deliberate resets (AT+RESET, DFU
 * entry). After CC_RESCUE_AFTER_FAILED_BOOTS failed boots in a row the next
 * boot skips sensors and radio and runs only the console (AT+BOOT, AT+RESET,
 * queries, credential writes) for CC_RESCUE_WINDOW_MS, then retries normally.
 * Power loss clears the counter. */
#define CC_BOOT_STABLE_MS            90000UL
#define CC_RESCUE_AFTER_FAILED_BOOTS 3
#define CC_RESCUE_WINDOW_MS          (10UL * 60UL * 1000UL)
#define CC_BOOT_COUNT_MAGIC          0xB0070000UL /**< BKP13R high half; low half = count */

/* ---- Payload timestamp epoch --------------------------------------------- */
#define CC_TS_EPOCH_UNIX        1735689600UL /**< 2025-01-01T00:00:00Z */
#define CC_TZ_OFFSET_MIN        0            /**< 0 = UTC */

/* ---- Robustness ----------------------------------------------------------- */
/* Graceful self-reboot after 14 days, end-of-cycle only: keeps every
 * HAL_GetTick()-based age and the software clock far from the 49.7-day wrap.
 * Costs one cold cycle (sensor caches lost); the GPS time survives in the
 * RTC-anchored clock (cc_clock.h), so timestamps stay valid. */
#define CC_REBOOT_UPTIME_MS     (14UL * 24UL * 3600UL * 1000UL)

#endif /* CC_CONF_H */
