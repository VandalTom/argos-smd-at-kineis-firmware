/**
 * @file    cc_sleep.h
 * @brief   CC_TRACKER deep sleep: STOP2 until an absolute HAL-tick deadline.
 *
 * Ported (trimmed) from MGR_LPM_UW_enterStop2TimedMs, which cannot be linked
 * into CC (UW-only, stubs without BSP_HAS_PWR_LATCH, pulls SWS/REED/PMLOG).
 *
 * Wake sources: the RTC wake-up timer (the deadline) and a falling edge on
 * PA3 = console RX (the first byte is lost — the host sends a CR/LF
 * preamble). A console wake holds the unit awake CC_CONSOLE_EDGE_HOLD_MS for
 * the host's line; every console line holds it CC_CONSOLE_AWAKE_MS so a
 * dialogue stays interactive. Edge wakes that never bring a line (noise on
 * an unconnected RX) trip the storm guard in cc_rxwake.h.
 *
 * HAL_GetTick() is corrected from the RTC across the sleep (lpm.c
 * LPM_saveRtcTime / LPM_compensateTick), so every tick-based timer in the
 * tracker keeps wall-clock time. The IWDG must be frozen in STOP (option
 * byte, MGR_WDG_ensureIwdgStopOptionByte) — a STOP2 longer than its 16 s
 * window would otherwise reset the unit.
 */

#ifndef CC_SLEEP_H
#define CC_SLEEP_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Sleep in STOP2 until HAL_GetTick() reaches @p deadline (capped at
 *        CC_STOP2_MAX_MS per call; the caller simply calls again).
 * @return true when STOP2 was entered (clocks/peripherals restored on return),
 *         false when a gate refused: too close to the deadline, console
 *         active, an event or AT line pending, or the MAC using the radio.
 */
bool CC_SLEEP_stop2Until(uint32_t deadline);

/** Keep the unit out of STOP2 for @p ms (console activity). */
void CC_SLEEP_holdAwake(uint32_t ms);

/** A console line arrived: a host is present. Holds the unit awake
 *  CC_CONSOLE_AWAKE_MS and clears the RX-wake storm guard (cc_rxwake.h);
 *  a bare RX-edge wake alone only holds CC_CONSOLE_EDGE_HOLD_MS. */
void CC_SLEEP_consoleLine(void);

/** Power-step bench build only (CC_POWER_STEPS): hold P9 (PC0, the PA
 *  supply enable) HIGH through the following STOP2s instead of low. */
void CC_SLEEP_benchPaEn(bool on);

/** STOP2 accounting since boot (RTC-measured). */
uint32_t CC_SLEEP_totalMs(void);
uint32_t CC_SLEEP_count(void);

#endif /* CC_SLEEP_H */
