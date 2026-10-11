/**
 * @file    cc_sleep.h
 * @brief   CC_TRACKER deep sleep: STOP2 until an absolute HAL-tick deadline.
 *
 * Ported (trimmed) from MGR_LPM_UW_enterStop2TimedMs, which cannot be linked
 * into CC (UW-only, stubs without BSP_HAS_PWR_LATCH, pulls SWS/REED/PMLOG).
 *
 * Wake sources: the RTC wake-up timer (the deadline) and a falling edge on
 * PA3 = console RX (the first byte is lost — the host sends a CR/LF
 * preamble). After a console wake, or any console line, the unit stays out
 * of STOP2 for CC_CONSOLE_AWAKE_MS so a dialogue stays interactive.
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

/** STOP2 accounting since boot (RTC-measured). */
uint32_t CC_SLEEP_totalMs(void);
uint32_t CC_SLEEP_count(void);

#endif /* CC_SLEEP_H */
