/**
 * @file    cc_clock.h
 * @brief   CC_TRACKER wall clock: GPS time anchored to the RTC calendar.
 *
 * The RTC calendar keeps counting through every reset (MX_RTC_Init leaves a
 * running calendar alone) and is cleared, together with the TAMP backup
 * registers, only by power loss. A GPS fix stores anchor = Unix - RTC seconds
 * in TAMP (value + complement), so watchdog resets, AT+RESET, updates and the
 * 14-day reboot keep packet timestamps valid instead of sending ts=0 until
 * the next fix. Drift is the LSE crystal's (~2 s/day), re-anchored at every
 * fix; the payload carries minutes.
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_CLOCK_H
#define CC_CLOCK_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Seconds since 2000-01-01 00:00:00 for an RTC calendar reading.
 * @param yy 0..99 (2000..2099)   @param mo 1..12   @param dd 1..31
 * @param hh 0..23   @param mi 0..59   @param ss 0..59
 */
uint32_t cc_clock_rtc_seconds(uint8_t yy, uint8_t mo, uint8_t dd,
	uint8_t hh, uint8_t mi, uint8_t ss);

/** A persisted anchor is trusted only with its complement beside it (a stray
 *  write or a half-done store cannot validate) and never as 0 (reset value). */
bool cc_clock_anchor_valid(uint32_t anchor, uint32_t anchor_inv);

/** GPS time worth anchoring to: not before @p min_epoch (the payload epoch;
 *  rejects receivers reporting 1980/GPS-week-rollover dates). */
bool cc_clock_epoch_plausible(uint32_t epoch, uint32_t min_epoch);

#endif /* CC_CLOCK_H */
