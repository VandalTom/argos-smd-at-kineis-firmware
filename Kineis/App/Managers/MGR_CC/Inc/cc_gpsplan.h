/**
 * @file    cc_gpsplan.h
 * @brief   When to power the GPS within a cycle (pure logic, host-tested).
 *
 * The cycle is: battery -> [GPS on] -> WS85 -> env -> [GPS on] -> listen for a
 * fix -> TX. USART1 serves one sensor at a time, but the receiver needs no
 * listener to acquire, so powering it at cycle start overlaps its time to
 * first fix with the WS85 wait (up to ~35 s) and shortens the cycle.
 *
 * That head start only pays when the fix takes a while. A receiver that
 * hot-starts (ephemeris kept, fix in ~1-2 s) would just burn ~25 mA through
 * the whole WS85 wait. So the plan learns: it starts EARLY by default, and
 * LATE (after the sensors) only once a recent fix has proven the receiver
 * hot. A fix that took long, a missed window, or an old last fix send it back
 * to EARLY.
 */

#ifndef CC_GPSPLAN_H
#define CC_GPSPLAN_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool     hot;          /**< the last attempt proved a quick (hot) start */
	bool     ever_fixed;
	uint32_t last_fix_ms;  /**< tick of the last fix */
} cc_gpsplan_t;

void cc_gpsplan_init(cc_gpsplan_t *g);

/**
 * @brief Power the receiver at cycle start (true) or after the sensors (false).
 * @param now_ms     current tick
 * @param hot_age_ms a fix older than this no longer predicts a hot start
 *                   (broadcast ephemeris ages out after ~4 h)
 */
bool cc_gpsplan_early(const cc_gpsplan_t *g, uint32_t now_ms, uint32_t hot_age_ms);

/**
 * @brief Record a fix.
 * @param early           this attempt powered up at cycle start
 * @param since_power_ms  power-up -> fix (true TTFF for a late start)
 * @param since_listen_ms UART opened -> fix (an early start can't see the
 *                        true TTFF: a fix right after listening means it was
 *                        already waiting)
 * @param hot_ms          a quick fix: TTFF (late) or listen delay (early) below this
 */
void cc_gpsplan_on_fix(cc_gpsplan_t *g, uint32_t now_ms, bool early,
	uint32_t since_power_ms, uint32_t since_listen_ms, uint32_t hot_ms);

/** Record a window that ended without a fix. */
void cc_gpsplan_on_miss(cc_gpsplan_t *g);

#endif /* CC_GPSPLAN_H */
