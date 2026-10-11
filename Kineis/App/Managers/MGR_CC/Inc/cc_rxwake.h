/**
 * @file    cc_rxwake.h
 * @brief   Console RX-wake storm guard for STOP2 (pure logic, host-tested).
 *
 * A falling edge on PA3 (console RX) wakes the unit so a host can talk to a
 * sleeping tracker. With nothing connected the line only has the internal
 * pull-up, and on UW_DOPPLER that was bench-confirmed NOT to hold it idle
 * against ambient EMI: noise edges kept waking the unit and it never reached
 * STOP2. Here a bare edge only holds the unit awake briefly; a real console
 * line proves a host. After @c storm_count edge wakes with no line in
 * between, the RX wake is disarmed for a backoff (doubling up to a cap,
 * reset by the next real line). While disarmed the console still answers
 * whenever the unit is awake for its own cycle.
 */

#ifndef CC_RXWAKE_H
#define CC_RXWAKE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint8_t  streak;      /**< edge wakes since the last console line */
	uint32_t backoff_ms;  /**< current backoff length; 0 = none taken yet */
	uint32_t off_until;   /**< tick when the wake re-arms (valid when off) */
	bool     off;         /**< RX wake disarmed */
} cc_rxwake_t;

void cc_rxwake_init(cc_rxwake_t *w);

/** Should the next STOP2 arm the PA3 wake? Re-arms once the backoff ends. */
bool cc_rxwake_armed(cc_rxwake_t *w, uint32_t now_ms);

/**
 * @brief An RX edge woke the unit.
 * @return true = plausibly a host: stay awake for a line;
 *         false = this edge completed a storm: the wake is now disarmed.
 */
bool cc_rxwake_on_edge(cc_rxwake_t *w, uint32_t now_ms, uint8_t storm_count,
	uint32_t backoff_ms, uint32_t backoff_max_ms);

/** A real console line arrived: a host is there, forget the storm history. */
void cc_rxwake_on_line(cc_rxwake_t *w);

#endif /* CC_RXWAKE_H */
