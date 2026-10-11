/**
 * @file    cc_commission.h
 * @brief   CC_TRACKER deployment (commissioning) burst — decision logic.
 *
 * Units are switched on indoors and deployed later, with no button or other
 * external input. The first GPS fix after power-up is the passive "it is
 * outside now" signal:
 *
 *   WAIT_FIX --first valid fix (or AT+COMMISSION=START)--> BURST --30 min--> DONE
 *
 *   WAIT_FIX : normal operation (battery-tier cadence, normal payload), but
 *              for the first CC_COMM_FAST_GPS_MS after power-up GPS is tried
 *              every cycle with one fixed window — no every-Nth-TX schedule,
 *              no backoff — so the burst starts within about a cycle of the
 *              unit seeing sky. After that window the normal GPS schedule
 *              resumes (a unit stored indoors stops burning GPS power), and
 *              a later fix still starts the burst.
 *   BURST    : transmit every CC_COMM_BURST_PERIOD_MS (+ random jitter so
 *              units commissioned together don't stay in lockstep) until
 *              CC_COMM_BURST_MS has elapsed since the burst started; GPS is
 *              skipped (we just got a fix).
 *   DONE     : normal operation. Persisted (by the caller) in a TAMP backup
 *              register: survives watchdog/soft resets, the 14-day reboot and
 *              DFU updates; cleared only when power is actually removed.
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_COMMISSION_H
#define CC_COMMISSION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
	CC_COMM_WAIT_FIX = 0,
	CC_COMM_BURST,
	CC_COMM_DONE,
} cc_comm_phase_t;

typedef struct {
	cc_comm_phase_t phase;
	uint32_t burst_start_ms;  /**< tick when the burst started */
	uint16_t burst_tx;        /**< transmissions completed in the burst */
	uint16_t gps_tries;       /**< GPS attempts while waiting for the first fix */
} cc_comm_t;

/** Boot: DONE when the persisted flag says so, else WAIT_FIX. */
void cc_comm_init(cc_comm_t *c, bool already_done);

/** A valid GPS fix arrived. Returns true when this starts the burst. */
bool cc_comm_on_fix(cc_comm_t *c, uint32_t now_ms);

/** Force the burst (bench / AT+COMMISSION=START). */
void cc_comm_start(cc_comm_t *c, uint32_t now_ms);

/** Mark commissioned without a burst (AT+COMMISSION=SKIP). */
void cc_comm_skip(cc_comm_t *c);

/**
 * @brief A TX attempt finished (done, timeout or error alike).
 * @return true when this ends the burst (caller persists DONE).
 */
bool cc_comm_on_tx(cc_comm_t *c, uint32_t now_ms, uint32_t burst_ms);

/** This cycle's period: burst cadence + jitter during BURST, else normal_ms. */
uint32_t cc_comm_period_ms(const cc_comm_t *c, uint32_t normal_ms, uint32_t burst_period_ms,
                           uint32_t burst_jitter_ms, uint32_t rand32);

/** GPS policy for this cycle. */
typedef enum {
	CC_COMM_GPS_NORMAL = 0, /**< normal every-Nth-TX schedule with backoff */
	CC_COMM_GPS_FAST,       /**< try now, one fixed window, ignore schedule/backoff */
	CC_COMM_GPS_SKIP,       /**< no GPS this cycle (burst running) */
} cc_comm_gps_t;

cc_comm_gps_t cc_comm_gps_policy(const cc_comm_t *c, uint32_t uptime_ms, uint32_t fast_gps_ms);

const char *cc_comm_phase_name(cc_comm_phase_t p);

/**
 * @brief "+COMMISSION=WAIT_FIX,fastgps=1,gps_tries=3\r\n" |
 *        "+COMMISSION=BURST,7/30,left=1380\r\n" | "+COMMISSION=DONE\r\n"
 */
int cc_comm_format(char *buf, size_t n, const cc_comm_t *c, uint32_t now_ms, uint32_t uptime_ms,
                   uint32_t fast_gps_ms, uint32_t burst_ms, uint32_t burst_period_ms);

#endif /* CC_COMMISSION_H */
