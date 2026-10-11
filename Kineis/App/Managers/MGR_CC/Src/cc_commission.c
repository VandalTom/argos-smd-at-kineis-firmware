/**
 * @file    cc_commission.c
 * @brief   CC_TRACKER deployment burst decision logic.
 */

#include <stdio.h>
#include "cc_commission.h"

void cc_comm_init(cc_comm_t *c, bool already_done)
{
	c->phase = already_done ? CC_COMM_DONE : CC_COMM_WAIT_FIX;
	c->burst_start_ms = 0;
	c->burst_tx = 0;
	c->gps_tries = 0;
}

void cc_comm_start(cc_comm_t *c, uint32_t now_ms)
{
	c->phase = CC_COMM_BURST;
	c->burst_start_ms = now_ms;
	c->burst_tx = 0;
}

bool cc_comm_on_fix(cc_comm_t *c, uint32_t now_ms)
{
	if (c->phase != CC_COMM_WAIT_FIX)
		return false;
	cc_comm_start(c, now_ms);
	return true;
}

void cc_comm_skip(cc_comm_t *c)
{
	c->phase = CC_COMM_DONE;
}

bool cc_comm_on_tx(cc_comm_t *c, uint32_t now_ms, uint32_t burst_ms)
{
	if (c->phase != CC_COMM_BURST)
		return false;
	c->burst_tx++;
	if (now_ms - c->burst_start_ms >= burst_ms) {
		c->phase = CC_COMM_DONE;
		return true;
	}
	return false;
}

uint32_t cc_comm_period_ms(const cc_comm_t *c, uint32_t normal_ms, uint32_t burst_period_ms,
                           uint32_t burst_jitter_ms, uint32_t rand32)
{
	if (c->phase != CC_COMM_BURST)
		return normal_ms;
	return burst_period_ms + (rand32 % (burst_jitter_ms + 1u));
}

cc_comm_gps_t cc_comm_gps_policy(const cc_comm_t *c, uint32_t uptime_ms, uint32_t fast_gps_ms)
{
	if (c->phase == CC_COMM_BURST)
		return CC_COMM_GPS_SKIP;
	if (c->phase == CC_COMM_WAIT_FIX && uptime_ms < fast_gps_ms)
		return CC_COMM_GPS_FAST;
	return CC_COMM_GPS_NORMAL;
}

const char *cc_comm_phase_name(cc_comm_phase_t p)
{
	switch (p) {
	case CC_COMM_WAIT_FIX: return "WAIT_FIX";
	case CC_COMM_BURST:    return "BURST";
	default:               return "DONE";
	}
}

int cc_comm_format(char *buf, size_t n, const cc_comm_t *c, uint32_t now_ms, uint32_t uptime_ms,
                   uint32_t fast_gps_ms, uint32_t burst_ms, uint32_t burst_period_ms)
{
	switch (c->phase) {
	case CC_COMM_WAIT_FIX:
		return snprintf(buf, n, "+COMMISSION=WAIT_FIX,fastgps=%u,gps_tries=%u\r\n",
		                uptime_ms < fast_gps_ms ? 1u : 0u, (unsigned)c->gps_tries);
	case CC_COMM_BURST: {
		uint32_t el = now_ms - c->burst_start_ms;
		uint32_t left_s = el < burst_ms ? (burst_ms - el) / 1000u : 0u;

		return snprintf(buf, n, "+COMMISSION=BURST,%u/%lu,left=%lu\r\n", (unsigned)c->burst_tx,
		                (unsigned long)(burst_ms / burst_period_ms), (unsigned long)left_s);
	}
	default:
		return snprintf(buf, n, "+COMMISSION=DONE\r\n");
	}
}
