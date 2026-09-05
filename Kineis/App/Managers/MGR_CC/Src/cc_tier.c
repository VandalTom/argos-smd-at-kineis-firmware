/**
 * @file    cc_tier.c
 * @brief   Battery tier machine — port of Board::txBasePeriodMs() (WS85-Tracker).
 */

#include "cc_tier.h"
#include "cc_conf.h"

void cc_tier_init(cc_tier_state_t *s)
{
	s->tier = CC_TIER_HIGH;
}

uint32_t cc_tier_update(cc_tier_state_t *s, uint16_t batt_mv, bool nimh)
{
	const uint16_t hi_mv   = nimh ? CC_BATT_NIMH_HIGH_MV : CC_BATT_HIGH_MV;
	const uint16_t mid_mv  = nimh ? CC_BATT_NIMH_MID_MV  : CC_BATT_MID_MV;
	const uint16_t hyst_mv = nimh ? CC_BATT_NIMH_HYST_MV : CC_BATT_HYST_MV;

	switch (s->tier) {
	case CC_TIER_HIGH:
		if (batt_mv < mid_mv)
			s->tier = CC_TIER_LOW;
		else if (batt_mv < hi_mv)
			s->tier = CC_TIER_MID;
		break;
	case CC_TIER_MID:
		if (batt_mv < mid_mv)
			s->tier = CC_TIER_LOW;
		else if (batt_mv >= hi_mv + hyst_mv)
			s->tier = CC_TIER_HIGH;
		break;
	case CC_TIER_LOW:
	default:
		if (batt_mv >= hi_mv + hyst_mv)
			s->tier = CC_TIER_HIGH;
		else if (batt_mv >= mid_mv + hyst_mv)
			s->tier = CC_TIER_MID;
		break;
	}

	return (s->tier == CC_TIER_HIGH) ? CC_TX_PERIOD_MS
	     : (s->tier == CC_TIER_MID)  ? CC_TX_PERIOD_MID_MS
	                                 : CC_TX_PERIOD_LOW_MS;
}

uint32_t cc_tier_jittered_period_ms(uint32_t base_ms, uint8_t jitter_pct, uint32_t rand32)
{
	/* max jitter = base * pct / 100; modulo bias negligible (2^32 >> maxMs),
	 * matching the original randomJitterMs(). uint64 keeps base*pct exact. */
	uint32_t max_jitter = (uint32_t)((uint64_t)base_ms * jitter_pct / 100ULL);
	return base_ms + (rand32 % (max_jitter + 1U));
}
