/**
 * @file    cc_tier.c
 * @brief   Battery tier machine — port of Board::txBasePeriodMs() (WS85-Tracker),
 *          plus the CRIT tier below LOW.
 */

#include "cc_tier.h"
#include "cc_conf.h"

void cc_tier_init(cc_tier_state_t *s)
{
	s->tier = CC_TIER_HIGH;
}

bool cc_tier_is_critical(uint16_t batt_mv, bool nimh)
{
	const uint16_t crit_mv = nimh ? CC_BATT_NIMH_CRIT_MV : CC_BATT_CRIT_MV;

	return batt_mv >= CC_BATT_SENSE_MIN_MV && batt_mv < crit_mv;
}

/** Where a LOW unit goes on this reading (promotion needs the margin). */
static cc_tier_t from_low(uint16_t batt_mv, uint16_t hi_mv, uint16_t mid_mv, uint16_t hyst_mv)
{
	if (batt_mv >= hi_mv + hyst_mv)
		return CC_TIER_HIGH;
	if (batt_mv >= mid_mv + hyst_mv)
		return CC_TIER_MID;
	return CC_TIER_LOW;
}

uint32_t cc_tier_update(cc_tier_state_t *s, uint16_t batt_mv, bool nimh)
{
	const uint16_t hi_mv   = nimh ? CC_BATT_NIMH_HIGH_MV : CC_BATT_HIGH_MV;
	const uint16_t mid_mv  = nimh ? CC_BATT_NIMH_MID_MV  : CC_BATT_MID_MV;
	const uint16_t crit_mv = nimh ? CC_BATT_NIMH_CRIT_MV : CC_BATT_CRIT_MV;
	const uint16_t hyst_mv = nimh ? CC_BATT_NIMH_HYST_MV : CC_BATT_HYST_MV;

	if (cc_tier_is_critical(batt_mv, nimh)) {
		s->tier = CC_TIER_CRIT;
	} else {
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
		case CC_TIER_CRIT:
			/* A pack recovering from flat (solar) leaves only with the
			 * margin, so it can't bounce straight back into GPS + TX
			 * load. A failed read is not evidence either way: LOW. */
			if (batt_mv >= CC_BATT_SENSE_MIN_MV && batt_mv < crit_mv + hyst_mv)
				break;
			s->tier = from_low(batt_mv, hi_mv, mid_mv, hyst_mv);
			break;
		case CC_TIER_LOW:
		default:
			s->tier = from_low(batt_mv, hi_mv, mid_mv, hyst_mv);
			break;
		}
	}

	return (s->tier == CC_TIER_HIGH) ? CC_TX_PERIOD_MS
	     : (s->tier == CC_TIER_MID)  ? CC_TX_PERIOD_MID_MS
	     : (s->tier == CC_TIER_LOW)  ? CC_TX_PERIOD_LOW_MS
	                                 : CC_TX_PERIOD_CRIT_MS;
}

const char *cc_tier_name(cc_tier_t t)
{
	static const char *const names[] = { "HIGH", "MID", "LOW", "CRIT" };

	return ((unsigned)t < sizeof(names) / sizeof(names[0])) ? names[t] : "?";
}

uint32_t cc_tier_jittered_period_ms(uint32_t base_ms, uint8_t jitter_pct, uint32_t rand32)
{
	/* max jitter = base * pct / 100; modulo bias negligible (2^32 >> maxMs),
	 * matching the original randomJitterMs(). uint64 keeps base*pct exact. */
	uint32_t max_jitter = (uint32_t)((uint64_t)base_ms * jitter_pct / 100ULL);
	return base_ms + (rand32 % (max_jitter + 1U));
}
