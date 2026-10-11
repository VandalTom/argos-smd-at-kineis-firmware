/**
 * @file    test_cc_tier.c
 * @brief   Unit tests for the CC_TRACKER battery tier machine (cc_tier.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_tier.c"

/** Boot at HIGH; healthy LiPo-set battery stays HIGH. */
void test_boot_high(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_MS, cc_tier_update(&s, 4100, false));
	ASSERT_EQ(CC_TIER_HIGH, s.tier);
	TEST_PASS();
}

/** Demotion is immediate at the raw thresholds. */
void test_demotion_immediate(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_MID_MS, cc_tier_update(&s, CC_BATT_HIGH_MV - 1, false));
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, CC_BATT_MID_MV - 1, false));
	TEST_PASS();
}

/** Promotion needs the hysteresis margin — a straddling reading can't flap. */
void test_promotion_hysteresis(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	(void)cc_tier_update(&s, CC_BATT_MID_MV - 1, false); /* -> LOW */
	ASSERT_EQ(CC_TIER_LOW, s.tier);

	/* Just under MID + hyst: stays LOW */
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS,
		cc_tier_update(&s, CC_BATT_MID_MV + CC_BATT_HYST_MV - 1, false));
	/* MID + hyst promotes one tier */
	ASSERT_EQ(CC_TX_PERIOD_MID_MS,
		cc_tier_update(&s, CC_BATT_MID_MV + CC_BATT_HYST_MV, false));
	/* Exactly at HIGH from MID: not enough (needs + hysteresis) — stays MID */
	ASSERT_EQ(CC_TX_PERIOD_MID_MS, cc_tier_update(&s, CC_BATT_HIGH_MV, false));
	/* HIGH + hyst promotes to full rate */
	ASSERT_EQ(CC_TX_PERIOD_MS,
		cc_tier_update(&s, CC_BATT_HIGH_MV + CC_BATT_HYST_MV, false));
	TEST_PASS();
}

/** LOW -> HIGH directly when the reading clears HIGH + hysteresis. */
void test_low_to_high_direct(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	(void)cc_tier_update(&s, 0, false); /* failed read -> LOW */
	ASSERT_EQ(CC_TIER_LOW, s.tier);
	ASSERT_EQ(CC_TX_PERIOD_MS,
		cc_tier_update(&s, CC_BATT_HIGH_MV + CC_BATT_HYST_MV, false));
	TEST_PASS();
}

/** A failed battery read (~0 mV) lands LOW — conservative. */
void test_failed_read_lands_low(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, 0, false));
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, 0, true));
	TEST_PASS();
}

/** The NiMH threshold set: plateau ~2.47 V is full rate, knee demotes. */
void test_nimh_set(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_MS, cc_tier_update(&s, 2470, true));
	ASSERT_EQ(CC_TX_PERIOD_MID_MS, cc_tier_update(&s, CC_BATT_NIMH_HIGH_MV - 1, true));
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, CC_BATT_NIMH_MID_MV - 1, true));
	/* NiMH hysteresis is the smaller 50 mV margin */
	ASSERT_EQ(CC_TX_PERIOD_MS,
		cc_tier_update(&s, CC_BATT_NIMH_HIGH_MV + CC_BATT_NIMH_HYST_MV, true));
	TEST_PASS();
}

/** A measured-and-flat pack drops to CRIT from any tier, immediately. */
void test_crit_entry(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_CRIT_MS, cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV - 1, true));
	ASSERT_EQ(CC_TIER_CRIT, s.tier);

	cc_tier_init(&s);
	(void)cc_tier_update(&s, CC_BATT_NIMH_MID_MV - 1, true); /* -> LOW */
	ASSERT_EQ(CC_TIER_LOW, s.tier);
	ASSERT_EQ(CC_TX_PERIOD_CRIT_MS, cc_tier_update(&s, CC_BATT_SENSE_MIN_MV, true));

	/* VBAT set has its own threshold */
	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_CRIT_MS, cc_tier_update(&s, CC_BATT_CRIT_MV - 1, false));
	TEST_PASS();
}

/** A failed or unfitted sense is never taken for a flat pack: LOW, not CRIT. */
void test_crit_needs_plausible_reading(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, 0, true));
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, CC_BATT_SENSE_MIN_MV - 1, true));
	ASSERT_FALSE(cc_tier_is_critical(0, true));
	ASSERT_FALSE(cc_tier_is_critical(CC_BATT_SENSE_MIN_MV - 1, true));
	ASSERT_TRUE(cc_tier_is_critical(CC_BATT_SENSE_MIN_MV, true));
	ASSERT_FALSE(cc_tier_is_critical(CC_BATT_NIMH_CRIT_MV, true));

	/* ...and a failed read while CRIT is not evidence of a flat pack */
	(void)cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV - 1, true);
	ASSERT_EQ(CC_TIER_CRIT, s.tier);
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS, cc_tier_update(&s, 0, true));
	TEST_PASS();
}

/** Recovery needs the margin, then promotes like LOW (straight to HIGH ok). */
void test_crit_recovery_hysteresis(void)
{
	cc_tier_state_t s;

	cc_tier_init(&s);
	(void)cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV - 1, true);
	ASSERT_EQ(CC_TX_PERIOD_CRIT_MS, cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV, true));
	ASSERT_EQ(CC_TX_PERIOD_CRIT_MS,
		cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV + CC_BATT_NIMH_HYST_MV - 1, true));
	ASSERT_EQ(CC_TX_PERIOD_LOW_MS,
		cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV + CC_BATT_NIMH_HYST_MV, true));

	(void)cc_tier_update(&s, CC_BATT_NIMH_CRIT_MV - 1, true);
	ASSERT_EQ(CC_TX_PERIOD_MS,
		cc_tier_update(&s, CC_BATT_NIMH_HIGH_MV + CC_BATT_NIMH_HYST_MV, true));
	TEST_PASS();
}

void test_tier_names(void)
{
	ASSERT_STR_EQ("HIGH", cc_tier_name(CC_TIER_HIGH));
	ASSERT_STR_EQ("CRIT", cc_tier_name(CC_TIER_CRIT));
	ASSERT_STR_EQ("?", cc_tier_name((cc_tier_t)9));
	TEST_PASS();
}

/** Jitter: 0 at rand=0, capped at base*pct/100, base always included. */
void test_jitter_bounds(void)
{
	ASSERT_EQ(420000UL, cc_tier_jittered_period_ms(420000UL, 30, 0));

	uint32_t max_j = 420000UL * 30UL / 100UL; /* 126000 */

	ASSERT_EQ(420000UL + max_j,
		cc_tier_jittered_period_ms(420000UL, 30, max_j));
	/* modulo wraps: rand = max_j+1 -> jitter 0 */
	ASSERT_EQ(420000UL,
		cc_tier_jittered_period_ms(420000UL, 30, max_j + 1));
	/* pct 0 -> always exactly base */
	ASSERT_EQ(420000UL, cc_tier_jittered_period_ms(420000UL, 0, 0xFFFFFFFFUL));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER battery tier + jitter");
	RUN_TEST(test_boot_high);
	RUN_TEST(test_demotion_immediate);
	RUN_TEST(test_promotion_hysteresis);
	RUN_TEST(test_low_to_high_direct);
	RUN_TEST(test_failed_read_lands_low);
	RUN_TEST(test_nimh_set);
	RUN_TEST(test_crit_entry);
	RUN_TEST(test_crit_needs_plausible_reading);
	RUN_TEST(test_crit_recovery_hysteresis);
	RUN_TEST(test_tier_names);
	RUN_TEST(test_jitter_bounds);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
