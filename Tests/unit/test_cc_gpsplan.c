/**
 * @file    test_cc_gpsplan.c
 * @brief   Unit tests for the GPS power-up timing plan (cc_gpsplan.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_gpsplan.c"

#define HOT_AGE (2UL * 3600UL * 1000UL)
#define HOT_MS  10000UL

/** Never fixed (boot, indoors): head start at cycle start. */
void test_default_early(void)
{
	cc_gpsplan_t g;

	cc_gpsplan_init(&g);
	ASSERT_TRUE(cc_gpsplan_early(&g, 1000, HOT_AGE));
	TEST_PASS();
}

/** Early start, fix waiting the moment we listen -> hot -> next time late. */
void test_early_quick_listen_goes_late(void)
{
	cc_gpsplan_t g;

	cc_gpsplan_init(&g);
	cc_gpsplan_on_fix(&g, 100000, true, 36000, 1200, HOT_MS);
	ASSERT_FALSE(cc_gpsplan_early(&g, 100000 + 21UL * 60000UL, HOT_AGE));
	TEST_PASS();
}

/** Early start but the fix still took a while after listening: stay early. */
void test_early_slow_stays_early(void)
{
	cc_gpsplan_t g;

	cc_gpsplan_init(&g);
	cc_gpsplan_on_fix(&g, 100000, true, 70000, 34000, HOT_MS);
	ASSERT_TRUE(cc_gpsplan_early(&g, 200000, HOT_AGE));
	TEST_PASS();
}

/** Late start: the true TTFF decides (hot keeps it late, slow sends it early). */
void test_late_ttff_decides(void)
{
	cc_gpsplan_t g;

	cc_gpsplan_init(&g);
	cc_gpsplan_on_fix(&g, 0, true, 36000, 900, HOT_MS);        /* -> late */
	cc_gpsplan_on_fix(&g, 1000000, false, 2500, 2500, HOT_MS); /* hot again */
	ASSERT_FALSE(cc_gpsplan_early(&g, 2000000, HOT_AGE));
	cc_gpsplan_on_fix(&g, 3000000, false, 28000, 28000, HOT_MS); /* warm start */
	ASSERT_TRUE(cc_gpsplan_early(&g, 3100000, HOT_AGE));
	TEST_PASS();
}

/** A missed window, or a last fix too old to predict a hot start: early. */
void test_miss_or_age_goes_early(void)
{
	cc_gpsplan_t g;

	cc_gpsplan_init(&g);
	cc_gpsplan_on_fix(&g, 0, false, 1500, 1500, HOT_MS);
	ASSERT_FALSE(cc_gpsplan_early(&g, HOT_AGE - 1, HOT_AGE));
	ASSERT_TRUE(cc_gpsplan_early(&g, HOT_AGE, HOT_AGE));         /* aged out */
	cc_gpsplan_on_fix(&g, 0, false, 1500, 1500, HOT_MS);
	cc_gpsplan_on_miss(&g);
	ASSERT_TRUE(cc_gpsplan_early(&g, 1000, HOT_AGE));
	TEST_PASS();
}

/** Age test is wrap-safe across the 32-bit tick. */
void test_tick_wrap(void)
{
	cc_gpsplan_t g;
	uint32_t t = 0xFFFFFFFFUL - 5000UL;

	cc_gpsplan_init(&g);
	cc_gpsplan_on_fix(&g, t, false, 1500, 1500, HOT_MS);
	ASSERT_FALSE(cc_gpsplan_early(&g, t + 60000UL, HOT_AGE));    /* wrapped, recent */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER GPS power-up plan");
	RUN_TEST(test_default_early);
	RUN_TEST(test_early_quick_listen_goes_late);
	RUN_TEST(test_early_slow_stays_early);
	RUN_TEST(test_late_ttff_decides);
	RUN_TEST(test_miss_or_age_goes_early);
	RUN_TEST(test_tick_wrap);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
