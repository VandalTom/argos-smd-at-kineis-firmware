/**
 * @file    test_cc_rxwake.c
 * @brief   Unit tests for the console RX-wake storm guard (cc_rxwake.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_rxwake.c"

#define STORM   6
#define BACKOFF 1800000UL   /* 30 min */
#define MAX     14400000UL  /* 4 h */

/** A host: preamble edge, then a line - never trips, however often. */
void test_host_never_trips(void)
{
	cc_rxwake_t w;

	cc_rxwake_init(&w);
	for (int i = 0; i < 100; i++) {
		ASSERT_TRUE(cc_rxwake_armed(&w, (uint32_t)i * 1000u));
		ASSERT_TRUE(cc_rxwake_on_edge(&w, (uint32_t)i * 1000u, STORM, BACKOFF, MAX));
		cc_rxwake_on_line(&w);
	}
	TEST_PASS();
}

/** Noise: the storm_count-th bare edge disarms the wake for the backoff. */
void test_storm_disarms_then_rearms(void)
{
	cc_rxwake_t w;

	cc_rxwake_init(&w);
	for (int i = 1; i < STORM; i++)
		ASSERT_TRUE(cc_rxwake_on_edge(&w, 1000u, STORM, BACKOFF, MAX));
	ASSERT_FALSE(cc_rxwake_on_edge(&w, 1000u, STORM, BACKOFF, MAX));
	ASSERT_FALSE(cc_rxwake_armed(&w, 1000u));
	ASSERT_FALSE(cc_rxwake_armed(&w, 1000u + BACKOFF - 1u));
	ASSERT_TRUE(cc_rxwake_armed(&w, 1000u + BACKOFF));
	TEST_PASS();
}

/** Repeated storms double the backoff up to the cap; a line resets it. */
void test_backoff_doubles_and_resets(void)
{
	cc_rxwake_t w;
	uint32_t now = 0;
	const uint32_t expect[] = { BACKOFF, 2 * BACKOFF, 4 * BACKOFF, MAX, MAX };

	cc_rxwake_init(&w);
	for (unsigned k = 0; k < sizeof(expect) / sizeof(expect[0]); k++) {
		for (int i = 0; i < STORM; i++)
			(void)cc_rxwake_on_edge(&w, now, STORM, BACKOFF, MAX);
		ASSERT_EQ(expect[k], w.backoff_ms);
		now = w.off_until;
		ASSERT_TRUE(cc_rxwake_armed(&w, now));
	}
	cc_rxwake_on_line(&w);
	for (int i = 0; i < STORM; i++)
		(void)cc_rxwake_on_edge(&w, now, STORM, BACKOFF, MAX);
	ASSERT_EQ(BACKOFF, w.backoff_ms);
	TEST_PASS();
}

/** A line while disarmed re-arms at once (a host proved itself while the
 *  unit was awake for its cycle). */
void test_line_rearms(void)
{
	cc_rxwake_t w;

	cc_rxwake_init(&w);
	for (int i = 0; i < STORM; i++)
		(void)cc_rxwake_on_edge(&w, 0, STORM, BACKOFF, MAX);
	ASSERT_FALSE(cc_rxwake_armed(&w, 10u));
	cc_rxwake_on_line(&w);
	ASSERT_TRUE(cc_rxwake_armed(&w, 10u));
	TEST_PASS();
}

/** Re-arm time is wrap-safe across the 32-bit tick. */
void test_tick_wrap(void)
{
	cc_rxwake_t w;
	uint32_t now = 0xFFFFFFFFUL - 1000u;

	cc_rxwake_init(&w);
	for (int i = 0; i < STORM; i++)
		(void)cc_rxwake_on_edge(&w, now, STORM, BACKOFF, MAX);
	ASSERT_FALSE(cc_rxwake_armed(&w, now + 5000u));            /* wrapped, still off */
	ASSERT_TRUE(cc_rxwake_armed(&w, now + BACKOFF));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER console RX-wake storm guard");
	RUN_TEST(test_host_never_trips);
	RUN_TEST(test_storm_disarms_then_rearms);
	RUN_TEST(test_backoff_doubles_and_resets);
	RUN_TEST(test_line_rearms);
	RUN_TEST(test_tick_wrap);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
