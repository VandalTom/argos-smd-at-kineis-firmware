/**
 * @file    test_cc_commission.c
 * @brief   Unit tests for the CC_TRACKER deployment burst logic (cc_commission.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Inc/cc_conf.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_commission.c"

#define H (3600UL * 1000UL)

/** Fresh power-up waits for a fix; a persisted flag means already done. */
void test_init(void)
{
	cc_comm_t c;

	cc_comm_init(&c, false);
	ASSERT_EQ(CC_COMM_WAIT_FIX, c.phase);
	cc_comm_init(&c, true);
	ASSERT_EQ(CC_COMM_DONE, c.phase);
	TEST_PASS();
}

/** Indoors (no fix): normal cadence; GPS fast for 48 h, then normal schedule. */
void test_waiting_indoors(void)
{
	cc_comm_t c;

	cc_comm_init(&c, false);
	ASSERT_EQ(420000UL, cc_comm_period_ms(&c, 420000UL, CC_COMM_BURST_PERIOD_MS,
		CC_COMM_BURST_JITTER_MS, 12345));
	ASSERT_EQ(CC_COMM_GPS_FAST, cc_comm_gps_policy(&c, 1000, CC_COMM_FAST_GPS_MS));
	ASSERT_EQ(CC_COMM_GPS_FAST, cc_comm_gps_policy(&c, 47 * H, CC_COMM_FAST_GPS_MS));
	ASSERT_EQ(CC_COMM_GPS_NORMAL, cc_comm_gps_policy(&c, 49 * H, CC_COMM_FAST_GPS_MS));
	/* a TX while waiting changes nothing */
	ASSERT_FALSE(cc_comm_on_tx(&c, 5000, CC_COMM_BURST_MS));
	ASSERT_EQ(CC_COMM_WAIT_FIX, c.phase);
	TEST_PASS();
}

/** First fix starts the burst; later fixes don't restart it. */
void test_first_fix_starts_burst(void)
{
	cc_comm_t c;

	cc_comm_init(&c, false);
	ASSERT_TRUE(cc_comm_on_fix(&c, 60 * H)); /* deployed days later still works */
	ASSERT_EQ(CC_COMM_BURST, c.phase);
	ASSERT_FALSE(cc_comm_on_fix(&c, 60 * H + 5000));
	ASSERT_EQ(CC_COMM_GPS_SKIP, cc_comm_gps_policy(&c, 60 * H, CC_COMM_FAST_GPS_MS));
	TEST_PASS();
}

/** Burst cadence is 60 s + 0..5 s jitter. */
void test_burst_period(void)
{
	cc_comm_t c;

	cc_comm_init(&c, false);
	cc_comm_on_fix(&c, 0);
	for (uint32_t r = 0; r < 20000; r += 997) {
		uint32_t p = cc_comm_period_ms(&c, 420000UL, CC_COMM_BURST_PERIOD_MS,
			CC_COMM_BURST_JITTER_MS, r * 2654435761UL);

		ASSERT_TRUE(p >= CC_COMM_BURST_PERIOD_MS);
		ASSERT_TRUE(p <= CC_COMM_BURST_PERIOD_MS + CC_COMM_BURST_JITTER_MS);
	}
	TEST_PASS();
}

/** The burst ends on the first TX completing at/after 30 min, then stays done. */
void test_burst_ends_after_window(void)
{
	cc_comm_t c;
	uint32_t t0 = 10 * H;
	uint32_t now = t0;
	int ended_at = -1;

	cc_comm_init(&c, false);
	cc_comm_on_fix(&c, t0);
	for (int i = 0; i < 40; i++) {
		now += 61000; /* one burst cycle */
		if (cc_comm_on_tx(&c, now, CC_COMM_BURST_MS)) {
			ended_at = i;
			break;
		}
	}
	ASSERT_EQ(29, ended_at);             /* TX #30 at 30.5 min ends it */
	ASSERT_EQ(30, (int)c.burst_tx);
	ASSERT_EQ(CC_COMM_DONE, c.phase);
	ASSERT_FALSE(cc_comm_on_fix(&c, now + 1000));
	ASSERT_EQ(CC_COMM_GPS_NORMAL, cc_comm_gps_policy(&c, 1000, CC_COMM_FAST_GPS_MS));
	ASSERT_EQ(420000UL, cc_comm_period_ms(&c, 420000UL, CC_COMM_BURST_PERIOD_MS,
		CC_COMM_BURST_JITTER_MS, 7));
	TEST_PASS();
}

/** Tick wrap inside the burst is handled (unsigned elapsed). */
void test_burst_tick_wrap(void)
{
	cc_comm_t c;

	cc_comm_init(&c, false);
	cc_comm_on_fix(&c, 0xFFFFFFFFUL - 60000UL);
	ASSERT_FALSE(cc_comm_on_tx(&c, 0x00010000UL, CC_COMM_BURST_MS));
	ASSERT_TRUE(cc_comm_on_tx(&c, CC_COMM_BURST_MS, CC_COMM_BURST_MS));
	TEST_PASS();
}

/** Console overrides and the status line. */
void test_overrides_and_format(void)
{
	cc_comm_t c;
	char buf[96];

	cc_comm_init(&c, false);
	c.gps_tries = 3;
	cc_comm_format(buf, sizeof(buf), &c, 0, 1000, CC_COMM_FAST_GPS_MS, CC_COMM_BURST_MS,
		CC_COMM_BURST_PERIOD_MS);
	ASSERT_STR_EQ("+COMMISSION=WAIT_FIX,fastgps=1,gps_tries=3\r\n", buf);

	cc_comm_start(&c, 1000);
	(void)cc_comm_on_tx(&c, 61000, CC_COMM_BURST_MS);
	cc_comm_format(buf, sizeof(buf), &c, 61000, 61000, CC_COMM_FAST_GPS_MS, CC_COMM_BURST_MS,
		CC_COMM_BURST_PERIOD_MS);
	ASSERT_STR_EQ("+COMMISSION=BURST,1/30,left=1740\r\n", buf);

	cc_comm_skip(&c);
	cc_comm_format(buf, sizeof(buf), &c, 0, 0, CC_COMM_FAST_GPS_MS, CC_COMM_BURST_MS,
		CC_COMM_BURST_PERIOD_MS);
	ASSERT_STR_EQ("+COMMISSION=DONE\r\n", buf);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER commissioning burst");
	RUN_TEST(test_init);
	RUN_TEST(test_waiting_indoors);
	RUN_TEST(test_first_fix_starts_burst);
	RUN_TEST(test_burst_period);
	RUN_TEST(test_burst_ends_after_window);
	RUN_TEST(test_burst_tick_wrap);
	RUN_TEST(test_overrides_and_format);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
