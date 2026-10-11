/**
 * @file    test_cc_clock.c
 * @brief   Unit tests for the CC_TRACKER RTC-anchored clock helpers (cc_clock.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_clock.c"

/* Unix time of 2000-01-01T00:00:00Z: Unix = RTC seconds + this */
#define UNIX_2000 946684800UL

/** The RTC's power-on reset value (2000-01-01 00:00:00) is second 0. */
void test_epoch_zero(void)
{
	ASSERT_EQ(0UL, cc_clock_rtc_seconds(0, 1, 1, 0, 0, 0));
	ASSERT_EQ(86399UL, cc_clock_rtc_seconds(0, 1, 1, 23, 59, 59));
	ASSERT_EQ(86400UL, cc_clock_rtc_seconds(0, 1, 2, 0, 0, 0));
	TEST_PASS();
}

/** Known dates (checked against Unix timestamps). */
void test_known_dates(void)
{
	/* 2000-03-01: 2000 is a leap year (Jan 31 + Feb 29) */
	ASSERT_EQ(60UL * 86400UL, cc_clock_rtc_seconds(0, 3, 1, 0, 0, 0));
	/* 2001-01-01 = 366 days */
	ASSERT_EQ(366UL * 86400UL, cc_clock_rtc_seconds(1, 1, 1, 0, 0, 0));
	/* 2025-01-01T00:00:00Z = 1735689600 (the payload epoch) */
	ASSERT_EQ(1735689600UL - UNIX_2000, cc_clock_rtc_seconds(25, 1, 1, 0, 0, 0));
	/* 2024-02-29T12:34:56Z = 1709210096 */
	ASSERT_EQ(1709210096UL - UNIX_2000, cc_clock_rtc_seconds(24, 2, 29, 12, 34, 56));
	/* 2024-03-01T00:00:00Z = 1709251200 (after a leap day) */
	ASSERT_EQ(1709251200UL - UNIX_2000, cc_clock_rtc_seconds(24, 3, 1, 0, 0, 0));
	/* 2026-10-11T08:00:00Z = 1791705600 */
	ASSERT_EQ(1791705600UL - UNIX_2000, cc_clock_rtc_seconds(26, 10, 11, 8, 0, 0));
	/* 2099-12-31T23:59:59Z = 4102444799 (end of the RTC range) */
	ASSERT_EQ(4102444799UL - UNIX_2000, cc_clock_rtc_seconds(99, 12, 31, 23, 59, 59));
	TEST_PASS();
}

/** Monotonic across a month, year and leap boundary. */
void test_boundaries_monotonic(void)
{
	ASSERT_EQ(cc_clock_rtc_seconds(23, 12, 31, 23, 59, 59) + 1,
		cc_clock_rtc_seconds(24, 1, 1, 0, 0, 0));
	ASSERT_EQ(cc_clock_rtc_seconds(24, 2, 29, 23, 59, 59) + 1,
		cc_clock_rtc_seconds(24, 3, 1, 0, 0, 0));
	ASSERT_EQ(cc_clock_rtc_seconds(25, 2, 28, 23, 59, 59) + 1,
		cc_clock_rtc_seconds(25, 3, 1, 0, 0, 0));
	TEST_PASS();
}

/** Garbage month/day fields never index out of bounds. */
void test_bad_fields_safe(void)
{
	ASSERT_EQ(cc_clock_rtc_seconds(0, 1, 1, 0, 0, 0), cc_clock_rtc_seconds(0, 0, 0, 0, 0, 0));
	(void)cc_clock_rtc_seconds(0, 13, 1, 0, 0, 0);
	(void)cc_clock_rtc_seconds(0, 255, 255, 0, 0, 0);
	TEST_PASS();
}

/** Anchor round trip: what a fix stores reproduces Unix time after a reset. */
void test_anchor_round_trip(void)
{
	uint32_t rtc_at_fix = cc_clock_rtc_seconds(0, 1, 9, 6, 0, 0);    /* 8.25 d after power-up */
	uint32_t gps_epoch = 1791705600UL;
	uint32_t anchor = gps_epoch - rtc_at_fix;
	uint32_t a = anchor, a_inv = ~anchor;                           /* TAMP pair */

	ASSERT_TRUE(cc_clock_anchor_valid(a, a_inv));
	/* 14-day reboot later: RTC kept counting */
	uint32_t rtc_later = cc_clock_rtc_seconds(0, 1, 23, 6, 0, 0);

	ASSERT_EQ(gps_epoch + 14UL * 86400UL, a + rtc_later);
	TEST_PASS();
}

/** Reset values, stray writes and torn stores never validate. */
void test_anchor_validation(void)
{
	ASSERT_FALSE(cc_clock_anchor_valid(0, 0));              /* backup-domain reset */
	ASSERT_FALSE(cc_clock_anchor_valid(0, 0xFFFFFFFFUL));   /* zero anchor */
	ASSERT_FALSE(cc_clock_anchor_valid(0x12345678UL, 0));   /* complement missing */
	ASSERT_FALSE(cc_clock_anchor_valid(0x12345678UL, ~0x12345679UL));
	ASSERT_TRUE(cc_clock_anchor_valid(0x12345678UL, ~0x12345678UL));
	TEST_PASS();
}

void test_epoch_plausible(void)
{
	ASSERT_FALSE(cc_clock_epoch_plausible(0, 1735689600UL));
	ASSERT_FALSE(cc_clock_epoch_plausible(315964800UL, 1735689600UL)); /* 1980-01-06 GPS epoch */
	ASSERT_TRUE(cc_clock_epoch_plausible(1735689600UL, 1735689600UL));
	ASSERT_TRUE(cc_clock_epoch_plausible(1791705600UL, 1735689600UL));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER RTC-anchored clock");
	RUN_TEST(test_epoch_zero);
	RUN_TEST(test_known_dates);
	RUN_TEST(test_boundaries_monotonic);
	RUN_TEST(test_bad_fields_safe);
	RUN_TEST(test_anchor_round_trip);
	RUN_TEST(test_anchor_validation);
	RUN_TEST(test_epoch_plausible);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
