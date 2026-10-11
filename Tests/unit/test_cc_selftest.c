/**
 * @file    test_cc_selftest.c
 * @brief   Unit tests for the CC_TRACKER self-test bookkeeping (cc_selftest.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_selftest.c"

/** Item lists: empty = all, names, ALL, rejects unknown/empty tokens. */
void test_parse_mask(void)
{
	ASSERT_EQ(CC_ST_ALL, cc_selftest_parse_mask(NULL));
	ASSERT_EQ(CC_ST_ALL, cc_selftest_parse_mask(""));
	ASSERT_EQ(CC_ST_ALL, cc_selftest_parse_mask("\r\n"));
	ASSERT_EQ(CC_ST_ALL, cc_selftest_parse_mask("ALL\r"));
	ASSERT_EQ(CC_ST_BIT(CC_STI_GPS) | CC_ST_BIT(CC_STI_TX), cc_selftest_parse_mask("GPS,TX\r\n"));
	ASSERT_EQ(CC_ST_BIT(CC_STI_WS85), cc_selftest_parse_mask("WS85"));
	ASSERT_EQ(0, cc_selftest_parse_mask("GPS,FOO"));
	ASSERT_EQ(0, cc_selftest_parse_mask("GPS,"));
	ASSERT_EQ(0, cc_selftest_parse_mask("gps"));
	TEST_PASS();
}

/** Verdict: requested items must all PASS; SKIP is neutral; NOTRUN fails. */
void test_verdict(void)
{
	cc_selftest_t st;

	memset(&st, 0, sizeof(st));
	cc_selftest_begin(&st, CC_ST_BIT(CC_STI_GPS) | CC_ST_BIT(CC_STI_TX), false);
	ASSERT_FALSE(cc_selftest_passed(&st)); /* nothing run yet */
	cc_selftest_set(&st, CC_STI_GPS, CC_STV_PASS, "bytes=10,nmea=2");
	cc_selftest_set(&st, CC_STI_TX, CC_STV_SKIP, "disabled");
	ASSERT_TRUE(cc_selftest_passed(&st));
	cc_selftest_set(&st, CC_STI_TX, CC_STV_FAIL, "timeout");
	ASSERT_FALSE(cc_selftest_passed(&st));
	/* items outside the mask never count */
	cc_selftest_set(&st, CC_STI_WS85, CC_STV_FAIL, "silent");
	cc_selftest_set(&st, CC_STI_TX, CC_STV_PASS, "ms=900");
	ASSERT_TRUE(cc_selftest_passed(&st));
	cc_selftest_end(&st);
	ASSERT_TRUE(st.done);
	ASSERT_FALSE(st.running);
	TEST_PASS();
}

/** Report lines are exactly what the web tool parses. */
void test_format(void)
{
	cc_selftest_t st;
	char buf[96];

	memset(&st, 0, sizeof(st));
	cc_selftest_begin(&st, CC_ST_ALL, true);
	cc_selftest_set(&st, CC_STI_GPS, CC_STV_PASS, "bytes=812,nmea=9");
	cc_selftest_set(&st, CC_STI_WS85, CC_STV_FAIL, "silent");
	cc_selftest_set(&st, CC_STI_ENV, CC_STV_PASS, NULL);
	cc_selftest_set(&st, CC_STI_ADC, CC_STV_PASS, "nimh=2650,scap=4800");
	cc_selftest_set(&st, CC_STI_TX, CC_STV_FAIL, "timeout");

	cc_selftest_format_item(buf, sizeof(buf), &st, CC_STI_GPS);
	ASSERT_STR_EQ("+ST=GPS,PASS,bytes=812,nmea=9\r\n", buf);
	cc_selftest_format_item(buf, sizeof(buf), &st, CC_STI_ENV);
	ASSERT_STR_EQ("+ST=ENV,PASS\r\n", buf);
	cc_selftest_format_summary(buf, sizeof(buf), &st);
	ASSERT_STR_EQ("+SELFTEST=FAIL,WS85,TX\r\n", buf);

	cc_selftest_set(&st, CC_STI_WS85, CC_STV_PASS, "frames=1");
	cc_selftest_set(&st, CC_STI_TX, CC_STV_PASS, "ms=1234");
	cc_selftest_format_summary(buf, sizeof(buf), &st);
	ASSERT_STR_EQ("+SELFTEST=PASS\r\n", buf);
	TEST_PASS();
}

/** Long details are truncated, never overflowed. */
void test_detail_truncation(void)
{
	cc_selftest_t st;
	char longd[200];

	memset(longd, 'x', sizeof(longd) - 1);
	longd[sizeof(longd) - 1] = '\0';
	memset(&st, 0, sizeof(st));
	cc_selftest_begin(&st, CC_ST_ALL, false);
	cc_selftest_set(&st, CC_STI_GPS, CC_STV_FAIL, longd);
	ASSERT_EQ(CC_ST_DETAIL_LEN - 1, (int)strlen(st.item[CC_STI_GPS].detail));
	TEST_PASS();
}

/** Item rules. */
void test_item_rules(void)
{
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_gps(500, 0));   /* bytes but no valid sentence */
	ASSERT_EQ(CC_STV_PASS, cc_selftest_eval_gps(80, 1));
	ASSERT_EQ(CC_STV_PASS, cc_selftest_eval_env(21.5f, 1013.2f, 45.0f));
	ASSERT_EQ(CC_STV_PASS, cc_selftest_eval_env(-12.0f, 980.0f, -1.0f)); /* BMP280: no RH */
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_env(21.5f, 0.0f, 45.0f));    /* pressure garbage */
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_env(150.0f, 1013.0f, 45.0f));
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_env(21.5f, 1013.0f, 140.0f));
	ASSERT_EQ(CC_STV_PASS, cc_selftest_eval_adc(2650, 1800, 3300));
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_adc(0, 1800, 3300));
	ASSERT_EQ(CC_STV_FAIL, cc_selftest_eval_adc(3600, 1800, 3300));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER self-test");
	RUN_TEST(test_parse_mask);
	RUN_TEST(test_verdict);
	RUN_TEST(test_format);
	RUN_TEST(test_detail_truncation);
	RUN_TEST(test_item_rules);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
