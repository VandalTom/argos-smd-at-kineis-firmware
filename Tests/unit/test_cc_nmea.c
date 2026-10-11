/**
 * @file    test_cc_nmea.c
 * @brief   Unit tests for the NMEA RMC+GGA parser (cc_nmea.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_nmea.c"

/* Feed a full sentence, appending the correct checksum + CRLF. */
static bool feed_sentence(cc_nmea_parser_t *p, const char *body)
{
	uint8_t cs = 0;
	char buf[8];
	bool fix = false;

	cc_nmea_feed(p, '$');
	for (const char *c = body; *c; c++) {
		cs ^= (uint8_t)*c;
		if (cc_nmea_feed(p, (uint8_t)*c))
			fix = true;
	}
	snprintf(buf, sizeof(buf), "*%02X", cs);
	for (const char *c = buf; *c; c++)
		if (cc_nmea_feed(p, (uint8_t)*c))
			fix = true;
	if (cc_nmea_feed(p, '\r'))
		fix = true;
	if (cc_nmea_feed(p, '\n'))
		fix = true;
	return fix;
}

/* 51°30.0' N, 0°7.2' W = (51.5, -0.12); 2025-07-01 12:00:00 UTC */
#define GGA_GOOD "GNGGA,120000.00,5130.00000,N,00007.20000,W,1,08,1.0,10.0,M,45.0,M,,"
#define RMC_GOOD "GNRMC,120000.00,A,5130.00000,N,00007.20000,W,0.5,0.0,010725,,,A"

void test_fix_accepted(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	ASSERT_FALSE(feed_sentence(&p, GGA_GOOD)); /* GGA alone is not a fix */
	ASSERT_TRUE(feed_sentence(&p, RMC_GOOD));

	const cc_gps_fix_t *f = cc_nmea_fix(&p);

	ASSERT_TRUE(f->valid);
	ASSERT_TRUE(f->lat > 51.4999 && f->lat < 51.5001);
	ASSERT_TRUE(f->lon > -0.1201 && f->lon < -0.1199);
	ASSERT_EQ(8, f->sats);
	ASSERT_EQ(1, f->quality);
	/* 2025-07-01 = Unix day 20270; 12:00:00 -> +43200 s */
	ASSERT_EQ(1751371200UL, f->epoch);
	ASSERT_EQ(2, p.sentences);
	ASSERT_EQ(0, p.cs_errors);
	TEST_PASS();
}

/** RMC "V" (void) is never a fix, even with a good GGA. */
void test_rmc_void_rejected(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	(void)feed_sentence(&p, GGA_GOOD);
	ASSERT_FALSE(feed_sentence(&p,
		"GNRMC,120000.00,V,5130.00000,N,00007.20000,W,0.5,0.0,010725,,,N"));
	ASSERT_FALSE(cc_nmea_fix(&p)->valid);
	TEST_PASS();
}

/** Quality gate: fewer than 4 sats is a coarse fix -> rejected. */
void test_quality_gate(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	(void)feed_sentence(&p,
		"GNGGA,120000.00,5130.00000,N,00007.20000,W,1,03,1.0,10.0,M,45.0,M,,");
	ASSERT_FALSE(feed_sentence(&p, RMC_GOOD));
	ASSERT_FALSE(cc_nmea_fix(&p)->valid);
	TEST_PASS();
}

/** A corrupted checksum drops the sentence, counted, no state change. */
void test_bad_checksum_dropped(void)
{
	cc_nmea_parser_t p;
	const char *s = RMC_GOOD;

	cc_nmea_init(&p);
	cc_nmea_feed(&p, '$');
	for (const char *c = s; *c; c++)
		(void)cc_nmea_feed(&p, (uint8_t)*c);
	/* wrong checksum on purpose */
	(void)cc_nmea_feed(&p, '*');
	(void)cc_nmea_feed(&p, '0');
	(void)cc_nmea_feed(&p, '0');
	ASSERT_FALSE(cc_nmea_feed(&p, '\r'));
	ASSERT_EQ(1, p.cs_errors);
	ASSERT_EQ(0, p.sentences);
	ASSERT_FALSE(cc_nmea_fix(&p)->valid);
	TEST_PASS();
}

/** Southern/eastern hemisphere signs. */
void test_hemispheres(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	(void)feed_sentence(&p,
		"GNGGA,120000.00,3357.00000,S,15113.10000,E,1,08,1.0,10.0,M,45.0,M,,");
	ASSERT_TRUE(feed_sentence(&p,
		"GNRMC,120000.00,A,3357.00000,S,15113.10000,E,0.5,0.0,010725,,,A"));

	const cc_gps_fix_t *f = cc_nmea_fix(&p);

	/* 33°57' S = -33.95, 151°13.1' E = 151.21833 */
	ASSERT_TRUE(f->lat > -33.9501 && f->lat < -33.9499);
	ASSERT_TRUE(f->lon > 151.2182 && f->lon < 151.2185);
	TEST_PASS();
}

/** Garbage bytes between sentences don't derail the parser. */
void test_noise_immunity(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	(void)feed_sentence(&p, GGA_GOOD);
	for (int i = 0; i < 300; i++)
		(void)cc_nmea_feed(&p, (uint8_t)(i & 0xFF)); /* noise burst */
	ASSERT_TRUE(feed_sentence(&p, RMC_GOOD));
	TEST_PASS();
}

/* ---- GSV sky view ---- */

/** Indoors: satellites listed but weak or untracked -> no sky after 45 s. */
void test_gsv_indoor_no_sky(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	feed_sentence(&p, "GPGSV,2,1,06,02,45,120,18,05,30,200,,12,60,045,15,15,10,300,");
	feed_sentence(&p, "GPGSV,2,2,06,24,20,080,21,29,05,150,");
	feed_sentence(&p, "GLGSV,1,1,02,65,40,100,,72,25,210,12");
	ASSERT_EQ(1, cc_nmea_sky_strong(&p));          /* only the 21 dBHz one */
	ASSERT_EQ(21, p.max_cno);
	ASSERT_FALSE(cc_nmea_no_sky(&p, 30000, 45000, 2)); /* too early to judge */
	ASSERT_TRUE(cc_nmea_no_sky(&p, 45000, 45000, 2));
	TEST_PASS();
}

/** Outdoors: several strong satellites across constellations -> keep going. */
void test_gsv_outdoor_sky(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	feed_sentence(&p, "GPGSV,2,1,05,02,45,120,38,05,30,200,33,12,60,045,41,15,10,300,");
	feed_sentence(&p, "GPGSV,2,2,05,24,20,080,29");
	feed_sentence(&p, "GLGSV,1,1,02,65,40,100,35,72,25,210,19");
	ASSERT_EQ(5, cc_nmea_sky_strong(&p));          /* 4 GP + 1 GL */
	ASSERT_FALSE(cc_nmea_no_sky(&p, 120000, 45000, 2));
	TEST_PASS();
}

/** A fresh group replaces that constellation's count (sky improved). */
void test_gsv_group_replaces(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	feed_sentence(&p, "GPGSV,1,1,02,02,45,120,,05,30,200,");
	ASSERT_EQ(0, cc_nmea_sky_strong(&p));
	feed_sentence(&p, "GPGSV,1,1,02,02,45,120,31,05,30,200,27");
	ASSERT_EQ(2, cc_nmea_sky_strong(&p));
	TEST_PASS();
}

/** NMEA 4.1: trailing signal id after the last block is ignored. */
void test_gsv_nmea41_signal_id(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	feed_sentence(&p, "GPGSV,1,1,03,02,45,120,30,05,30,200,25,12,60,045,10,1");
	ASSERT_EQ(2, cc_nmea_sky_strong(&p));
	TEST_PASS();
}

/** No GSV from this receiver: never cut a window short. */
void test_gsv_absent_never_aborts(void)
{
	cc_nmea_parser_t p;

	cc_nmea_init(&p);
	feed_sentence(&p, GGA_GOOD);
	ASSERT_FALSE(cc_nmea_no_sky(&p, 600000, 45000, 2));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER NMEA parser");
	RUN_TEST(test_fix_accepted);
	RUN_TEST(test_rmc_void_rejected);
	RUN_TEST(test_quality_gate);
	RUN_TEST(test_bad_checksum_dropped);
	RUN_TEST(test_hemispheres);
	RUN_TEST(test_noise_immunity);
	RUN_TEST(test_gsv_indoor_no_sky);
	RUN_TEST(test_gsv_outdoor_sky);
	RUN_TEST(test_gsv_group_replaces);
	RUN_TEST(test_gsv_nmea41_signal_id);
	RUN_TEST(test_gsv_absent_never_aborts);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
