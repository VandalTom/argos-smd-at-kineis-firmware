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

int main(void)
{
	TEST_SUITE_START("CC_TRACKER NMEA parser");
	RUN_TEST(test_fix_accepted);
	RUN_TEST(test_rmc_void_rejected);
	RUN_TEST(test_quality_gate);
	RUN_TEST(test_bad_checksum_dropped);
	RUN_TEST(test_hemispheres);
	RUN_TEST(test_noise_immunity);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
