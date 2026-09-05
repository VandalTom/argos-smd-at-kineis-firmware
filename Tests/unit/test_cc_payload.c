/**
 * @file    test_cc_payload.c
 * @brief   Unit tests for the CC_TRACKER payload v3 packer (cc_payload.c).
 *
 * The layout is frozen (deployed decoders parse it), so these tests pin the
 * wire format with hand-computed golden vectors — not by re-running the
 * packer's own math.
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_payload.c"

/* Golden vector A: every group valid.
 * lat 51.5          -> rnd(51.5 * 8388607/90)  =  4800147 = 0x493E93
 * lon -0.12         -> rnd(-0.12 * 8388607/180) =   -5592 = 0xFFEA28 (24-bit)
 * epoch base+6e6 s  -> 100000 min = 0x0186A0
 * wake16 0x1234
 * WS85: spd 5.0->50, gust 8.2->82, dir 270, rain 123.4->1234
 *   [rain:13|dir:9|gust:9|spd:9] = 0010011010010 100001110 001010010 000110010
 *   = 26 94 38 A4 32
 * BMP: temp 21.37 -> 614, press 1013.25 -> 10133; [temp:10|press:14] = 99 A7 95
 * batt 2470 -> (2470-2000)/10 = 47 = 0x2F
 * hum 55.5 -> 111 = 0x6F
 * scap 4200 = 0x1068
 */
void test_golden_all_valid(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.lat = 51.5;
	f.lon = -0.12;
	f.epoch = CC_TS_EPOCH_UNIX + 6000000UL;
	f.wake16 = 0x1234;
	f.ws_valid = true;
	f.wind_ms = 5.0f;
	f.gust_ms = 8.2f;
	f.dir_deg = 270;
	f.rain_mm = 123.4f;
	f.temp_valid = true;
	f.temp_c = 21.37f;
	f.press_valid = true;
	f.press_hpa = 1013.25f;
	f.hum_valid = true;
	f.hum_pct = 55.5f;
	f.batt_mv = 2470;
	f.scap_valid = true;
	f.scap_mv = 4200;

	cc_payload_pack(out, &f);

	const uint8_t expect[CC_PAYLOAD_BYTES] = {
		0xC1, 0x49, 0x3E, 0x93, 0xFF, 0xEA, 0x28, 0x01, 0x86, 0xA0,
		0x12, 0x34, 0x26, 0x94, 0x38, 0xA4, 0x32, 0x99, 0xA7, 0x95,
		0x2F, 0x6F, 0x10, 0x68,
	};

	ASSERT_MEM_EQ(expect, out, CC_PAYLOAD_BYTES);
	TEST_PASS();
}

/* Golden vector B: everything invalid/unknown — all-ones sentinels. */
void test_golden_all_invalid(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	cc_payload_pack(out, &f);

	const uint8_t expect[CC_PAYLOAD_BYTES] = {
		0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x00, 0xFF, 0xFF, 0xFF,
	};

	ASSERT_MEM_EQ(expect, out, CC_PAYLOAD_BYTES);
	TEST_PASS();
}

/** Latitude beyond +/-90 clamps to the 24-bit signed extreme. */
void test_lat_clamp(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.lat = 95.0;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0x7F, out[1]);
	ASSERT_EQ(0xFF, out[2]);
	ASSERT_EQ(0xFF, out[3]);

	f.lat = -95.0;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0x80, out[1]);
	ASSERT_EQ(0x00, out[2]);
	ASSERT_EQ(0x01, out[3]); /* -8388607 (clamp), not -8388608 */
	TEST_PASS();
}

/** Battery clamps: <=2 V packs 0, >=4.55 V packs 255. */
void test_batt_clamp(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.batt_mv = 1500;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0x00, out[20]);

	f.batt_mv = 5000;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0xFF, out[20]);
	TEST_PASS();
}

/** Rain raw 8191 (the sentinel) is remapped to 8190 — sentinel stays unambiguous. */
void test_rain_sentinel_remap(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.ws_valid = true;
	f.rain_mm = 819.1f; /* *10 = 8191 = 0x1FFF */
	cc_payload_pack(out, &f);

	/* rain = bits 39..27 of the 40-bit group */
	uint16_t rain = (uint16_t)((((uint32_t)out[12] << 8) | out[13]) >> 3);

	ASSERT_EQ(0x1FFE, rain);
	TEST_PASS();
}

/** Epoch below the base packs 0 minutes (never underflows). */
void test_epoch_below_base(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.epoch = CC_TS_EPOCH_UNIX - 1;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0, out[7]);
	ASSERT_EQ(0, out[8]);
	ASSERT_EQ(0, out[9]);
	TEST_PASS();
}

/** A real supercap value of 0xFFFF clamps to 0xFFFE (sentinel unambiguous). */
void test_scap_never_sentinel(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.scap_valid = true;
	f.scap_mv = 0xFFFF;
	cc_payload_pack(out, &f);
	ASSERT_EQ(0xFF, out[22]);
	ASSERT_EQ(0xFE, out[23]);
	TEST_PASS();
}

/** Wind speed clamps at 51.0 m/s (0x1FE), never emitting the 0x1FF sentinel. */
void test_spd_clamp(void)
{
	cc_payload_fields_t f = {0};
	uint8_t out[CC_PAYLOAD_BYTES];

	f.ws_valid = true;
	f.wind_ms = 99.0f;
	cc_payload_pack(out, &f);

	uint16_t spd = (uint16_t)((((uint16_t)out[15] << 8) | out[16]) & 0x1FF);

	ASSERT_EQ(0x1FE, spd);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER payload v3 packer");
	RUN_TEST(test_golden_all_valid);
	RUN_TEST(test_golden_all_invalid);
	RUN_TEST(test_lat_clamp);
	RUN_TEST(test_batt_clamp);
	RUN_TEST(test_rain_sentinel_remap);
	RUN_TEST(test_epoch_below_base);
	RUN_TEST(test_scap_never_sentinel);
	RUN_TEST(test_spd_clamp);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
