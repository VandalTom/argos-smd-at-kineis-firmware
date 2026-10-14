/**
 * @file    test_cc_ws85.c
 * @brief   Unit tests for the WS85 broadcast-frame parser (cc_ws85.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_ws85.c"

/* Build a 33-byte broadcast frame: 85 03 1C <14 regs BE> CRC(lo,hi).
 * Register order: model, baud, address, id_hi, id_lo, light, uv, temp, hum,
 * wind, gust, dir, rain, press. */
static uint8_t build_broadcast(uint8_t *out, const uint16_t regs[14])
{
	out[0] = 0x85;
	out[1] = 0x03;
	out[2] = 0x1C;
	for (int i = 0; i < 14; i++) {
		out[3 + 2 * i] = (uint8_t)(regs[i] >> 8);
		out[4 + 2 * i] = (uint8_t)(regs[i] & 0xFF);
	}

	uint16_t crc = cc_ws85_crc16(out, 31);

	out[31] = (uint8_t)(crc & 0xFF); /* low byte first */
	out[32] = (uint8_t)(crc >> 8);
	return 33;
}

/* WS85-typical register content: temp 21.5 C (=615), wind 5.0, gust 8.2,
 * dir 270, rain 123.4 (=1234), light/uv/hum/press absent (0xFFFF). */
static const uint16_t k_regs[14] = {
	0x0085, 0x0004, 0x0085, 0x1234, 0x5678,
	0xFFFF, 0xFFFF, /* light, uv */
	615,    0xFFFF, /* temp, hum */
	50, 82, 270, 1234,
	0xFFFF,         /* press */
};

void test_broadcast_decodes(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint8_t frame[33];
	uint8_t n = build_broadcast(frame, k_regs);
	bool got = false;

	cc_ws85_init(&p, 0x85);
	for (uint8_t i = 0; i < n; i++)
		if (cc_ws85_feed(&p, frame[i], &f))
			got = true;

	ASSERT_TRUE(got);
	ASSERT_TRUE(f.valid);
	ASSERT_EQ(0x0085, f.model);
	ASSERT_TRUE(f.has_temp);
	ASSERT_TRUE(f.temp_c > 21.49f && f.temp_c < 21.51f);
	ASSERT_TRUE(f.has_wind);
	ASSERT_TRUE(f.wind_ms > 4.99f && f.wind_ms < 5.01f);
	ASSERT_TRUE(f.has_gust);
	ASSERT_TRUE(f.gust_ms > 8.19f && f.gust_ms < 8.21f);
	ASSERT_TRUE(f.has_dir);
	ASSERT_EQ(270, f.dir_deg);
	ASSERT_TRUE(f.has_rain);
	ASSERT_TRUE(f.rain_mm > 123.39f && f.rain_mm < 123.41f);
	ASSERT_EQ(1, p.frames);
	ASSERT_EQ(0, p.crc_errors);
	TEST_PASS();
}

void test_crc_error_rejected(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint8_t frame[33];
	uint8_t n = build_broadcast(frame, k_regs);
	bool got = false;

	frame[10] ^= 0x40; /* corrupt one data byte */
	cc_ws85_init(&p, 0x85);
	for (uint8_t i = 0; i < n; i++)
		if (cc_ws85_feed(&p, frame[i], &f))
			got = true;

	ASSERT_FALSE(got);
	ASSERT_EQ(0, p.frames);
	ASSERT_EQ(1, p.crc_errors);
	TEST_PASS();
}

/** Garbage before the frame: parser resyncs on the address byte. */
void test_resync_after_garbage(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint8_t frame[40];
	const uint8_t junk[7] = { 0x00, 0xFF, 0x85, 0x99, 0x12, 0x85, 0x03 };
	bool got = false;

	/* junk includes a fake 0x85 0x99 (bad func) and a truncated 0x85 0x03 —
	 * the trailing 0x85 0x03 is a REAL frame start, so append from func on */
	uint8_t n = build_broadcast(frame, k_regs);

	cc_ws85_init(&p, 0x85);
	for (uint8_t i = 0; i < 5; i++) /* only the pure junk part */
		(void)cc_ws85_feed(&p, junk[i], &f);
	for (uint8_t i = 0; i < n; i++)
		if (cc_ws85_feed(&p, frame[i], &f))
			got = true;

	ASSERT_TRUE(got);
	ASSERT_EQ(1, p.frames);
	TEST_PASS();
}

/** Two frames back-to-back both decode. */
void test_two_frames(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint8_t frame[33];
	uint8_t n = build_broadcast(frame, k_regs);
	int got = 0;

	cc_ws85_init(&p, 0x85);
	for (int r = 0; r < 2; r++)
		for (uint8_t i = 0; i < n; i++)
			if (cc_ws85_feed(&p, frame[i], &f))
				got++;

	ASSERT_EQ(2, got);
	ASSERT_EQ(2, p.frames);
	TEST_PASS();
}

/** Absent channels (raw 0xFFFF) come back has_* = false. */
void test_absent_channels(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint16_t regs[14];
	uint8_t frame[33];

	memcpy(regs, k_regs, sizeof(regs));
	regs[7] = 0xFFFF;  /* temp absent */
	regs[11] = 0xFFFF; /* dir absent */

	uint8_t n = build_broadcast(frame, regs);
	bool got = false;

	cc_ws85_init(&p, 0x85);
	for (uint8_t i = 0; i < n; i++)
		if (cc_ws85_feed(&p, frame[i], &f))
			got = true;

	ASSERT_TRUE(got);
	ASSERT_FALSE(f.has_temp);
	ASSERT_FALSE(f.has_dir);
	ASSERT_TRUE(f.has_wind);
	TEST_PASS();
}

/** Known CRC16 vector: Modbus poly over 85 03 1C of zeros — round-trip only. */
void test_crc16_known(void)
{
	/* Classic Modbus check: CRC16("123456789") = 0x4B37 */
	ASSERT_EQ_HEX(0x4B37, cc_ws85_crc16((const uint8_t *)"123456789", 9));
	TEST_PASS();
}

/* The bench capture (unit SMD60C00360040, WS85 V1.1.8 at 115200): the line
 * idled low, so the header 85 03 1C 00 arrived as E1 E0 0C. */
static const uint8_t k_bench[32] = {
	0xE1, 0xE0, 0x0C, 0x85, 0x00, 0x04, 0x00, 0x85, 0x00, 0x00, 0x27, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF,
	0x02, 0x7A, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0x37, 0x00, 0x00, 0xFF, 0xFF, 0x7B, 0x5A,
};

void test_recovers_bench_capture(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	int got = 0;

	cc_ws85_init(&p, 0x85);
	for (uint8_t i = 0; i < sizeof(k_bench); i++)
		if (cc_ws85_feed(&p, k_bench[i], &f))
			got++;
	ASSERT_EQ(1, got);
	ASSERT_EQ(1, p.frames);
	ASSERT_EQ(1, p.recovered);
	ASSERT_EQ(0x0085, f.model);
	ASSERT_TRUE(f.has_temp && f.temp_c > 23.39f && f.temp_c < 23.41f);
	ASSERT_TRUE(f.has_wind && f.wind_ms < 0.01f);
	ASSERT_EQ(311, f.dir_deg);
	TEST_PASS();
}

void test_recovers_each_lost_length(void)
{
	uint8_t frame[33];

	build_broadcast(frame, k_regs);
	for (uint8_t lost = 1; lost <= 4; lost++) {
		cc_ws85_parser_t p;
		cc_ws85_frame_t f = {0};
		int got = 0;

		cc_ws85_init(&p, 0x85);
		cc_ws85_feed(&p, 0x00, &f);              /* the break byte the line leaves */
		for (uint8_t i = 0; i < lost - 1; i++)   /* fewer garbled bytes than lost */
			cc_ws85_feed(&p, 0xE1, &f);
		for (uint8_t i = lost; i < 33; i++)
			if (cc_ws85_feed(&p, frame[i], &f))
				got++;
		ASSERT_EQ(1, got);
		ASSERT_EQ(1, p.recovered);
		ASSERT_EQ(270, f.dir_deg);
	}
	TEST_PASS();
}

void test_intact_frame_not_double_counted(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint8_t frame[33];
	int got = 0;

	build_broadcast(frame, k_regs);
	cc_ws85_init(&p, 0x85);
	for (int rep = 0; rep < 3; rep++)
		for (uint8_t i = 0; i < 33; i++)
			if (cc_ws85_feed(&p, frame[i], &f))
				got++;
	ASSERT_EQ(3, got);
	ASSERT_EQ(3, p.frames);
	ASSERT_EQ(0, p.recovered);
	TEST_PASS();
}

void test_recovery_needs_crc_and_fields(void)
{
	uint8_t frame[33];
	uint16_t regs[14];
	cc_ws85_frame_t f = {0};

	/* a corrupted data byte behind a lost header: CRC rejects it */
	{
		cc_ws85_parser_t p;
		bool got = false;

		build_broadcast(frame, k_regs);
		frame[20] ^= 0x01;
		cc_ws85_init(&p, 0x85);
		for (uint8_t i = 4; i < 33; i++)
			got |= cc_ws85_feed(&p, frame[i], &f);
		ASSERT_FALSE(got);
	}
	/* another sensor's address, valid CRC: not ours */
	{
		cc_ws85_parser_t p;
		bool got = false;

		memcpy(regs, k_regs, sizeof(regs));
		regs[2] = 0x0090;
		build_broadcast(frame, regs);
		cc_ws85_init(&p, 0x85);
		for (uint8_t i = 4; i < 33; i++)
			got |= cc_ws85_feed(&p, frame[i], &f);
		ASSERT_FALSE(got);
	}
	/* an impossible baud code, valid CRC */
	{
		cc_ws85_parser_t p;
		bool got = false;

		memcpy(regs, k_regs, sizeof(regs));
		regs[1] = 0x0009;
		build_broadcast(frame, regs);
		cc_ws85_init(&p, 0x85);
		for (uint8_t i = 4; i < 33; i++)
			got |= cc_ws85_feed(&p, frame[i], &f);
		ASSERT_FALSE(got);
	}
	TEST_PASS();
}

void test_noise_never_decodes(void)
{
	cc_ws85_parser_t p;
	cc_ws85_frame_t f = {0};
	uint32_t x = 12345u;
	int got = 0;

	cc_ws85_init(&p, 0x85);
	for (uint32_t i = 0; i < 200000u; i++) {
		x = x * 1664525u + 1013904223u;
		if (cc_ws85_feed(&p, (uint8_t)(x >> 24), &f))
			got++;
	}
	ASSERT_EQ(0, got);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER WS85 frame parser");
	RUN_TEST(test_broadcast_decodes);
	RUN_TEST(test_crc_error_rejected);
	RUN_TEST(test_resync_after_garbage);
	RUN_TEST(test_two_frames);
	RUN_TEST(test_absent_channels);
	RUN_TEST(test_crc16_known);
	RUN_TEST(test_recovers_bench_capture);
	RUN_TEST(test_recovers_each_lost_length);
	RUN_TEST(test_intact_frame_not_double_counted);
	RUN_TEST(test_recovery_needs_crc_and_fields);
	RUN_TEST(test_noise_never_decodes);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
