/**
 * @file    cc_payload.c
 * @brief   Payload v3 packer — byte-exact port of WS85-Tracker Payload.h.
 *
 * Every encoder mirrors the Arduino original exactly (same rounding, same
 * clamps, same sentinel remaps) so packets from the ported firmware are
 * indistinguishable from the XIAO-built ones on the decoder side.
 */

#include <string.h>
#include "cc_payload.h"
#include "cc_conf.h"

/* rnd(): round-half-away-from-zero via C truncation, as the original. */
static long cc_rnd(double x)
{
	return (long)(x >= 0 ? x + 0.5 : x - 0.5);
}

static long cc_clampl(long v, long lo, long hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)(v & 0xFF);
}

static void put_u24(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 16);
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v & 0xFF);
}

/* signed 24-bit two's complement */
static void put_i24(uint8_t *p, long v)
{
	v = cc_clampl(v, -8388607L, 8388607L);
	put_u24(p, (uint32_t)v & 0xFFFFFFu);
}

/* --- v3 bit-field encoders: each returns its all-ones sentinel when invalid --- */

static uint16_t enc_spd9(bool valid, float ms) /* also used for gust */
{
	return valid ? (uint16_t)cc_clampl(cc_rnd(ms * 10.0), 0, 0x1FE) : 0x1FF;
}

static uint16_t enc_dir9(bool valid, uint16_t deg)
{
	return valid ? (uint16_t)cc_clampl((long)deg, 0, 359) : 0x1FF;
}

static uint16_t enc_rain13(bool valid, float mm)
{
	if (!valid)
		return 0x1FFF;
	uint16_t r = (uint16_t)(cc_rnd(mm * 10.0) & 0x1FFF); /* counter mod 819.2 mm */
	return (r == 0x1FFF) ? 0x1FFE : r; /* never emit the sentinel as data */
}

static uint16_t enc_temp10(bool valid, float c)
{
	return valid ? (uint16_t)cc_clampl(cc_rnd((c + 40.0) * 10.0), 0, 0x3FE) : 0x3FF;
}

static uint16_t enc_press14(bool valid, float hpa)
{
	return valid ? (uint16_t)cc_clampl(cc_rnd(hpa * 10.0), 0, 0x3FFE) : 0x3FFF;
}

static uint8_t enc_batt8(uint16_t mv)
{
	return (uint8_t)cc_clampl(((long)mv - 2000) / 10, 0, 255);
}

static uint8_t enc_hum8(bool valid, float rh) /* 0.5 %RH steps */
{
	return valid ? (uint8_t)cc_clampl(cc_rnd(rh * 2.0), 0, 0xFE) : 0xFF;
}

void cc_payload_pack(uint8_t out[CC_PAYLOAD_BYTES], const cc_payload_fields_t *f)
{
	memset(out, 0, CC_PAYLOAD_BYTES);

	out[0] = CC_PAYLOAD_ADDR;
	put_i24(&out[1], cc_rnd(f->lat * (8388607.0 / 90.0)));
	put_i24(&out[4], cc_rnd(f->lon * (8388607.0 / 180.0)));

	uint32_t minutes = (f->epoch >= CC_TS_EPOCH_UNIX)
		? ((f->epoch - CC_TS_EPOCH_UNIX) / 60UL + (uint32_t)CC_TZ_OFFSET_MIN) : 0;
	put_u24(&out[7], minutes & 0xFFFFFFu);

	put_u16(&out[10], f->wake16);

	/* WS85 group: rain<<27 | dir<<18 | gust<<9 | spd (40 bits, MSB first) */
	uint64_t ws = ((uint64_t)enc_rain13(f->ws_valid, f->rain_mm) << 27)
		| ((uint32_t)enc_dir9(f->ws_valid, f->dir_deg) << 18)
		| ((uint32_t)enc_spd9(f->ws_valid, f->gust_ms) << 9)
		| enc_spd9(f->ws_valid, f->wind_ms);
	out[12] = (uint8_t)(ws >> 32);
	out[13] = (uint8_t)(ws >> 24);
	out[14] = (uint8_t)(ws >> 16);
	out[15] = (uint8_t)(ws >> 8);
	out[16] = (uint8_t)ws;

	/* BMP280 group: temp<<14 | press (24 bits, MSB first) */
	uint32_t bp = ((uint32_t)enc_temp10(f->temp_valid, f->temp_c) << 14)
		| enc_press14(f->press_valid, f->press_hpa);
	out[17] = (uint8_t)(bp >> 16);
	out[18] = (uint8_t)(bp >> 8);
	out[19] = (uint8_t)bp;

	out[20] = enc_batt8(f->batt_mv);
	out[21] = enc_hum8(f->hum_valid, f->hum_pct);
	/* supercap: raw mV big-endian; 0xFFFF = invalid (real values clamp below) */
	put_u16(&out[22], f->scap_valid
		? (uint16_t)cc_clampl((long)f->scap_mv, 0, 0xFFFE) : 0xFFFF);
}
