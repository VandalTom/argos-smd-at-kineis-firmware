/**
 * @file    cc_i2c_sens.c
 * @brief   SHT45 / BMP280-BME280 / AHT20 drivers — ports of the WS85-Tracker
 *          Arduino originals onto the cc_i2c_ops_t bus vtable.
 *
 * Register sequences, compensation math (Bosch datasheet integer routines)
 * and CRC checks are ported verbatim; only the bus plumbing changed.
 */

#include <string.h>
#include "cc_i2c_sens.h"

/* Sensirion CRC-8: poly 0x31, init 0xFF, over each 2-byte word.
 * Shared by SHT45 (per-word) and AHT20 (over 6 bytes). */
static uint8_t crc8_31(const uint8_t *p, uint8_t n)
{
	uint8_t crc = 0xFF;

	for (uint8_t i = 0; i < n; i++) {
		crc ^= p[i];
		for (uint8_t b = 0; b < 8; b++)
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
	}
	return crc;
}

/*******************************************************************************
 * SHT45
 ******************************************************************************/

static bool sht45_words_ok(const uint8_t b[6])
{
	return crc8_31(b, 2) == b[2] && crc8_31(b + 3, 2) == b[5];
}

bool cc_sht45_begin(cc_sht45_t *s, const cc_i2c_ops_t *bus, uint8_t addr)
{
	uint8_t cmd = 0x89; /* read serial number */
	uint8_t b[6];

	s->bus = bus;
	s->addr = addr;
	if (!bus->write(bus->ctx, addr, &cmd, 1))
		return false;
	bus->delay_ms(bus->ctx, 1);
	return bus->read(bus->ctx, addr, b, 6) && sht45_words_ok(b);
}

bool cc_sht45_read(cc_sht45_t *s, float *rh, float *tc)
{
	const cc_i2c_ops_t *bus = s->bus;
	uint8_t cmd = 0xFD; /* measure, high precision, no heater */
	uint8_t b[6];       /* tempMSB tempLSB crc rhMSB rhLSB crc */

	if (!bus)
		return false;
	if (!bus->write(bus->ctx, s->addr, &cmd, 1))
		return false;
	bus->delay_ms(bus->ctx, 10); /* chip NACKs reads until done (max 8.3 ms) */
	if (!bus->read(bus->ctx, s->addr, b, 6) || !sht45_words_ok(b))
		return false;

	uint16_t raw_t  = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
	uint16_t raw_rh = (uint16_t)(((uint16_t)b[3] << 8) | b[4]);

	*tc = -45.0f + 175.0f * raw_t / 65535.0f; /* datasheet §4.6 */
	float h = -6.0f + 125.0f * raw_rh / 65535.0f;
	*rh = (h < 0.0f) ? 0.0f : (h > 100.0f) ? 100.0f : h;
	return true;
}

/*******************************************************************************
 * BMP280 / BME280
 ******************************************************************************/

static bool bmp_write8(cc_bmp280_t *b, uint8_t reg, uint8_t val)
{
	uint8_t d[2] = { reg, val };

	return b->bus->write(b->bus->ctx, b->addr, d, 2);
}

static bool bmp_readn(cc_bmp280_t *b, uint8_t reg, uint8_t *dst, uint8_t n)
{
	if (!b->bus->write(b->bus->ctx, b->addr, &reg, 1))
		return false;
	return b->bus->read(b->bus->ctx, b->addr, dst, n);
}

static uint8_t bmp_read8(cc_bmp280_t *b, uint8_t reg, bool *ok)
{
	uint8_t v = 0;

	*ok = bmp_readn(b, reg, &v, 1);
	return v;
}

static uint16_t u16le(uint8_t lo, uint8_t hi)
{
	return (uint16_t)lo | ((uint16_t)hi << 8);
}

static bool bmp_probe(cc_bmp280_t *b)
{
	bool ok;

	b->chip_id = bmp_read8(b, 0xD0, &ok);
	return ok && (b->chip_id == 0x58 || b->chip_id == 0x60);
}

static bool bmp_read_calibration(cc_bmp280_t *b)
{
	uint8_t c[24]; /* 0x88..0x9F: dig_T1..dig_P9, little-endian */

	if (!bmp_readn(b, 0x88, c, 24))
		return false;
	b->dig_t1 = u16le(c[0], c[1]);
	b->dig_t2 = (int16_t)u16le(c[2], c[3]);
	b->dig_t3 = (int16_t)u16le(c[4], c[5]);
	b->dig_p1 = u16le(c[6], c[7]);
	b->dig_p2 = (int16_t)u16le(c[8], c[9]);
	b->dig_p3 = (int16_t)u16le(c[10], c[11]);
	b->dig_p4 = (int16_t)u16le(c[12], c[13]);
	b->dig_p5 = (int16_t)u16le(c[14], c[15]);
	b->dig_p6 = (int16_t)u16le(c[16], c[17]);
	b->dig_p7 = (int16_t)u16le(c[18], c[19]);
	b->dig_p8 = (int16_t)u16le(c[20], c[21]);
	b->dig_p9 = (int16_t)u16le(c[22], c[23]);
	if (b->dig_t1 == 0 || b->dig_p1 == 0) /* all-zero calib = bus/power problem */
		return false;

	if (cc_bmp280_has_humidity(b)) { /* BME280: H1 at 0xA1, H2..H6 at 0xE1..0xE7 */
		bool ok;

		b->dig_h1 = bmp_read8(b, 0xA1, &ok);
		if (!ok)
			return false;
		uint8_t h[7];

		if (!bmp_readn(b, 0xE1, h, 7))
			return false;
		b->dig_h2 = (int16_t)u16le(h[0], h[1]);
		b->dig_h3 = h[2];
		b->dig_h4 = (int16_t)(((int16_t)(int8_t)h[3] << 4) | (h[4] & 0x0F));
		b->dig_h5 = (int16_t)(((int16_t)(int8_t)h[5] << 4) | (h[4] >> 4));
		b->dig_h6 = (int8_t)h[6];
	}
	return true;
}

bool cc_bmp280_begin(cc_bmp280_t *b, const cc_i2c_ops_t *bus, uint8_t addr)
{
	memset(b, 0, sizeof(*b));
	b->bus = bus;
	if (addr) {
		b->addr = addr;
		if (!bmp_probe(b))
			return false;
	} else {
		b->addr = 0x76;
		if (!bmp_probe(b)) {
			b->addr = 0x77;
			if (!bmp_probe(b))
				return false;
		}
	}
	return bmp_read_calibration(b);
}

/* Datasheet §3.11.3: returns temperature in degC * 100, sets tfine. */
static int32_t bmp_compensate_t(cc_bmp280_t *b, int32_t adc_t)
{
	int32_t var1 = ((((adc_t >> 3) - ((int32_t)b->dig_t1 << 1))) * ((int32_t)b->dig_t2)) >> 11;
	int32_t var2 = (((((adc_t >> 4) - ((int32_t)b->dig_t1)) *
			  ((adc_t >> 4) - ((int32_t)b->dig_t1))) >> 12) * ((int32_t)b->dig_t3)) >> 14;

	b->tfine = var1 + var2;
	return (b->tfine * 5 + 128) >> 8;
}

/* Datasheet §3.11.3 (64-bit): returns pressure in Pa as unsigned Q24.8. */
static uint32_t bmp_compensate_p(cc_bmp280_t *b, int32_t adc_p)
{
	int64_t var1 = ((int64_t)b->tfine) - 128000;
	int64_t var2 = var1 * var1 * (int64_t)b->dig_p6;

	var2 = var2 + ((var1 * (int64_t)b->dig_p5) << 17);
	var2 = var2 + (((int64_t)b->dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)b->dig_p3) >> 8) + ((var1 * (int64_t)b->dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)b->dig_p1) >> 33;
	if (var1 == 0)
		return 0;

	int64_t p = 1048576 - adc_p;

	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)b->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)b->dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)b->dig_p7) << 4);
	return (uint32_t)p;
}

/* BME280 datasheet §4.2.3: returns %RH in Q22.10 (0..100*1024). Needs tfine. */
static uint32_t bmp_compensate_h(cc_bmp280_t *b, int32_t adc_h)
{
	int32_t v = b->tfine - 76800;

	v = (((((adc_h << 14) - (((int32_t)b->dig_h4) << 20) - (((int32_t)b->dig_h5) * v)) +
	       16384) >> 15) *
	     (((((((v * (int32_t)b->dig_h6) >> 10) *
	          (((v * (int32_t)b->dig_h3) >> 11) + 32768)) >> 10) + 2097152) *
	       (int32_t)b->dig_h2 + 8192) >> 14));
	v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)b->dig_h1) >> 4;
	v = (v < 0) ? 0 : (v > 419430400) ? 419430400 : v; /* clamp 0..100 %RH */
	return (uint32_t)(v >> 12);
}

bool cc_bmp280_read_forced(cc_bmp280_t *b, float *tc, float *hpa, float *rh)
{
	const cc_i2c_ops_t *bus = b->bus;

	if (!bus)
		return false;
	/* BME280 only: ctrl_hum osrs_h=x1 — latched by the ctrl_meas write below. */
	if (cc_bmp280_has_humidity(b) && !bmp_write8(b, 0xF2, 0x01))
		return false;
	/* ctrl_meas: osrs_t=x1, osrs_p=x1, mode=forced -> 0x25 */
	if (!bmp_write8(b, 0xF4, 0x25))
		return false;
	bus->delay_ms(bus->ctx, 8); /* typ. conversion ~6.4 ms at x1/x1 */

	uint32_t t0 = bus->tick_ms(bus->ctx);

	for (;;) { /* wait for the measuring bit to clear */
		bool ok;
		uint8_t s = bmp_read8(b, 0xF3, &ok);

		if (ok && !(s & 0x08))
			break;
		if (bus->tick_ms(bus->ctx) - t0 > 50)
			return false;
		bus->delay_ms(bus->ctx, 1);
	}

	uint8_t d[8]; /* burst 0xF7..: press[3] temp[3] (hum[2] on BME) */

	if (!bmp_readn(b, 0xF7, d, cc_bmp280_has_humidity(b) ? 8 : 6))
		return false;

	int32_t adc_p = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
	int32_t adc_t = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));

	if (adc_t == 0x80000 || adc_p == 0x80000) /* reset value = no conversion ran */
		return false;

	*tc  = bmp_compensate_t(b, adc_t) / 100.0f; /* sets tfine (needed below) */
	*hpa = bmp_compensate_p(b, adc_p) / 25600.0f; /* Pa Q24.8 -> hPa */
	if (cc_bmp280_has_humidity(b) && rh) {
		int32_t adc_h = (int32_t)(((int32_t)d[6] << 8) | d[7]);

		if (adc_h != 0x8000) /* 0x8000 = channel skipped */
			*rh = bmp_compensate_h(b, adc_h) / 1024.0f;
	}
	return true;
}

/*******************************************************************************
 * AHT20
 ******************************************************************************/

#define CC_AHT20_ADDR 0x38

static uint8_t aht_status(cc_aht20_t *a, bool *ok)
{
	uint8_t s = 0;

	*ok = a->bus->read(a->bus->ctx, CC_AHT20_ADDR, &s, 1);
	return *ok ? s : 0;
}

bool cc_aht20_begin(cc_aht20_t *a, const cc_i2c_ops_t *bus)
{
	a->bus = bus;
	bus->delay_ms(bus->ctx, 40); /* power-on settle before first command */

	bool ok = false;
	uint8_t s = aht_status(a, &ok);

	if (!ok)
		return false; /* no ACK: not present / rail down */
	if (!(s & 0x08)) { /* CAL clear -> send init */
		const uint8_t init[3] = { 0xBE, 0x08, 0x00 };

		if (!bus->write(bus->ctx, CC_AHT20_ADDR, init, 3))
			return false;
		bus->delay_ms(bus->ctx, 10);
		s = aht_status(a, &ok);
		if (!ok || !(s & 0x08))
			return false;
	}
	return true;
}

bool cc_aht20_read(cc_aht20_t *a, float *rh, float *tc)
{
	const cc_i2c_ops_t *bus = a->bus;
	const uint8_t trig[3] = { 0xAC, 0x33, 0x00 };

	if (!bus)
		return false;
	if (!bus->write(bus->ctx, CC_AHT20_ADDR, trig, 3))
		return false;
	bus->delay_ms(bus->ctx, 80); /* typical conversion time */

	uint32_t t0 = bus->tick_ms(bus->ctx);

	for (;;) { /* poll BUSY until the data is ready */
		bool ok = false;
		uint8_t s = aht_status(a, &ok);

		if (ok && !(s & 0x80))
			break;
		if (bus->tick_ms(bus->ctx) - t0 > 100)
			return false;
		bus->delay_ms(bus->ctx, 5);
	}

	uint8_t b[7];

	if (!bus->read(bus->ctx, CC_AHT20_ADDR, b, 7))
		return false;
	if (crc8_31(b, 6) != b[6])
		return false;

	uint32_t raw_h = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
	uint32_t raw_t = (((uint32_t)b[3] & 0x0F) << 16) | ((uint32_t)b[4] << 8) | b[5];

	*rh = raw_h * (100.0f / 1048576.0f);
	*tc = raw_t * (200.0f / 1048576.0f) - 50.0f;
	return true;
}
