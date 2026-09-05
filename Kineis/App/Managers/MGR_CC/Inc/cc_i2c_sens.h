/**
 * @file    cc_i2c_sens.h
 * @brief   Env sensor drivers — SHT45, BMP280/BME280, AHT20 — over an injected
 *          I2C bus-ops table.
 *
 * Ports of the WS85-Tracker single-shot drivers, re-based from Arduino Wire
 * onto a tiny ops vtable so the register logic stays MCU-free (host-testable
 * with a scripted bus) and the target build plugs in HAL I2C2 (cc_hw.c).
 *
 * Power model (unchanged from the original): the sensors sit on the
 * VCCIO-gated port and are COLD-BOOTED every wake — each *_begin() re-probes
 * (and re-reads calibration) every session; nothing persists across rail-downs.
 */

#ifndef CC_I2C_SENS_H
#define CC_I2C_SENS_H

#include <stdint.h>
#include <stdbool.h>

/** Bus operations. Each returns false on NACK/bus error.
 *  read after write uses STOP+START (all three chips accept it). */
typedef struct {
	bool (*write)(void *ctx, uint8_t addr7, const uint8_t *data, uint16_t len);
	bool (*read)(void *ctx, uint8_t addr7, uint8_t *data, uint16_t len);
	void (*delay_ms)(void *ctx, uint32_t ms);
	uint32_t (*tick_ms)(void *ctx);          /**< monotonic ms, for poll timeouts */
	void *ctx;
} cc_i2c_ops_t;

/* ---- SHT45 (Sensirion SHT4x, addr 0x44) --------------------------------- */

typedef struct {
	const cc_i2c_ops_t *bus;
	uint8_t addr;
} cc_sht45_t;

/** Probe: request the serial number and CRC-check it (a random ACK can't
 *  false-positive). Call after every rail-up. */
bool cc_sht45_begin(cc_sht45_t *s, const cc_i2c_ops_t *bus, uint8_t addr);
/** One high-precision measurement (~10 ms). Outputs untouched on failure. */
bool cc_sht45_read(cc_sht45_t *s, float *rh, float *tc);

/* ---- BMP280 / BME280 (addr 0x76/0x77) ----------------------------------- */

typedef struct {
	const cc_i2c_ops_t *bus;
	uint8_t addr;
	uint8_t chip_id;      /**< 0x58 = BMP280, 0x60 = BME280 */
	uint16_t dig_t1; int16_t dig_t2, dig_t3;
	uint16_t dig_p1; int16_t dig_p2, dig_p3, dig_p4, dig_p5,
	                        dig_p6, dig_p7, dig_p8, dig_p9;
	uint8_t  dig_h1, dig_h3;
	int16_t  dig_h2, dig_h4, dig_h5;
	int8_t   dig_h6;
	int32_t  tfine;
} cc_bmp280_t;

/** Probe (chip id) + load calibration. @param addr 0 = auto-probe 0x76 then 0x77. */
bool cc_bmp280_begin(cc_bmp280_t *b, const cc_i2c_ops_t *bus, uint8_t addr);
/**
 * One forced measurement (osrs x1 — Bosch "weather monitoring"). ~10 ms.
 * @param rh on a BME280, filled with %RH; on a BMP280 left untouched (pass
 *           NULL if not wanted). Outputs untouched on failure.
 */
bool cc_bmp280_read_forced(cc_bmp280_t *b, float *tc, float *hpa, float *rh);
static inline bool cc_bmp280_has_humidity(const cc_bmp280_t *b)
{
	return b->chip_id == 0x60;
}

/* ---- AHT20 (fixed addr 0x38) --------------------------------------------- */

typedef struct {
	const cc_i2c_ops_t *bus;
} cc_aht20_t;

/** Probe + (re)initialise after a cold power-up (>= 40 ms settle inside). */
bool cc_aht20_begin(cc_aht20_t *a, const cc_i2c_ops_t *bus);
/** One triggered measurement (~80 ms). False on NACK/stuck-busy/CRC fail. */
bool cc_aht20_read(cc_aht20_t *a, float *rh, float *tc);

#endif /* CC_I2C_SENS_H */
