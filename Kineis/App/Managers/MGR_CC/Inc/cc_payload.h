/**
 * @file    cc_payload.h
 * @brief   CounterCurrent uplink payload v3: 24 bytes, big-endian, bit-packed.
 *
 * Byte-exact port of the WS85-Tracker Payload.h packer (address byte 0xC1).
 * The layout is frozen — the deployed decoder parses it today:
 *
 *      off  len  field          encoding (big-endian)
 *      ---  ---  -------------  ------------------------------------------
 *        0   1   address/ver    = 0xC1
 *        1   3   latitude       lat * (2^23 / 90),  signed 24-bit two's-comp
 *        4   3   longitude      lon * (2^23 / 180), signed 24-bit two's-comp
 *        7   3   GPS timestamp  minutes since CC_TS_EPOCH_UNIX; 0 = never seeded
 *       10   2   wake time      uptime in 65.536 s ticks; resets to 0 on reboot
 *       12   5   WS85 group     [ RAIN:13 | DIR:9 | GUST:9 | SPD:9 ]  (40 bits)
 *       17   3   BMP280 group   [ TEMP:10 | PRESS:14 ]                (24 bits)
 *       20   1   battery        (mV - 2000) / 10, 2.00-4.55 V (0 = <=2 V/unknown)
 *       21   1   humidity       %RH * 2 (0.5 % steps); 0xFF invalid
 *       22   2   supercap       mV, unsigned 16-bit BE; 0xFFFF invalid
 *      ---
 *       24 bytes = 192 bits (one LDA2 user-data frame)
 *
 * Each sensor-group field's ALL-ONES value means invalid/no data. Staleness
 * policy: a cache that is not fresh at publish time is sent as the invalid
 * sentinel, never as silent old data.
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_PAYLOAD_H
#define CC_PAYLOAD_H

#include <stdint.h>
#include <stdbool.h>

#define CC_PAYLOAD_BYTES 24
#define CC_PAYLOAD_ADDR  0xC1

/** Everything the packer needs for one uplink. Zero-init = "no data": an
 *  invalid group is transmitted as its all-ones sentinel. */
typedef struct {
	/* position / time */
	double   lat;            /**< last-known fix ((0,0) until the first ever) */
	double   lon;
	uint32_t epoch;          /**< Unix UTC s (software clock); 0 = never seeded */
	uint16_t wake16;         /**< uptime ms >> 16 — 65.536 s ticks since boot */
	/* WS85 group (wind + rain) */
	bool     ws_valid;       /**< fresh at publish; false -> group all-ones */
	float    wind_ms;
	float    gust_ms;
	uint16_t dir_deg;
	float    rain_mm;        /**< cumulative counter (packed modulo 819.2 mm) */
	/* BMP280 group (temp source priority handled by the caller) */
	bool     temp_valid;
	float    temp_c;
	bool     press_valid;
	float    press_hpa;
	/* humidity */
	bool     hum_valid;
	float    hum_pct;
	/* health */
	uint16_t batt_mv;        /**< packs to byte 0 when unknown or <= 2.00 V */
	bool     scap_valid;
	uint16_t scap_mv;        /**< supercap, divider already undone */
} cc_payload_fields_t;

/** Pack the fields into exactly 24 bytes (v3 layout). */
void cc_payload_pack(uint8_t out[CC_PAYLOAD_BYTES], const cc_payload_fields_t *f);

#endif /* CC_PAYLOAD_H */
