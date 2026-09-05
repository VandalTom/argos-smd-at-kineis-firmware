/**
 * @file    cc_ws85.h
 * @brief   Ecowitt WS85 broadcast-frame parser (RX-only port of WS85-Arduino).
 *
 * Decodes the sensor's auto-streamed measurement frame (Modbus-RTU style,
 * CRC16 poly 0xA001 reflected):
 *   85 03 1C <28 data> CRC   broadcast measurement (14 regs, 0x0160..0x016D)
 *
 * RX-only by design: the publish cycle never writes to the sensor (register
 * writes — baud, auto-send interval — are bench operations done with the
 * WS85-Arduino console). Byte-feed API with no time or UART dependency, so
 * the frame machine is unit-tested on the host; the app pumps bytes from the
 * USART1 session-B ring into it.
 *
 * Channel validity: each measurement has a has_* flag (raw 0xFFFF = absent).
 * On the WS85, temp/wind/gust/dir/rain are always present and light/UV/
 * humidity/pressure are always absent.
 */

#ifndef CC_WS85_H
#define CC_WS85_H

#include <stdint.h>
#include <stdbool.h>

/** One decoded broadcast measurement frame. */
typedef struct {
	bool     valid;          /**< true when this struct holds a parsed frame */
	uint16_t model;          /**< 0x0085 for WS85, 0x0090 for WS90 */
	float    temp_c;         bool has_temp;
	float    wind_ms;        bool has_wind;
	float    gust_ms;        bool has_gust;
	uint16_t dir_deg;        bool has_dir;   /**< 0..359 */
	float    rain_mm;        bool has_rain;  /**< cumulative counter */
} cc_ws85_frame_t;

/** Parser state — zero-init (or cc_ws85_init) before a capture session. */
typedef struct {
	uint8_t  addr;           /**< Modbus slave address (0x85) */
	uint8_t  buf[64];
	uint8_t  pos;
	uint8_t  func;
	uint8_t  frame_len;
	uint32_t frames;         /**< CRC-valid broadcasts seen */
	uint32_t crc_errors;     /**< frames dropped on CRC mismatch */
} cc_ws85_parser_t;

/** Reset the parser for a fresh capture session. */
void cc_ws85_init(cc_ws85_parser_t *p, uint8_t addr);

/**
 * @brief Feed one received byte.
 * @param out filled when a complete CRC-valid broadcast decodes on this byte.
 * @return true when @p out was just filled.
 */
bool cc_ws85_feed(cc_ws85_parser_t *p, uint8_t b, cc_ws85_frame_t *out);

/** Modbus CRC16 (poly 0xA001 reflected, init 0xFFFF) — exposed for tests. */
uint16_t cc_ws85_crc16(const uint8_t *data, uint16_t len);

#endif /* CC_WS85_H */
