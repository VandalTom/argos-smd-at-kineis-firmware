/**
 * @file    cc_ws85.c
 * @brief   WS85 broadcast-frame parser — RX-only port of WS85-Arduino WS85.h.
 *
 * The frame state machine and scaling are ported verbatim; the request/reply
 * register access (function 0x06 writes, targeted 0x03 reads) is dropped —
 * the tracker only ever listens to the ~30 s broadcast. Error and short-read
 * frames are still recognised so a config reply can't desync the parser.
 */

#include <string.h>
#include "cc_ws85.h"

void cc_ws85_init(cc_ws85_parser_t *p, uint8_t addr)
{
	memset(p, 0, sizeof(*p));
	p->addr = addr;
}

uint16_t cc_ws85_crc16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++)
			crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
	}
	return crc; /* transmitted low byte first */
}

/* Decode the 14-register broadcast into a frame. Raw 0xFFFF = channel absent. */
static void parse_measurement(const cc_ws85_parser_t *p, cc_ws85_frame_t *out)
{
	const uint8_t *b = p->buf;
	uint16_t regs[14];

	for (int i = 0; i < 14; i++)
		regs[i] = (uint16_t)((uint16_t)b[3 + 2 * i] << 8) | b[4 + 2 * i];

	memset(out, 0, sizeof(*out));
	out->valid = true;
	out->model = regs[0];

	/* reg 7 temp, 9 wind, 10 gust, 11 dir, 12 rain (regs 5/6/8/13 = light/UV/
	 * humidity/pressure — never present on a WS85, so not carried here) */
	if (regs[7] != 0xFFFF) {
		out->temp_c = regs[7] / 10.0f - 40.0f;
		out->has_temp = true;
	}
	if (regs[9] != 0xFFFF) {
		out->wind_ms = regs[9] / 10.0f;
		out->has_wind = true;
	}
	if (regs[10] != 0xFFFF) {
		out->gust_ms = regs[10] / 10.0f;
		out->has_gust = true;
	}
	if (regs[11] != 0xFFFF) {
		out->dir_deg = regs[11];
		out->has_dir = true;
	}
	if (regs[12] != 0xFFFF) {
		out->rain_mm = regs[12] / 10.0f;
		out->has_rain = true;
	}
}

/* Completed frame in buf[0..frame_len-1]: CRC-check then dispatch. */
static bool process(cc_ws85_parser_t *p, cc_ws85_frame_t *out)
{
	uint16_t rxcrc = (uint16_t)p->buf[p->frame_len - 2]
		| ((uint16_t)p->buf[p->frame_len - 1] << 8);

	if (cc_ws85_crc16(p->buf, (uint16_t)(p->frame_len - 2)) != rxcrc) {
		p->crc_errors++;
		return false;
	}

	if (p->func & 0x80)          /* error reply — recognised, not consumed */
		return false;
	if (p->func == 0x06)         /* write echo — bench traffic, ignore */
		return false;
	if (p->func == 0x03 && p->buf[2] == 0x1C) {  /* the broadcast */
		parse_measurement(p, out);
		p->frames++;
		return true;
	}
	return false;                 /* targeted register read — ignore */
}

bool cc_ws85_feed(cc_ws85_parser_t *p, uint8_t b, cc_ws85_frame_t *out)
{
	if (p->pos == 0) {
		if (b == p->addr)
			p->buf[p->pos++] = b;
		return false;
	}
	if (p->pos == 1) {
		if (b == 0x03 || b == 0x06 || b == 0x83 || b == 0x86) {
			p->buf[1] = b;
			p->func = b;
			p->pos = 2;
		} else {
			p->pos = 0;
			if (b == p->addr)
				p->buf[p->pos++] = b;
		}
		return false;
	}
	if (p->pos == 2) {
		p->buf[2] = b;
		if (p->func & 0x80) {
			p->frame_len = 5;          /* error frame: code + CRC */
		} else {
			if (b == 0 || b > 56) {    /* implausible byte count: resync */
				p->pos = 0;
				if (b == p->addr)
					p->buf[p->pos++] = b;
				return false;
			}
			p->frame_len = (uint8_t)(3 + b + 2);
		}
		p->pos = 3;
		return false;
	}

	p->buf[p->pos++] = b;
	if (p->pos < p->frame_len)
		return false;
	p->pos = 0;
	return process(p, out);
}
