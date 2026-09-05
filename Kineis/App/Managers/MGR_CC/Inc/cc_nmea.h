/**
 * @file    cc_nmea.h
 * @brief   Minimal talker-agnostic NMEA parser (RMC + GGA) — port of GpsNmea.h.
 *
 * RX-only. Feed bytes from the GPS UART session; each sentence is checksum-
 * verified, so a corrupted byte just drops that sentence. A fix is accepted
 * only when RMC says "active" AND the quality gate passes (GGA quality >= 1
 * and sats >= 4) — a real 3D fix, not a coarse one.
 *
 * Pure C, no time or UART dependency — unit-tested on the host. The caller
 * stamps accepted fixes with its own tick for TTFF bookkeeping.
 */

#ifndef CC_NMEA_H
#define CC_NMEA_H

#include <stdint.h>
#include <stdbool.h>

/** One GPS fix snapshot. */
typedef struct {
	bool     valid;      /**< RMC status A AND quality gate */
	double   lat;        /**< decimal degrees, +N / -S */
	double   lon;        /**< decimal degrees, +E / -W */
	uint8_t  sats;
	uint8_t  quality;    /**< GGA fix quality (0 none, 1 GPS, 2 DGPS...) */
	uint32_t epoch;      /**< Unix UTC seconds (from RMC date+time), 0 unknown */
} cc_gps_fix_t;

typedef struct {
	char     line[100];
	uint16_t len;
	bool     have;
	cc_gps_fix_t fix;
	uint32_t sentences;
	uint32_t cs_errors;
	uint8_t  last_sats;   /**< from the latest GGA (live, even with no fix) */
	uint8_t  last_qual;
} cc_nmea_parser_t;

/** Reset the parser (call before each acquisition session). */
void cc_nmea_init(cc_nmea_parser_t *p);

/** Feed one byte; returns true when a fresh VALID fix was just completed. */
bool cc_nmea_feed(cc_nmea_parser_t *p, uint8_t c);

/** The most recent RMC snapshot (check .valid). */
const cc_gps_fix_t *cc_nmea_fix(const cc_nmea_parser_t *p);

#endif /* CC_NMEA_H */
