/**
 * @file    cc_nmea.h
 * @brief   Minimal talker-agnostic NMEA parser (RMC + GGA + GSV) — port of GpsNmea.h.
 *
 * RX-only. Feed bytes from the GPS UART session; each sentence is checksum-
 * verified, so a corrupted byte just drops that sentence. A fix is accepted
 * only when RMC says "active" AND the quality gate passes (GGA quality >= 1
 * and sats >= 4) — a real 3D fix, not a coarse one.
 *
 * GSV feeds a "sky view": how many satellites the receiver tracks with a
 * usable carrier-to-noise ratio (>= CC_GPS_SKY_MIN_CNO dBHz), per
 * constellation from the latest complete GSV group. Indoors that stays at
 * 0..1 while outdoors it is several within seconds — the acquisition loop
 * uses it to end a hopeless window early (GPS is the dominant power draw).
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
	/* Sky view from GSV */
	uint32_t gsv_msgs;      /**< checksum-valid GSV sentences this session */
	uint8_t  gsv_strong[5]; /**< strong sats per constellation (GP, GL, GA, GB/BD, other) */
	uint8_t  gsv_cur;       /**< strong sats accumulating in the group being received */
	uint8_t  max_cno;       /**< best C/N0 seen this session, dBHz */
} cc_nmea_parser_t;

/** Reset the parser (call before each acquisition session). */
void cc_nmea_init(cc_nmea_parser_t *p);

/** Feed one byte; returns true when a fresh VALID fix was just completed. */
bool cc_nmea_feed(cc_nmea_parser_t *p, uint8_t c);

/** The most recent RMC snapshot (check .valid). */
const cc_gps_fix_t *cc_nmea_fix(const cc_nmea_parser_t *p);

/** Satellites tracked at >= CC_GPS_SKY_MIN_CNO dBHz, all constellations. */
uint8_t cc_nmea_sky_strong(const cc_nmea_parser_t *p);

/**
 * @brief True when this window can be abandoned: at least @p check_ms in,
 *        GSV is being received (otherwise we cannot judge — never abort),
 *        and fewer than @p min_strong satellites are tracked strongly.
 */
bool cc_nmea_no_sky(const cc_nmea_parser_t *p, uint32_t elapsed_ms, uint32_t check_ms,
                    uint8_t min_strong);

#endif /* CC_NMEA_H */
