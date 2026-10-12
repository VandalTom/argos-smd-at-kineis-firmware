/**
 * @file    cc_selftest.h
 * @brief   CC_TRACKER self-test: item set, verdict rules and console report format.
 *
 * The self-test runs once at boot (after the MAC is ready) and on demand via
 * AT+SELFTEST[=items]. Every check is chosen to finish in seconds on a
 * production line, without waiting for the sky:
 *   GPS  : a checksum-valid NMEA sentence arrives (proves rail, wiring, UART
 *          swap and baud) — no fix needed. It also diagnoses a failure: the
 *          data line is probed with the receiver switched off and on, it
 *          listens while the receiver should be off (enable check), and a
 *          silent-but-driven line triggers a baud scan (a hit is adopted)
 *   WS85 : one CRC-valid broadcast (sensor broadcasts every ~30 s)
 *   ENV  : BMP/BME280 forced read with plausible values
 *   ADC  : NiMH pack within range (supercap reported)
 *   TX   : one payload frame, PASS on the MAC's TX_DONE (radio-level
 *          completion; no satellite pass involved)
 *
 * Report (one line per item that ran, then a summary), e.g.
 *   +ST=GPS,PASS,bytes=812,nmea=1,cs_err=0,fe=0,line=lo/hi
 *   +ST=TX,FAIL,timeout
 *   +SELFTEST=FAIL,TX
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_SELFTEST_H
#define CC_SELFTEST_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
	CC_STI_GPS = 0,
	CC_STI_WS85,
	CC_STI_ENV,
	CC_STI_ADC,
	CC_STI_TX,
	CC_STI_COUNT,
} cc_st_item_t;

#define CC_ST_BIT(i)   ((uint8_t)(1u << (i)))
#define CC_ST_ALL      ((uint8_t)((1u << CC_STI_COUNT) - 1u))

typedef enum {
	CC_STV_NOTRUN = 0,
	CC_STV_PASS,
	CC_STV_FAIL,
	CC_STV_SKIP,      /**< item compiled out (CC_ENABLE_* = 0) */
} cc_st_verdict_t;

#define CC_ST_DETAIL_LEN 80

typedef struct {
	cc_st_verdict_t v;
	char detail[CC_ST_DETAIL_LEN];
} cc_st_result_t;

typedef struct {
	cc_st_result_t item[CC_STI_COUNT];
	uint8_t mask;        /**< items requested for this run */
	bool    running;
	bool    done;        /**< a run has completed since boot */
	bool    at_boot;     /**< the completed run was the boot self-test */
} cc_selftest_t;

/** Start a run: clear all items, remember the mask. */
void cc_selftest_begin(cc_selftest_t *st, uint8_t mask, bool at_boot);

/** Record one item's verdict and detail (detail may be NULL). */
void cc_selftest_set(cc_selftest_t *st, cc_st_item_t i, cc_st_verdict_t v, const char *detail);

/** Close the run. */
void cc_selftest_end(cc_selftest_t *st);

/** PASS when no requested item FAILed or stayed NOTRUN (SKIP is neutral). */
bool cc_selftest_passed(const cc_selftest_t *st);

/**
 * @brief Parse the AT+SELFTEST parameter list into an item mask.
 * @param params text after '=' up to CR/LF/NUL; NULL or empty = all items.
 *        Comma-separated, case-sensitive: GPS, WS85, ENV, ADC, TX, ALL.
 * @return mask, or 0 when any token is unknown.
 */
uint8_t cc_selftest_parse_mask(const char *params);

const char *cc_selftest_item_name(cc_st_item_t i);
const char *cc_selftest_verdict_name(cc_st_verdict_t v);

/** "+ST=<ITEM>,<VERDICT>[,<detail>]\r\n" — returns chars written (snprintf-style). */
int cc_selftest_format_item(char *buf, size_t n, const cc_selftest_t *st, cc_st_item_t i);

/** "+SELFTEST=PASS\r\n" or "+SELFTEST=FAIL,<ITEM>[,<ITEM>...]\r\n". */
int cc_selftest_format_summary(char *buf, size_t n, const cc_selftest_t *st);

/* ---- Item verdict rules (thresholds passed in from cc_conf.h) ---- */

/** GPS: at least one checksum-valid NMEA sentence. */
cc_st_verdict_t cc_selftest_eval_gps(uint32_t bytes, uint32_t sentences);

/* ---- GPS diagnosis ----
 * What drives the receiver's data line, from two static samples of the RX
 * pin: one with the MCU's pull-down on, one with its pull-up on. A line that
 * follows both pulls is driven by nothing. */
typedef enum {
	CC_LINE_UNKNOWN = 0,  /**< not probed */
	CC_LINE_FLOAT,        /**< "fl": follows the pulls, nothing drives it */
	CC_LINE_HIGH,         /**< "hi": held high (an idle UART output, or a pull-up resistor) */
	CC_LINE_LOW,          /**< "lo": held low (an unpowered receiver clamps it, or a short) */
	CC_LINE_ACTIVE,       /**< "act": seen high and low against the pulls: data moving */
} cc_line_t;

/** What the receiver said while it should have been switched off. */
typedef enum {
	CC_OFF_SILENT = 0,    /**< nothing (as it should be) */
	CC_OFF_BYTES,         /**< bytes but no valid sentence */
	CC_OFF_NMEA,          /**< valid NMEA: the enable does not switch it off */
} cc_gps_off_t;

typedef struct {
	uint32_t     bytes;       /**< received with the receiver switched on, at baud */
	uint32_t     sentences;   /**< checksum-valid NMEA sentences in that window */
	uint32_t     cs_errors;
	uint32_t     line_errors; /**< UART framing/noise errors in that window */
	cc_line_t    off_line;    /**< data line before switching on */
	cc_line_t    on_line;     /**< data line after the window */
	cc_gps_off_t off_heard;
	uint32_t     baud;        /**< the window's baud */
	bool         scanned;     /**< other bauds were tried */
	uint32_t     scan_baud;   /**< the baud a scan found NMEA at (0 = none) */
} cc_st_gps_t;

cc_line_t cc_selftest_line_class(bool high_when_pulled_down, bool low_when_pulled_up);
const char *cc_selftest_line_name(cc_line_t l);

/** PASS: NMEA arrived (at the configured baud, or at one a scan found) and
 *  the receiver stayed quiet while switched off. */
cc_st_verdict_t cc_selftest_eval_gps_diag(const cc_st_gps_t *g);

/**
 * "bytes=<n>,nmea=<n>,cs_err=<n>,fe=<n>,line=<off>/<on>[,off=bytes|nmea][,baud=<b>|none]"
 * off=  only when the receiver was heard while switched off;
 * baud= only after a scan: the baud NMEA was found at, or none.
 */
int cc_selftest_format_gps(char *buf, size_t n, const cc_st_gps_t *g);

/** ENV: -40..85 C, 300..1100 hPa, humidity 0..100 %RH (rh < 0 = not a BME280). */
cc_st_verdict_t cc_selftest_eval_env(float tc, float hpa, float rh);

/** ADC: NiMH within [min_mv, max_mv]; the supercap is informational. */
cc_st_verdict_t cc_selftest_eval_adc(uint16_t nimh_mv, uint16_t min_mv, uint16_t max_mv);

#endif /* CC_SELFTEST_H */
