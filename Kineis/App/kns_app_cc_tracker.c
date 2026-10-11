/**
 * @file    kns_app_cc_tracker.c
 * @brief   CC_TRACKER application — state machine and publish cycle.
 *
 * Modeled on kns_app_doppler.c (MAC queue contract, watchdog/event-log/error
 * patterns) with the CounterCurrent WS85-Tracker cycle logic ported in:
 * freshness-gated sensor caches, GPS every-Nth-TX schedule with adaptive
 * backoff, battery-tiered TX cadence with anti-phase-lock jitter, and the
 * byte-exact payload v3 packer.
 *
 * State machine:
 *   BOOT -> INIT_MAC -> WAIT_MAC_READY -> SAMPLE_WS85 -> SAMPLE_ENV
 *        -> SAMPLE_GPS -> TX_SEND -> WAIT_TX_DONE -> CYCLE_WAIT -+
 *                              ^---------------------------------+
 *
 * DPL-001 note: like DOPPLER/UW_DOPPLER, this app owns KNS_Q_UL_MAC2APP and
 * must never call MGR_AT_CMD_macEvtProcess().
 *
 * Debug console (LPUART1, 115200): read-only queries, Kineis credential
 * writes, AT+BOOT (UART DFU), AT+RESET, AT+SELFTEST, AT+STATE and
 * AT+COMMISSION — everything else is refused (cc_console.c). Actions that
 * would disturb a transmission wait until the radio is idle.
 *
 * Self-test (cc_selftest.c): once per boot right after MAC ready, and on
 * demand. GPS = NMEA arrives, TX = MAC TX_DONE — line-test fast.
 *
 * Commissioning (cc_commission.c): the first GPS fix since power-up starts a
 * 30-minute burst of one TX per minute so a freshly deployed unit is heard
 * quickly; the "done" flag lives in TAMP BKP12R (cleared by power loss only).
 */

/**
 * @addtogroup KNS_APP_CC_TRACKER
 * @{
 */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "main.h"

#if defined(USE_CC_TRACKER_APP)

#include "kns_app_cc_tracker.h"
#include "stm32wlxx_hal.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kns_cfg.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mgr_lpm.h"
#include "mgr_wdg.h"
#include "mgr_err.h"
#include "mgr_evtlog.h"
#include "mgr_bat.h"
#include "mgr_at_cmd.h"
#include "mgr_at_cmd_common.h"
#include "mcu_at_console.h"
#include "mcu_aes.h"
#include "mcu_misc.h"

#include "cc_conf.h"
#include "cc_console.h"
#include "cc_selftest.h"
#include "cc_commission.h"
#include "cc_sleep.h"
#include "cc_clock.h"
#include "rtc.h"
#include "mgr_cred.h"
#include "cc_hw.h"
#include "cc_payload.h"
#include "cc_tier.h"
#include "cc_ws85.h"
#include "cc_nmea.h"
#include "cc_i2c_sens.h"

#ifndef USE_MAC_PRFL_BASIC
#error "CC_TRACKER owns its own scheduling: build with MAC_PRFL=BASIC."
#endif
#if defined(USE_SPI_DRIVER)
#error "CC_TRACKER is UART-console only: build with COMM=UART."
#endif

/* ---- State machine ---- */

typedef enum {
	CC_ST_BOOT,
	CC_ST_INIT_MAC,
	CC_ST_WAIT_MAC_READY,
	CC_ST_SAMPLE_WS85,
	CC_ST_SAMPLE_ENV,
	CC_ST_SAMPLE_GPS,
	CC_ST_TX_SEND,
	CC_ST_WAIT_TX_DONE,
	CC_ST_CYCLE_WAIT,
	CC_ST_SELFTEST,      /**< appended: keeps the evtlog/err numbering stable */
	CC_ST_RESCUE,        /**< console-only after repeated failed boots */
	CC_ST_NO_CREDS,      /**< unprovisioned: MAC never started, console up */
} cc_state_t;

static const char *const cc_state_names[] = {
	"BOOT", "INIT_MAC", "WAIT_MAC_READY", "SAMPLE_WS85", "SAMPLE_ENV",
	"SAMPLE_GPS", "TX_SEND", "WAIT_TX_DONE", "CYCLE_WAIT", "SELFTEST", "RESCUE",
	"NO_CREDS",
};
_Static_assert(sizeof(cc_state_names) / sizeof(cc_state_names[0]) == CC_ST_NO_CREDS + 1,
	"cc_state_names must list every cc_state_t");

static cc_state_t cc_state;

/* Exported for fault handlers (mgr_err.c / mgr_evtlog.c reference this name) */
volatile uint32_t g_uw_doppler_state_for_err = 0;

#define TIMEOUT_BOOT_MS 3000UL /**< console settle / banner window */

/* ---- Cycle bookkeeping ---- */

static uint32_t state_enter_tick;
static uint32_t cycle_start_tick;   /**< period is measured from cycle start */
static uint32_t cycle_period_ms;    /**< this cycle's tier base + jitter */
static uint8_t  mac_init_retries;
static uint32_t tx_count;           /**< TX attempts since boot (GPS schedule) */
static uint8_t  tx_fail_streak;     /**< cycles in a row whose frame never completed */
static bool     tx_queue_refused;   /**< last push_tx failed at the MAC queue */
static bool     tx_since_boot;      /**< a frame went out since boot */
static uint32_t last_tx_cycle;      /**< cycle_start_tick of that frame's cycle */

static cc_tier_state_t tier;
static uint32_t cycle_base_ms = CC_TX_PERIOD_MS; /**< tier base from this cycle's reading */

/** This cycle's sensor fields, frozen at the end of SAMPLE_ENV (the original
 *  fills each group straight after its refresh): freshness is judged there,
 *  so a long GPS window can't age wind/env/battery out of the packet or the
 *  battery out of the tier. TX_SEND adds position, time and wake counter. */
static cc_payload_fields_t cycle_fields;

/* ---- Console requests, self-test, commissioning ---- */

typedef enum {
	CC_ACT_NONE = 0,
	CC_ACT_RESET,
	CC_ACT_DFU,
} cc_act_t;

static cc_act_t pending_act;        /**< AT+RESET / AT+BOOT waiting for radio idle */
static bool mac_ready;              /**< MAC init OK at least once since boot */
static bool boot_selftest_done;
static bool boot_stable;            /**< up CC_BOOT_STABLE_MS: failed-boot counter cleared */
static bool rescue;                 /**< console-only boot (see cc_conf.h rescue mode) */
static bool creds_ok;               /**< ID/ADDR/SECKEY provisioned (or test creds allowed) */

typedef enum {
	STS_GPS, STS_WS85, STS_ENV, STS_ADC, STS_TX, STS_TX_WAIT, STS_DONE,
} cc_st_step_t;

static cc_selftest_t st;            /**< current / last self-test report */
static uint8_t st_req_mask;         /**< AT+SELFTEST waiting for a safe point */
static cc_st_step_t st_step;
static bool st_entry;               /**< first call of the current step */
static uint32_t st_step_tick;
static uint32_t st_bytes;
static int st_tx_evt;               /**< 0 waiting, 1 TX_DONE, -1 timeout, -2 abort, -3 MAC error */
static int st_tx_status;

static cc_comm_t comm;

static void after_mac_ready(void);
static void tx_finished(void);
static void tx_outcome(bool ok);

/* ---- Wall clock (GPS time anchored to the RTC calendar; see cc_clock.h) ----
 * Unix = anchor + RTC seconds. The anchor and its complement live in TAMP
 * BKP8R/BKP9R, so resets keep it; power loss clears it with the calendar.
 * 0 until the first fix since power-up. */

static uint32_t clk_anchor;
static bool     clk_valid;

static uint32_t rtc_seconds(void)
{
	RTC_TimeTypeDef t;
	RTC_DateTypeDef d;

	if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK)
		return 0;
	/* The date read also unlocks the shadow registers (RM: time, then date) */
	if (HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN) != HAL_OK)
		return 0;
	return cc_clock_rtc_seconds(d.Year, d.Month, d.Date, t.Hours, t.Minutes, t.Seconds);
}

static uint32_t clock_now(void)
{
	return clk_valid ? clk_anchor + rtc_seconds() : 0;
}

static void clock_seed(uint32_t epoch)
{
	if (!cc_clock_epoch_plausible(epoch, CC_TS_EPOCH_UNIX))
		return;
	clk_anchor = epoch - rtc_seconds();
	clk_valid = true;
	TAMP->BKP9R = 0;            /* invalid while the pair is half-written */
	TAMP->BKP8R = clk_anchor;
	TAMP->BKP9R = ~clk_anchor;
}

/** Boot: adopt the anchor a previous run (same power-up) stored. */
static void clock_restore(void)
{
	uint32_t a = TAMP->BKP8R;

	if (!cc_clock_anchor_valid(a, TAMP->BKP9R))
		return;
	clk_anchor = a;
	clk_valid = true;
	MGR_LOG_INFO("[CLK] kept across reset: unix %lu\r\n", (unsigned long)clock_now());
}

/** Time since power-up, not since the last reset: the RTC calendar starts at
 *  2000-01-01 00:00 when the backup domain powers up and runs through every
 *  reset (the commissioning windows are defined from power-up). Saturates
 *  instead of wrapping at 49.7 days. */
static uint32_t powerup_ms(void)
{
	uint32_t s = rtc_seconds();

	return (s >= 0xFFFFFFFFUL / 1000UL) ? 0xFFFFFFFFUL : s * 1000UL;
}

/* ---- Freshness-gated caches (WS85-Tracker Cached<T> pattern) ---- */

typedef struct {
	uint32_t at_ms;
	bool     valid;
} cc_cache_meta_t;

static bool cache_fresh(const cc_cache_meta_t *m, uint32_t max_age_ms)
{
	return m->valid && (HAL_GetTick() - m->at_ms) <= max_age_ms;
}

static void cache_stamp(cc_cache_meta_t *m)
{
	m->at_ms = HAL_GetTick();
	m->valid = true;
}

#if CC_ENABLE_WS85
static struct {
	cc_cache_meta_t meta;
	float temp_c, wind_ms, gust_ms, rain_mm;
	uint16_t dir_deg;
} ws_cache;
static cc_ws85_parser_t ws_parser;
static bool ws_session_open;
#endif

#if CC_ENABLE_ENV_I2C
#if CC_ENABLE_SHT45
static struct { cc_cache_meta_t meta; float rh, tc; }  sht_cache;   /* SHT45 */
#endif
#if CC_ENABLE_BMP280
static struct { cc_cache_meta_t meta; float tc, hpa; } baro_cache;  /* BMP/BME */
static struct { cc_cache_meta_t meta; float rh, tc; }  bmehum_cache;/* BME280 RH */
#endif
#if CC_ENABLE_AHT20
static struct { cc_cache_meta_t meta; float rh, tc; }  aht_cache;   /* AHT20 */
#endif
#endif

#if CC_ENABLE_ADC_SENSE
static struct { cc_cache_meta_t meta; uint16_t nimh_mv, scap_mv; } adc_cache;
#endif

/* GPS: last-known fix is always sent (lat/lon have no invalid sentinel) */
#if CC_ENABLE_GPS
static double  gps_lat, gps_lon;
static bool    gps_ever_fixed;
static cc_nmea_parser_t nmea;
static uint32_t gps_budget_ms;    /**< this acquisition's chained window budget */
static uint8_t gps_durations = 1; /**< windows next acquisition chains (1..MAX) */
static uint8_t gps_skip_cycles;   /**< backoff skips after a miss */
static uint8_t gps_countdown;     /**< scheduled skips until next attempt */
static bool    gps_fast;          /**< this acquisition is a commissioning probe */
#endif

/* ---- Helpers ---- */

static void transition_to(cc_state_t s)
{
	cc_state = s;
	g_uw_doppler_state_for_err = (uint32_t)s;
	state_enter_tick = HAL_GetTick();
	MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)s);
}

static uint32_t state_elapsed_ms(void)
{
	return HAL_GetTick() - state_enter_tick;
}

/* ---- MAC ---- */

static void start_mac_profile(void)
{
	struct KNS_MAC_appEvt_t appEvt;

	appEvt.id = KNS_MAC_INIT;
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BASIC;

	if (KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt) != KNS_STATUS_OK)
		MGR_LOG_DEBUG("[CC] MAC init push failed\r\n");
}

static bool process_mac_events(void)
{
	bool got = false;
	enum KNS_status_t status;
	struct KNS_MAC_srvcEvt_t evt;

	for (status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&evt);
	     status != KNS_STATUS_QEMPTY;
	     status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&evt)) {
		if (status != KNS_STATUS_OK)
			continue;
		got = true;

		switch (evt.id) {
		case KNS_MAC_OK:
			if (evt.app_evt == KNS_MAC_INIT) {
				MGR_LOG_DEBUG("[CC] MAC init OK\r\n");
				MGR_EVTLOG_log(EVT_MAC_READY, 0);
				if (cc_state == CC_ST_WAIT_MAC_READY)
					after_mac_ready();
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[CC] TX done (#%lu)\r\n", (unsigned long)tx_count);
			MGR_EVTLOG_log(EVT_TX_DONE, (uint16_t)tx_count);
			if (cc_state == CC_ST_SELFTEST) {
				st_tx_evt = 1;
			} else if (cc_state == CC_ST_WAIT_TX_DONE) {
				tx_outcome(true);
				tx_finished();
			}
			break;

		case KNS_MAC_TX_TIMEOUT:
		case KNS_MAC_TX_ABORT:
			MGR_LOG_DEBUG("[CC] TX %s\r\n",
				(evt.id == KNS_MAC_TX_ABORT) ? "abort" : "timeout");
			MGR_ERR_log(ERR_TX_TIMEOUT);
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, (uint16_t)tx_count);
			if (cc_state == CC_ST_SELFTEST) {
				st_tx_evt = (evt.id == KNS_MAC_TX_ABORT) ? -2 : -1;
			} else if (cc_state == CC_ST_WAIT_TX_DONE) {
				tx_outcome(false);
				tx_finished();
			}
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[CC] MAC error: %d\r\n", evt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)evt.status);
			if (cc_state == CC_ST_SELFTEST) {
				st_tx_evt = -3;
				st_tx_status = evt.status;
			} else if (cc_state == CC_ST_WAIT_TX_DONE) {
				tx_outcome(false);
				tx_finished();
			} else if (cc_state == CC_ST_WAIT_MAC_READY) {
				/* Counted like an init timeout: a deterministic init
				 * error must end in a reset, not an INIT<->ERROR spin
				 * that never idles */
				MGR_ERR_log(ERR_MAC_INIT_FAIL);
				if (++mac_init_retries >= CC_MAC_INIT_MAX_RETRIES)
					MGR_ERR_logAndReset(ERR_MAC_INIT_FAIL);
				transition_to(CC_ST_INIT_MAC);
			}
			break;

		default:
			break;
		}
	}
	return got;
}

/* ---- Sensor phases ---- */

#if CC_ENABLE_WS85
/** Feed pending WS85 bytes to the parser; returns how many arrived. */
static uint16_t ws85_pump(void)
{
	uint8_t buf[64];
	uint16_t n = CC_HW_uartRead(buf, sizeof(buf));
	cc_ws85_frame_t f;

	for (uint16_t i = 0; i < n; i++) {
		if (!cc_ws85_feed(&ws_parser, buf[i], &f))
			continue;
		ws_cache.temp_c  = f.has_temp ? f.temp_c  : 0;
		ws_cache.wind_ms = f.has_wind ? f.wind_ms : 0;
		ws_cache.gust_ms = f.has_gust ? f.gust_ms : 0;
		ws_cache.dir_deg = f.has_dir  ? f.dir_deg : 0;
		ws_cache.rain_mm = f.has_rain ? f.rain_mm : 0;
		cache_stamp(&ws_cache.meta);
		MGR_LOG_DEBUG("[WS85] %d.%u C  wind %d.%u  gust %d.%u  dir %u  rain %d.%u mm\r\n",
			(int)ws_cache.temp_c, (unsigned)((int)(ws_cache.temp_c * 10) % 10),
			(int)ws_cache.wind_ms, (unsigned)((int)(ws_cache.wind_ms * 10) % 10),
			(int)ws_cache.gust_ms, (unsigned)((int)(ws_cache.gust_ms * 10) % 10),
			ws_cache.dir_deg,
			(int)ws_cache.rain_mm, (unsigned)((int)(ws_cache.rain_mm * 10) % 10));
	}
	return n;
}
#endif /* CC_ENABLE_WS85 */

#if CC_ENABLE_ADC_SENSE
/** Supply voltages (internal ADC): NiMH via MGR_BAT's gated divider, supercap
 *  via ADC_IN3. Stamped only on a good NiMH read; raw values are returned for
 *  the self-test report. */
static void adc_sample(uint16_t *nimh_out, uint16_t *scap_out)
{
	uint16_t nimh = MGR_BAT_readVoltage_mV();
	uint16_t scap = CC_HW_readSupercap_mV();

	if (nimh > 0) {
		adc_cache.nimh_mv = nimh;
		adc_cache.scap_mv = scap;
		cache_stamp(&adc_cache.meta);
		MGR_LOG_DEBUG("[ADC] NiMH %umV  supercap %umV\r\n", nimh, scap);
	} else {
		MGR_LOG_DEBUG("[ADC] NiMH read failed\r\n");
	}
	if (nimh_out)
		*nimh_out = nimh;
	if (scap_out)
		*scap_out = scap;
}
#endif

#if CC_ENABLE_ENV_I2C
/* One shared session: rail up, one I2C lifecycle, each enabled chip read and
 * stamped independently (a miss keeps that chip's previous reading and the
 * cache just ages out). SHT45 preempts: when present, BMP/BME + AHT20 are
 * skipped that cycle (temp priority SHT45 > BMP > AHT > fresh WS85). */
static void i2c_sample(void)
{
	if (!CC_HW_i2cSessionOpen())
		return;

	const cc_i2c_ops_t *bus = CC_HW_i2cOps();
	bool sht_found = false;

	(void)sht_found;
#if CC_ENABLE_SHT45
	{
		cc_sht45_t sht;

		sht_found = cc_sht45_begin(&sht, bus, CC_SHT45_ADDR);
		if (sht_found) {
			float rh, tc;

			if (cc_sht45_read(&sht, &rh, &tc)) {
				sht_cache.rh = rh;
				sht_cache.tc = tc;
				cache_stamp(&sht_cache.meta);
				MGR_LOG_DEBUG("[SHT] %d %%RH  %d.%02u C (BMP/AHT skipped)\r\n",
					(int)rh, (int)tc, (unsigned)((int)(tc * 100) % 100));
			} else {
				MGR_LOG_DEBUG("[SHT] read failed — keeping previous\r\n");
			}
		}
	}
#endif

#if CC_ENABLE_BMP280
	if (!sht_found) {
		cc_bmp280_t bmp;

		if (cc_bmp280_begin(&bmp, bus, CC_BMP280_ADDR)) {
			float tc, hpa, rh = -1.0f;

			if (cc_bmp280_read_forced(&bmp, &tc, &hpa, &rh)) {
				baro_cache.tc = tc;
				baro_cache.hpa = hpa;
				cache_stamp(&baro_cache.meta);
				if (rh >= 0.0f) { /* only written on a BME280 */
					bmehum_cache.rh = rh;
					bmehum_cache.tc = tc;
					cache_stamp(&bmehum_cache.meta);
				}
				MGR_LOG_DEBUG("[BMP] %d.%02u C  %d.%u hPa\r\n",
					(int)tc, (unsigned)((int)(tc * 100) % 100),
					(int)hpa, (unsigned)((int)(hpa * 10) % 10));
			} else {
				MGR_LOG_DEBUG("[BMP] read failed — keeping previous\r\n");
			}
		} else {
			MGR_LOG_DEBUG("[BMP] not responding — keeping previous\r\n");
		}
	}
#endif

#if CC_ENABLE_AHT20
	if (!sht_found) {
		cc_aht20_t aht;

		if (cc_aht20_begin(&aht, bus)) {
			float rh, tc;

			if (cc_aht20_read(&aht, &rh, &tc)) {
				aht_cache.rh = rh;
				aht_cache.tc = tc;
				cache_stamp(&aht_cache.meta);
				MGR_LOG_DEBUG("[AHT] %d %%RH  %d.%02u C\r\n",
					(int)rh, (int)tc, (unsigned)((int)(tc * 100) % 100));
			} else {
				MGR_LOG_DEBUG("[AHT] read failed — keeping previous\r\n");
			}
		}
	}
#endif

	CC_HW_i2cSessionClose();
}
#endif /* CC_ENABLE_ENV_I2C */

/** The ENV phase: supply voltages (sampled even when the I2C bus is down —
 *  the battery tier depends on them), then the I2C sensors. */
static __attribute__((unused)) void env_sample(void)
{
#if CC_ENABLE_ADC_SENSE
	adc_sample(NULL, NULL);
#endif
#if CC_ENABLE_ENV_I2C
	i2c_sample();
#endif
}

#if CC_ENABLE_ENV_I2C
static bool env_stale(void)
{
	bool stale = false;

#if CC_ENABLE_SHT45
	stale = stale || !cache_fresh(&sht_cache.meta, CC_ENV_MAX_AGE_MS);
#endif
#if CC_ENABLE_BMP280
	stale = stale || !cache_fresh(&baro_cache.meta, CC_ENV_MAX_AGE_MS);
#endif
#if CC_ENABLE_AHT20
	stale = stale || !cache_fresh(&aht_cache.meta, CC_ENV_MAX_AGE_MS);
#endif
#if CC_ENABLE_ADC_SENSE
	stale = true; /* supply voltages: sample every cycle */
#endif
	return stale;
}
#endif /* CC_ENABLE_ENV_I2C */

/* ---- Payload ---- */

/** The fields that move after the sample phases: position (a fix lands in
 *  SAMPLE_GPS), time and the wake counter. */
static void fields_late(cc_payload_fields_t *f)
{
#if CC_ENABLE_GPS
	f->lat = gps_lat; /* last-known fix; (0,0) until the first ever */
	f->lon = gps_lon;
#endif
	f->epoch = clock_now();
	f->wake16 = (uint16_t)(HAL_GetTick() >> 16); /* 65.5 s ticks since boot */
}

static void build_fields(cc_payload_fields_t *f)
{
	memset(f, 0, sizeof(*f));
	fields_late(f);

#if CC_ENABLE_WS85
	f->ws_valid = cache_fresh(&ws_cache.meta, CC_WS85_MAX_AGE_MS);
	f->wind_ms = ws_cache.wind_ms;
	f->gust_ms = ws_cache.gust_ms;
	f->dir_deg = ws_cache.dir_deg;
	f->rain_mm = ws_cache.rain_mm;
#endif

#if CC_ENABLE_ENV_I2C
	/* Temp priority: SHT45 > BMP/BME > AHT20; humidity: SHT45 > BME > AHT20;
	 * pressure: BMP/BME only. (On the CC carrier only the BME280 is fitted,
	 * so it supplies all three fields.) */
#if CC_ENABLE_SHT45
	if (cache_fresh(&sht_cache.meta, CC_ENV_MAX_AGE_MS)) {
		f->hum_valid = true;
		f->hum_pct = sht_cache.rh;
		f->temp_valid = true;
		f->temp_c = sht_cache.tc;
	}
#endif
#if CC_ENABLE_BMP280
	if (cache_fresh(&baro_cache.meta, CC_ENV_MAX_AGE_MS)) {
		f->press_valid = true;
		f->press_hpa = baro_cache.hpa;
		if (!f->temp_valid) {
			f->temp_valid = true;
			f->temp_c = baro_cache.tc;
		}
	}
	if (cache_fresh(&bmehum_cache.meta, CC_ENV_MAX_AGE_MS) && !f->hum_valid) {
		f->hum_valid = true;
		f->hum_pct = bmehum_cache.rh;
	}
#endif
#if CC_ENABLE_AHT20
	if (cache_fresh(&aht_cache.meta, CC_ENV_MAX_AGE_MS)) {
		if (!f->hum_valid) {
			f->hum_valid = true;
			f->hum_pct = aht_cache.rh;
		}
		if (!f->temp_valid) {
			f->temp_valid = true;
			f->temp_c = aht_cache.tc;
		}
	}
#endif
#endif

#if CC_ENABLE_WS85
	/* Fresh WS85 air temperature is the last-resort temp source */
	if (!f->temp_valid && f->ws_valid) {
		f->temp_valid = true;
		f->temp_c = ws_cache.temp_c;
	}
#endif

#if CC_ENABLE_ADC_SENSE
	if (cache_fresh(&adc_cache.meta, CC_ENV_MAX_AGE_MS)) {
		f->batt_mv = adc_cache.nimh_mv;
		f->scap_valid = true;
		f->scap_mv = adc_cache.scap_mv;
	}
#endif
}

/** End of SAMPLE_ENV: freeze this cycle's sensor fields and move the battery
 *  tier on the NiMH reading (0 = not measured this cycle -> LOW, conservative). */
static void cycle_snapshot(void)
{
	cc_tier_t was = tier.tier;

	build_fields(&cycle_fields);
	cycle_base_ms = cc_tier_update(&tier, cycle_fields.batt_mv, true);
	if (tier.tier != was)
		MGR_LOG_INFO("[BAT] %umV: tier %s -> %s\r\n", cycle_fields.batt_mv,
			cc_tier_name(was), cc_tier_name(tier.tier));
}

static bool push_tx(const uint8_t payload[CC_PAYLOAD_BYTES])
{
	struct KNS_CFG_radio_t radio_cfg;

	if (KNS_CFG_getRadioInfo(&radio_cfg) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[CC] Radio config read failed\r\n");
		return false;
	}
	if (radio_cfg.modulation != KNS_TX_MOD_LDA2) {
		/* v3 payload is one 192-bit LDA2 frame — anything else would
		 * truncate or pad it. Fail safe rather than send garbage. */
		MGR_LOG_ERR("[CC] Radio conf is not LDA2 (%d) — TX skipped\r\n",
			radio_cfg.modulation);
		return false;
	}

	struct KNS_MAC_appEvt_t appEvt;

	appEvt.id = KNS_MAC_SEND_DATA;
	appEvt.data_ctxt.sf = KNS_SF_NO_SERVICE;
	memset(appEvt.data_ctxt.usrdata, 0, KNS_MAC_USRDATA_MAXLEN);
	memcpy(appEvt.data_ctxt.usrdata, payload, CC_PAYLOAD_BYTES);
	appEvt.data_ctxt.usrdata_bitlen = CC_PAYLOAD_BYTES * 8; /* 192 = LDA2 */

	tx_queue_refused = false;
	if (KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[CC] TX push failed\r\n");
		tx_queue_refused = true;
		return false;
	}
	return true;
}

/* ---- Radio-safe points and session cleanup ---- */

/** The MAC owns the radio from TX_SEND until TX_DONE/timeout (also for the
 *  self-test frame). DFU, reset, credential writes and console-started
 *  self-tests wait for that; every other state is safe — including a MAC
 *  that never comes ready, so a bad build can still be replaced over UART. */
static bool radio_busy(void)
{
	return cc_state == CC_ST_TX_SEND || cc_state == CC_ST_WAIT_TX_DONE ||
	       (cc_state == CC_ST_SELFTEST && st_step == STS_TX_WAIT);
}

/** Close whatever USART1 session is pumping and drop the GPS rail. */
static void quiesce_sessions(void)
{
#if CC_ENABLE_WS85
	ws_session_open = false;
#endif
#if CC_ENABLE_GPS
	gps_budget_ms = 0; /* an interrupted acquisition restarts next cycle */
	CC_HW_gpsPowerOff();
#endif
	CC_HW_uartClose(); /* no-op when no session is open */
}

/* ---- Commissioning persistence (TAMP BKP12R: survives resets, not power loss) ---- */

static bool comm_flag_read(void)
{
	return TAMP->BKP12R == CC_COMM_DONE_MAGIC;
}

static void comm_flag_write(bool done)
{
	HAL_PWR_EnableBkUpAccess();
	TAMP->BKP12R = done ? CC_COMM_DONE_MAGIC : 0u;
}

/* ---- Failed-boot counter (TAMP BKP13R; see cc_conf.h rescue mode) ---- */

static uint32_t boot_attempts_read(void)
{
	uint32_t v = TAMP->BKP13R;

	return ((v & 0xFFFF0000UL) == CC_BOOT_COUNT_MAGIC) ? (v & 0xFFFFUL) : 0u;
}

static void boot_attempts_write(uint32_t n)
{
	HAL_PWR_EnableBkUpAccess();
	TAMP->BKP13R = CC_BOOT_COUNT_MAGIC | (n & 0xFFFFUL);
}

/** The cycle's TX attempt ended (done, timeout, error or push failure). */
static void tx_finished(void)
{
	if (cc_comm_on_tx(&comm, HAL_GetTick(), CC_COMM_BURST_MS)) {
		comm_flag_write(true);
		MGR_LOG_INFO("[COMM] burst complete (%u TX) - commissioned until power is removed\r\n",
			(unsigned)comm.burst_tx);
	}
	transition_to(CC_ST_CYCLE_WAIT);
}

/** A cycle's frame completed (TX_DONE) or never did (timeout, abort, MAC
 *  error, queue refused). A MAC that keeps failing is wedged — and a stuck
 *  modemOn also refuses STOP2 — so reset rather than idle at full power
 *  until the 14-day reboot. Radio-config refusals (not LDA2) don't count:
 *  a reset can't fix those. */
static void tx_outcome(bool ok)
{
	if (ok) {
		tx_fail_streak = 0;
		return;
	}
	if (++tx_fail_streak >= CC_TX_FAIL_RESET_COUNT) {
		MGR_LOG_ERR("[CC] %u TX failures in a row - resetting the radio stack\r\n",
			(unsigned)tx_fail_streak);
		MGR_ERR_logAndReset(ERR_TX_TIMEOUT);
	}
}

/** CRIT tier: at most one frame per CC_TX_PERIOD_CRIT_MS, measured between
 *  cycle starts, with boot counting as a frame — a flat pack that browned
 *  out under PA load must not transmit again straight after the reset. */
static bool crit_tx_due(void)
{
	return cycle_start_tick - (tx_since_boot ? last_tx_cycle : 0u) >= CC_TX_PERIOD_CRIT_MS;
}

static void note_tx(void)
{
	tx_since_boot = true;
	last_tx_cycle = cycle_start_tick;
}

/* ---- Self-test ---- */

static bool st_tx_pushed; /**< this run put a frame on the air (counts as the cycle's TX) */
static bool st_resume_cycle; /**< this run interrupted a cycle before its TX */

static uint32_t st_elapsed(void)
{
	return HAL_GetTick() - st_step_tick;
}

static void st_goto(cc_st_step_t s)
{
	st_step = s;
	st_entry = true;
}

/** Record one item's verdict and print its "+ST=" line. */
static void st_report(cc_st_item_t i, cc_st_verdict_t v, const char *detail)
{
	char line[96];

	cc_selftest_set(&st, i, v, detail);
	cc_selftest_format_item(line, sizeof(line), &st, i);
	MCU_AT_CONSOLE_send("%s", line);
}

static bool st_wants(cc_st_item_t i)
{
	return (st.mask & CC_ST_BIT(i)) != 0;
}

static bool stamped_since(const cc_cache_meta_t *m, uint32_t t0)
{
	return m->valid && (int32_t)(m->at_ms - t0) >= 0;
}

/** "t=-3.25,p=1013.2[,rh=45]" without float printf (newlib-nano). */
static void fmt_env(char *d, size_t n, float tc, float hpa, float rh)
{
	int t100 = (int)(tc * 100.0f + (tc < 0.0f ? -0.5f : 0.5f));
	int p10 = (int)(hpa * 10.0f + 0.5f);
	const char *sign = (t100 < 0) ? "-" : "";

	if (t100 < 0)
		t100 = -t100;
	if (rh >= 0.0f)
		snprintf(d, n, "t=%s%d.%02d,p=%d.%d,rh=%d", sign, t100 / 100, t100 % 100,
			p10 / 10, p10 % 10, (int)(rh + 0.5f));
	else
		snprintf(d, n, "t=%s%d.%02d,p=%d.%d", sign, t100 / 100, t100 % 100,
			p10 / 10, p10 % 10);
}

static void selftest_start(uint8_t mask, bool at_boot)
{
	/* Started mid-cycle (before its TX): that cycle resumes afterwards */
	st_resume_cycle = cc_state == CC_ST_SAMPLE_WS85 || cc_state == CC_ST_SAMPLE_ENV ||
	                  cc_state == CC_ST_SAMPLE_GPS || cc_state == CC_ST_TX_SEND;
	quiesce_sessions();
	cc_selftest_begin(&st, mask, at_boot);
	st_tx_pushed = false;
	st_goto(STS_GPS);
	MGR_LOG_INFO("[ST] self-test start (items 0x%02X%s)\r\n", mask, at_boot ? ", boot" : "");
	MCU_AT_CONSOLE_send("+SELFTEST=RUNNING\r\n");
	transition_to(CC_ST_SELFTEST);
}

static void selftest_finish(void)
{
	char line[64];
	bool transmitted = st_tx_pushed;

	cc_selftest_end(&st);
	cc_selftest_format_summary(line, sizeof(line), &st);
	MCU_AT_CONSOLE_send("%s", line);
	MGR_LOG_INFO("[ST] self-test %s\r\n", cc_selftest_passed(&st) ? "PASS" : "FAIL");

	if (!creds_ok) {
		transition_to(CC_ST_NO_CREDS); /* still waiting for provisioning */
		return;
	}

	if (st.at_boot) {
		/* Boot run: straight into the first cycle (its GPS step is the
		 * commissioning probe). Sensor caches are fresh from the test. */
		transition_to(CC_ST_SAMPLE_WS85);
		return;
	}
	if (transmitted) {
		/* The self-test frame was this cycle's transmission. */
		cycle_start_tick = HAL_GetTick();
		cycle_period_ms = 0;
		note_tx();
	} else if (st_resume_cycle) {
		/* It interrupted a cycle before its TX: finish that cycle (caches
		 * are fresh from the test; an interrupted GPS window restarts) */
		transition_to(CC_ST_SAMPLE_WS85);
		return;
	}
	transition_to(CC_ST_CYCLE_WAIT);
}

/** One self-test step per loop call; never blocks longer than the ENV read. */
static void selftest_run(void)
{
	char d[CC_ST_DETAIL_LEN];

	if (st_entry)
		st_step_tick = HAL_GetTick();

	switch (st_step) {
	case STS_GPS:
		if (!st_wants(CC_STI_GPS)) {
			st_goto(STS_WS85);
			return;
		}
#if CC_ENABLE_GPS
		if (st_entry) {
			st_entry = false;
			st_bytes = 0;
			cc_nmea_init(&nmea);
			CC_HW_gpsPowerOn();
			if (!CC_HW_uartOpen(CC_UART_GPS)) {
				CC_HW_gpsPowerOff();
				st_report(CC_STI_GPS, CC_STV_FAIL, "uart");
				st_goto(STS_WS85);
				return;
			}
		}
		{
			uint8_t buf[64];
			uint16_t n = CC_HW_uartRead(buf, sizeof(buf));

			st_bytes += n;
			for (uint16_t k = 0; k < n; k++)
				(void)cc_nmea_feed(&nmea, buf[k]);
		}
		if (nmea.sentences == 0 && st_elapsed() <= CC_SELFTEST_GPS_MS)
			return;
		CC_HW_uartClose();
		CC_HW_gpsPowerOff();
		snprintf(d, sizeof(d), "bytes=%lu,nmea=%lu,cs_err=%lu", (unsigned long)st_bytes,
			(unsigned long)nmea.sentences, (unsigned long)nmea.cs_errors);
		st_report(CC_STI_GPS, cc_selftest_eval_gps(st_bytes, nmea.sentences), d);
#else
		st_report(CC_STI_GPS, CC_STV_SKIP, "disabled");
#endif
		st_goto(STS_WS85);
		return;

	case STS_WS85:
		if (!st_wants(CC_STI_WS85)) {
			st_goto(STS_ENV);
			return;
		}
#if CC_ENABLE_WS85
		if (st_entry) {
			st_entry = false;
			st_bytes = 0;
			cc_ws85_init(&ws_parser, CC_WS85_ADDR);
			if (!CC_HW_uartOpen(CC_UART_WS85)) {
				st_report(CC_STI_WS85, CC_STV_FAIL, "uart");
				st_goto(STS_ENV);
				return;
			}
		}
		st_bytes += ws85_pump();
		if (ws_parser.frames == 0 && st_elapsed() <= CC_SELFTEST_WS85_MS)
			return;
		CC_HW_uartClose();
		if (ws_parser.frames > 0) {
			snprintf(d, sizeof(d), "wind=%d.%u,dir=%u,frames=%lu",
				(int)ws_cache.wind_ms, (unsigned)((int)(ws_cache.wind_ms * 10) % 10),
				ws_cache.dir_deg, (unsigned long)ws_parser.frames);
			st_report(CC_STI_WS85, CC_STV_PASS, d);
		} else if (st_bytes) {
			snprintf(d, sizeof(d), "bytes=%lu,crc_err=%lu", (unsigned long)st_bytes,
				(unsigned long)ws_parser.crc_errors);
			st_report(CC_STI_WS85, CC_STV_FAIL, d);
		} else {
			st_report(CC_STI_WS85, CC_STV_FAIL, "silent");
		}
#else
		st_report(CC_STI_WS85, CC_STV_SKIP, "disabled");
#endif
		st_goto(STS_ENV);
		return;

	case STS_ENV:
		if (!st_wants(CC_STI_ENV)) {
			st_goto(STS_ADC);
			return;
		}
#if CC_ENABLE_ENV_I2C && CC_ENABLE_BMP280
		{
			uint32_t t0 = HAL_GetTick();

			i2c_sample();
			if (stamped_since(&baro_cache.meta, t0)) {
				float rh = stamped_since(&bmehum_cache.meta, t0) ? bmehum_cache.rh : -1.0f;

				fmt_env(d, sizeof(d), baro_cache.tc, baro_cache.hpa, rh);
				st_report(CC_STI_ENV, cc_selftest_eval_env(baro_cache.tc, baro_cache.hpa, rh), d);
			} else {
				st_report(CC_STI_ENV, CC_STV_FAIL, "no_response");
			}
		}
#else
		st_report(CC_STI_ENV, CC_STV_SKIP, "disabled");
#endif
		st_goto(STS_ADC);
		return;

	case STS_ADC:
		if (!st_wants(CC_STI_ADC)) {
			st_goto(STS_TX);
			return;
		}
#if CC_ENABLE_ADC_SENSE
		{
			uint16_t nimh = 0, scap = 0;

			adc_sample(&nimh, &scap);
			snprintf(d, sizeof(d), "nimh=%u,scap=%u", nimh, scap);
			st_report(CC_STI_ADC, cc_selftest_eval_adc(nimh, CC_SELFTEST_NIMH_MIN_MV,
				CC_SELFTEST_NIMH_MAX_MV), d);
		}
#else
		st_report(CC_STI_ADC, CC_STV_SKIP, "disabled");
#endif
		st_goto(STS_TX);
		return;

	case STS_TX:
		if (!st_wants(CC_STI_TX)) {
			st_goto(STS_DONE);
			return;
		}
		{
			cc_payload_fields_t f;
			uint8_t payload[CC_PAYLOAD_BYTES];

			build_fields(&f);
			if (st.at_boot && cc_tier_is_critical(f.batt_mv, true)) {
				/* Every reboot would transmit at once on a flat pack
				 * (brownout loop); AT+SELFTEST still forces it. */
				st_report(CC_STI_TX, CC_STV_SKIP, "batt_crit");
				st_goto(STS_DONE);
				return;
			}
			cc_payload_pack(payload, &f);
			st_tx_evt = 0;
			if (!mac_ready) {
				st_report(CC_STI_TX, CC_STV_FAIL, creds_ok ? "mac_not_ready" : "unprovisioned");
				st_goto(STS_DONE);
				return;
			}
			if (!push_tx(payload)) {
				st_report(CC_STI_TX, CC_STV_FAIL, "push");
				st_goto(STS_DONE);
				return;
			}
			tx_count++;
			st_tx_pushed = true;
			MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);
		}
		st_goto(STS_TX_WAIT);
		return;

	case STS_TX_WAIT:
		st_entry = false;
		process_mac_events();
		if (st_tx_evt == 0 && st_elapsed() <= CC_SELFTEST_TX_MS)
			return;
		if (st_tx_evt == 1) {
			snprintf(d, sizeof(d), "ms=%lu", (unsigned long)st_elapsed());
			st_report(CC_STI_TX, CC_STV_PASS, d);
		} else if (st_tx_evt == -3) {
			snprintf(d, sizeof(d), "mac_err=%d", st_tx_status);
			st_report(CC_STI_TX, CC_STV_FAIL, d);
		} else {
			st_report(CC_STI_TX, CC_STV_FAIL, st_tx_evt == -1 ? "timeout" :
				st_tx_evt == -2 ? "abort" : "no_tx_done");
		}
		st_goto(STS_DONE);
		return;

	case STS_DONE:
	default:
		selftest_finish();
		return;
	}
}

/** AT+SELFTEST=? — replay the last report (or its status). */
static void selftest_replay(void)
{
	char line[96];

	if (st.running) {
		MCU_AT_CONSOLE_send("+SELFTEST=RUNNING\r\n");
	} else if (!st.done) {
		MCU_AT_CONSOLE_send("+SELFTEST=NONE\r\n");
	} else {
		for (int i = 0; i < CC_STI_COUNT; i++) {
			if (!st_wants((cc_st_item_t)i))
				continue;
			cc_selftest_format_item(line, sizeof(line), &st, (cc_st_item_t)i);
			MCU_AT_CONSOLE_send("%s", line);
		}
		cc_selftest_format_summary(line, sizeof(line), &st);
		MCU_AT_CONSOLE_send("%s", line);
	}
	bMGR_AT_CMD_logSucceedMsg();
}

/** After MAC init: pending/boot self-test first, else the first cycle. */
static void after_mac_ready(void)
{
	bool at_boot = CC_SELFTEST_AT_BOOT && !boot_selftest_done;
	uint8_t mask = st_req_mask;

	mac_ready = true;
	st_req_mask = 0;
	if (at_boot)
		mask |= (uint8_t)CC_SELFTEST_BOOT_ITEMS;
	boot_selftest_done = true;

	if (mask)
		selftest_start(mask, at_boot);
	else
		transition_to(CC_ST_SAMPLE_WS85);
}

/* ---- Debug console ---- */

static void enter_dfu(void)
{
	quiesce_sessions();
	boot_attempts_write(0); /* deliberate: not a failed boot */
	MCU_MISC_turn_off_pa(); /* bootloader never drives PA_PSU_EN (upstream 9e0ef44) */
	MGR_LOG_INFO("[CC] entering UART DFU bootloader\r\n");
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0xDFU);
	MGR_LOG_flush_all();
	MCU_AT_CONSOLE_send("+BOOT=OK\r\n");
	HAL_Delay(50); /* let the response leave the UART */
	request_dfu_mode(DFU_PROTO_UART);
	/* Only returns when no valid bootloader is installed */
	pending_act = CC_ACT_NONE;
	MCU_AT_CONSOLE_send("+ERROR=NO_BOOTLOADER\r\n");
}

static void do_reset(void)
{
	quiesce_sessions();
	boot_attempts_write(0); /* deliberate: not a failed boot */
	MGR_LOG_INFO("[CC] reset requested from console\r\n");
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0x5EU);
	MGR_LOG_flush_all();
	MCU_AT_CONSOLE_send("+RESET=OK\r\n");
	HAL_Delay(50);
	NVIC_SystemReset();
}

static void send_state(void)
{
	const char *st_name = st.running ? "RUNNING" : !st.done ? "NONE" :
	                      cc_selftest_passed(&st) ? "PASS" : "FAIL";

	MCU_AT_CONSOLE_send("+STATE=%s,up=%lu,tx=%lu,mac=%u,comm=%s,st=%s,slp=%lu,prov=%u,"
		"bat=%u,tier=%s,utc=%lu\r\n",
		cc_state_names[cc_state], (unsigned long)(HAL_GetTick() / 1000UL),
		(unsigned long)tx_count, mac_ready ? 1u : 0u,
		cc_comm_phase_name(comm.phase), st_name,
		(unsigned long)(CC_SLEEP_totalMs() / 1000UL), creds_ok ? 1u : 0u,
		cycle_fields.batt_mv, cc_tier_name(tier.tier), (unsigned long)clock_now());
	bMGR_AT_CMD_logSucceedMsg();
}

static void send_commission(void)
{
	char line[80];

	cc_comm_format(line, sizeof(line), &comm, HAL_GetTick(), powerup_ms(),
		CC_COMM_FAST_GPS_MS, CC_COMM_BURST_MS, CC_COMM_BURST_PERIOD_MS);
	MCU_AT_CONSOLE_send("%s", line);
	bMGR_AT_CMD_logSucceedMsg();
}

/** AT+COMMISSION=START|SKIP|ARM (bench / production overrides). */
static void commission_cmd(const char *line)
{
	const char *p = cc_console_params(line);

	if (strncmp(p, "START", 5) == 0) {
		cc_comm_start(&comm, HAL_GetTick());
		comm_flag_write(false);
		if (cc_state == CC_ST_CYCLE_WAIT)
			cycle_period_ms = 1; /* burst TX #1 now: end this wait */
		MGR_LOG_INFO("[COMM] burst forced from console\r\n");
	} else if (strncmp(p, "SKIP", 4) == 0) {
		cc_comm_skip(&comm);
		comm_flag_write(true);
	} else {
		cc_comm_init(&comm, false);
		comm_flag_write(false);
	}
	send_commission();
}

/** AT+SECKEY=? — CRC-32 fingerprint only; the key is never echoed. */
static void send_key_crc(void)
{
	uint8_t key[16];

	if (MCU_AES_get_device_sec_key(key) != KNS_STATUS_OK) {
		bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN);
		return;
	}
	unsigned long crc = cc_console_crc32(key, sizeof(key));

	memset(key, 0, sizeof(key));
	MCU_AT_CONSOLE_send("+SECKEYCRC=%08lX\r\n", crc);
	bMGR_AT_CMD_logSucceedMsg();
}

static void console_poll(void)
{
	uint8_t *line;

	while ((line = MGR_AT_CMD_popNextAt()) != NULL) {
		const char *l = (const char *)line;

		CC_SLEEP_consoleLine();

		switch (cc_console_classify(l)) {
		case CC_CON_BOOT:
			pending_act = CC_ACT_DFU;
			if (radio_busy())
				MCU_AT_CONSOLE_send("+BOOT=PENDING\r\n");
			break;
		case CC_CON_RESET:
			if (pending_act == CC_ACT_NONE)
				pending_act = CC_ACT_RESET;
			if (radio_busy())
				MCU_AT_CONSOLE_send("+RESET=PENDING\r\n");
			break;
		case CC_CON_SELFTEST: {
			uint8_t m = cc_selftest_parse_mask(cc_console_params(l));

			if (!m || rescue) {
				bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
			} else if (cc_state == CC_ST_SELFTEST) {
				MCU_AT_CONSOLE_send("+SELFTEST=RUNNING\r\n");
			} else {
				st_req_mask |= m; /* started by service_requests() */
				if ((!mac_ready && cc_state != CC_ST_NO_CREDS) || radio_busy())
					MCU_AT_CONSOLE_send("+SELFTEST=PENDING\r\n");
			}
			break;
		}
		case CC_CON_SELFTEST_QUERY:
			selftest_replay();
			break;
		case CC_CON_STATE:
			send_state();
			break;
		case CC_CON_COMMISSION_QUERY:
			send_commission();
			break;
		case CC_CON_COMMISSION:
			commission_cmd(l);
			break;
		case CC_CON_CRED:
			/* Flash write; takes effect at the next MAC init (AT+RESET) */
			if (radio_busy())
				MCU_AT_CONSOLE_send("+ERROR=BUSY\r\n");
			else
				MGR_AT_CMD_decodeAt(line);
			break;
		case CC_CON_KEYCRC:
			send_key_crc();
			break;
		case CC_CON_DISPATCH:
			MGR_AT_CMD_decodeAt(line);
			break;
		default:
			bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
			break;
		}
	}
}

/** Act on console requests that had to wait for a safe point. */
static void service_requests(void)
{
	if (pending_act != CC_ACT_NONE && !radio_busy()) {
		if (pending_act == CC_ACT_DFU)
			enter_dfu();
		else
			do_reset();
	}
	if (st_req_mask && (mac_ready || cc_state == CC_ST_NO_CREDS) && !radio_busy() &&
	    cc_state != CC_ST_SELFTEST &&
	    cc_state != CC_ST_BOOT && cc_state != CC_ST_INIT_MAC &&
	    cc_state != CC_ST_WAIT_MAC_READY) {
		uint8_t m = st_req_mask;

		st_req_mask = 0;
		selftest_start(m, false);
	}
}

/* ---- LPM client (scaffold: SLEEP only; STOP2+RTC deadline is the planned
 * production path once the tick-compensation wiring is ported) ---- */

static enum MgrLpm_LPM_t cc_lpm_req(void)
{
	return LOW_POWER_MODE_SLEEP;
}

static bool cc_lpm_enter(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	return true;
}

static bool cc_lpm_exit(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	return true;
}

static struct MgrLpmClientCb_t cc_lpm_client = {
	.fpMGR_LPM_LpmReqCb        = cc_lpm_req,
	.fpMGR_LPM_LpmNotifEnterCb = cc_lpm_enter,
	.fpMGR_LPM_LpmNotifExitCb  = cc_lpm_exit,
};

/* ---- Public API ---- */

bool KNS_APP_cc_tracker_isBusy(void)
{
	return cc_state != CC_ST_CYCLE_WAIT && cc_state != CC_ST_WAIT_TX_DONE;
}

void KNS_APP_cc_tracker_init(void)
{
	/* IWDG early — before any slow init. STOP2 sleeps far longer than the
	 * 16 s window, so the IWDG must be frozen in STOP first (option byte;
	 * on a fresh chip this programs it and does one OBL reset, VDD-gated).
	 * Without it MGR_WDG_init leaves the watchdog off ("not frozen yet"). */
#if CC_USE_STOP2
	(void)MGR_WDG_ensureIwdgStopOptionByte();
#endif
	MGR_WDG_init();

	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);
	MGR_ERR_init();

	{
		uint32_t attempts = boot_attempts_read() + 1;

		boot_attempts_write(attempts);
		if (attempts > CC_RESCUE_AFTER_FAILED_BOOTS) {
			/* Repeated failed boots: skip everything that could be what
			 * crashes (sensor bring-up, ADC, MAC) and keep only the console,
			 * so the unit stays updatable over UART. */
			rescue = true;
			MGR_LOG_ERR("[CC] %lu failed boots in a row - RESCUE: console only for %lu min\r\n",
				(unsigned long)(attempts - 1),
				(unsigned long)(CC_RESCUE_WINDOW_MS / 60000UL));
			MCU_AT_CONSOLE_send("+RESCUE=failed_boots=%lu,window_s=%lu\r\n",
				(unsigned long)(attempts - 1),
				(unsigned long)(CC_RESCUE_WINDOW_MS / 1000UL));
			cc_comm_init(&comm, comm_flag_read());
			transition_to(CC_ST_RESCUE);
			return;
		}
	}

	{
		MGR_CRED_Result_t cr = MGR_CRED_syncAndRestore();

		creds_ok = (cr != MGR_CRED_FAIL_LOUD) || CC_ALLOW_TEST_CREDS;
		if (cr == MGR_CRED_RESTORED)
			MGR_LOG_WARN("[CC] credentials restored from the mirror\r\n");
		if (!creds_ok)
			MGR_LOG_ERR("[CC] NOT PROVISIONED (no ID/ADDR/SECKEY) - radio stays off until "
				"credentials are written over the console\r\n");
	}

	CC_HW_init();
	MGR_BAT_init();
	cc_tier_init(&tier);

	MGR_LPM_registerClient(cc_lpm_client);

	cc_comm_init(&comm, comm_flag_read());
	clock_restore();

	cycle_start_tick = HAL_GetTick();

	MGR_LOG_INFO("[CC] CounterCurrent tracker init (payload v3, LDA2)\r\n");
	MGR_LOG_INFO("[CC] periods %lu/%lu/%lu s, jitter %u%%, GPS every %u TX\r\n",
		(unsigned long)(CC_TX_PERIOD_MS / 1000),
		(unsigned long)(CC_TX_PERIOD_MID_MS / 1000),
		(unsigned long)(CC_TX_PERIOD_LOW_MS / 1000),
		(unsigned)CC_TX_JITTER_PCT, (unsigned)CC_GPS_FIX_EVERY_N_TX);
	MGR_LOG_INFO("[COMM] %s\r\n", comm.phase == CC_COMM_DONE ?
		"commissioned (flag kept since power-up)" :
		"waiting for the first GPS fix to start the deployment burst");

	transition_to(CC_ST_BOOT);
}

void KNS_APP_cc_tracker_loop(void)
{
	MGR_WDG_refresh();
	console_poll();
	service_requests();

	if (!boot_stable && HAL_GetTick() >= CC_BOOT_STABLE_MS) {
		boot_stable = true;
		boot_attempts_write(0);
	}

	switch (cc_state) {

	case CC_ST_RESCUE:
		if (HAL_GetTick() >= CC_RESCUE_WINDOW_MS) {
			MGR_LOG_INFO("[CC] rescue window over - rebooting for a normal attempt\r\n");
			boot_attempts_write(0);
			MGR_LOG_flush_all();
			NVIC_SystemReset();
		}
		return;

	case CC_ST_SELFTEST:
		selftest_run();
		return;

	case CC_ST_BOOT:
		if (state_elapsed_ms() > TIMEOUT_BOOT_MS)
			transition_to(CC_ST_INIT_MAC);
		return;

	case CC_ST_INIT_MAC:
		if (!creds_ok) {
			MGR_EVTLOG_log(EVT_ERROR, (uint16_t)ERR_CREDS_BLANK);
			MGR_ERR_log(ERR_CREDS_BLANK);
			MCU_AT_CONSOLE_send("+PROV=NONE\r\n");
			transition_to(CC_ST_NO_CREDS);
			if (!boot_selftest_done && CC_SELFTEST_AT_BOOT) {
				/* Factory flow: sensors are checked before provisioning */
				boot_selftest_done = true;
				selftest_start((uint8_t)CC_SELFTEST_BOOT_ITEMS, true);
			}
			return;
		}
		start_mac_profile();
		transition_to(CC_ST_WAIT_MAC_READY);
		return;

	case CC_ST_NO_CREDS: {
		/* Re-check ~1 Hz; console credential writes complete the set */
		static uint32_t recheck_tick;

		if (HAL_GetTick() - recheck_tick < 1000u)
			return;
		recheck_tick = HAL_GetTick();
		if (MGR_CRED_syncAndRestore() != MGR_CRED_FAIL_LOUD) {
			creds_ok = true;
			MGR_LOG_INFO("[CC] credentials provisioned - starting the radio\r\n");
			MCU_AT_CONSOLE_send("+PROV=OK\r\n");
			transition_to(CC_ST_INIT_MAC);
		}
		return;
	}

	case CC_ST_WAIT_MAC_READY:
		process_mac_events();
		if (cc_state != CC_ST_WAIT_MAC_READY)
			return;
		if (state_elapsed_ms() > CC_MAC_READY_TIMEOUT_MS) {
			MGR_LOG_DEBUG("[CC] MAC ready timeout (retry %u/%u)\r\n",
				mac_init_retries + 1, CC_MAC_INIT_MAX_RETRIES);
			MGR_ERR_log(ERR_MAC_TIMEOUT);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)CC_ST_WAIT_MAC_READY);
			if (++mac_init_retries >= CC_MAC_INIT_MAX_RETRIES)
				MGR_ERR_logAndReset(ERR_MAC_INIT_FAIL);
			transition_to(CC_ST_INIT_MAC);
		}
		return;

	case CC_ST_SAMPLE_WS85:
#if CC_ENABLE_WS85
		/* Open the capture session once, on entry */
		if (!ws_session_open) {
			if (cache_fresh(&ws_cache.meta, CC_WS85_MAX_AGE_MS)) {
				MGR_LOG_DEBUG("[WS85] cache fresh — skipping sample\r\n");
				transition_to(CC_ST_SAMPLE_ENV);
				return;
			}
			cc_ws85_init(&ws_parser, CC_WS85_ADDR);
			if (!CC_HW_uartOpen(CC_UART_WS85)) {
				transition_to(CC_ST_SAMPLE_ENV);
				return;
			}
			ws_session_open = true;
		}
		{
			uint32_t frames_before = ws_parser.frames;

			ws85_pump();
			if (ws_parser.frames > frames_before) {
				CC_HW_uartClose();
				ws_session_open = false;
				transition_to(CC_ST_SAMPLE_ENV);
				return;
			}
		}
		if (state_elapsed_ms() > CC_WS85_WAIT_MS) {
			MGR_LOG_DEBUG("[WS85] no fresh frame (crc_err=%lu ovr=%u) — keeping previous\r\n",
				(unsigned long)ws_parser.crc_errors, CC_HW_uartOverruns());
			CC_HW_uartClose();
			ws_session_open = false;
			transition_to(CC_ST_SAMPLE_ENV);
		}
#else
		transition_to(CC_ST_SAMPLE_ENV);
#endif
		return;

	case CC_ST_SAMPLE_ENV:
#if CC_ENABLE_ENV_I2C
		if (env_stale())
			env_sample(); /* single blocking pass (~300 ms) */
		else
			MGR_LOG_DEBUG("[ENV] caches fresh — skipping sample\r\n");
#elif CC_ENABLE_ADC_SENSE
		adc_sample(NULL, NULL); /* the battery tier needs it every cycle */
#endif
		cycle_snapshot();
		transition_to(CC_ST_SAMPLE_GPS);
		return;

	case CC_ST_SAMPLE_GPS:
#if CC_ENABLE_GPS
		/* Schedule gates, evaluated once on entry (budget==0 marks entry) */
		if (gps_budget_ms == 0) {
			cc_comm_gps_t pol = cc_comm_gps_policy(&comm, powerup_ms(), CC_COMM_FAST_GPS_MS);

			if (tier.tier == CC_TIER_CRIT) {
				MGR_LOG_DEBUG("[GPS] skipped (battery critical)\r\n");
				transition_to(CC_ST_TX_SEND);
				return;
			}
			if (pol == CC_COMM_GPS_SKIP) {
				MGR_LOG_DEBUG("[GPS] skipped (deployment burst)\r\n");
				transition_to(CC_ST_TX_SEND);
				return;
			}
			gps_fast = (pol == CC_COMM_GPS_FAST);
			if (!gps_fast && gps_countdown > 0) {
				gps_countdown--;
				MGR_LOG_DEBUG("[GPS] scheduled skip (%u left)\r\n", gps_countdown);
				transition_to(CC_ST_TX_SEND);
				return;
			}
			if (!gps_fast && gps_skip_cycles > 0) {
				gps_skip_cycles--;
				MGR_LOG_DEBUG("[GPS] backoff skip (%u left)\r\n", gps_skip_cycles);
				transition_to(CC_ST_TX_SEND);
				return;
			}
			if (gps_fast) {
				/* Waiting to be deployed: every cycle, one fixed window */
				comm.gps_tries++;
				gps_budget_ms = CC_COMM_GPS_WINDOW_MS;
			} else {
				gps_budget_ms = (uint32_t)gps_durations * CC_GPS_TTFF_BUDGET_MS;
			}
			cc_nmea_init(&nmea);
			CC_HW_gpsPowerOn();
			if (!CC_HW_uartOpen(CC_UART_GPS)) {
				CC_HW_gpsPowerOff();
				gps_budget_ms = 0;
				transition_to(CC_ST_TX_SEND);
				return;
			}
			MGR_LOG_DEBUG("[GPS] acquiring (budget %lus)\r\n",
				(unsigned long)(gps_budget_ms / 1000));
		}
		{
			uint8_t buf[64];
			uint16_t n = CC_HW_uartRead(buf, sizeof(buf));
			bool fix = false;

			for (uint16_t i = 0; i < n; i++)
				if (cc_nmea_feed(&nmea, buf[i]))
					fix = true;

			if (fix && cc_nmea_fix(&nmea)->valid) {
				const cc_gps_fix_t *g = cc_nmea_fix(&nmea);

				gps_lat = g->lat;
				gps_lon = g->lon;
				gps_ever_fixed = true;
				clock_seed(g->epoch);
				MGR_LOG_INFO("[GPS] fix %d.%06lu,%d.%06lu sats %u TTFF %lus\r\n",
					(int)g->lat,
					(unsigned long)((g->lat - (int)g->lat) * 1e6 * ((g->lat < 0) ? -1 : 1)),
					(int)g->lon,
					(unsigned long)((g->lon - (int)g->lon) * 1e6 * ((g->lon < 0) ? -1 : 1)),
					g->sats,
					(unsigned long)(state_elapsed_ms() / 1000));
				CC_HW_uartClose();
				CC_HW_gpsPowerOff();
				gps_durations = 1; /* hot from here */
				gps_skip_cycles = 0;
				gps_countdown = CC_GPS_FIX_EVERY_N_TX - 1;
				gps_budget_ms = 0;
				if (cc_comm_on_fix(&comm, HAL_GetTick()))
					MGR_LOG_INFO("[COMM] first fix since power-up: deployment burst, "
						"TX every %lus for %lu min\r\n",
						(unsigned long)(CC_COMM_BURST_PERIOD_MS / 1000UL),
						(unsigned long)(CC_COMM_BURST_MS / 60000UL));
				transition_to(CC_ST_TX_SEND); /* burst TX #1 right now */
				return;
			}
		}
		bool no_sky = cc_nmea_no_sky(&nmea, state_elapsed_ms(), CC_GPS_SKY_CHECK_MS,
			CC_GPS_SKY_MIN_SATS);

		if (no_sky || state_elapsed_ms() > gps_budget_ms) {
			if (no_sky)
				MGR_LOG_INFO("[GPS] no sky after %lus (%u strong sats, best %u dBHz) - window ended early\r\n",
					(unsigned long)(state_elapsed_ms() / 1000),
					cc_nmea_sky_strong(&nmea), nmea.max_cno);
			MGR_LOG_DEBUG("[GPS] no fix in %lus (%lu sentences, %lu cs-err) — backoff %u\r\n",
				(unsigned long)(state_elapsed_ms() / 1000),
				(unsigned long)nmea.sentences,
				(unsigned long)nmea.cs_errors, gps_durations);
			CC_HW_uartClose();
			CC_HW_gpsPowerOff();
			if (!gps_fast) {
				gps_skip_cycles = gps_durations;
				if (gps_durations < CC_GPS_MAX_DURATIONS)
					gps_durations++;
			}
			gps_budget_ms = 0;
			transition_to(CC_ST_TX_SEND);
		}
#else
		transition_to(CC_ST_TX_SEND);
#endif
		return;

	case CC_ST_TX_SEND:
	{
		cc_payload_fields_t f;
		uint8_t payload[CC_PAYLOAD_BYTES];

		if (tier.tier == CC_TIER_CRIT && !crit_tx_due()) {
			MGR_LOG_INFO("[BAT] critical: TX held (one per %lu min, boot counts as one)\r\n",
				(unsigned long)(CC_TX_PERIOD_CRIT_MS / 60000UL));
			transition_to(CC_ST_CYCLE_WAIT);
			return;
		}

		f = cycle_fields;
		fields_late(&f);
		cc_payload_pack(payload, &f);

		MGR_EVTLOG_log(EVT_BAT, f.batt_mv);
		MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);

		if (push_tx(payload)) {
			tx_count++;
			note_tx();
			transition_to(CC_ST_WAIT_TX_DONE);
		} else {
			if (tx_queue_refused)
				tx_outcome(false);
			tx_finished();
		}
		return;
	}

	case CC_ST_WAIT_TX_DONE:
		process_mac_events();
		if (cc_state != CC_ST_WAIT_TX_DONE)
			return;
		if (state_elapsed_ms() > CC_TX_RESULT_TIMEOUT_MS) {
			MGR_LOG_DEBUG("[CC] TX result timeout\r\n");
			MGR_ERR_log(ERR_TX_TIMEOUT);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)CC_ST_WAIT_TX_DONE);
			tx_outcome(false);
			tx_finished();
		}
		return;

	case CC_ST_CYCLE_WAIT:
	{
		/* Compute this cycle's period once, on entry (marked by 0) */
		if (cycle_period_ms == 0) {
			bool crit = (tier.tier == CC_TIER_CRIT);
			uint32_t normal = cc_tier_jittered_period_ms(cycle_base_ms,
				CC_TX_JITTER_PCT, CC_HW_rand32());

			/* A critical pack sits the deployment burst out too */
			cycle_period_ms = crit ? normal : cc_comm_period_ms(&comm, normal,
				CC_COMM_BURST_PERIOD_MS, CC_COMM_BURST_JITTER_MS, CC_HW_rand32());
			MGR_LOG_INFO("[SLP] batt %umV (%s) -> base %lus, period %lus%s\r\n",
				cycle_fields.batt_mv, cc_tier_name(tier.tier),
				(unsigned long)(cycle_base_ms / 1000),
				(unsigned long)(cycle_period_ms / 1000),
				(!crit && comm.phase == CC_COMM_BURST) ? " (deployment burst)" : "");
		}

		process_mac_events(); /* drain strays */

		uint32_t elapsed = HAL_GetTick() - cycle_start_tick;

		if (elapsed < cycle_period_ms) {
#if CC_USE_STOP2
			/* Deep sleep to the deadline; refused (console active, MAC
			 * busy, < CC_STOP2_MIN_MS left...) -> the idle task WFIs */
			(void)CC_SLEEP_stop2Until(cycle_start_tick + cycle_period_ms);
#endif
			return;
		}

		/* End-of-cycle maintenance: graceful reboot at the uptime cap,
		 * never mid-TX (keeps tick-based ages far from the 49.7-day wrap;
		 * costs one cold cycle). */
		if (HAL_GetTick() > CC_REBOOT_UPTIME_MS) {
			MGR_LOG_INFO("[CC] end-of-cycle reboot (uptime cap)\r\n");
			MGR_EVTLOG_log(EVT_SHUTDOWN, 0xCCU);
			NVIC_SystemReset();
		}

		cycle_start_tick = HAL_GetTick();
		cycle_period_ms = 0;
		transition_to(CC_ST_SAMPLE_WS85);
		return;
	}

	default:
		kns_assert(0);
		return;
	}
}

#endif /* USE_CC_TRACKER_APP */

/**
 * @}
 */
