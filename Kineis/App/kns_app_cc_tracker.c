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
 */

/**
 * @addtogroup KNS_APP_CC_TRACKER
 * @{
 */

#include <stdbool.h>
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

#include "cc_conf.h"
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
} cc_state_t;

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

static cc_tier_state_t tier;

/* ---- Software clock (GPS-seeded Unix time; 0 until first fix) ---- */

static uint32_t clk_epoch;   /**< Unix seconds at seed; 0 = never seeded */
static uint32_t clk_at_ms;   /**< HAL tick at seed */

static uint32_t clock_now(void)
{
	if (!clk_epoch)
		return 0;
	return clk_epoch + (HAL_GetTick() - clk_at_ms) / 1000UL;
}

static void clock_seed(uint32_t epoch)
{
	if (!epoch)
		return;
	clk_epoch = epoch;
	clk_at_ms = HAL_GetTick();
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
static struct { cc_cache_meta_t meta; float rh, tc; }  sht_cache;   /* SHT45 */
static struct { cc_cache_meta_t meta; float tc, hpa; } baro_cache;  /* BMP/BME */
static struct { cc_cache_meta_t meta; float rh, tc; }  bmehum_cache;/* BME280 RH */
static struct { cc_cache_meta_t meta; float rh, tc; }  aht_cache;   /* AHT20 */
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
					transition_to(CC_ST_SAMPLE_WS85);
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[CC] TX done (#%lu)\r\n", (unsigned long)tx_count);
			MGR_EVTLOG_log(EVT_TX_DONE, (uint16_t)tx_count);
			if (cc_state == CC_ST_WAIT_TX_DONE)
				transition_to(CC_ST_CYCLE_WAIT);
			break;

		case KNS_MAC_TX_TIMEOUT:
		case KNS_MAC_TX_ABORT:
			MGR_LOG_DEBUG("[CC] TX %s\r\n",
				(evt.id == KNS_MAC_TX_ABORT) ? "abort" : "timeout");
			MGR_ERR_log(ERR_TX_TIMEOUT);
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, (uint16_t)tx_count);
			if (cc_state == CC_ST_WAIT_TX_DONE)
				transition_to(CC_ST_CYCLE_WAIT);
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[CC] MAC error: %d\r\n", evt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)evt.status);
			if (cc_state == CC_ST_WAIT_TX_DONE)
				transition_to(CC_ST_CYCLE_WAIT);
			else if (cc_state == CC_ST_WAIT_MAC_READY)
				transition_to(CC_ST_INIT_MAC);
			break;

		default:
			break;
		}
	}
	return got;
}

/* ---- Sensor phases ---- */

#if CC_ENABLE_WS85
static void ws85_pump(void)
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
}
#endif /* CC_ENABLE_WS85 */

#if CC_ENABLE_ENV_I2C
/* One shared session: rail up, one I2C lifecycle, each enabled chip read and
 * stamped independently (a miss keeps that chip's previous reading and the
 * cache just ages out). SHT45 preempts: when present, BMP/BME + AHT20 are
 * skipped that cycle (temp priority SHT45 > BMP > AHT > fresh WS85). */
static void env_sample(void)
{
	if (!CC_HW_i2cSessionOpen())
		return;

	const cc_i2c_ops_t *bus = CC_HW_i2cOps();

#if CC_ENABLE_ADC_SENSE
	/* Supply voltages first (internal ADC — not on the I2C bus, but sampled
	 * in the same phase as the original ADS1115 was). NiMH via MGR_BAT's
	 * gated divider; supercap via ADC_IN3. */
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
	}
#endif

	bool sht_found = false;
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
		}

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

	CC_HW_i2cSessionClose();
}

static bool env_stale(void)
{
	bool stale = false;

	stale = stale || !cache_fresh(&sht_cache.meta, CC_ENV_MAX_AGE_MS);
	stale = stale || !cache_fresh(&baro_cache.meta, CC_ENV_MAX_AGE_MS);
	stale = stale || !cache_fresh(&aht_cache.meta, CC_ENV_MAX_AGE_MS);
#if CC_ENABLE_ADC_SENSE
	stale = true; /* supply voltages: sample every cycle */
#endif
	return stale;
}
#endif /* CC_ENABLE_ENV_I2C */

/* ---- Payload ---- */

static void build_fields(cc_payload_fields_t *f)
{
	memset(f, 0, sizeof(*f));

#if CC_ENABLE_GPS
	f->lat = gps_lat; /* last-known fix; (0,0) until the first ever */
	f->lon = gps_lon;
#endif
	f->epoch = clock_now();
	f->wake16 = (uint16_t)(HAL_GetTick() >> 16); /* 65.5 s ticks since boot */

#if CC_ENABLE_WS85
	f->ws_valid = cache_fresh(&ws_cache.meta, CC_WS85_MAX_AGE_MS);
	f->wind_ms = ws_cache.wind_ms;
	f->gust_ms = ws_cache.gust_ms;
	f->dir_deg = ws_cache.dir_deg;
	f->rain_mm = ws_cache.rain_mm;
#endif

#if CC_ENABLE_ENV_I2C
	/* Temp priority: SHT45 > BMP/BME > AHT20; humidity: SHT45 > BME > AHT20;
	 * pressure: BMP/BME only. */
	if (cache_fresh(&sht_cache.meta, CC_ENV_MAX_AGE_MS)) {
		f->hum_valid = true;
		f->hum_pct = sht_cache.rh;
		f->temp_valid = true;
		f->temp_c = sht_cache.tc;
	}
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

/** This cycle's battery view for the tier machine: the NiMH pack when it was
 *  measured this cycle, else 0 (failed read lands LOW — conservative). */
static uint16_t battery_for_tier(bool *nimh)
{
#if CC_ENABLE_ADC_SENSE
	if (cache_fresh(&adc_cache.meta, CC_ENV_MAX_AGE_MS)) {
		*nimh = true;
		return adc_cache.nimh_mv;
	}
#endif
	*nimh = true;
	return 0;
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

	if (KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[CC] TX push failed\r\n");
		return false;
	}
	return true;
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
	/* IWDG early — before any slow init */
	MGR_WDG_init();

	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);
	MGR_ERR_init();

	CC_HW_init();
	MGR_BAT_init();
	cc_tier_init(&tier);

	MGR_LPM_registerClient(cc_lpm_client);

	cycle_start_tick = HAL_GetTick();

	MGR_LOG_INFO("[CC] CounterCurrent tracker init (payload v3, LDA2)\r\n");
	MGR_LOG_INFO("[CC] periods %lu/%lu/%lu s, jitter %u%%, GPS every %u TX\r\n",
		(unsigned long)(CC_TX_PERIOD_MS / 1000),
		(unsigned long)(CC_TX_PERIOD_MID_MS / 1000),
		(unsigned long)(CC_TX_PERIOD_LOW_MS / 1000),
		(unsigned)CC_TX_JITTER_PCT, (unsigned)CC_GPS_FIX_EVERY_N_TX);

	transition_to(CC_ST_BOOT);
}

void KNS_APP_cc_tracker_loop(void)
{
	MGR_WDG_refresh();

	switch (cc_state) {

	case CC_ST_BOOT:
		if (state_elapsed_ms() > TIMEOUT_BOOT_MS)
			transition_to(CC_ST_INIT_MAC);
		return;

	case CC_ST_INIT_MAC:
		start_mac_profile();
		transition_to(CC_ST_WAIT_MAC_READY);
		return;

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
#endif
		transition_to(CC_ST_SAMPLE_GPS);
		return;

	case CC_ST_SAMPLE_GPS:
#if CC_ENABLE_GPS
		/* Schedule gates, evaluated once on entry (budget==0 marks entry) */
		if (gps_budget_ms == 0) {
			if (gps_countdown > 0) {
				gps_countdown--;
				MGR_LOG_DEBUG("[GPS] scheduled skip (%u left)\r\n", gps_countdown);
				transition_to(CC_ST_TX_SEND);
				return;
			}
			if (gps_skip_cycles > 0) {
				gps_skip_cycles--;
				MGR_LOG_DEBUG("[GPS] backoff skip (%u left)\r\n", gps_skip_cycles);
				transition_to(CC_ST_TX_SEND);
				return;
			}
			gps_budget_ms = (uint32_t)gps_durations * CC_GPS_TTFF_BUDGET_MS;
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
				transition_to(CC_ST_TX_SEND);
				return;
			}
		}
		if (state_elapsed_ms() > gps_budget_ms) {
			MGR_LOG_DEBUG("[GPS] no fix in %lus (%lu sentences, %lu cs-err) — backoff %u\r\n",
				(unsigned long)(gps_budget_ms / 1000),
				(unsigned long)nmea.sentences,
				(unsigned long)nmea.cs_errors, gps_durations);
			CC_HW_uartClose();
			CC_HW_gpsPowerOff();
			gps_skip_cycles = gps_durations;
			if (gps_durations < CC_GPS_MAX_DURATIONS)
				gps_durations++;
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

		build_fields(&f);
		cc_payload_pack(payload, &f);

		MGR_EVTLOG_log(EVT_BAT, f.batt_mv);
		MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);

		if (push_tx(payload)) {
			tx_count++;
			transition_to(CC_ST_WAIT_TX_DONE);
		} else {
			transition_to(CC_ST_CYCLE_WAIT);
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
			transition_to(CC_ST_CYCLE_WAIT);
		}
		return;

	case CC_ST_CYCLE_WAIT:
	{
		/* Compute this cycle's period once, on entry (marked by 0) */
		if (cycle_period_ms == 0) {
			bool nimh;
			uint16_t batt = battery_for_tier(&nimh);
			uint32_t base = cc_tier_update(&tier, batt, nimh);

			cycle_period_ms = cc_tier_jittered_period_ms(base,
				CC_TX_JITTER_PCT, CC_HW_rand32());
			MGR_LOG_INFO("[SLP] batt %umV -> base %lus, period %lus\r\n",
				batt, (unsigned long)(base / 1000),
				(unsigned long)(cycle_period_ms / 1000));
		}

		process_mac_events(); /* drain strays */

		uint32_t elapsed = HAL_GetTick() - cycle_start_tick;

		if (elapsed < cycle_period_ms)
			return;

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
