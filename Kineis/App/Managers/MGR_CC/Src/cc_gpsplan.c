/**
 * @file    cc_gpsplan.c
 * @brief   When to power the GPS within a cycle (see cc_gpsplan.h).
 */

#include "cc_gpsplan.h"

void cc_gpsplan_init(cc_gpsplan_t *g)
{
	g->hot = false;
	g->ever_fixed = false;
	g->last_fix_ms = 0;
}

bool cc_gpsplan_early(const cc_gpsplan_t *g, uint32_t now_ms, uint32_t hot_age_ms)
{
	bool recent = g->ever_fixed && (now_ms - g->last_fix_ms) < hot_age_ms;

	return !(g->hot && recent);
}

void cc_gpsplan_on_fix(cc_gpsplan_t *g, uint32_t now_ms, bool early,
	uint32_t since_power_ms, uint32_t since_listen_ms, uint32_t hot_ms)
{
	g->hot = early ? since_listen_ms < hot_ms : since_power_ms < hot_ms;
	g->ever_fixed = true;
	g->last_fix_ms = now_ms;
}

void cc_gpsplan_on_miss(cc_gpsplan_t *g)
{
	g->hot = false;
}
