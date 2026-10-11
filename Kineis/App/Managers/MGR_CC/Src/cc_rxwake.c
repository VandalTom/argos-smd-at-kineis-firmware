/**
 * @file    cc_rxwake.c
 * @brief   Console RX-wake storm guard (see cc_rxwake.h).
 */

#include "cc_rxwake.h"

void cc_rxwake_init(cc_rxwake_t *w)
{
	w->streak = 0;
	w->backoff_ms = 0;
	w->off_until = 0;
	w->off = false;
}

bool cc_rxwake_armed(cc_rxwake_t *w, uint32_t now_ms)
{
	if (w->off && (int32_t)(now_ms - w->off_until) >= 0)
		w->off = false;
	return !w->off;
}

bool cc_rxwake_on_edge(cc_rxwake_t *w, uint32_t now_ms, uint8_t storm_count,
	uint32_t backoff_ms, uint32_t backoff_max_ms)
{
	if (++w->streak < storm_count)
		return true;
	w->streak = 0;
	if (w->backoff_ms == 0)
		w->backoff_ms = backoff_ms;
	else if (w->backoff_ms <= backoff_max_ms / 2u)
		w->backoff_ms *= 2u;
	else
		w->backoff_ms = backoff_max_ms;
	w->off = true;
	w->off_until = now_ms + w->backoff_ms;
	return false;
}

void cc_rxwake_on_line(cc_rxwake_t *w)
{
	w->streak = 0;
	w->backoff_ms = 0;
	w->off = false;
}
