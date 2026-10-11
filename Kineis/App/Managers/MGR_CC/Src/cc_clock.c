/**
 * @file    cc_clock.c
 * @brief   CC_TRACKER wall clock helpers (see cc_clock.h).
 */

#include "cc_clock.h"

uint32_t cc_clock_rtc_seconds(uint8_t yy, uint8_t mo, uint8_t dd,
	uint8_t hh, uint8_t mi, uint8_t ss)
{
	static const uint16_t cum_days[12] = {
		0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
	};
	uint32_t y = yy;
	/* Leap days in 2000..(2000+y-1): every 4th year from 2000 (2000 was
	 * leap, 2100 is out of the RTC's range) */
	uint32_t days = y * 365UL + (y + 3UL) / 4UL;

	if (mo >= 1 && mo <= 12) {
		days += cum_days[mo - 1];
		if (mo > 2 && (y % 4UL) == 0)
			days++;
	}
	if (dd >= 1)
		days += dd - 1UL;
	return days * 86400UL + hh * 3600UL + mi * 60UL + ss;
}

bool cc_clock_anchor_valid(uint32_t anchor, uint32_t anchor_inv)
{
	return anchor != 0 && anchor_inv == ~anchor;
}

bool cc_clock_epoch_plausible(uint32_t epoch, uint32_t min_epoch)
{
	return epoch >= min_epoch;
}
