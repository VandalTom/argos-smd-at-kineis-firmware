/**
 * @file    cc_selftest.c
 * @brief   CC_TRACKER self-test bookkeeping and report formatting.
 */

#include <stdio.h>
#include <string.h>
#include "cc_selftest.h"

static const char *const item_names[CC_STI_COUNT] = { "GPS", "WS85", "ENV", "ADC", "TX" };

void cc_selftest_begin(cc_selftest_t *st, uint8_t mask, bool at_boot)
{
	memset(st->item, 0, sizeof(st->item));
	st->mask = mask & CC_ST_ALL;
	st->running = true;
	st->at_boot = at_boot;
}

void cc_selftest_set(cc_selftest_t *st, cc_st_item_t i, cc_st_verdict_t v, const char *detail)
{
	if (i >= CC_STI_COUNT)
		return;
	st->item[i].v = v;
	snprintf(st->item[i].detail, sizeof(st->item[i].detail), "%s", detail ? detail : "");
}

void cc_selftest_end(cc_selftest_t *st)
{
	st->running = false;
	st->done = true;
}

bool cc_selftest_passed(const cc_selftest_t *st)
{
	for (int i = 0; i < CC_STI_COUNT; i++) {
		if (!(st->mask & CC_ST_BIT(i)))
			continue;
		if (st->item[i].v == CC_STV_FAIL || st->item[i].v == CC_STV_NOTRUN)
			return false;
	}
	return true;
}

uint8_t cc_selftest_parse_mask(const char *params)
{
	if (params == NULL)
		return CC_ST_ALL;

	size_t len = strcspn(params, "\r\n");
	uint8_t mask = 0;
	size_t pos = 0;

	if (len == 0)
		return CC_ST_ALL;

	while (pos <= len) {
		size_t tok = strcspn(params + pos, ",\r\n");
		bool known = false;

		if (pos + tok > len)
			tok = len - pos;
		if (tok == 3 && strncmp(params + pos, "ALL", 3) == 0) {
			mask |= CC_ST_ALL;
			known = true;
		}
		for (int i = 0; i < CC_STI_COUNT && !known; i++) {
			if (strlen(item_names[i]) == tok && strncmp(params + pos, item_names[i], tok) == 0) {
				mask |= CC_ST_BIT(i);
				known = true;
			}
		}
		if (!known)
			return 0;
		pos += tok + 1;
	}
	return mask;
}

const char *cc_selftest_item_name(cc_st_item_t i)
{
	return (i < CC_STI_COUNT) ? item_names[i] : "?";
}

const char *cc_selftest_verdict_name(cc_st_verdict_t v)
{
	switch (v) {
	case CC_STV_PASS: return "PASS";
	case CC_STV_FAIL: return "FAIL";
	case CC_STV_SKIP: return "SKIP";
	default:          return "NOTRUN";
	}
}

int cc_selftest_format_item(char *buf, size_t n, const cc_selftest_t *st, cc_st_item_t i)
{
	const cc_st_result_t *r = &st->item[i];

	if (r->detail[0])
		return snprintf(buf, n, "+ST=%s,%s,%s\r\n", cc_selftest_item_name(i),
		                cc_selftest_verdict_name(r->v), r->detail);
	return snprintf(buf, n, "+ST=%s,%s\r\n", cc_selftest_item_name(i),
	                cc_selftest_verdict_name(r->v));
}

int cc_selftest_format_summary(char *buf, size_t n, const cc_selftest_t *st)
{
	if (cc_selftest_passed(st))
		return snprintf(buf, n, "+SELFTEST=PASS\r\n");

	char failed[48] = "";

	for (int i = 0; i < CC_STI_COUNT; i++) {
		if (!(st->mask & CC_ST_BIT(i)))
			continue;
		if (st->item[i].v == CC_STV_FAIL || st->item[i].v == CC_STV_NOTRUN) {
			strcat(failed, ",");
			strcat(failed, item_names[i]);   /* <= 5 x ",WS85" fits */
		}
	}
	return snprintf(buf, n, "+SELFTEST=FAIL%s\r\n", failed);
}

cc_st_verdict_t cc_selftest_eval_gps(uint32_t bytes, uint32_t sentences)
{
	(void)bytes;
	return sentences > 0 ? CC_STV_PASS : CC_STV_FAIL;
}

cc_st_verdict_t cc_selftest_eval_env(float tc, float hpa, float rh)
{
	if (tc < -40.0f || tc > 85.0f)
		return CC_STV_FAIL;
	if (hpa < 300.0f || hpa > 1100.0f)
		return CC_STV_FAIL;
	if (rh >= 0.0f && rh > 100.0f)
		return CC_STV_FAIL;
	return CC_STV_PASS;
}

cc_st_verdict_t cc_selftest_eval_adc(uint16_t nimh_mv, uint16_t min_mv, uint16_t max_mv)
{
	return (nimh_mv >= min_mv && nimh_mv <= max_mv) ? CC_STV_PASS : CC_STV_FAIL;
}
