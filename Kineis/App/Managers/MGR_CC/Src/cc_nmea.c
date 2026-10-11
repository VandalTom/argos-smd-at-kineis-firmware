/**
 * @file    cc_nmea.c
 * @brief   NMEA RMC+GGA+GSV parser — port of WS85-Tracker GpsNmea.h.
 */

#include <string.h>
#include <stdlib.h>
#include "cc_nmea.h"
#include "cc_conf.h"

void cc_nmea_init(cc_nmea_parser_t *p)
{
	memset(p, 0, sizeof(*p));
}

const cc_gps_fix_t *cc_nmea_fix(const cc_nmea_parser_t *p)
{
	return &p->fix;
}

static uint8_t hexv(char c)
{
	if (c >= '0' && c <= '9')
		return (uint8_t)(c - '0');
	if (c >= 'A' && c <= 'F')
		return (uint8_t)(c - 'A' + 10);
	if (c >= 'a' && c <= 'f')
		return (uint8_t)(c - 'a' + 10);
	return 0;
}

static bool checksum_ok(cc_nmea_parser_t *p)
{
	int star = -1;

	for (uint16_t i = 0; i < p->len; i++) {
		if (p->line[i] == '*') {
			star = (int)i;
			break;
		}
	}
	if (star < 0 || star + 2 >= (int)p->len)
		return false;

	uint8_t cs = 0;
	for (int i = 0; i < star; i++)
		cs ^= (uint8_t)p->line[i];

	uint8_t want = (uint8_t)((hexv(p->line[star + 1]) << 4) | hexv(p->line[star + 2]));
	return cs == want;
}

/* Split on ',' in place; '*' terminates. Returns field count. */
static int split(cc_nmea_parser_t *p, char **f, int maxf)
{
	int n = 0;

	f[n++] = p->line;
	for (uint16_t i = 0; i < p->len && n < maxf; i++) {
		if (p->line[i] == '*') {
			p->line[i] = 0;
			break;
		}
		if (p->line[i] == ',') {
			p->line[i] = 0;
			f[n++] = p->line + i + 1;
		}
	}
	return n;
}

static double nmea_coord(const char *s, char hemi)
{
	if (!s || !*s)
		return 0;

	double v = atof(s);
	int deg = (int)(v / 100);
	double minutes = v - deg * 100.0;
	double dec = deg + minutes / 60.0;

	if (hemi == 'S' || hemi == 'W')
		dec = -dec;
	return dec;
}

static int d2(const char *s)
{
	return (s[0] - '0') * 10 + (s[1] - '0');
}

/* Howard Hinnant days_from_civil */
static long days_from_civil(int y, unsigned m, unsigned d)
{
	y -= m <= 2;
	long era = (y >= 0 ? y : y - 399) / 400;
	unsigned yoe = (unsigned)(y - era * 400);
	unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
	unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097L + (long)doe - 719468L;
}

static uint32_t to_epoch(const char *tm, const char *dt)
{
	if (!tm || strlen(tm) < 6 || !dt || strlen(dt) < 6)
		return 0;

	int hh = d2(tm), mm = d2(tm + 2), ss = d2(tm + 4);
	int day = d2(dt), mon = d2(dt + 2), yr = 2000 + d2(dt + 4);

	return (uint32_t)days_from_civil(yr, (unsigned)mon, (unsigned)day) * 86400UL
		+ (uint32_t)hh * 3600UL + (uint32_t)mm * 60UL + (uint32_t)ss;
}

static void parse_gga(cc_nmea_parser_t *p)
{
	char *f[15];
	int n = split(p, f, 15);

	if (n < 8)
		return;
	p->last_qual = (uint8_t)atoi(f[6]);
	p->last_sats = (uint8_t)atoi(f[7]);
}

static bool parse_rmc(cc_nmea_parser_t *p)
{
	char *f[14];
	int n = split(p, f, 14);

	if (n < 10)
		return false;

	bool active = (f[2][0] == 'A');
	cc_gps_fix_t nf;

	memset(&nf, 0, sizeof(nf));
	nf.lat = nmea_coord(f[3], f[4][0]);
	nf.lon = nmea_coord(f[5], f[6][0]);
	nf.epoch = to_epoch(f[1], f[9]);
	nf.sats = p->last_sats;
	nf.quality = p->last_qual;
	nf.valid = active && p->last_qual >= 1 && p->last_sats >= 4;

	p->fix = nf;
	return nf.valid;
}

/* GSV: numMsg,msgNum,numSV, then per satellite svid,elev,az,cno (cno may be
 * empty when not tracked). NMEA 4.1 appends a signal id after the last
 * block; the 4-field stride ignores it. */
static int talker_slot(const char *t)
{
	if (t[0] == 'G' && t[1] == 'P')
		return 0;
	if (t[0] == 'G' && t[1] == 'L')
		return 1;
	if (t[0] == 'G' && t[1] == 'A')
		return 2;
	if ((t[0] == 'G' && t[1] == 'B') || (t[0] == 'B' && t[1] == 'D'))
		return 3;
	return 4;
}

static void parse_gsv(cc_nmea_parser_t *p)
{
	char *f[24];
	int slot = talker_slot(p->line);
	int n = split(p, f, 24);

	if (n < 4)
		return;
	int num = atoi(f[1]);
	int idx = atoi(f[2]);

	if (idx == 1) {
		p->gsv_cur = 0;
		p->gsv_in_group = true;
	}
	for (int k = 4; k + 3 < n; k += 4) {
		int cno = f[k + 3][0] ? atoi(f[k + 3]) : 0;

		if (cno > p->max_cno)
			p->max_cno = (uint8_t)cno;
		if (cno >= CC_GPS_SKY_MIN_CNO && p->gsv_cur < 255)
			p->gsv_cur++;
	}
	/* Commit only groups seen from message 1: joining mid-group (listening
	 * starts late, or a lost sentence) would under-count the sky */
	if (idx == num && p->gsv_in_group) {
		p->gsv_strong[slot] = p->gsv_cur;
		p->gsv_in_group = false;
		if (p->gsv_groups < 0xFFFFu)
			p->gsv_groups++;
	}
	p->gsv_msgs++;
}

uint8_t cc_nmea_sky_strong(const cc_nmea_parser_t *p)
{
	unsigned sum = 0;

	for (int i = 0; i < 5; i++)
		sum += p->gsv_strong[i];
	return (uint8_t)(sum > 255 ? 255 : sum);
}

bool cc_nmea_no_sky(const cc_nmea_parser_t *p, uint32_t elapsed_ms, uint32_t check_ms,
                    uint8_t min_strong)
{
	return elapsed_ms >= check_ms && p->gsv_groups >= CC_GPS_SKY_MIN_GROUPS &&
	       cc_nmea_sky_strong(p) < min_strong;
}

static bool parse(cc_nmea_parser_t *p)
{
	p->line[p->len] = 0;
	if (!checksum_ok(p)) {
		p->cs_errors++;
		return false;
	}
	p->sentences++;

	const char *t = (p->len >= 5) ? p->line + 2 : p->line; /* skip talker id */
	if (!strncmp(t, "RMC", 3))
		return parse_rmc(p);
	if (!strncmp(t, "GGA", 3))
		parse_gga(p);
	else if (!strncmp(t, "GSV", 3))
		parse_gsv(p);
	return false;
}

bool cc_nmea_feed(cc_nmea_parser_t *p, uint8_t c)
{
	if (c == '$') {
		p->len = 0;
		p->have = true;
		return false;
	}
	if (!p->have)
		return false;
	if (c == '\r' || c == '\n') {
		bool f = p->len ? parse(p) : false;

		p->len = 0;
		p->have = false;
		return f;
	}
	if (p->len < sizeof(p->line) - 1)
		p->line[p->len++] = (char)c;
	return false;
}
