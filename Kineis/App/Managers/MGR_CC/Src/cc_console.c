/**
 * @file    cc_console.c
 * @brief   CC_TRACKER debug-console command filter.
 */

#include <string.h>
#include "cc_console.h"

/** Read-only queries forwarded to the standard AT handlers. */
static const char *const cc_con_queries[] = {
	"AT+FW=?",
	"AT+VERSION=?",
	"AT+PING=?",
	"AT+ID=?",
	"AT+ADDR=?",
	"AT+SN=?",
	"AT+RCONF=?",
};

/** Credential writes (need a parameter) forwarded to the standard handlers. */
static const char *const cc_con_creds[] = {
	"AT+ID",
	"AT+ADDR",
	"AT+SECKEY",
	"AT+RCONF",
};

static int body_is(const char *line, size_t len, const char *s)
{
	return strlen(s) == len && strncmp(line, s, len) == 0;
}

/** Bare command, or command followed by '=' (optionally with parameters). */
static int is_action(const char *line, size_t len, const char *cmd, int allow_params)
{
	size_t c = strlen(cmd);

	if (len < c || strncmp(line, cmd, c) != 0)
		return 0;
	if (len == c)
		return 1;
	if (line[c] != '=')
		return 0;
	if (len == c + 1)
		return 1;
	return allow_params && line[c + 1] != '?';
}

cc_con_action_t cc_console_classify(const char *line)
{
	if (line == NULL)
		return CC_CON_REJECT;

	size_t len = strcspn(line, "\r\n");

	if (body_is(line, len, "AT+BOOT=?") || is_action(line, len, "AT+BOOT", 0))
		return CC_CON_BOOT;
	if (is_action(line, len, "AT+RESET", 0))
		return CC_CON_RESET;
	if (body_is(line, len, "AT+SELFTEST=?"))
		return CC_CON_SELFTEST_QUERY;
	if (is_action(line, len, "AT+SELFTEST", 1))
		return CC_CON_SELFTEST;
	if (body_is(line, len, "AT+STATE=?"))
		return CC_CON_STATE;
	if (body_is(line, len, "AT+COMMISSION=?"))
		return CC_CON_COMMISSION_QUERY;
	if (body_is(line, len, "AT+COMMISSION=START") || body_is(line, len, "AT+COMMISSION=SKIP") ||
	    body_is(line, len, "AT+COMMISSION=ARM"))
		return CC_CON_COMMISSION;
	if (body_is(line, len, "AT+SECKEY=?"))
		return CC_CON_KEYCRC;
	if (body_is(line, len, "AT+PROV=ERASE"))
		return CC_CON_PROV_ERASE;
	if (is_action(line, len, "AT+SAVE_RCONF", 0))
		return CC_CON_CRED;

	for (size_t i = 0; i < sizeof(cc_con_queries) / sizeof(cc_con_queries[0]); i++)
		if (body_is(line, len, cc_con_queries[i]))
			return CC_CON_DISPATCH;

	/* Writes need a value: "AT+ID=123456" yes, bare "AT+ID" / "AT+ID=" no */
	for (size_t i = 0; i < sizeof(cc_con_creds) / sizeof(cc_con_creds[0]); i++) {
		size_t c = strlen(cc_con_creds[i]);

		if (len > c + 1 && strncmp(line, cc_con_creds[i], c) == 0 && line[c] == '=')
			return CC_CON_CRED;
	}

	return CC_CON_REJECT;
}

const char *cc_console_params(const char *line)
{
	const char *eq = line ? strchr(line, '=') : NULL;

	return eq ? eq + 1 : "";
}

unsigned long cc_console_crc32(const unsigned char *data, unsigned long len)
{
	unsigned long crc = 0xFFFFFFFFUL;

	while (len--) {
		crc ^= *data++;
		for (int b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
	}
	return (crc ^ 0xFFFFFFFFUL) & 0xFFFFFFFFUL;
}
