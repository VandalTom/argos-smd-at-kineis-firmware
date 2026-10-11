/**
 * @file    cc_console.h
 * @brief   CC_TRACKER debug-console command filter.
 *
 * The tracker owns the MAC queues and its own TX schedule, so the full GUI AT
 * set (AT+TX, AT+KMAC, credential writes...) must not reach it from the
 * console. Lines framed by MGR_AT_CMD on LPUART1 are classified here:
 *   - AT+BOOT / AT+BOOT= / AT+BOOT=?        -> CC_CON_BOOT (UART DFU bootloader
 *                                               at the next safe point)
 *   - AT+RESET / AT+RESET=                   -> CC_CON_RESET (reboot, same deferral)
 *   - AT+SELFTEST / AT+SELFTEST=[items]      -> CC_CON_SELFTEST (run now)
 *   - AT+SELFTEST=?                          -> CC_CON_SELFTEST_QUERY (last report)
 *   - AT+STATE=?                             -> CC_CON_STATE
 *   - AT+COMMISSION=?                        -> CC_CON_COMMISSION_QUERY
 *   - AT+COMMISSION=START|SKIP|ARM           -> CC_CON_COMMISSION
 *   - AT+ID= / ADDR= / SECKEY= / RCONF=<v>,
 *     AT+SAVE_RCONF                          -> CC_CON_CRED (standard handler,
 *                                               refused while transmitting)
 *   - AT+SECKEY=?                            -> CC_CON_KEYCRC (CRC-32 of the key;
 *                                               the key itself is never echoed)
 *   - read-only status queries (=?) below    -> CC_CON_DISPATCH (standard handler)
 *   - anything else                          -> CC_CON_REJECT (+ERROR)
 *
 * Only the text before the first CR/LF is compared; the caller passes the
 * original line (terminator included) on to MGR_AT_CMD_decodeAt().
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_CONSOLE_H
#define CC_CONSOLE_H

typedef enum {
	CC_CON_REJECT = 0,
	CC_CON_DISPATCH,
	CC_CON_BOOT,
	CC_CON_RESET,
	CC_CON_SELFTEST,
	CC_CON_SELFTEST_QUERY,
	CC_CON_STATE,
	CC_CON_COMMISSION,
	CC_CON_COMMISSION_QUERY,
	CC_CON_CRED,
	CC_CON_KEYCRC,
} cc_con_action_t;

/**
 * @brief Classify one console line queued by MGR_AT_CMD.
 * @param line NUL-terminated line starting with "AT+" (NULL -> reject)
 */
cc_con_action_t cc_console_classify(const char *line);

/**
 * @brief Parameter text of an action command ("AT+SELFTEST=GPS,TX\r" -> "GPS,TX\r").
 * @return pointer just past the first '=', or "" when there is none.
 */
const char *cc_console_params(const char *line);

/**
 * @brief CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF) — the device
 *        key fingerprint reported by AT+SECKEY=? ("123456789" -> 0xCBF43926).
 */
unsigned long cc_console_crc32(const unsigned char *data, unsigned long len);

#endif /* CC_CONSOLE_H */
