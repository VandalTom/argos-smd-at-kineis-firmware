/**
 * @file    test_cc_console.c
 * @brief   Unit tests for the CC_TRACKER console command filter (cc_console.c).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Src/cc_console.c"

/** Every accepted spelling of AT+BOOT, with any line ending, requests DFU. */
void test_boot_forms(void)
{
	ASSERT_EQ(CC_CON_BOOT, cc_console_classify("AT+BOOT=\r\n"));
	ASSERT_EQ(CC_CON_BOOT, cc_console_classify("AT+BOOT=?\r"));
	ASSERT_EQ(CC_CON_BOOT, cc_console_classify("AT+BOOT\n"));
	ASSERT_EQ(CC_CON_BOOT, cc_console_classify("AT+BOOT"));
	TEST_PASS();
}

/** Read-only status queries go to the standard handlers. */
void test_queries_dispatched(void)
{
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+FW=?\r\n"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+VERSION=?\r"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+PING=?\n"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+ID=?\r\n"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+ADDR=?\r\n"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+SN=?\r\n"));
	ASSERT_EQ(CC_CON_DISPATCH, cc_console_classify("AT+RCONF=?\r\n"));
	TEST_PASS();
}

/** Tracker control commands. */
void test_control_commands(void)
{
	ASSERT_EQ(CC_CON_RESET, cc_console_classify("AT+RESET=\r\n"));
	ASSERT_EQ(CC_CON_RESET, cc_console_classify("AT+RESET\r\n"));
	ASSERT_EQ(CC_CON_SELFTEST, cc_console_classify("AT+SELFTEST\r\n"));
	ASSERT_EQ(CC_CON_SELFTEST, cc_console_classify("AT+SELFTEST=\r\n"));
	ASSERT_EQ(CC_CON_SELFTEST, cc_console_classify("AT+SELFTEST=GPS,TX\r\n"));
	ASSERT_EQ(CC_CON_SELFTEST_QUERY, cc_console_classify("AT+SELFTEST=?\r\n"));
	ASSERT_EQ(CC_CON_STATE, cc_console_classify("AT+STATE=?\r\n"));
	ASSERT_EQ(CC_CON_COMMISSION_QUERY, cc_console_classify("AT+COMMISSION=?\r\n"));
	ASSERT_EQ(CC_CON_COMMISSION, cc_console_classify("AT+COMMISSION=START\r\n"));
	ASSERT_EQ(CC_CON_COMMISSION, cc_console_classify("AT+COMMISSION=SKIP\r\n"));
	ASSERT_EQ(CC_CON_COMMISSION, cc_console_classify("AT+COMMISSION=ARM\r\n"));
	TEST_PASS();
}

/** Kineis credential writes are forwarded; the key is only ever fingerprinted. */
void test_credentials(void)
{
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+ID=123456\r\n"));
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+ADDR=0A1B2C3D\r\n"));
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+SECKEY=00112233445566778899AABBCCDDEEFF\r\n"));
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+RCONF=44cd3a299068292a74d2126f3c9e0bf1\r\n"));
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+SAVE_RCONF=\r\n"));
	ASSERT_EQ(CC_CON_CRED, cc_console_classify("AT+SAVE_RCONF\r\n"));
	ASSERT_EQ(CC_CON_KEYCRC, cc_console_classify("AT+SECKEY=?\r\n"));
	/* a write needs a value */
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+ID=\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+SECKEY\r\n"));
	TEST_PASS();
}

/** Erasing the credentials takes the exact, explicit form only. */
void test_prov_erase(void)
{
	ASSERT_EQ(CC_CON_PROV_ERASE, cc_console_classify("AT+PROV=ERASE\r\n"));
	ASSERT_EQ(CC_CON_PROV_ERASE, cc_console_classify("AT+PROV=ERASE"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+PROV\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+PROV=\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+PROV=?\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+PROV=erase\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+PROV=ERASEX\r\n"));
	TEST_PASS();
}

/** The console log detail: a query, or one digit 0..4. */
void test_loglvl(void)
{
	ASSERT_EQ(CC_CON_LOGLVL_QUERY, cc_console_classify("AT+LOGLVL=?\r\n"));
	ASSERT_EQ(CC_CON_LOGLVL, cc_console_classify("AT+LOGLVL=0\r\n"));
	ASSERT_EQ(CC_CON_LOGLVL, cc_console_classify("AT+LOGLVL=1"));
	ASSERT_EQ(CC_CON_LOGLVL, cc_console_classify("AT+LOGLVL=4\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+LOGLVL=5\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+LOGLVL=10\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+LOGLVL=\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+LOGLVL\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+LOGLVL=x\r\n"));
	ASSERT_EQ('3', cc_console_params("AT+LOGLVL=3\r\n")[0]);
	TEST_PASS();
}

/** Anything that could drive the MAC directly is refused. */
void test_actions_rejected(void)
{
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+TX=00112233\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+KMAC=1\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+FW\r\n"));       /* action form */
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+CW=1\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+RCONFRAW=?\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+COMMISSION=NOW\r\n"));
	TEST_PASS();
}

/** Near-misses and junk: no prefix matching, no case folding. */
void test_near_misses_rejected(void)
{
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+BOOTX\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+BOOT=1\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("at+boot\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+FW=?X\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+RESET=1\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify("AT+\r\n"));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify(""));
	ASSERT_EQ(CC_CON_REJECT, cc_console_classify(NULL));
	TEST_PASS();
}

/** Parameter extraction and the key fingerprint CRC. */
void test_params_and_crc(void)
{
	ASSERT_TRUE(strncmp(cc_console_params("AT+SELFTEST=GPS,TX\r\n"), "GPS,TX\r", 7) == 0);
	ASSERT_TRUE(cc_console_params("AT+SELFTEST\r\n")[0] == '\0');
	ASSERT_EQ_HEX(0xCBF43926UL, cc_console_crc32((const unsigned char *)"123456789", 9));
	ASSERT_EQ_HEX(0x00000000UL, cc_console_crc32((const unsigned char *)"", 0));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("CC_TRACKER console filter");
	RUN_TEST(test_boot_forms);
	RUN_TEST(test_queries_dispatched);
	RUN_TEST(test_control_commands);
	RUN_TEST(test_credentials);
	RUN_TEST(test_prov_erase);
	RUN_TEST(test_loglvl);
	RUN_TEST(test_actions_rejected);
	RUN_TEST(test_near_misses_rejected);
	RUN_TEST(test_params_and_crc);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
