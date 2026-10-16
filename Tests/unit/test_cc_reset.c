/**
 * @file    test_cc_reset.c
 * @brief   Unit tests for the deliberate-reset check behind rescue-mode counting (cc_reset.h).
 */

#include "test_framework.h"
#include "../../Kineis/App/Managers/MGR_CC/Inc/cc_reset.h"

/** A reset button or a debug probe sets PINRSTF only. */
void test_pin_only_is_deliberate(void)
{
	ASSERT_TRUE(cc_reset_from_pin_only(CC_CSR_PINRSTF));
	/* other RCC_CSR bits (clock settings) do not matter */
	ASSERT_TRUE(cc_reset_from_pin_only(CC_CSR_PINRSTF | 0x00000F00UL));
	TEST_PASS();
}

/** Every internal reset drives NRST too: its own flag makes it count. */
void test_internal_resets_count(void)
{
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_IWDGRSTF));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_WWDGRSTF));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_SFTRSTF));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_BORRSTF));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_LPWRRSTF));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_PINRSTF | CC_CSR_OBLRSTF));
	TEST_PASS();
}

/** No flags at all (the bootloader jumped in after an update) is not a pin reset. */
void test_no_flags(void)
{
	ASSERT_FALSE(cc_reset_from_pin_only(0u));
	ASSERT_FALSE(cc_reset_from_pin_only(CC_CSR_SFTRSTF));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("cc_reset");
	RUN_TEST(test_pin_only_is_deliberate);
	RUN_TEST(test_internal_resets_count);
	RUN_TEST(test_no_flags);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
