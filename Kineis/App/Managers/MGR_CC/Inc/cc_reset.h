/**
 * @file    cc_reset.h
 * @brief   CC_TRACKER: was this boot a deliberate reset? (rescue-mode counting)
 *
 * Every boot counts towards rescue mode (cc_conf.h) unless it was deliberate.
 * RCC_CSR, captured at the start of main() before it is cleared, says why the
 * chip reset. Every internal reset also drives the NRST pin, so it sets
 * PINRSTF next to its own flag; a reset from the pin alone (a reset button,
 * a debug probe) sets PINRSTF only.
 *
 * Pure C, no MCU dependencies — unit-tested on the host. The tracker checks
 * these bit positions against the CMSIS definitions at compile time.
 */

#ifndef CC_RESET_H
#define CC_RESET_H

#include <stdbool.h>
#include <stdint.h>

/* RCC_CSR reset flags (STM32WL RM0453) */
#define CC_CSR_OBLRSTF    (1UL << 25)  /**< option-byte load */
#define CC_CSR_PINRSTF    (1UL << 26)  /**< NRST pin (also set by every internal reset) */
#define CC_CSR_BORRSTF    (1UL << 27)  /**< brownout or power-on */
#define CC_CSR_SFTRSTF    (1UL << 28)  /**< software: crash handlers, NVIC_SystemReset */
#define CC_CSR_IWDGRSTF   (1UL << 29)  /**< independent watchdog */
#define CC_CSR_WWDGRSTF   (1UL << 30)  /**< window watchdog */
#define CC_CSR_LPWRRSTF   (1UL << 31)  /**< illegal low-power mode entry */
#define CC_CSR_CAUSE_RSTF (CC_CSR_OBLRSTF | CC_CSR_BORRSTF | CC_CSR_SFTRSTF | \
                           CC_CSR_IWDGRSTF | CC_CSR_WWDGRSTF | CC_CSR_LPWRRSTF)

/** A reset from the NRST pin alone: a button press or a debug probe. */
static inline bool cc_reset_from_pin_only(uint32_t csr)
{
	return (csr & CC_CSR_PINRSTF) != 0u && (csr & CC_CSR_CAUSE_RSTF) == 0u;
}

#endif /* CC_RESET_H */
