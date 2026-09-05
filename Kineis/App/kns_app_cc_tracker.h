/**
 * @file    kns_app_cc_tracker.h
 * @brief   CC_TRACKER application — CounterCurrent satellite weather tracker.
 *
 * Timer-based read-and-transmit cycle ported from the CounterCurrent
 * WS85-Tracker (XIAO nRF52840 + AT-driven module), collapsed onto the
 * STM32WL55: sensors are read by this MCU and the 24-byte payload v3 goes
 * straight to the Kineis MAC.
 *
 * Publish cycle, every battery-tiered period (+ RNG jitter):
 *   1. refresh each sensor cache that has aged past its max age
 *      (WS85 wind/rain, SHT45/BMP-BME280/AHT20 env, supercap/NiMH sense)
 *   2. GPS fix on the every-Nth-TX schedule with adaptive TTFF backoff
 *   3. pack the caches into the 24-byte v3 payload — a stale group goes out
 *      as its all-ones "invalid" sentinel, never as old data
 *   4. KNS_MAC_SEND_DATA (BASIC profile, LDA2) and wait for TX_DONE
 *   5. wait out the rest of the period; scheduled reboot at 14 days
 *
 * Configuration is compile-time (cc_conf.h) by design; the LPUART console
 * carries MGR_LOG diagnostics.
 */

#ifndef KNS_APP_CC_TRACKER_H
#define KNS_APP_CC_TRACKER_H

/** One-time init: hardware safe-states, watchdog, event log, MAC config. */
void KNS_APP_cc_tracker_init(void);

/** App task body, called forever by KNS_OS_main round-robin. */
void KNS_APP_cc_tracker_loop(void);

/** True while a sensor/TX phase is active (IDLE task LPM gate: only the
 *  CYCLE_WAIT and MAC-wait phases are safe for low-power entry). */
bool KNS_APP_cc_tracker_isBusy(void);

#endif /* KNS_APP_CC_TRACKER_H */
