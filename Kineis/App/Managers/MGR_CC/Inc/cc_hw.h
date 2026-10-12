/**
 * @file    cc_hw.h
 * @brief   CC_TRACKER hardware glue: rails, USART1 sensor sessions, I2C2
 *          session, ADC sense, RNG jitter.
 *
 * Everything electrical for the CounterCurrent carrier, mirroring the roles of
 * the Arduino Board.h but on STM32WL55 HAL:
 *
 *  - Power: one always-on 3.3 V rail feeds module + BME280 + I2C pull-ups
 *    (no switched VCCIO — the AP2112K era ended with the XIAO board). Only
 *    the GPS is hard-switched, via CC_GPS_EN.
 *  - USART1 time-share: session A = GPS on PB6 (CR2.SWAP) @ CC_GPS_BAUD (or
 *    the baud the self-test found it at), session B = WS85 on PA10 @ 115200.
 *    The publish cycle is sequential, so one peripheral serves both; each
 *    open is a full deinit / pin-mux / init, with the RX pin pulled up. RX is
 *    interrupt-driven into a ring buffer the app drains each loop pass.
 *  - I2C2 session: bus-idle check (bit-banged bus-clear if a slave holds SDA
 *    low) -> HAL init. Close parks SDA/SCL high-Z (analog): the pull-ups sit
 *    on the live rail, so the bus idles high at zero static current. (The
 *    old pulldown-parking trick was for a DEAD switched rail — against a
 *    live rail it would burn ~1.4 mA through the pull-ups.)
 *  - ADC: supercap on ADC_IN3 (PB4) with VREFINT-calibrated true-VDDA math
 *    (same approach as MGR_BAT, which owns the NiMH/ADC_IN0 read).
 *  - RNG: one hardware-random word per cycle for the TX-period jitter.
 */

#ifndef CC_HW_H
#define CC_HW_H

#include <stdint.h>
#include <stdbool.h>
#include "cc_i2c_sens.h"

typedef enum {
	CC_UART_NONE = 0,
	CC_UART_GPS,   /**< session A: PB6 (swapped to RX), CC_HW_gpsBaud() */
	CC_UART_WS85,  /**< session B: PA10 RX, 115200 */
} cc_uart_session_t;

/** One-time init: drive every rail/control pin to its safe state
 *  (VCCIO off, GPS off, I2C parked pulled-down, sense pins analog). */
void CC_HW_init(void);

/* ---- USART1 sensor sessions ---- */

/** Open a sensor UART session at its baud (closes any previous one). */
bool CC_HW_uartOpen(cc_uart_session_t s);
/** Same, at an explicit baud (the self-test's GPS baud scan). */
bool CC_HW_uartOpenBaud(cc_uart_session_t s, uint32_t baud);
/** Drain up to @p max ring bytes into @p buf; returns count (non-blocking). */
uint16_t CC_HW_uartRead(uint8_t *buf, uint16_t max);
/** Count of ring overruns since open (diagnostic). */
uint16_t CC_HW_uartOverruns(void);
/** Framing/noise errors since open: a sender at another baud, or noise. */
uint16_t CC_HW_uartLineErrors(void);
/** Close the session: UART off, pins parked, rail released. */
void CC_HW_uartClose(void);

/* ---- GPS power (CC_GPS_EN; UART handled by the session) ---- */

void CC_HW_gpsPowerOn(void);
void CC_HW_gpsPowerOff(void);
/** How long the receiver has been switched off (0 while on). */
uint32_t CC_HW_gpsOffForMs(void);
/** The GPS session baud: CC_GPS_BAUD until the self-test finds another. */
uint32_t CC_HW_gpsBaud(void);
void CC_HW_gpsSetBaud(uint32_t baud);
/**
 * Probe the GPS data line (PB6) with the MCU's own pulls, ~16 ms: is it
 * high while pulled down, low while pulled up? (cc_selftest_line_class).
 * Not while the GPS session is open (returns false). Leaves PB6 parked.
 */
bool CC_HW_gpsLineProbe(bool *high_when_pulled_down, bool *low_when_pulled_up);

/* ---- I2C2 env-sensor session ---- */

/** Bus-idle check / bus-clear + HAL init. False = bus stuck low (skip this
 *  sample; retried next cycle — with an always-on rail the 9-clock bus-clear
 *  is the only unstick lever, so a persistent failure is a hardware fault). */
bool CC_HW_i2cSessionOpen(void);
/** Bus ops for the cc_i2c_sens drivers (valid between open and close). */
const cc_i2c_ops_t *CC_HW_i2cOps(void);
/** HAL deinit, park pins pulled-down, release the rail. */
void CC_HW_i2cSessionClose(void);

/* ---- ADC sense ---- */

/** Supercap voltage at the pack (divider de-applied), mV. 0 = read failed. */
uint16_t CC_HW_readSupercap_mV(void);

/* ---- RNG ---- */

/** One true-random 32-bit word (RNG clocked up only for the read). */
uint32_t CC_HW_rand32(void);

#endif /* CC_HW_H */
