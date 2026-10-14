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
#include <stddef.h>
#include "cc_i2c_sens.h"
#include "cc_conf.h"

typedef enum {
	CC_UART_NONE = 0,
	CC_UART_GPS,   /**< session A: PB6 (swapped to RX), CC_HW_gpsBaud() */
	CC_UART_WS85,  /**< session B: PA10 RX, CC_HW_ws85Baud() */
#if CC_GPS_LOOPBACK
	CC_UART_GPS_LOOP, /**< bench: session A plus PB7 (swapped to TX) */
#endif
} cc_uart_session_t;

/** One-time init: drive every rail/control pin to its safe state
 *  (VCCIO off, GPS off, I2C parked pulled-down, sense pins analog). */
void CC_HW_init(void);

/* ---- USART1 sensor sessions ---- */

/** Open a sensor UART session at its baud (closes any previous one). */
bool CC_HW_uartOpen(cc_uart_session_t s);
/** Same, at an explicit baud (the self-test's GPS and WS85 baud scans). */
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
/** The WS85 session baud: CC_WS85_BAUD until the self-test finds another
 *  (a factory-reset or freshly flashed WS85 talks at 9600). */
uint32_t CC_HW_ws85Baud(void);
void CC_HW_ws85SetBaud(uint32_t baud);
/**
 * Probe the GPS data line (PB6) with the MCU's own pulls, ~16 ms: is it
 * high while pulled down, low while pulled up? (cc_selftest_line_class).
 * Not while the GPS session is open (returns false). Leaves PB6 parked.
 */
bool CC_HW_gpsLineProbe(bool *high_when_pulled_down, bool *low_when_pulled_up);

#if CC_GPS_LOOPBACK
/* ---- GPS UART loopback (bench, PB6/P11 bridged to a USART1 TX pin) ---- */

/** The pins CR2.SWAP turns into USART1 TX, either of which can feed PB6. */
typedef enum {
	CC_LOOP_TX_NONE = 0,
	CC_LOOP_TX_PB7,   /**< P12 */
	CC_LOOP_TX_PA10,  /**< P33, the WS85 data pad (WS85 unplugged) */
} cc_loop_tx_t;

/** Open the CC_UART_GPS_LOOP session: RX on PB6, TX on @p tx. */
bool CC_HW_gpsLoopOpen(cc_loop_tx_t tx, uint32_t baud);
/** Send @p len bytes on the CC_UART_GPS_LOOP session (blocking). */
bool CC_HW_uartWrite(const uint8_t *buf, uint16_t len);
/**
 * DC continuity through the bridge, ~4 ms: @p tx driven low with PB6 pulled
 * up, then high with PB6 pulled down. Not while a session is open, nor for
 * CC_LOOP_TX_NONE (returns false). Leaves both pins parked.
 */
bool CC_HW_gpsLoopDc(cc_loop_tx_t tx, bool *pb6_low_when_tx_low, bool *pb6_high_when_tx_high);
/**
 * Which MCU pins are wired to P11: PB6 driven low then high while every pin
 * parked analog (bar PB6 and the LSE crystal pins) is read against the
 * opposite pull. Writes their names to @p out ("PB7 ", or "none") and the
 * TX pin to loop through to @p tx. Only call when nothing else drives P11
 * (idle "fl"). Leaves every pin as found.
 */
bool CC_HW_gpsLoopFind(char *out, size_t n, cc_loop_tx_t *tx);
#endif

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
