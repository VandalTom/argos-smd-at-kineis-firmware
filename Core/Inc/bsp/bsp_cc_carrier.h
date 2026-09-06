/**
 * @file    bsp_cc_carrier.h
 * @brief   BSP pin definitions for the CounterCurrent carrier (CC_CARRIER)
 *
 * The argos-smd module on the CounterCurrent weather-tracker carrier: the
 * STM32WL55 drives every sensor itself (no host MCU). Assignments follow the
 * "CounterCurrent Carrier Pinout" plan, confirmed against the module's
 * definitive pad mapping (P-numbers in comments).
 *
 * Features: External PA (module's own), VBAT ADC (NiMH via gated divider).
 * Comm: UART only (LPUART1 console for AT diagnostics).
 *
 * USART1 is TIME-SHARED between two sensors that never run simultaneously:
 *   session A: GPS  on PB6 (TX, P11) / PB7 (RX, P12) @ 9600
 *   session B: WS85 on PA10 (RX, P33) @ 115200 (PA9/P34 reserved for bench TX)
 */

#ifndef BSP_CC_CARRIER_H
#define BSP_CC_CARRIER_H

/* ---- Board capabilities ----
 * NOTE: LED/reed/latch capabilities are tested with #if defined(...) across
 * the codebase — a board WITHOUT the hardware must leave them UNDEFINED
 * (defining them 0 still links the LED/reed managers in). */
#define BSP_BOARD_NAME          "CC_CARRIER"
#define BSP_HAS_EXTERNAL_PA     1
#define BSP_HAS_VBAT_ADC        1   /* NiMH pack via gated divider (MGR_BAT) */
/* Krd v11.1.0 features — value-tested, 0 = absent */
#define BSP_HAS_DOWNLINK        0
#define BSP_HAS_GNSS            0   /* no Kineis-stack GNSS; the app owns the GPS */
#define BSP_HAS_SATDET          0

/* ---- Debug pins (P26/P25 SWD, P23 SWO) ---- */
#define JTMS_SWCLK_Pin          GPIO_PIN_14
#define JTMS_SWCLK_GPIO_Port    GPIOA
#define JTMS_SWDIO_Pin          GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port    GPIOA
#define SWO_Pin                 GPIO_PIN_3
#define SWO_GPIO_Port           GPIOB

/* ---- External PA control (module-internal, P9/P10) ---- */
#define PA_PSU_EN_Pin           GPIO_PIN_0   /* PC0: VDD_EXTPA_EN */
#define PA_PSU_EN_GPIO_Port     GPIOC
#define PA_PSU_SEL_Pin          GPIO_PIN_1   /* PC1: PGOOD or SEL */
#define PA_PSU_SEL_GPIO_Port    GPIOC

/* ---- Wakeup button alias (GUI-app compatibility; PB3 = WKUP3 spare) ---- */
#define EXT_WKUP_BUTTON_Pin         GPIO_PIN_3
#define EXT_WKUP_BUTTON_GPIO_Port   GPIOB

/* ---- Battery monitoring: NiMH pack sensed DIRECTLY on PB13/P0 = ADC_IN0
 * through a 10 k series resistor (R6 on the carrier netlist) — no divider
 * (2S NiMH tops out ~2.9 V, inside VDDA) and no gate. PB9/VBAT_EN is NOT
 * wired on this carrier: MGR_BAT still toggles it, harmlessly, into a
 * floating pad kept here so the shared code compiles unchanged. ---- */
#define VBAT_EN_Pin             GPIO_PIN_9
#define VBAT_EN_GPIO_Port       GPIOB
#define VBAT_ADC_Pin            GPIO_PIN_13
#define VBAT_ADC_GPIO_Port      GPIOB
/* Direct sense: ratio (TOP+BOT)/BOT must equal 1 -> TOP = 0. */
#define MGR_BAT_DIV_R_TOP_KOHM  0
#define MGR_BAT_DIV_R_BOT_KOHM  100

/* ---- CC carrier rails and sensors ----
 * Power architecture (rev-next): one always-on 3.3 V buck (TPS62842) feeds
 * the module (VDD/VBAT/VDD_EXTPA), the BME280, and the I2C pull-ups — no
 * switched VCCIO rail (AP2112K deleted). Only the GPS is hard-switched,
 * via its own enable. PB5 (P22) is a spare GPIO. */

/* GPS power switch (HIGH = on). PA1 = P3. */
#define CC_GPS_EN_Pin           GPIO_PIN_1
#define CC_GPS_EN_GPIO_Port     GPIOA

/* USART1 session A — GPS @ 9600, RX-ONLY as routed: the receiver's NMEA
 * output lands on PB6 (P11), which is USART1's TX-capable pad, so the GPS
 * session runs with CR2.SWAP enabled (CC_GPS_UART_SWAP) making PB6 the
 * receive pin. PB7 (P12) is not wired on this carrier. */
#define CC_GPS_NMEA_Pin         GPIO_PIN_6
#define CC_GPS_NMEA_GPIO_Port   GPIOB

/* USART1 session B — WS85 @ 115200: PA10 RX (P33), AF7. The WS85 lives on
 * the always-on 3V3 rail (rain counter) and broadcasts unprompted; PA9 (P34)
 * is reserved for bench-only config TX and left untouched here. */
#define CC_WS85_RX_Pin          GPIO_PIN_10
#define CC_WS85_RX_GPIO_Port    GPIOA

/* I2C2 — BME280: PA11 SDA (P28) / PA12 SCL (P27), AF4.
 * Pull-ups to the always-on 3.3 V rail; between samples the firmware parks
 * the pins high-Z (analog) and the pull-ups hold the bus idle-high at zero
 * static current. */
#define CC_I2C_SDA_Pin          GPIO_PIN_11
#define CC_I2C_SDA_GPIO_Port    GPIOA
#define CC_I2C_SCL_Pin          GPIO_PIN_12
#define CC_I2C_SCL_GPIO_Port    GPIOA

/* Supercap sense: PB4 (P2) = ADC_IN3 via 1M:1M divider + ~100 nF at the pin. */
#define CC_SCAP_ADC_Pin         GPIO_PIN_4
#define CC_SCAP_ADC_GPIO_Port   GPIOB

#endif /* BSP_CC_CARRIER_H */
