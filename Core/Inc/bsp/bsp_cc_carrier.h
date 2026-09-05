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

/* ---- Battery monitoring: NiMH pack via gated divider (SMD_STDALONE
 * pattern: PB9/P1 gates the divider, PB13/P0 = ADC_IN0 senses it) ---- */
#define VBAT_EN_Pin             GPIO_PIN_9
#define VBAT_EN_GPIO_Port       GPIOB
#define VBAT_ADC_Pin            GPIO_PIN_13
#define VBAT_ADC_GPIO_Port      GPIOB
/* Divider ratio override for MGR_BAT (values in kOhm; only the ratio matters).
 * CARRIER TODO: set to the routed divider before flight; 1:1 shown here. */
#define MGR_BAT_DIV_R_TOP_KOHM  1000
#define MGR_BAT_DIV_R_BOT_KOHM  1000

/* ---- CC carrier rails and sensors ---- */

/* VCCIO rail switch (AP2112K EN): powers GPS + external I2C. PB5 = P22. */
#define CC_IO_EN_Pin            GPIO_PIN_5
#define CC_IO_EN_GPIO_Port      GPIOB

/* GPS power switch (HIGH = on). PA1 = P3. */
#define CC_GPS_EN_Pin           GPIO_PIN_1
#define CC_GPS_EN_GPIO_Port     GPIOA

/* USART1 session A — GPS @ 9600: PB6 TX (P11) / PB7 RX (P12), AF7. */
#define CC_GPS_TX_Pin           GPIO_PIN_6
#define CC_GPS_TX_GPIO_Port     GPIOB
#define CC_GPS_RX_Pin           GPIO_PIN_7
#define CC_GPS_RX_GPIO_Port     GPIOB

/* USART1 session B — WS85 @ 115200: PA10 RX (P33), AF7. The WS85 lives on
 * the always-on 3V3 rail (rain counter) and broadcasts unprompted; PA9 (P34)
 * is reserved for bench-only config TX and left untouched here. */
#define CC_WS85_RX_Pin          GPIO_PIN_10
#define CC_WS85_RX_GPIO_Port    GPIOA

/* I2C2 — env sensors: PA11 SDA (P28) / PA12 SCL (P27), AF4.
 * Pull-ups to VCCIO on the carrier; firmware parks the pins pulled-down
 * whenever the rail is off (kills the ESD-diode back-feed). */
#define CC_I2C_SDA_Pin          GPIO_PIN_11
#define CC_I2C_SDA_GPIO_Port    GPIOA
#define CC_I2C_SCL_Pin          GPIO_PIN_12
#define CC_I2C_SCL_GPIO_Port    GPIOA

/* Supercap sense: PB4 (P2) = ADC_IN3 via 1M:1M divider + ~100 nF at the pin. */
#define CC_SCAP_ADC_Pin         GPIO_PIN_4
#define CC_SCAP_ADC_GPIO_Port   GPIOB

#endif /* BSP_CC_CARRIER_H */
