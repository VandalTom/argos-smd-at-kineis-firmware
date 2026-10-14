/**
 * @file    cc_hw.c
 * @brief   CC_TRACKER hardware glue for the CounterCurrent carrier.
 *
 * Power choreography ported from the WS85-Tracker Board.h (rail refcounting,
 * bus-clear, pulldown parking); UART mux replaced by USART1 pin re-muxing.
 */

#include "main.h"

#if defined(USE_CC_TRACKER_APP)

#include <stdio.h>
#include <string.h>
#include "stm32wlxx_hal.h"
#include "cc_hw.h"
#include "cc_conf.h"
#include "adc.h"
#include "mgr_log.h"

/*******************************************************************************
 * Power state (single translation unit — plain statics)
 ******************************************************************************/

static bool s_gps_powered;
static bool s_i2c_open;
static uint32_t s_gps_off_tick;            /**< when the receiver was last switched off */
static uint32_t s_gps_baud = CC_GPS_BAUD;  /**< survives CC_HW_init (re-run after every sleep) */

/*******************************************************************************
 * Init: safe pin states
 ******************************************************************************/

static void park_i2c_pins(void)
{
	GPIO_InitTypeDef g = {0};

	/* High-Z (analog): the external pull-ups sit on the always-on 3.3 V
	 * rail and hold the bus idle-high at zero static current. */
	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	g.Pin = CC_I2C_SDA_Pin;
	HAL_GPIO_Init(CC_I2C_SDA_GPIO_Port, &g);
	g.Pin = CC_I2C_SCL_Pin;
	HAL_GPIO_Init(CC_I2C_SCL_GPIO_Port, &g);
}

void CC_HW_init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* GPS switch: output, OFF (the only switched load on this carrier) */
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin = CC_GPS_EN_Pin;
	HAL_GPIO_Init(CC_GPS_EN_GPIO_Port, &g);
	HAL_GPIO_WritePin(CC_GPS_EN_GPIO_Port, CC_GPS_EN_Pin, GPIO_PIN_RESET);

	park_i2c_pins();

	/* Supercap sense pin: analog (its natural low-leak state) */
	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	g.Pin = CC_SCAP_ADC_Pin;
	HAL_GPIO_Init(CC_SCAP_ADC_GPIO_Port, &g);

	/* Sensor UART pins idle analog (WS85 TX line is hot while we sleep —
	 * analog input neither loads it nor back-feeds anything). */
	g.Pin = CC_WS85_RX_Pin;
	HAL_GPIO_Init(CC_WS85_RX_GPIO_Port, &g);
	g.Pin = CC_GPS_NMEA_Pin;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);

	s_gps_powered = false;
	s_gps_off_tick = HAL_GetTick();
	s_i2c_open = false;
}

/*******************************************************************************
 * USART1 sensor sessions (IRQ-driven RX ring)
 ******************************************************************************/

#define CC_UART_RING_SIZE 512U /* power of two; WS85 frame burst = 33 B @ 115200 */

static UART_HandleTypeDef cc_huart1;
static volatile uint8_t  s_ring[CC_UART_RING_SIZE];
static volatile uint16_t s_ring_wr;
static volatile uint16_t s_ring_rd;
static volatile uint16_t s_ring_ovr;
static volatile uint16_t s_line_err;
static cc_uart_session_t s_session = CC_UART_NONE;

#if CC_GPS_LOOPBACK
static cc_loop_tx_t s_loop_tx = CC_LOOP_TX_PB7;

/** USART1 pins that turn TX under CR2.SWAP: PB7 (P12) or PA10 (P33). */
static uint16_t loop_tx_pin(cc_loop_tx_t tx, GPIO_TypeDef **port)
{
	if (tx == CC_LOOP_TX_PA10) {
		*port = CC_WS85_RX_GPIO_Port;
		return CC_WS85_RX_Pin;
	}
	*port = CC_GPS_LOOP_TX_GPIO_Port;
	return CC_GPS_LOOP_TX_Pin;
}
#endif

void USART1_IRQHandler(void)
{
	uint32_t isr = USART1->ISR;

	/* Framing/noise errors: a sender at another baud, or a noisy line */
	if ((isr & (USART_ISR_FE | USART_ISR_NE)) && s_line_err != 0xFFFFU)
		s_line_err++;
	if (isr & USART_ISR_RXNE_RXFNE) {
		uint8_t b = (uint8_t)USART1->RDR;
		uint16_t next = (uint16_t)((s_ring_wr + 1U) & (CC_UART_RING_SIZE - 1U));

		if (next != s_ring_rd) {
			s_ring[s_ring_wr] = b;
			s_ring_wr = next;
		} else {
			s_ring_ovr++;
		}
	}
	/* Clear sticky errors so RX keeps flowing (noise/framing on session swap
	 * is expected — the frame parsers CRC/checksum every byte anyway). */
	USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
}

void CC_HW_uartClose(void)
{
	if (s_session == CC_UART_NONE)
		return;

	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_UART_DeInit(&cc_huart1);
	__HAL_RCC_USART1_CLK_SLEEP_DISABLE();
	__HAL_RCC_USART1_CLK_DISABLE();

	/* Park the session's pins analog (never back-feed an unpowered device) */
	GPIO_InitTypeDef g = {0};

	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	if (s_session != CC_UART_WS85) {
		g.Pin = CC_GPS_NMEA_Pin;
		HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
#if CC_GPS_LOOPBACK
		if (s_session == CC_UART_GPS_LOOP) {
			GPIO_TypeDef *port;

			g.Pin = loop_tx_pin(s_loop_tx, &port);
			HAL_GPIO_Init(port, &g);
		}
#endif
	} else {
		g.Pin = CC_WS85_RX_Pin;
		HAL_GPIO_Init(CC_WS85_RX_GPIO_Port, &g);
	}
	s_session = CC_UART_NONE;
}

bool CC_HW_uartOpen(cc_uart_session_t s)
{
	return CC_HW_uartOpenBaud(s, s == CC_UART_GPS ? s_gps_baud : CC_WS85_BAUD);
}

bool CC_HW_uartOpenBaud(cc_uart_session_t s, uint32_t baud)
{
	if (s == CC_UART_NONE) {
		CC_HW_uartClose();
		return true;
	}
	CC_HW_uartClose();

	GPIO_InitTypeDef g = {0};

	/* Pull-up: an undriven RX line (sensor off, unplugged, mis-wired) then
	 * idles instead of floating into noise that decodes as junk bytes. Both
	 * senders are push-pull and powered while their session is open. */
	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_PULLUP;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF7_USART1;

	if (s != CC_UART_WS85) {
		/* NMEA arrives on PB6 (USART1's TX-capable pad) — CR2.SWAP below
		 * turns it into the receive pin. PB7 is unwired on this carrier. */
		g.Pin = CC_GPS_NMEA_Pin;
		HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
#if CC_GPS_LOOPBACK
		if (s == CC_UART_GPS_LOOP) {
			/* bench loopback: PB7 or PA10 (swapped to TX) drives the bridge */
			GPIO_TypeDef *port;

			g.Pull = GPIO_NOPULL;
			g.Pin = loop_tx_pin(s_loop_tx, &port);
			HAL_GPIO_Init(port, &g);
		}
#endif
	} else {
		/* WS85: RX only — PA9 (bench config TX) is deliberately untouched */
		g.Pin = CC_WS85_RX_Pin;
		HAL_GPIO_Init(CC_WS85_RX_GPIO_Port, &g);
	}

	__HAL_RCC_USART1_CLK_ENABLE();
	/* LPM_Init clears every SLEEP-mode clock enable but LPUART1's, so without
	 * this USART1 stops at each WFI between loop passes and the byte in flight
	 * comes out as a framing error: a 9600 NMEA stream then yields a few
	 * broken bytes a second. Cleared again in CC_HW_uartClose. */
	__HAL_RCC_USART1_CLK_SLEEP_ENABLE();

	memset(&cc_huart1, 0, sizeof(cc_huart1));
	cc_huart1.Instance = USART1;
	cc_huart1.Init.BaudRate = baud;
	cc_huart1.Init.WordLength = UART_WORDLENGTH_8B;
	cc_huart1.Init.StopBits = UART_STOPBITS_1;
	cc_huart1.Init.Parity = UART_PARITY_NONE;
	cc_huart1.Init.Mode = UART_MODE_RX; /* both sensor sessions only listen */
#if CC_GPS_LOOPBACK
	if (s == CC_UART_GPS_LOOP)
		cc_huart1.Init.Mode = UART_MODE_TX_RX;
#endif
	cc_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	cc_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	cc_huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	cc_huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
#if CC_GPS_UART_SWAP
	if (s != CC_UART_WS85) {
		cc_huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
		cc_huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
	}
#endif
	if (HAL_UART_Init(&cc_huart1) != HAL_OK) {
		MGR_LOG_WARN("[CCHW] USART1 init failed (session %d)\r\n", (int)s);
		__HAL_RCC_USART1_CLK_SLEEP_DISABLE();
		__HAL_RCC_USART1_CLK_DISABLE();
		return false;
	}

	s_ring_wr = 0;
	s_ring_rd = 0;
	s_ring_ovr = 0;
	s_line_err = 0;
	s_session = s;

	/* RXNE interrupt only — the ring is drained from the app loop */
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE_RXFNEIE);
	HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	return true;
}

uint16_t CC_HW_uartRead(uint8_t *buf, uint16_t max)
{
	uint16_t n = 0;

	while (n < max && s_ring_rd != s_ring_wr) {
		buf[n++] = s_ring[s_ring_rd];
		s_ring_rd = (uint16_t)((s_ring_rd + 1U) & (CC_UART_RING_SIZE - 1U));
	}
	return n;
}

uint16_t CC_HW_uartOverruns(void)
{
	return s_ring_ovr;
}

uint16_t CC_HW_uartLineErrors(void)
{
	return s_line_err;
}

/*******************************************************************************
 * GPS power
 ******************************************************************************/

void CC_HW_gpsPowerOn(void)
{
	if (s_gps_powered)
		return;
	MGR_LOG_DEBUG("[CCHW] GPS on\r\n");
	HAL_GPIO_WritePin(CC_GPS_EN_GPIO_Port, CC_GPS_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(10); /* receiver power-on settle before opening the UART */
	s_gps_powered = true;
}

void CC_HW_gpsPowerOff(void)
{
	if (!s_gps_powered)
		return;
	MGR_LOG_DEBUG("[CCHW] GPS off\r\n");
	HAL_GPIO_WritePin(CC_GPS_EN_GPIO_Port, CC_GPS_EN_Pin, GPIO_PIN_RESET);
	s_gps_powered = false;
	s_gps_off_tick = HAL_GetTick();
}

uint32_t CC_HW_gpsOffForMs(void)
{
	return s_gps_powered ? 0U : HAL_GetTick() - s_gps_off_tick;
}

uint32_t CC_HW_gpsBaud(void)
{
	return s_gps_baud;
}

void CC_HW_gpsSetBaud(uint32_t baud)
{
	if (baud != 0U && baud != s_gps_baud) {
		MGR_LOG_INFO("[CCHW] GPS baud %lu -> %lu\r\n", (unsigned long)s_gps_baud, (unsigned long)baud);
		s_gps_baud = baud;
	}
}

#define CC_LINE_PROBE_SAMPLES 8U /* per pull, ~1 ms apart */

bool CC_HW_gpsLineProbe(bool *high_when_pulled_down, bool *low_when_pulled_up)
{
	GPIO_InitTypeDef g = {0};
	bool hi = false, lo = false;

	*high_when_pulled_down = false;
	*low_when_pulled_up = false;
	if (s_session == CC_UART_GPS)
		return false; /* the pin belongs to the open session */

	g.Mode = GPIO_MODE_INPUT;
	g.Pin = CC_GPS_NMEA_Pin;
	g.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	for (uint32_t i = 0; i < CC_LINE_PROBE_SAMPLES; i++) {
		HAL_Delay(1);
		if (HAL_GPIO_ReadPin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin) == GPIO_PIN_SET)
			hi = true;
	}
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	for (uint32_t i = 0; i < CC_LINE_PROBE_SAMPLES; i++) {
		HAL_Delay(1);
		if (HAL_GPIO_ReadPin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin) == GPIO_PIN_RESET)
			lo = true;
	}
	/* back to the low-leak parking state */
	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);

	*high_when_pulled_down = hi;
	*low_when_pulled_up = lo;
	return true;
}

#if CC_GPS_LOOPBACK

bool CC_HW_uartWrite(const uint8_t *buf, uint16_t len)
{
	if (s_session != CC_UART_GPS_LOOP)
		return false;
	/* 10 bits a byte at the session baud, doubled, + 10 ms */
	uint32_t ms = (uint32_t)len * 20000UL / cc_huart1.Init.BaudRate + 10UL;

	return HAL_UART_Transmit(&cc_huart1, buf, len, ms) == HAL_OK;
}

bool CC_HW_gpsLoopOpen(cc_loop_tx_t tx, uint32_t baud)
{
	if (tx == CC_LOOP_TX_NONE)
		return false;
	CC_HW_uartClose(); /* parks the previous TX pin before switching */
	s_loop_tx = tx;
	return CC_HW_uartOpenBaud(CC_UART_GPS_LOOP, baud);
}

bool CC_HW_gpsLoopDc(cc_loop_tx_t tx, bool *pb6_low_when_tx_low, bool *pb6_high_when_tx_high)
{
	GPIO_InitTypeDef g = {0};
	GPIO_TypeDef *port;
	uint16_t pin;

	*pb6_low_when_tx_low = false;
	*pb6_high_when_tx_high = false;
	if (tx == CC_LOOP_TX_NONE || s_session != CC_UART_NONE)
		return false; /* nothing to drive, or the pins belong to the open session */
	pin = loop_tx_pin(tx, &port);

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin = pin;
	HAL_GPIO_Init(port, &g);

	/* TX pin low against PB6's pull-up */
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	g.Pin = CC_GPS_NMEA_Pin;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	HAL_Delay(2);
	*pb6_low_when_tx_low =
		HAL_GPIO_ReadPin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin) == GPIO_PIN_RESET;

	/* TX pin high against PB6's pull-down */
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	g.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	HAL_Delay(2);
	*pb6_high_when_tx_high =
		HAL_GPIO_ReadPin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin) == GPIO_PIN_SET;

	/* back to the low-leak parking state */
	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	g.Pin = pin;
	HAL_GPIO_Init(port, &g);
	return true;
}

bool CC_HW_gpsLoopFind(char *out, size_t n, cc_loop_tx_t *tx)
{
	static const struct {
		GPIO_TypeDef *port;
		char name;
		uint16_t skip;
	} ports[] = {
		{ GPIOA, 'A', 0U },
		{ GPIOB, 'B', CC_GPS_NMEA_Pin },
		{ GPIOC, 'C', GPIO_PIN_14 | GPIO_PIN_15 }, /* LSE crystal */
	};
	enum { NP = sizeof(ports) / sizeof(ports[0]) };
	uint16_t cand[NP], low[NP], high[NP];
	GPIO_InitTypeDef g = {0};
	size_t used = 0;

	out[0] = '\0';
	if (s_session != CC_UART_NONE)
		return false;
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* candidates: every pin parked analog (MODER = 11), i.e. unused */
	for (unsigned i = 0; i < NP; i++) {
		uint32_t moder = ports[i].port->MODER;

		cand[i] = 0;
		for (unsigned p = 0; p < 16U; p++)
			if (((moder >> (2U * p)) & 3U) == 3U && !(ports[i].skip & (1U << p)))
				cand[i] |= (uint16_t)(1U << p);
	}

	/* PB6 low, candidates pulled up */
	HAL_GPIO_WritePin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin, GPIO_PIN_RESET);
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Pin = CC_GPS_NMEA_Pin;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	for (unsigned i = 0; i < NP; i++) {
		if (cand[i]) {
			g.Pin = cand[i];
			HAL_GPIO_Init(ports[i].port, &g);
		}
	}
	HAL_Delay(2);
	for (unsigned i = 0; i < NP; i++)
		low[i] = (uint16_t)(~ports[i].port->IDR & cand[i]);

	/* PB6 high, candidates pulled down */
	HAL_GPIO_WritePin(CC_GPS_NMEA_GPIO_Port, CC_GPS_NMEA_Pin, GPIO_PIN_SET);
	g.Pull = GPIO_PULLDOWN;
	for (unsigned i = 0; i < NP; i++) {
		if (cand[i]) {
			g.Pin = cand[i];
			HAL_GPIO_Init(ports[i].port, &g);
		}
	}
	HAL_Delay(2);
	for (unsigned i = 0; i < NP; i++)
		high[i] = (uint16_t)(ports[i].port->IDR & cand[i]);

	/* everything back to analog */
	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	for (unsigned i = 0; i < NP; i++) {
		if (cand[i]) {
			g.Pin = cand[i];
			HAL_GPIO_Init(ports[i].port, &g);
		}
	}
	g.Pin = CC_GPS_NMEA_Pin;
	HAL_GPIO_Init(CC_GPS_NMEA_GPIO_Port, &g);

	for (unsigned i = 0; i < NP; i++)
		for (unsigned p = 0; p < 16U; p++)
			if ((low[i] & high[i] & (1U << p)) && used + 6U < n)
				used += (size_t)snprintf(out + used, n - used, "P%c%u ", ports[i].name, p);
	if (used == 0U)
		snprintf(out, n, "none");

	/* a USART1 pin that turns TX under CR2.SWAP: PB7 (P12), else PA10 (P33) */
	if (low[1] & high[1] & CC_GPS_LOOP_TX_Pin)
		*tx = CC_LOOP_TX_PB7;
	else if (low[0] & high[0] & CC_WS85_RX_Pin)
		*tx = CC_LOOP_TX_PA10;
	return true;
}

#endif /* CC_GPS_LOOPBACK */

/*******************************************************************************
 * I2C2 session
 ******************************************************************************/

static I2C_HandleTypeDef cc_hi2c2;

/* Standard bus-clear: the I2C peripheral can't start on a bus a slave holds
 * low. Emulate open-drain with output-low / input-pullup, clock out up to 9
 * bits, then issue a STOP. Ported from Board::i2cBusIdle(). */
static bool i2c_bus_idle(void)
{
	GPIO_InitTypeDef g = {0};

	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	g.Pin = CC_I2C_SDA_Pin;
	HAL_GPIO_Init(CC_I2C_SDA_GPIO_Port, &g);
	g.Pin = CC_I2C_SCL_Pin;
	HAL_GPIO_Init(CC_I2C_SCL_GPIO_Port, &g);

	/* ~10 us settle */
	for (volatile int i = 0; i < 100; i++)
		;

	if (HAL_GPIO_ReadPin(CC_I2C_SDA_GPIO_Port, CC_I2C_SDA_Pin) == GPIO_PIN_SET &&
	    HAL_GPIO_ReadPin(CC_I2C_SCL_GPIO_Port, CC_I2C_SCL_Pin) == GPIO_PIN_SET)
		return true;

	/* SCL held low is slave clock-stretch: only waiting helps. */
	uint32_t t0 = HAL_GetTick();

	while (HAL_GPIO_ReadPin(CC_I2C_SCL_GPIO_Port, CC_I2C_SCL_Pin) == GPIO_PIN_RESET) {
		if (HAL_GetTick() - t0 > 20)
			return false;
		HAL_Delay(1);
	}

	/* SDA held low: a slave stuck mid-byte. Clock until it releases (<= 9). */
	for (int i = 0;
	     i < 9 && HAL_GPIO_ReadPin(CC_I2C_SDA_GPIO_Port, CC_I2C_SDA_Pin) == GPIO_PIN_RESET;
	     i++) {
		g.Mode = GPIO_MODE_OUTPUT_OD;
		g.Pull = GPIO_NOPULL;
		g.Pin = CC_I2C_SCL_Pin;
		HAL_GPIO_Init(CC_I2C_SCL_GPIO_Port, &g);
		HAL_GPIO_WritePin(CC_I2C_SCL_GPIO_Port, CC_I2C_SCL_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		g.Mode = GPIO_MODE_INPUT;
		g.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(CC_I2C_SCL_GPIO_Port, &g);
		HAL_Delay(1);
	}

	/* STOP condition: SDA low -> high while SCL high. */
	g.Mode = GPIO_MODE_OUTPUT_OD;
	g.Pull = GPIO_NOPULL;
	g.Pin = CC_I2C_SDA_Pin;
	HAL_GPIO_Init(CC_I2C_SDA_GPIO_Port, &g);
	HAL_GPIO_WritePin(CC_I2C_SDA_GPIO_Port, CC_I2C_SDA_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CC_I2C_SDA_GPIO_Port, &g);
	HAL_Delay(1);

	return HAL_GPIO_ReadPin(CC_I2C_SDA_GPIO_Port, CC_I2C_SDA_Pin) == GPIO_PIN_SET &&
	       HAL_GPIO_ReadPin(CC_I2C_SCL_GPIO_Port, CC_I2C_SCL_Pin) == GPIO_PIN_SET;
}

bool CC_HW_i2cSessionOpen(void)
{
	if (!i2c_bus_idle()) {
		MGR_LOG_WARN("[CCHW] I2C bus stuck low — skipping env sample\r\n");
		park_i2c_pins();
		return false;
	}
	s_i2c_open = true;

	GPIO_InitTypeDef g = {0};

	g.Mode = GPIO_MODE_AF_OD;
	g.Pull = GPIO_NOPULL; /* external pull-ups to VCCIO on the carrier */
	g.Speed = GPIO_SPEED_FREQ_LOW;
	g.Alternate = GPIO_AF4_I2C2;
	g.Pin = CC_I2C_SDA_Pin;
	HAL_GPIO_Init(CC_I2C_SDA_GPIO_Port, &g);
	g.Pin = CC_I2C_SCL_Pin;
	HAL_GPIO_Init(CC_I2C_SCL_GPIO_Port, &g);

	__HAL_RCC_I2C2_CLK_ENABLE();

	memset(&cc_hi2c2, 0, sizeof(cc_hi2c2));
	cc_hi2c2.Instance = I2C2;
	/* 100 kHz from a 48 MHz kernel clock (CubeMX-computed standard-mode
	 * timing: PRESC=0xB, SCLDEL/SDADEL=0x4/0x2, SCLH/SCLL=0x0F/0x13). */
	cc_hi2c2.Init.Timing = 0xB0420F13U;
	cc_hi2c2.Init.OwnAddress1 = 0;
	cc_hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	cc_hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	cc_hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	cc_hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&cc_hi2c2) != HAL_OK) {
		MGR_LOG_WARN("[CCHW] I2C2 init failed\r\n");
		__HAL_RCC_I2C2_CLK_DISABLE();
		park_i2c_pins();
		s_i2c_open = false;
		return false;
	}
	return true;
}

void CC_HW_i2cSessionClose(void)
{
	if (!s_i2c_open)
		return;
	HAL_I2C_DeInit(&cc_hi2c2);
	__HAL_RCC_I2C2_CLK_DISABLE();
	park_i2c_pins();
	s_i2c_open = false;
}

/* --- cc_i2c_ops_t implementation over HAL (polling, 100 ms guard) --- */

#define CC_I2C_XFER_TIMEOUT_MS 100

static bool i2c_ops_write(void *ctx, uint8_t addr7, const uint8_t *data, uint16_t len)
{
	(void)ctx;
	return HAL_I2C_Master_Transmit(&cc_hi2c2, (uint16_t)(addr7 << 1),
		(uint8_t *)data, len, CC_I2C_XFER_TIMEOUT_MS) == HAL_OK;
}

static bool i2c_ops_read(void *ctx, uint8_t addr7, uint8_t *data, uint16_t len)
{
	(void)ctx;
	return HAL_I2C_Master_Receive(&cc_hi2c2, (uint16_t)(addr7 << 1),
		data, len, CC_I2C_XFER_TIMEOUT_MS) == HAL_OK;
}

static void i2c_ops_delay(void *ctx, uint32_t ms)
{
	(void)ctx;
	HAL_Delay(ms);
}

static uint32_t i2c_ops_tick(void *ctx)
{
	(void)ctx;
	return HAL_GetTick();
}

static const cc_i2c_ops_t s_i2c_ops = {
	.write = i2c_ops_write,
	.read = i2c_ops_read,
	.delay_ms = i2c_ops_delay,
	.tick_ms = i2c_ops_tick,
	.ctx = NULL,
};

const cc_i2c_ops_t *CC_HW_i2cOps(void)
{
	return &s_i2c_ops;
}

/*******************************************************************************
 * ADC: supercap sense (VREFINT-calibrated, MGR_BAT-style)
 ******************************************************************************/

#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR ((uint16_t *)0x1FFF75AAUL)
#endif
#define CC_VREFINT_CAL_MV 3300U

static bool adc_read_raw(uint32_t channel, uint32_t *raw)
{
	ADC_ChannelConfTypeDef c = {0};

	c.Channel = channel;
	c.Rank = ADC_REGULAR_RANK_1;
	c.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc, &c) != HAL_OK)
		return false;

	bool ok = false;

	if (HAL_ADC_Start(&hadc) == HAL_OK) {
		if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
			*raw = HAL_ADC_GetValue(&hadc);
			ok = true;
		}
		HAL_ADC_Stop(&hadc);
	}
	return ok;
}

uint16_t CC_HW_readSupercap_mV(void)
{
	uint32_t raw_vref = 0, raw_scap = 0;

	/* VREFINT first for true-VDDA math (factory calibrated at 3300 mV) */
	if (!adc_read_raw(ADC_CHANNEL_VREFINT, &raw_vref) ||
	    !adc_read_raw(ADC_CHANNEL_3, &raw_scap) ||
	    raw_vref == 0)
		return 0;

	uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
	uint32_t vdda_mv = (uint32_t)CC_VREFINT_CAL_MV * vrefint_cal / raw_vref;

	/* pin mV = raw * VDDA / 4095, then de-divide (1M:1M -> x2) */
	uint32_t scap_mv = (uint32_t)((uint64_t)raw_scap * vdda_mv * CC_SCAP_DIV_NUM /
		(4095ULL * CC_SCAP_DIV_DEN));

	return (scap_mv > 0xFFFF) ? 0xFFFF : (uint16_t)scap_mv;
}

/*******************************************************************************
 * RNG
 ******************************************************************************/

uint32_t CC_HW_rand32(void)
{
	RNG_HandleTypeDef hrng = {0};
	uint32_t v = 0;

	/* RNG kernel clock: RNGSEL resets to PLLQ, an output this firmware never
	 * enables (and at PLLQ/2 = 64 MHz it would exceed the RNG's 48 MHz).
	 * MSI (4 MHz, on from reset and the STOP2 wake clock) is in range and
	 * above HCLK/32 as clock-error detection requires. */
	if (!__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY)) {
		uint32_t t0 = HAL_GetTick();

		__HAL_RCC_MSI_ENABLE();
		while (!__HAL_RCC_GET_FLAG(RCC_FLAG_MSIRDY) && HAL_GetTick() - t0 < 3u)
			;
	}
	__HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_MSI);
	__HAL_RCC_RNG_CLK_ENABLE();
	hrng.Instance = RNG;
	hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
	if (HAL_RNG_Init(&hrng) == HAL_OK) {
		if (HAL_RNG_GenerateRandomNumber(&hrng, &v) != HAL_OK)
			v = HAL_GetTick() * 2654435761U; /* degraded fallback */
		HAL_RNG_DeInit(&hrng);
	} else {
		v = HAL_GetTick() * 2654435761U;
	}
	__HAL_RCC_RNG_CLK_DISABLE();
	return v;
}

#endif /* USE_CC_TRACKER_APP */
