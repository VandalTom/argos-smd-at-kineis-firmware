/**
 * @file    cc_hw.c
 * @brief   CC_TRACKER hardware glue for the CounterCurrent carrier.
 *
 * Power choreography ported from the WS85-Tracker Board.h (rail refcounting,
 * bus-clear, pulldown parking); UART mux replaced by USART1 pin re-muxing.
 */

#include "main.h"

#if defined(USE_CC_TRACKER_APP)

#include <string.h>
#include "stm32wlxx_hal.h"
#include "cc_hw.h"
#include "cc_conf.h"
#include "adc.h"
#include "mgr_log.h"

/*******************************************************************************
 * Rail state (single translation unit — plain statics)
 ******************************************************************************/

static bool s_gps_powered;
static bool s_i2c_open;

/* VCCIO settle: rail + attached chips' power-on reset (a chip probed mid-boot
 * can jam the I2C bus). Same 10 ms as the Arduino original. */
#define CC_VCCIO_SETTLE_MS 10

static void io_enable(void)
{
	HAL_GPIO_WritePin(CC_IO_EN_GPIO_Port, CC_IO_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(CC_VCCIO_SETTLE_MS);
}

static void io_maybe_disable(void)
{
	if (!s_gps_powered && !s_i2c_open)
		HAL_GPIO_WritePin(CC_IO_EN_GPIO_Port, CC_IO_EN_Pin, GPIO_PIN_RESET);
}

/*******************************************************************************
 * Init: safe pin states
 ******************************************************************************/

static void park_i2c_pins(void)
{
	GPIO_InitTypeDef g = {0};

	/* Pulled DOWN, not hi-Z: bleeds the dead VCCIO rail through the sensor
	 * pull-ups instead of floating mid-rail (see header). */
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLDOWN;
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

	/* Rails: outputs, OFF */
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;

	g.Pin = CC_IO_EN_Pin;
	HAL_GPIO_Init(CC_IO_EN_GPIO_Port, &g);
	HAL_GPIO_WritePin(CC_IO_EN_GPIO_Port, CC_IO_EN_Pin, GPIO_PIN_RESET);

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
	g.Pin = CC_GPS_RX_Pin;
	HAL_GPIO_Init(CC_GPS_RX_GPIO_Port, &g);
	g.Pin = CC_GPS_TX_Pin;
	HAL_GPIO_Init(CC_GPS_TX_GPIO_Port, &g);

	s_gps_powered = false;
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
static cc_uart_session_t s_session = CC_UART_NONE;

void USART1_IRQHandler(void)
{
	uint32_t isr = USART1->ISR;

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
	__HAL_RCC_USART1_CLK_DISABLE();

	/* Park the session's pins analog (never back-feed an unpowered device) */
	GPIO_InitTypeDef g = {0};

	g.Mode = GPIO_MODE_ANALOG;
	g.Pull = GPIO_NOPULL;
	if (s_session == CC_UART_GPS) {
		g.Pin = CC_GPS_TX_Pin;
		HAL_GPIO_Init(CC_GPS_TX_GPIO_Port, &g);
		g.Pin = CC_GPS_RX_Pin;
		HAL_GPIO_Init(CC_GPS_RX_GPIO_Port, &g);
	} else {
		g.Pin = CC_WS85_RX_Pin;
		HAL_GPIO_Init(CC_WS85_RX_GPIO_Port, &g);
	}
	s_session = CC_UART_NONE;
}

bool CC_HW_uartOpen(cc_uart_session_t s)
{
	if (s == CC_UART_NONE) {
		CC_HW_uartClose();
		return true;
	}
	CC_HW_uartClose();

	GPIO_InitTypeDef g = {0};

	g.Mode = GPIO_MODE_AF_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	g.Alternate = GPIO_AF7_USART1;

	uint32_t baud;

	if (s == CC_UART_GPS) {
		g.Pin = CC_GPS_TX_Pin;
		HAL_GPIO_Init(CC_GPS_TX_GPIO_Port, &g);
		g.Pin = CC_GPS_RX_Pin;
		HAL_GPIO_Init(CC_GPS_RX_GPIO_Port, &g);
		baud = CC_GPS_BAUD;
	} else {
		/* WS85: RX only — PA9 (bench config TX) is deliberately untouched */
		g.Pin = CC_WS85_RX_Pin;
		HAL_GPIO_Init(CC_WS85_RX_GPIO_Port, &g);
		baud = CC_WS85_BAUD;
	}

	__HAL_RCC_USART1_CLK_ENABLE();

	memset(&cc_huart1, 0, sizeof(cc_huart1));
	cc_huart1.Instance = USART1;
	cc_huart1.Init.BaudRate = baud;
	cc_huart1.Init.WordLength = UART_WORDLENGTH_8B;
	cc_huart1.Init.StopBits = UART_STOPBITS_1;
	cc_huart1.Init.Parity = UART_PARITY_NONE;
	cc_huart1.Init.Mode = (s == CC_UART_GPS) ? UART_MODE_TX_RX : UART_MODE_RX;
	cc_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	cc_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	cc_huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	cc_huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	if (HAL_UART_Init(&cc_huart1) != HAL_OK) {
		MGR_LOG_WARN("[CCHW] USART1 init failed (session %d)\r\n", (int)s);
		__HAL_RCC_USART1_CLK_DISABLE();
		return false;
	}

	s_ring_wr = 0;
	s_ring_rd = 0;
	s_ring_ovr = 0;
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

/*******************************************************************************
 * GPS power
 ******************************************************************************/

void CC_HW_gpsPowerOn(void)
{
	if (s_gps_powered)
		return;
	MGR_LOG_DEBUG("[CCHW] GPS on\r\n");
	io_enable();
	HAL_GPIO_WritePin(CC_GPS_EN_GPIO_Port, CC_GPS_EN_Pin, GPIO_PIN_SET);
	s_gps_powered = true;
}

void CC_HW_gpsPowerOff(void)
{
	if (!s_gps_powered)
		return;
	MGR_LOG_DEBUG("[CCHW] GPS off\r\n");
	HAL_GPIO_WritePin(CC_GPS_EN_GPIO_Port, CC_GPS_EN_Pin, GPIO_PIN_RESET);
	s_gps_powered = false;
	io_maybe_disable();
}

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
	s_i2c_open = true; /* holds the rail through the whole session */
	io_enable();

	if (!i2c_bus_idle()) {
		MGR_LOG_WARN("[CCHW] I2C bus stuck low — skipping env sample\r\n");
		park_i2c_pins();
		s_i2c_open = false;
		io_maybe_disable(); /* full rail power-cycle = best unstick */
		return false;
	}

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
		io_maybe_disable();
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
	io_maybe_disable();
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
