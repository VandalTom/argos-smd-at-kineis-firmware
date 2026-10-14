/**
 * @file    cc_sleep.c
 * @brief   CC_TRACKER STOP2-until-deadline (see cc_sleep.h).
 *
 * Sequence mirrors the proven UW_DOPPLER path (mgr_lpm_uw.c
 * MGR_LPM_UW_enterStop2TimedMs) minus the SWS/reed/LED handling.
 */

#include "main.h"

#if defined(USE_CC_TRACKER_APP)

#include "stm32wlxx_hal.h"
#include "rtc.h"
#include "subghz.h"
#include "adc.h"
#include "lpm.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "mgr_at_cmd.h"
#include "mgr_err.h"
#include "mgr_log.h"
#include "mcu_misc.h"
#include "cc_conf.h"
#include "cc_hw.h"
#include "cc_rxwake.h"
#include "cc_sleep.h"

extern void MX_GPIO_Init(void);

static volatile bool s_rx_edge;
static uint32_t s_awake_until;     /**< 0 = no hold-off */
static uint32_t s_total_ms;
static uint32_t s_count;
static cc_rxwake_t s_rxwake;   /**< zero-init = armed, no history */

/* PA3 falling edge = console RX while asleep. Armed only during STOP2
 * (overrides the weak default; MGR_REED, the other EXTI3 user, is
 * SMD_STDALONE-only). */
void EXTI3_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
	s_rx_edge = true;
}

void CC_SLEEP_holdAwake(uint32_t ms)
{
	uint32_t until = HAL_GetTick() + ms;

	s_awake_until = until ? until : 1u;
}

static bool console_awake(void)
{
	if (s_awake_until == 0u)
		return false;
	if ((int32_t)(s_awake_until - HAL_GetTick()) > 0)
		return true;
	s_awake_until = 0u;
	return false;
}

void CC_SLEEP_consoleLine(void)
{
	cc_rxwake_on_line(&s_rxwake);
	CC_SLEEP_holdAwake(CC_CONSOLE_AWAKE_MS);
}

#if CC_POWER_STEPS
static bool s_bench_pa_en;

void CC_SLEEP_benchPaEn(bool on)
{
	s_bench_pa_en = on;
}
#endif

uint32_t CC_SLEEP_totalMs(void)
{
	return s_total_ms;
}

uint32_t CC_SLEEP_count(void)
{
	return s_count;
}

/* lpm.c's restore is static; STOP2 exits on MSI — back to HSI + PLL so the
 * console baud and every peripheral clock match the rest of the run. */
static void restore_clock(void)
{
	RCC_OscInitTypeDef osc = {0};
	RCC_ClkInitTypeDef clk = {0};
	uint32_t flat = 0;

	HAL_PWR_EnableBkUpAccess();
	HAL_RCC_GetOscConfig(&osc);
	osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
	osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	osc.HSIState            = RCC_HSI_ON;
	osc.PLL.PLLState        = RCC_PLL_ON;
	osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
	(void)HAL_RCC_OscConfig(&osc);

	HAL_RCC_GetClockConfig(&clk, &flat);
	clk.ClockType    = RCC_CLOCKTYPE_SYSCLK;
	clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	(void)HAL_RCC_ClockConfig(&clk, flat);
}

/** RTC liveness. RSF only clears with the RTC write protection off (HAL
 *  note on HAL_RTC_WaitForSynchro; RTC_ICSR is write-protected): with it on,
 *  the clear is ignored, RSF stays set and a dead clock passes. Unlocked,
 *  RSF re-sets within ~2 RTCCLK cycles on a live clock; a dead one times out. */
static bool rtc_ticking(void)
{
	HAL_StatusTypeDef st;

	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	st = HAL_RTC_WaitForSynchro(&hrtc);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	return st == HAL_OK;
}

static bool arm_wakeup_timer(uint32_t ms)
{
	HAL_StatusTypeDef st;

	/* A dead LSE leaves the RTC frozen: STOP2 would never wake (and the
	 * IWDG is frozen too). Same liveness gate + LSI fallback as UW. */
	if (!rtc_ticking()) {
		SystemClock_armLsiFallback();
		MGR_LOG_ERR("[SLP] RTC not ticking (LSE dead?) - reset to LSI fallback\r\n");
		MGR_ERR_logAndReset(ERR_RTC_DEAD);
	}
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
	if (ms < 29000u) {
		uint32_t ticks = (ms * 2048u) / 1000u; /* RTCCLK/16 = 2048 Hz */

		if (ticks == 0u)
			ticks = 1u;
		st = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (uint16_t)(ticks - 1u),
			RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);
	} else {
		st = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (uint16_t)(ms / 1000u - 1u),
			RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
	}
	if (st != HAL_OK)
		return false;

	/* WUTF reaches the core through the PWR internal wake-up line; a stale
	 * WUFI makes STOP2 "work once" */
	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
	HAL_PWREx_EnableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	return true;
}

bool CC_SLEEP_stop2Until(uint32_t deadline)
{
	int32_t d = (int32_t)(deadline - HAL_GetTick());

	if (d < (int32_t)CC_STOP2_MIN_MS)
		return false;
	if (console_awake() || MGR_AT_CMD_isPendingAt() || KNS_Q_isEvtInSomeQ())
		return false;
	{
		union KNS_MAC_rsrcStatus_t r = KNS_MAC_getRsrcStatus();

		if (r.modemOn || r.txTimeout || r.rxTimeout || r.satDetTimeout)
			return false;
	}
#if defined(DEBUG)
	/* Under a debugger STOP2 becomes a wake storm */
	if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
		return false;
#endif

	uint32_t ms = (uint32_t)d;

	if (ms > CC_STOP2_MAX_MS)
		ms = CC_STOP2_MAX_MS;
	MGR_LOG_flush_all();
	if (!arm_wakeup_timer(ms))
		return false;

	/* Peripherals down, every pin at its lowest-leak state; CC_HW_init
	 * re-drives the GPS enable LOW (the analog teardown would float it) */
	MCU_MISC_turn_off_pa(); /* PA supply enable driven LOW, never floating */
	MX_ADC_DeInit();
	GPIO_DisableAllToAnalogInput();
	CC_HW_init();
	(void)HAL_SUBGHZ_DeInit(&hsubghz);
#if CC_POWER_STEPS
	if (s_bench_pa_en)
		HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_SET);
#endif

	bool rx_wake_armed = cc_rxwake_armed(&s_rxwake, HAL_GetTick());

	{
		/* Console RX held high either way; the falling-edge wake only
		 * while the storm guard allows it (cc_rxwake.h) */
		GPIO_InitTypeDef rx_wake = {0};

		rx_wake.Pin = GPIO_PIN_3;
		rx_wake.Mode = rx_wake_armed ? GPIO_MODE_IT_FALLING : GPIO_MODE_INPUT;
		rx_wake.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &rx_wake);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		s_rx_edge = false;
		if (rx_wake_armed) {
			HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		}
	}

	/* CPU2 must not cap the system low-power mode */
	MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, PWR_LOWPOWERMODE_SHUTDOWN);

	__disable_irq();
	/* s_rx_edge: an edge between arming and here already ran the ISR
	 * (pending bit cleared), so WFI would not see it */
	bool skip = MGR_AT_CMD_isPendingAt() || KNS_Q_isEvtInSomeQ() || s_rx_edge;

	HAL_SuspendTick();
	LPM_saveRtcTime();
	uint32_t t0 = HAL_GetTick();

	if (!skip)
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	HAL_ResumeTick();
	LPM_compensateTick();
	__enable_irq();
	s_total_ms += HAL_GetTick() - t0;
	s_count++;

	/* Console pin back to the LPUART */
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
	{
		GPIO_InitTypeDef rx_af = {0};

		rx_af.Pin = GPIO_PIN_3;
		rx_af.Mode = GPIO_MODE_AF_PP;
		rx_af.Pull = GPIO_NOPULL;
		rx_af.Speed = GPIO_SPEED_FREQ_LOW;
		rx_af.Alternate = GPIO_AF8_LPUART1;
		HAL_GPIO_Init(GPIOA, &rx_af);
	}
	if (s_rx_edge) {
		s_rx_edge = false;
		/* Briefly, until a line proves a host (CC_SLEEP_consoleLine) */
		if (cc_rxwake_on_edge(&s_rxwake, HAL_GetTick(), CC_RX_WAKE_STORM_COUNT,
		    CC_RX_WAKE_BACKOFF_MS, CC_RX_WAKE_BACKOFF_MAX_MS))
			CC_SLEEP_holdAwake(CC_CONSOLE_EDGE_HOLD_MS);
		else
			MGR_LOG_WARN("[SLP] console RX wake storm (no host) - wake disarmed %lus\r\n",
				(unsigned long)(s_rxwake.backoff_ms / 1000UL));
	}

	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
	restore_clock();
	MX_GPIO_Init();
	CC_HW_init();
	MX_SUBGHZ_Init();
	MX_ADC_Init();
	return true;
}

#endif /* USE_CC_TRACKER_APP */
