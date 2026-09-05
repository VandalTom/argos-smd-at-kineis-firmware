/**
 * @file    cc_tier.h
 * @brief   Battery-tiered TX cadence + anti-phase-lock jitter.
 *
 * Port of the WS85-Tracker Board::txBasePeriodMs() SoC-aware tier machine.
 * Each cycle's battery reading selects the base publish period; the caller
 * adds 0..CC_TX_JITTER_PCT% of hardware-RNG jitter on top so TX times don't
 * phase-lock against the Kineis satellite pass schedule.
 *
 * Two threshold sets, picked per cycle by what measured the battery:
 *   - nimh=true  : the 2S NiMH pack via the gated divider (NIMH set)
 *   - nimh=false : fallback supply measurement (VBAT/LiPo set)
 *
 * Demotion is immediate at the raw threshold; promotion needs the hysteresis
 * margin so a reading straddling a boundary can't flap the cadence. A failed
 * battery read (~0 mV) lands LOW — conservative.
 *
 * Pure C, no MCU dependencies — unit-tested on the host.
 */

#ifndef CC_TIER_H
#define CC_TIER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	CC_TIER_HIGH = 0,
	CC_TIER_MID,
	CC_TIER_LOW,
} cc_tier_t;

/** Tier state — persists across cycles (resets to HIGH on reboot; the first
 *  cycle's reading corrects it straight away). */
typedef struct {
	cc_tier_t tier;
} cc_tier_state_t;

/** Reset to full rate (call once at boot). */
void cc_tier_init(cc_tier_state_t *s);

/**
 * @brief Update the tier from this cycle's battery reading and return the
 *        base publish period for the coming sleep.
 * @param s      tier state
 * @param batt_mv this cycle's battery reading (0 = failed read -> LOW)
 * @param nimh   true = reading is the NiMH pack (NIMH threshold set)
 * @return base period in ms (CC_TX_PERIOD_MS / _MID_MS / _LOW_MS)
 */
uint32_t cc_tier_update(cc_tier_state_t *s, uint16_t batt_mv, bool nimh);

/**
 * @brief This cycle's jittered period: base + rand32 % (base*pct/100 + 1).
 * @param base_ms tier base period
 * @param jitter_pct max stretch percent
 * @param rand32 a fresh hardware-RNG word
 */
uint32_t cc_tier_jittered_period_ms(uint32_t base_ms, uint8_t jitter_pct, uint32_t rand32);

#endif /* CC_TIER_H */
