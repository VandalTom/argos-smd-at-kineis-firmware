#!/usr/bin/env python3
"""
Naiad (CC_TRACKER) power budget model.

Duty-cycle model of one publish cycle, per phase: current on the 3.3 V rail
x time, averaged over the cycle period, then converted to battery-side power
through the TPS61094 boost -> TPS62842 buck chain. Every number below is an
ASSUMPTION with its source tag — replace with bench measurements (a PPK2 or
uCurrent on the 3.3 V rail) and re-run. The point is the relative weight of
each phase, which tells you what to optimise first.

Usage:
    python naiad_power_model.py
    python naiad_power_model.py --nimh-mah 2000 --period-s 420
"""

import argparse

# ---- Assumptions (3.3 V rail) ----------------------------------------------
A = {
    # MCU (STM32WL55)
    "mcu_run_ma":        3.5,    # ~72 uA/MHz @ 48 MHz run (datasheet class figure)        [est]
    "mcu_sleep_ma":      1.5,    # SLEEP with HSI/PLL + LPUART awake                     [est]
    "mcu_stop2_ua":      5.0,    # ~1 uA datasheet; board leakage typically adds more    [est]
    # Always-on loads
    "ws85_avg_ma":       0.30,   # vendor: 1 yr on 2xAA => ~0.3 mA average               [vendor]
    "bme280_ua":         0.1,    # sleep                                                 [datasheet]
    # GPS (SAM-M8Q)
    "gps_acq_ma":        28.0,   # 25-32 mA acquisition (secondary sources)              [est]
    "gps_fix_s":         35.0,   # typical warm/cold-ish start without backup power      [est]
    # Kineis TX (module PA at 27 dBm)
    "tx_ma":             250.0,  # PA + radio during the frame                           [est]
    "tx_s":              1.2,    # LDA2 frame incl. ramp/preamble                        [est]
    # Phase durations
    "ws85_listen_s":     15.0,   # mean wait for the next ~30 s broadcast               [design]
    "env_s":             0.3,    # I2C session + forced read                             [design]
    "awake_overhead_s":  2.0,    # MAC/queue/log per cycle at run current                [est]
    # Power chain
    "chain_eff":         0.85,   # boost + buck end-to-end                               [est]
    "rail_v":            3.3,
    "nimh_v":            2.4,    # 2S NiMH nominal
}


def cycle_ma(period_s, *, ws85_every, gps_every, gps_s, mcu_idle, tx_per_cycle=1.0):
    """Average rail current (mA) over one cycle."""
    t_ws = A["ws85_listen_s"] / ws85_every
    t_gps = gps_s / gps_every
    t_tx = A["tx_s"] * tx_per_cycle
    t_active = t_ws + t_gps + A["env_s"] + t_tx + A["awake_overhead_s"]
    t_idle = max(period_s - t_active, 0.0)
    idle_ma = A["mcu_sleep_ma"] if mcu_idle == "SLEEP" else A["mcu_stop2_ua"] / 1000.0
    charge = (t_ws * A["mcu_sleep_ma"]                       # MCU listening (SLEEP)
              + t_gps * (A["gps_acq_ma"] + A["mcu_sleep_ma"])
              + A["env_s"] * A["mcu_run_ma"]
              + t_tx * (A["tx_ma"] + A["mcu_run_ma"])
              + A["awake_overhead_s"] * A["mcu_run_ma"]
              + t_idle * idle_ma)
    always_on = A["ws85_avg_ma"] + A["bme280_ua"] / 1000.0
    parts = {
        "WS85 listen": t_ws * A["mcu_sleep_ma"] / period_s,
        "GPS": t_gps * (A["gps_acq_ma"] + A["mcu_sleep_ma"]) / period_s,
        "TX": t_tx * (A["tx_ma"] + A["mcu_run_ma"]) / period_s,
        "MCU idle": t_idle * idle_ma / period_s,
        "MCU active": (A["env_s"] + A["awake_overhead_s"]) * A["mcu_run_ma"] / period_s,
        "WS85 + BME always-on": always_on,
    }
    return charge / period_s + always_on, parts


def battery_days(rail_ma, nimh_mah):
    batt_ma = rail_ma * A["rail_v"] / (A["nimh_v"] * A["chain_eff"])
    return nimh_mah / batt_ma / 24.0, batt_ma


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--period-s", type=float, default=420.0, help="base publish period (default 7 min)")
    ap.add_argument("--nimh-mah", type=float, default=2000.0, help="2S NiMH capacity")
    args = ap.parse_args()
    P = args.period_s

    scenarios = [
        ("Deployed, today's firmware (SLEEP idle, WS85 every cycle)",
         dict(ws85_every=1, gps_every=3, gps_s=A["gps_fix_s"], mcu_idle="SLEEP")),
        ("Deployed + STOP2 between phases",
         dict(ws85_every=1, gps_every=3, gps_s=A["gps_fix_s"], mcu_idle="STOP2")),
        ("Deployed + STOP2 + WS85 field cadence (every 4th cycle)",
         dict(ws85_every=4, gps_every=3, gps_s=A["gps_fix_s"], mcu_idle="STOP2")),
        ("Waiting indoors: fast GPS probe 120 s every cycle (no sky check)",
         dict(ws85_every=1, gps_every=1, gps_s=120.0, mcu_idle="SLEEP")),
        ("Waiting indoors: probe ends at 45 s (sky check) + STOP2",
         dict(ws85_every=1, gps_every=1, gps_s=45.0, mcu_idle="STOP2")),
    ]
    print(f"Period {P:.0f} s, 2S NiMH {args.nimh_mah:.0f} mAh, chain eff {A['chain_eff']:.0%}\n")
    print(f"{'scenario':66} {'rail mA':>8} {'batt mA':>8} {'days':>7}")
    for name, kw in scenarios:
        ma, parts = cycle_ma(P, **kw)
        days, batt = battery_days(ma, args.nimh_mah)
        print(f"{name:66} {ma:8.3f} {batt:8.3f} {days:7.0f}")
        top = sorted(parts.items(), key=lambda kv: -kv[1])
        print("      " + ", ".join(f"{k} {v / ma:.0%}" for k, v in top if v / ma >= 0.02))

    burst_ma, _ = cycle_ma(60.0, ws85_every=1, gps_every=10 ** 9, gps_s=0.0, mcu_idle="STOP2")
    print(f"\nDeployment burst (30 min, 1 TX/min): {burst_ma * 0.5:.2f} mAh on the rail "
          f"- negligible vs. a day of operation.")
    print("\nAll figures are estimates; measure the rail per phase and update the table at the top.")


if __name__ == "__main__":
    main()
