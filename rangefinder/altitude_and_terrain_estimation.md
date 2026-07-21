# Rangefinder: altitude and terrain estimation

Status: **investigation notes**. Code claims verified against `PX4/PX4-Autopilot` main @ `841bb40365` (v1.18.0-beta1) on 2026-07-21. Flight evidence from ARK DIST SR characterization, log `20260720_2039`, ARK FPV + BMP390 + AFBR-S50 (rangefinder logged but **not** fused).

Goal in one line: make the rangefinder actually improve the Z estimate and the terrain state, so that altitude holds are tight, terrain following does not jump, and optical flow scaling is accurate.

Related PX4 work:

| Item | State | What it is |
|---|---|---|
| [PR #26924](https://github.com/PX4/PX4-Autopilot/pull/26924) | open | baro thrust (propwash) compensation + online RLS estimator — **mine** |
| [PR #26921](https://github.com/PX4/PX4-Autopilot/pull/26921) | **closed, unmerged** | baro/mag rate limiting moved to EKF2 — #26924 declares a dependency on this |
| [Issue #27110](https://github.com/PX4/PX4-Autopilot/issues/27110) | open | rangefinder step-changes cause altitude jumps in Terrain Hold + flow — **mine**, has a 4-PR arc |
| [Issue #27313](https://github.com/PX4/PX4-Autopilot/issues/27313) | open | `_position_sensor_ref` plumbing unused / half-finished — **mine** |
| [Issue #26226](https://github.com/PX4/PX4-Autopilot/issues/26226) | open, stale | implicit altitude semantics & uncoupled height-source roles |
| [Issue #24653](https://github.com/PX4/PX4-Autopilot/issues/24653) | open, stale | altitude estimation issues (static offset/bias class) |
| [PR #25206](https://github.com/PX4/PX4-Autopilot/pull/25206) | closed, unmerged | `[WIP] ekf2: range fusion overhaul` — stalled on scope/review bandwidth |
| [Issue #25258](https://github.com/PX4/PX4-Autopilot/issues/25258) | closed, not planned | refactor LRF usage — source of bresch's framing, see §1 |
| [Issue #24106](https://github.com/PX4/PX4-Autopilot/issues/24106) | closed | new filter for distance sensor validation |
| [PR #27094](https://github.com/PX4/PX4-Autopilot/pull/27094), [#27304](https://github.com/PX4/PX4-Autopilot/pull/27304), [#26794](https://github.com/PX4/PX4-Autopilot/pull/26794) | merged | bias-estimator guard fixes already landed |
| [PR #26975](https://github.com/PX4/PX4-Autopilot/pull/26975) | merged | terrain following setpoint fix (`MPC_ALT_MODE=1`) |
| [Issue #22221](https://github.com/PX4/PX4-Autopilot/issues/22221) | closed, completed | 2023 original: Terrain Hold local Z doesn't follow distance sensor — symptom has recurred |

---

## 1. The shape of the problem

PX4's vertical channel answers three different questions with one machine:

1. **What is my altitude**, in some datum?
2. **What is the ground doing** underneath me?
3. **Which sensor defines the datum?**

Question 3 is answered by a single scalar, `_height_sensor_ref`, and the answer is enforced by making every *other* source's bias estimator absorb its disagreement with the reference. The reference's own bias estimator is a hard no-op:

```cpp
// height_bias_estimator.hpp:56-68
virtual void predict(float dt) override {
    if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) { BiasEstimator::predict(dt); }
}
virtual void fuseBias(float bias, float bias_var) override {
    if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) { BiasEstimator::fuseBias(bias, bias_var); }
}
```

**The reference is therefore infallible by construction.** There is no mechanism anywhere in EKF2 that opposes drift of the reference source. When the reference is wrong, every other sensor's correct disagreement is relabeled "bias", all innovations stay near zero, and nothing is flagged. This is the single structural fact that unifies #26226, #24653, #27313 and the flight evidence in §3.1.

bresch's principle from #25258 is the right north star and is worth restating because it cuts against how the code is currently organized:

> "In general, the LRF doesn't measure altitude but distance to the ground ... altitude hold [should be] a control problem."

The rangefinder is a **terrain** sensor. Today it is wired as a height sensor that is sometimes allowed to touch terrain, and the result is that it is either overtrusted (`EKF2_HGT_REF=RANGE`, altitude follows furniture) or structurally silenced (§2, terrain-only mode).

---

## 2. What each source is good at, and how it lies

| Source | Trustworthy for | Fails by | Detected today by |
|---|---|---|---|
| **Baro** | short-term relative altitude, high rate, always available | propwash/thrust offset; ground effect near surfaces; thermal drift (`TC_B_ENABLE` defaults **off**); weather over long flights | nothing quantitative — a 5σ gate at `EKF2_BARO_NOISE=3.5` means rejection needs **≥17.5 m** |
| **GNSS** | absolute altitude, no drift over hours | cold-start vertical convergence (metres, minutes); multipath; constellation changes | `epv`, which reports **noise, not bias** — measured 1.4 m while true error was 13 m (§3.1) |
| **Rangefinder** | terrain distance, cm accuracy, fast | measures the wrong thing (furniture, vegetation, slopes); dropouts; aliasing beyond unambiguous range | `RangeFinderConsistencyCheck` — a binary reject with **1 s hardcoded** hysteresis |
| **Flow** | velocity, when scaled by a correct HAGL | garbage without HAGL; needs the terrain state to be right | — |

The failure modes are **disjoint in time and regime**, which is exactly why fusion should work and exactly why the current architecture cannot exploit it: a single designated reference cannot be right in every regime, and the bias machinery destroys the evidence that it is wrong.

---

## 3. Three failures, and what they share

### 3.1 The reference is infallible, so reference drift is unopposed

ARK DIST SR characterization flight, 13 minutes, pure vertical profile over grass at night, `EKF2_HGT_REF=1` (GNSS), `EKF2_GPS_CTRL=7`, `EKF2_RNG_CTRL=0`. Rangefinder logged but not fused, so it is a clean independent truth reference.

Error against rangefinder truth, referenced to settled end-of-flight ground:

| | at takeoff | mid-flight | ground-to-ground change |
|---|---|---|---|
| **Baro** (raw pressure) | — | ±1.3 m, no trend | **+1.22 m** |
| **GNSS** | +5.75 m | walks down monotonically | **−6.19 m** |
| **EKF `lpos.z`** | — | tracks GNSS within 0.3 m | **−6.66 m** |

The GNSS altitude was still converging from cold start at takeoff (13.1 m error at t=40 s, 5.8 m at liftoff, settling only ~6 min into the flight; 37 s elapsed between first fix and takeoff). The barometer was the *best sensor on the aircraft* and contributed essentially nothing: its bias estimator moved **+8.5 m** to make it agree with the drifting reference, holding its own innovation at a median of 0.056 m.

Nothing was flagged. Both height sources fused **100%** of the time, **0%** rejected, `cs_baro_fault` never set. Quantitatively it could not have been otherwise — the baro rejection threshold is ≈17.5 m and the divergence was 7 m; the bias estimator's ×1000 "offset detected" boost needs |innov| ≳ 4.7 m and **0 of 1060 samples** qualified. The bias estimator's process noise is hardcoded (`baro_bias_nsd{0.13f}`, `common.h:346`) giving τ ≈ 6 s, so against a drift unfolding over minutes it tracks perfectly and by design.

Consequence in the air: in altitude hold the controller holds the *estimate* at setpoint, so the airframe physically climbed as the estimate sank — +1.35 m, +2.29 m, +0.37 m, +0.35 m across four holds, with the climb rate tracking the GNSS convergence rate and stopping when it settled.

**This is the general mechanism, not a GNSS story.** Swap the reference and the same thing happens with baro drift instead, which is precisely #24653 and #26226.

### 3.2 Ground-effect detection is a heuristic, and it fires at 38 m

The compensation is a **one-sided deadzone applied to already-computed baro innovations**, suppressing only the negative direction — "baro says we are lower" — by up to `EKF2_GND_EFF_DZ` (default 4 m):

```cpp
// baro_height_control.cpp:92-105
if (_control_status.flags.gnd_effect && (_params.ekf2_gnd_eff_dz > 0.f)) {
    if (aid_src.innovation < -deadzone_start) {
        if (aid_src.innovation <= -deadzone_end) { aid_src.innovation += deadzone_end; }
        else                                     { aid_src.innovation = -deadzone_start; }
    }
}
```

The trigger (`ekf_helper.cpp:950-972`) uses **estimated** HAGL vs `EKF2_GND_MAX_HGT` (0.5 m) when HAGL is valid; otherwise the flag can only be latched by the land detector, which sets it on *descent without horizontal movement*, `dist_bottom < LNDMC_ALT_GND`, or takeoff ramp (`MulticopterLandDetector.cpp:316-321`), self-clearing after a 10 s timeout.

In the §3.1 flight — a purely vertical profile, `dist_bottom_valid` false for 100% of the flight — `cs_gnd_effect` was set **twice at altitude**: t=355.9–369.8 s (~28–32 m AGL) and t=618.8–654.7 s (during the descent from 38 m). 50 s of a 710 s flight, nowhere near the ground.

It was immaterial there (baro innovations were ~0.2 m, far below the deadzone, and baro was not the reference). But it is a live hazard for any baro-referenced vehicle that descends vertically: up to 4 m of the correction pulling the estimate back up gets zeroed, in exactly the direction of the drift. **Ground-effect "detection" today is a proxy for a proxy.**

### 3.3 A rangefinder step becomes an altitude jump (#27110)

Full chain is in the issue. The load-bearing steps: a range step trips the kinematic-consistency check, which rejects for a **hardcoded 1 s** (`range_finder_consistency_check.cpp:88`); on re-entry `resetTerrainToRng()` snaps terrain but not Z, producing an **implicit HAGL step that is never signalled as an EKF reset**; `_dist_to_ground_lock` in `FlightTaskManualAltitude` stays stale and the setpoint steps.

Underneath it sits the fact that makes the rangefinder structurally unable to help:

```cpp
// range_height_fusion.cpp:54-58  — verified
if (!update_height) {
    const float k_terrain = K(State::terrain.idx);
    K.zero();
    K(State::terrain.idx) = k_terrain;
}
```

In terrain-only mode — the common case under `EKF2_RNG_CTRL=1` (conditional) — **the entire Kalman gain is zeroed except the terrain state.** The rangefinder cannot correct vehicle altitude at all. And the terrain state is given `EKF2_TERR_NOISE=5.0` ⇒ 25 m²/s of variance growth (`covariance.cpp:227-229`), so it absorbs *any* Z error instantly.

The practical trap: **`dist_bottom` stays correct while `lpos.z` is wrong, range innovations stay ~0, and nothing is flagged.** A user who enables the rangefinder expecting it to fix altitude drift gets a sensor that looks like it is helping and is not.

---

## 4. The dependency web — evaluating the thesis

Working thesis going in: *baro thrust compensation is a prerequisite for all of it, because reliable ground-effect detection requires compensating thrust first; and rangefinder kinematic consistency only works if corrupted baro can be reliably discarded.*

**Mostly right, with one important correction.**

### Where the dependency is real

**Thrust compensation → ground-effect detection.** These are the same physics — rotor wash raising static pressure at the sensor. An uncompensated propwash offset is indistinguishable from ground effect using pressure alone, because both scale with thrust and both push the same direction. Today's detector dodges this by not measuring at all, and §3.2 shows what the dodge costs. You cannot build a *measured* ground-effect detector (innovation-based, or pressure-residual-based) on top of an uncompensated signal, because the thing you would trigger on is already present at every thrust level. **This dependency holds.**

**Baro integrity → kinematic consistency.** Real but narrower than stated, and worth making precise because the precise version is falsifiable. The consistency check compares d(range)/dt against estimated `vz`. A *constant* baro offset does not corrupt `vz` at all — only a *changing* one does. So the coupling is through **d(thrust)/dt**, not thrust. Which means it bites exactly in the regimes where the rangefinder matters most: takeoff, landing, aggressive vertical maneuvers, and gusts. In steady cruise it is nearly absent. **Dependency holds where it counts, but the mechanism is the derivative.**

### Where it does not hold

**The #27110 controller-layer arc does not depend on baro compensation at all.** PRs A and B (signal the HAGL reset; slew the terrain-hold setpoint) fix a stale-lock bug in `FlightTaskManualAltitude`. That bug fires with a perfect barometer. Gating it behind #26924 would stall a low-risk, independently-defensible fix behind a large sensors-layer PR that is itself blocked (below).

So the picture is **two arcs, not one chain**:

```
Arc 1 (controller layer)  — independent, ship now
    #27110 PR A (reset signal) → PR B (setpoint slew) → PR D (observability)

Arc 2 (sensor → estimator) — genuinely sequential
    #26924 baro thrust comp
        → measured ground-effect detection (replaces the land-detector proxy)
            → trustworthy vz in the near-ground regime
                → kinematic consistency that can be made continuous instead of binary
                    → honest Z-vs-terrain gain split
```

### The blocker nobody has flagged

**#26924 declares "Depends on #26921", and #26921 is closed unmerged.** If Arc 2 is the long game, that dependency has to be resolved first — either rebase #26924 free of it, or revive #26921. Worth checking why it was closed before doing either.

---

## 5. Direction

**Sequencing, in the order I would actually do it:**

1. **#27110 PR D (observability) first, not last.** Log terrain-reset events and the Z-vs-terrain gain split. Everything else in this doc took forensic log reconstruction to establish; §3.1 needed an unfused rangefinder as an independent truth reference just to *notice* the drift. The reason #22221 and #24653 died is that nobody could see the mechanism. This is additive, zero-risk, and makes every later argument cheap to make.

2. **#27110 PRs A and B.** Independent of everything else, fix a real customer symptom, small and reviewable.

3. **Unblock #26924** (resolve the #26921 dependency), then land it. This is the gate for Arc 2.

4. **Replace the ground-effect proxy with a measurement.** Once baro is thrust-compensated, the residual near-surface pressure rise *is* the ground-effect signal. It can be detected rather than guessed, and it should not be a one-sided innovation deadzone — that is a hack that hides evidence in the same way the bias estimators do.

5. **Separate "reference" from "active"** — the #26226 / #27313 theme. `_position_sensor_ref` (#27313) is the horizontal instance of a pattern that is already correct for height; fixing it is a cheap way to establish the shape of the general fix. The vertical version is the harder and more valuable one: a source should be able to be *the datum* without being *immune to evidence*.

6. **Then, and only then, range aiding proper**: continuous-confidence kinematic consistency (a real 2-state z/terrain validation filter, per #24106 / #25206) and a gain split that reflects actual confidence rather than a hardcoded zeroing.

**The principle that should govern all of it:** the rangefinder measures terrain, not altitude. Its job in the Z channel is not to be a height sensor — it is to make the terrain state observable enough that (a) flow scaling is right, and (b) altitude hold can become a control problem over a trustworthy terrain estimate, as bresch argued. Every failure in §3 comes from a source being asked to answer a question it does not measure.

---

## 6. Open questions, and what would change my mind

- **Is the §3.1 mechanism reproducible with baro as the reference?** It should be — the guard is symmetric. A deliberate flight with `EKF2_HGT_REF=0` on a vehicle with a known thrust-correlated baro would demonstrate the *same* structural failure with the roles swapped, which is a far stronger argument upstream than either case alone.
- **Does the §3.1 flight say anything about #26924?** Only weakly, and I want to be honest about it rather than overclaim. Thrust spanned just 5.4% across the four holds, so at the PR's identified K ≈ −21 m the predicted baro error span is ~0.31 m — below the ±1.3 m scatter. **The flight is consistent with #26924 but cannot confirm it.** It does supply one useful constraint: the effect is *proportional*, so hover-only logs cannot identify K, and the calibration flight profile must deliberately excite thrust.
- **How much of the drift in §3.1 would `EKF2_HGT_REF=0` actually have removed?** Baro was +1.22 m ground-to-ground vs the EKF's −6.66 m, so ~5× better on this flight. But with baro as reference there is nothing opposing baro drift either, and `TC_B_ENABLE` defaults off, so this trades one unopposed reference for another. It is the right call for short characterization flights and **not** a general fix.
- **Should the deadzone exist at all?** It suppresses evidence rather than modelling the disturbance. If ground effect were compensated as a *measured* offset like thrust, the deadzone could be deleted rather than fixed. Worth checking whether anything else depends on it before proposing removal.
- **What is the actual failure rate in the field?** #27110 has one forum report plus #22221's 2023 recurrence. Upstream will want more evidence than that before accepting estimator-layer changes; PR D is what makes collecting it possible.

---

## 7. Receipts

Verified by direct read against `841bb40365` on 2026-07-21:

| Claim | Location |
|---|---|
| Reference source's bias estimator is a hard no-op | `EKF/bias_estimator/height_bias_estimator.hpp:56-68` |
| Range gain zeroed for all states except terrain in terrain-only mode | `EKF/aid_sources/range_finder/range_height_fusion.cpp:54-58` |
| One-sided ground-effect deadzone on negative baro innovations | `EKF/aid_sources/barometer/baro_height_control.cpp:92-105` |
| Terrain process noise, gated on `_height_sensor_ref != RANGE` | `EKF/covariance.cpp:222-232`, `EKF2_TERR_NOISE` default 5.0 |
| Bias-estimator process noise hardcoded, not a parameter | `EKF/common.h:346` (baro), `:371` (gnss hgt), `:477` (ev hgt) — all `0.13f` |

Reported by code-reading agent, consistent with observed log behavior but **not** individually re-verified — check before citing upstream:

| Claim | Location |
|---|---|
| `lpos.z = -(altitude - _local_origin_alt)`, origin latched once | `EKF/estimator_interface.cpp:628`, `EKF/ekf_helper.cpp:229-240` |
| Ground-effect trigger conditions | `EKF/ekf_helper.cpp:950-972`, `land_detector/MulticopterLandDetector.cpp:316-321` |
| Height-reference silent fallback ordering | `EKF/height_control.cpp:61-122` |
| Kinematic consistency 1 s hysteresis, hardcoded | `EKF/aid_sources/range_finder/range_finder_consistency_check.cpp:40-93`, `:88` |
| GNSS height uses `EKF2_GPS_P_GATE`, not `EKF2_BARO_GATE` (doc string is stale) | `EKF/aid_sources/gnss/gnss_height_control.cpp:86` |
| Altitude from pressure uses fixed ISA profile; measured temperature never enters | `lib/atmosphere/atmosphere.cpp:55-73` |

Flight evidence: `~/Downloads/afbr_testing/sr/grass_home_evening_sr.ulg`, recorded as flight `20260720_2039` in the ARK DIST characterization set (`private_px4`, `src/drivers/distance_sensor/broadcom/afbrs50/analysis/results/`).
