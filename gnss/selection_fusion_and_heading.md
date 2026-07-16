# GNSS: heading/position separation, selection, and fusion

Status: **investigation notes**. §6 (blending vs selection) investigated 2026-07-16 against main @ `eddc929db3`, with code, git-history, GitHub, and ecosystem receipts (§11) — verdict: **remove blending, replace with sticky health-gated selection**. §5 (autoconfigure) is still idea-stage.

Goals in one line: easier and more automatic GNSS configuration so PX4 fuses the **best and most reliable** position solution, without mid-flight flapping or coupling heading to the wrong receiver.

Related PX4 work:

- [PR #27102](https://github.com/PX4/PX4-Autopilot/pull/27102) — separate GNSS heading from position (`vehicle_gnss_heading`)
- [PR #27868](https://github.com/PX4/PX4-Autopilot/pull/27868) — MAVLink `GPS_RAW_INT` / `GPS2_RAW` follow `SENS_GPS_PRIME` (boot-order race for CAN dual-GPS)
- [PR #19907](https://github.com/PX4/PX4-Autopilot/pull/19907) — enabled blending **by default** (2022); the heading crutch, see §6.2
- [PR #25516](https://github.com/PX4/PX4-Autopilot/pull/25516) — heading/blending decoupling predecessor (stale-botted, succeeded by #27102)
- Existing `SENS_GPS_PRIME`, `SENS_GPS_MASK` blending, dual u-blox moving-baseline setup

---

## 1. Why this is on my mind

Dual GNSS is common now: dual independent RTK, rover + moving base for heading, sometimes plus a GCS fixed base feeding RTCM. Configuration is still too manual and too easy to get wrong:

- uORB `sensor_gps` instance order is a **boot-order race** for DroneCAN receivers, so “main GPS” is random unless `SENS_GPS_PRIME` is set to a node ID.
- Default `SENS_GPS_PRIME = 0` pins instance 0 — not dual-RTK-aware.
- Heading was historically bundled with position in the same sample/timestamp path, so choosing a position primary could starve or corrupt heading fusion.
- Blending (`SENS_GPS_MASK`) is not an opt-in extra — it has been the **default** dual-GPS behavior since 2022 (#19907, default 7), enabled fleet-wide as an explicit data-gathering measure and kept because heading rode on it. Investigated in §6; verdict: replace with selection.
- Interference and RF environment hit receivers unequally (antenna placement, constellation/band config, local radio self-interference). Static “use GPS0” is brittle; naive “always switch to best” can be worse than a slightly worse but continuous solution.

The desired end state is roughly:

1. **Heading and position are independent pipelines** (EKF consumes them separately).
2. **Position path** automatically prefers the right physical receiver (or fuses both safely).
3. **Users do not have to know** node IDs / instance indices for the common dual-RTK setups, but intentional manual config is never clobbered.

---

## 2. Heading / position separation

### Problem (today / pre-separation)

GNSS yaw from dual-antenna / moving-baseline RTK was carried on the same path as position (`sensor_gps` → blended/selected `vehicle_gps_position` → one EKF sample). That couples:

- **Timestamps** — heading can inherit the position path’s time
- **Quality gates** — position checks (sats, PDOP, eph) can block heading
- **Rate** — heading updates limited by position observation interval
- **Starting conditions** — position intermittency blocks heading startup

### Direction (PR #27102)

```
sensor_gps             → VehicleGPSPosition → vehicle_gps_position → EKF position / velocity
sensor_gnss_relative   → VehicleGPSPosition → vehicle_gnss_heading → EKF yaw (own buffer)
                         (fallback: sensor_gps.heading for drivers without relative topic)
```

Implications for selection/fusion ideas below:

- Choosing **moving base** as position primary must **not** be required to get heading.
- Autoconfigure of “primary GPS” is a **position/velocity** decision only.
- Heading source stays the rover / relative path regardless of who wins position selection.

This separation is a prerequisite for sane dual-RTK primary selection. Without it, “prefer MB for position” fights “need rover for heading” in one message.

---

## 3. What `SENS_GPS_PRIME` does today

| Value | Behavior (when blending is **off**) |
|-------|-------------------------------------|
| **0** (default) | Prefer uORB instance 0 while it has a usable fix; secondary on timeout |
| **1** | Prefer instance 1 |
| **2–127** | Prefer DroneCAN node ID matched via `device_id` |
| **−1** | “Auto”: equal priority — best `fix_type`, then most sats |

When blending is **on** (`SENS_GPS_MASK != 0`), prime does not drive EKF position selection (weighted blend instead). Since `SENS_GPS_MASK` **defaults to 7**, prime is inert for the EKF on any healthy dual-receiver vehicle unless the user disabled blending — the ARK RTK pages even instruct enabling blending (for heading) *and* setting prime to the MB node (for position) together, without noting the second is dead while the first is active. MAVLink reporting historically ignored prime entirely (hardcoded instance 0/1); #27868 aligns streams with prime.

Important: default **0 is indistinguishable from “user intentionally wants instance 0.”** You cannot safely auto-rewrite prime without an explicit enable or a different default/sentinel.

Also: bare **−1 auto is a bad dual-heading policy**. The rover often shows RTK Fixed while the moving base is only 3D Fix → metric auto picks the **rover**, which is often the worse *navigation* source for rate/latency (see topologies below).

---

## 4. Dual-receiver topologies

Same hardware count (two on-vehicle receivers), different physics. Autoconfigure / selection must classify these.

### Case A — Dual independent RTK (no moving baseline)

- Both receivers are “rovers” relative to a fixed base (or both standalone / both RTK from ground).
- No heading baseline between them (or heading not the point).
- **Want:** pick the unit that is performing best right now (or fuse both if possible).
- Metrics: fix type, sats, eph/epv, update rate / continuity, maybe jamming/spoofing flags.

### Case B — Rover + Moving Base, no fixed-base RTCM

- Rover computes relative RTK to the moving base → **heading** (and often better *reported* accuracy).
- Moving base is typically standalone 3D for absolute position.
- Rover nav rate can **degrade** when MB RTCM is late (u-blox moving-base app note; ARK docs already say prefer MB for this reason).
- **Tension:** rover often wins eph/fix_type; MB wins rate and latency stability.
- **Want:** role-aware policy, not pure eph. Product call: rate-first (MB) vs accuracy-first with rate veto.

### Case C — Rover + Moving Base + GCS fixed base RTCM

- Fixed base corrections to the vehicle (ideally into the moving base, or both).
- MB can be RTK-class absolute **and** high rate.
- Rover still provides heading via MB relative; may still be rate-limited on the MB link.
- **Want:** position primary = **moving base**. Clear win once MB has RTK-grade fix.

Serial dual-F9P convention in PX4 (`GPS_UBX_MODE` heading modes): main = rover, secondary = moving base — so “best position primary” is often **instance 1**, opposite of default prime 0.

CAN: instance order is boot-order; prime should be the MB **node ID**, not “0”.

---

## 5. Autoconfigure idea (position primary only)

### Motivation

Users of dual RTK should not have to set node IDs correctly for good defaults. Intentional setups must not be silently rewritten.

### Sketch

Keep `SENS_GPS_PRIME` as the sticky/manual selection.

Add something like **`SENS_GPS_SEL`** (name TBD):

| Mode | Behavior |
|------|----------|
| **Manual** | Honor `SENS_GPS_PRIME` only. Never write it. Safe default. |
| **Auto once** | Detect topology + roles, write `SENS_GPS_PRIME`, then stop (or flip to Manual). Opt-in. |
| **Auto continuous** | Runtime resolve primary with hysteresis; optional mirror into status. Freeze after arm. |

### Classification → action

| Topology | Autoconfigure position primary |
|----------|--------------------------------|
| A (dual independent) | Best performer by metrics (richer than fix+sats only) |
| B (rover+MB, no fixed base) | Prefer MB for rate/latency by default; document accuracy tradeoff; optional hybrid later |
| C (rover+MB + fixed base) | MB (rate + accuracy) |
| Unknown / single GPS | Do nothing |

### Role detection (confidence order)

1. Serial / FC-owned drivers: `GPS_UBX_MODE` heading modes already encode rover vs MB.
2. Runtime: exactly one receiver publishes valid heading / `sensor_gnss_relative` with heading → that one is rover; the other is MB candidate.
3. Future: explicit role field on `sensor_gps` (standalone / moving_base / heading_rover) from driver/CAN node — best long-term.

Proxy for B vs C: MB has RTK-class fix ⇒ treat as C-like; MB 3D-only while rover RTK ⇒ B.

### Non-goals for autoconfigure

- Does not choose heading source (post-separation: always relative/rover path).
- Does not replace fusion policy (blend vs multi-source EKF vs single select).
- Does not thrash mid-flight; write-once or arm-freeze first.

---

## 6. Fusion: blending is the wrong architecture — replace it with selection

Investigated 2026-07-16 (code on main @ `eddc929db3`, git history, GitHub archaeology, ArduPilot/industry/GNSS-engineering evidence). This section states what blending actually does, why it is architecturally wrong rather than merely buggy, what it legitimately provides (and how each item is replaced), and the replacement plan. Receipts in §11.

### 6.1 What blending actually is (code truth)

Everything lives in `src/modules/sensors/vehicle_gps_position/` (`GpsBlending` + `VehicleGPSPosition`), hard-capped at 2 receivers:

- **It is the default.** `SENS_GPS_MASK` has defaulted to 7 (all three accuracy bits) since July 2022 (#19907). Any vehicle with two healthy, accuracy-reporting receivers blends out of the box. Meanwhile the official EKF docs (`tuning_the_ecl_ekf.md`) still claim the default disables blending, and still reference a topic that no longer exists — the feature’s own documentation has rotted.
- **Weights:** recomputed every output sample as normalized inverse variance of each receiver’s *self-reported* `s_variance_m_s` / `eph` / `epv` (per mask bit). No cross-check, no innovation feedback — the blend believes whatever each receiver claims about itself, and the official docs of both PX4 and ArduPilot admit the claimed accuracies are not comparable across manufacturers (CEP vs 1σ …).
- **Position:** weighted mean, applied through per-receiver NE/alt offsets low-pass filtered with `SENS_GPS_TAU` (default 10 s). Key property: once offsets converge (`oᵢ → b − pᵢ`), the output `Σ wᵢ(pᵢ + oᵢ)` equals the old blend point **for any weights** — a weight change therefore does not step the output, it *slews* it toward the new weighted mean over ~tau. Remember this; it is the core defect (§6.3.1). PX4’s own header admits the underlying instability: “This internal state cannot be used directly by estimators because if physical receivers have significant position differences, variation in receiver estimated accuracy will cause undesirable variation in the position solution” (`gps_blending.hpp`). ArduPilot’s implementation of the same algorithm says the same about itself (“statistically the most likely location, but will be not stable enough for direct use by the autopilot”).
- **Metadata:** the output takes the **min** of eph/epv/sacc/hdop/vdop and the **max** of fix_type/satellites across contributors (`gps_blend_states()`); all misc fields — including `jamming_state` / `spoofing_state` — are struct-copied from the dominant receiver only; `device_id` is zeroed. The blend’s own unit test pins min-eph with a `// TODO: should be greater than` (`gps_blending_test.cpp:213`); the same inconsistency was flagged in 2021 (#16531) and never fixed.
- **Cadence:** receivers >20% apart in rate → output locked to the **slowest** receiver. Same-rate receivers → publish gated on the two arrivals landing within half a period of each other, so publish timing jitters between the receivers’ arrival times and the blend carries up to half an epoch of staleness from the other receiver (timestamps are weight-averaged to keep this self-consistent for constant velocity, but the effective measurement lags the freshest receiver).
- **Heading:** passthrough from the highest-weighted receiver publishing a finite heading — i.e. position weights decide who provides yaw. This is the coupling #27102 kills (its PR body calls out “timestamp corruption: heading gets the blended-position timestamp instead of its actual measurement time”).
- **Fallback:** fewer than 2 blendable receivers (fix < 2D, 2 s timeout, missing accuracy fields) → hard switch to best-fix/most-sats selection (with `SENS_GPS_PRIME` override) on **raw** receiver data, and all learned offsets are zeroed — the smooth-transition machinery is discarded exactly when a receiver degrades.
- **EKF2 consumes the result 1:1:** blended hacc/vacc/sacc feed measurement noise (`gps_control.cpp:316,354`) and the `EKF2_REQ_*` quality gates (`gnss_checks.cpp:129-133`); blended spoof/jam flags and the per-sample antenna offset come along (`EKF2.cpp:2658-2672`). ~30 other consumers (home position, RemoteID, ADSB-out, OSD, navigator, mag cal …) treat the same output as “the GPS”.

### 6.2 How we got here (short history)

| When | What | Why |
|------|------|-----|
| 2018 | #9765 adds blending inside ekf2 (priseborough) | answer to #9431 “RTK to normal GPS handover” — the actual need was **failover**; PR merged with “Noise on blended lat/lon in log needs investigating” still open |
| 2020 | CarlOlsson replay-testing campaign: #13953 #14223 #14232 #14251 #14276; EPIC #14252 | failover didn’t fail over, offsets stuck, weights double-counted, samples dropped; EPIC still open today with two never-fixed defects (#13896 lever arms, #14267 altitude sink) |
| 2020 | #14447 moves blending from ekf2 to sensors | untangle the estimator frontend — and it **broke EKF2 replay**, the tool that had made the 2020 bugs findable (“a step backwards” — CarlOlsson) |
| 2021 | #16682 extracts the `GpsBlending` class, adds `SENS_GPS_PRIME` (bresch) | PR body: “GPS blending is not used very often and has no SITL nor unit test coverage” |
| 2022 | #19796 “always publish GPS heading” closed **unmerged** (“not an issue with GPS blending enabled”) → #19907 enables blending **by default** (AlexKlimaj) | selection flapped (EKF altitude error in the example log) and CAN moving-baseline setups lost heading whenever the MB won selection; blending “ensures the heading is always used” |
| 2023 | #22575 anti-flap + fallback quick patches (bresch) | continuous instance switching on sat count after primary timeout; antenna-disconnect incidents with no failover |
| 2025–26 | #25516 (stale-botted) → #27102; per-receiver offsets #26634/#26660; prime→CAN node #26126; telemetry alignment #27868 | investment moved to selection and per-receiver metadata; nobody is investing in the blender itself |

The two most honest quotes in the record: bresch approving default-on (2022): “the algorithm only had limited testing and we’re not aware about its corner cases. At the same time, if we don’t enable it on more vehicles, we don’t have more data and cannot fix the potential issues” — the fleet became the test bench. bresch again while patching it (2023): “**We should refactor the GNSS selector completely.** This is just a quick patch to fix this specific bug.” And oystub (2025, #25516 + dev call): blending is “hacky and semantically incorrect … mixes concepts that don’t really belong together”; “I think we all agree that the current blending system isn’t optimal.”

Pattern: every inflection point was driven by a need blending does not own — failover (#9431), heading availability (#19796/#19907/#21574), CAN instance identity (#21574/#26126) — and each of those now has, or is getting, a proper home.

### 6.3 Why blending is the wrong architecture

**1. It launders receiver faults into sub-gate drift — the EKF’s blind spot.** Because converged offsets make the output weight-invariant (§6.1), any change in *reported* accuracy — an RTK float↔fixed transition on one receiver swings inverse-variance weights by 2–3 orders of magnitude — does not step the output; it slews it toward the new weighted mean over ~`SENS_GPS_TAU` (10 s). Two receivers disagreeing by 2 m produce ~0.1 m/s of synthetic drift for tens of seconds: far below any innovation gate, so the EKF *tracks* it. A hard receiver switch, by contrast, is a step the EKF can gate, reject, or absorb as an accounted reset — and controllers already consume reset deltas. Blending converts the detectable failure mode into the undetectable one; that inversion is the core architectural sin. Field signature: #14267 (open since 2020) — two receivers whose height data disagreed “well in excess of the reported vertical position accuracy” (priseborough’s analysis), blended altitude sat in between, vehicle sank on takeoff with GPS height primary. The fusion literature states the general form: pre-mixing destroys fault isolation — once a faulty sensor is averaged in, the filter sees one corrupted pseudo-sensor and can neither attribute nor excise the fault (federated-filter / solution-separation literature, §11).

**2. The statistics never paid for the risk.** The single-receiver GNSS error budget is dominated by ionosphere/troposphere/ephemeris/clock terms (~11 m of an ~11.3 m budget; receiver noise ≈ 0.3 m), and those dominant terms are **common-mode** between two receivers at zero baseline — that correlation is the entire premise of DGPS/RTK. The variance of a mean of two equally-good measurements is σ²(1+ρ)/2: at ρ ≈ 1 averaging buys almost nothing, and what actually decorrelates (antenna-local multipath, thermal noise) is the small tail. When receiver grades differ (RTK vs standard), inverse-variance weighting degenerates to ~100:0 anyway — you get no averaging benefit at all, just the failure modes. And because the weights assume independence, the blend’s implied accuracy is overconfident by construction.

**3. It reports fiction to every consumer.** Min-accuracy/max-fix composition means a blend of one RTK-fixed and one 3D receiver is labeled “RTK fixed, eph 0.02” while carrying the 3D receiver’s mass. Those numbers feed the EKF measurement noise and the `EKF2_REQ_*` gates, arming checks, the GCS display, RemoteID and ADSB-out. Integrity flags are worse: jamming/spoofing detection from the non-dominant receiver simply vanishes from the output while its measurements keep contributing. `device_id = 0` erases provenance — neither logs nor the GCS can say which hardware the vehicle was actually flying on.

**4. It degrades rate, latency, and availability — the opposite of what redundancy is for.** Rates >20% apart lock the output to the slowest receiver: a 1 Hz backup drags a 10 Hz RTK system to 1 Hz EKF updates. (A 2026 field report with exactly this configuration — 10 Hz RTK + 1 Hz backup, `SENS_GPS_MASK` among the configured params — describes meters of drift in position mode and a flyaway; thread died unresolved.) Same-rate pairs pay publish jitter plus up to half an epoch of staleness. Moving-baseline rovers additionally lag when corrections are late — ArduPilot refuses to blend in MB-yaw configurations for exactly this reason (“the rover is significant[ly] lagged and gives no more information on position or velocity”, plus a hard wiki prohibition), while PX4’s ARK pages *prescribe* blending in that same topology.

**5. Its transitions cliff at the worst possible times.** Blending needs two healthy accuracy-reporting receivers; the moment one degrades below 2D or times out, the module hard-switches to raw selected data and zeroes the learned offsets. CarlOlsson, 2020 (#13953): “If one GPS module timeouts ekf2 instantly switches out of blending and pushes the other GPS data directly into the EKF, resulting in high innovations.” So steady state is a slowly wandering average and the failure path is an unmanaged step — the worst of both worlds. (Ironically, jump-free receiver switching was #9765’s stated design goal for the offset machinery; the fallback discards it.)

**6. It is a perpetual bug tax on an algorithm nobody defends.** At least ten merged fixes to its own math/failover between 2020 and 2023 (stuck offsets #14232, double-counted weights #14251, dropped samples #14276, dead failover #14223, wrong init #20335, broken timing math #17383, zero UTC time #22498, rapid switching #22575 …), an EPIC (#14252) open since 2020, field recipes that include `SENS_GPS_MASK 0` as a step to make dual-F9P heading work (#21675), and the maintainer quotes in §6.2. The ecosystem context removes any remaining doubt about which posture is normal: ArduPilot runs the same Riseborough algorithm but ships **UseBest** as the default, restricts Blend to matched receivers, and forbids it with moving-baseline yaw; and none of the surveyed INS/avionics vendors (NovAtel SPAN/ALIGN, Applanix GAMS, VectorNav VN-300, Garmin G1000) averages two receivers’ fixes — industry practice is second-antenna-for-heading plus health-monitored primary/secondary reversion. The honest counter-voice exists (Randy Mackay personally flies blend and likes it; dual-Zubax users reported years of good service), but even ArduPilot’s dev team is on record as split, and their project never made it the default.

### 6.4 The steelman — what blending actually provides, and its replacement

| Blending provides | Assessment | Replacement |
|-------------------|------------|-------------|
| Zero-config failover (the original #9431 ask) | genuinely valuable | selection with automatic health demotion — same zero config, honest output |
| No hard-switch flapping between comparable receivers (ArduPilot anecdote: two identical receivers ping-ponged UseBest selection with position jumps) | valid complaint about *naive* selection, not an argument for averaging | stickiness + hysteresis + switch only on sustained failure; the rare step is absorbed by EKF gating/reset accounting |
| Graceful down-weighting as a receiver degrades | only works when the receiver honestly self-reports degradation; silent failures (multipath bias, spoof walk-off) keep their weight and poison the blend | health scoring uses the same self-reports *plus* cross-checks a blender cannot do (shadow innovation against the EKF state, §6.5) |
| Heading always available on dual-CAN MB setups — the real reason for default-on (#19796/#19907/#21574) | a workaround for the single-topic architecture bug | #27102: heading rides its own topic, independent of position selection |
| Noise averaging in benign conditions | ≈ nothing (correlated errors, §6.3.2) | not needed |

Nothing in the left column requires averaging. After #27102 lands, blending’s remaining constituency is empty.

Note: “just set `SENS_GPS_MASK = 0`” is not the answer either — today’s fallback selection is primitive (best-fix/most-sats + prime, minimal hysteresis; the 2023 community Q&A noted there was no quality-based auto-switch with blending off). The replacement below is a real selector, not the current fallback.

### 6.5 Replacement architecture

**Layer 1 — sticky health-gated selection (do this; it replaces blending).** `vehicle_gps_position` becomes a pure pass-through of ONE receiver’s `sensor_gps` — honest metadata, real `device_id`, real integrity flags, per-receiver antenna offset (already per-receiver since #26634/#26660 and consumed per-sample by EKF2) — chosen by a selection state machine that owns:

- **Primary designation** = `SENS_GPS_PRIME` (instance / CAN node ID / −1 auto), semantics unchanged; §5’s autoconfigure can write it later.
- **Demotion only on sustained, evidenced failure**: timeout, fix collapse, eph/epv blow-up (absolute and relative to the standby), jamming/spoofing flags, rate collapse — each with a dwell time (`T_fail`), never on a 10 cm eph beauty contest.
- **Promotion hysteresis**: the standby must be healthy for `T_promote` before a switch; optional arm-freeze (while armed, switch only on hard failure).
- **Shadow innovation monitoring** — the honest generalization of what blending pretended to do: compare *both* receivers’ position/velocity against the current EKF state; demote a primary whose innovations grow while the standby’s stay consistent. External evidence instead of self-reported accuracy, and the bad receiver never touches the filter.
- **Selection status output** (who is primary, per-receiver health, why the last switch happened) so GCS and logs stop guessing — pairs with #27868 so telemetry shows the same receiver the EKF flies.

Switch cost is bounded and accounted: per-receiver antenna offsets remove the lever-arm component; what remains is true receiver disagreement, which the EKF absorbs through innovation gating and its existing reset-delta machinery — the same mechanism `EKF2Selector` already uses for whole-estimator handover (`EKF2Selector.cpp:433-447` synthesizes position/heading deltas on instance change). Rare accounted steps beat continuous unaccounted drag.

Sketch (unchanged from before, still right):

```
armed? → freeze current primary unless primary unhealthy for T_fail
unhealthy? → candidate secondary must be healthy for T_promote before switch
switch → optional innovation-compatible check / max step size before accepting
```

Seed code: `lib/gnss/SensorGpsSelector` (#27868) is the prime-resolution half; the health/hysteresis state machine replaces `GpsBlending`; `gps_blending_test.cpp`’s failover cases port over as the regression floor.

**Layer 2 — true redundancy, later, optional.** Two candidates, both downstream of Layer 1 and neither a blocker for removing blending:

- **Multi-EKF per receiver** (`EKF2_MULTI_GPS` alongside `EKF2_MULTI_IMU`/`EKF2_MULTI_MAG`): per-receiver estimator lanes plus the existing selector. Complete fault isolation (a bad receiver corrupts only its own lane) and continuous handover via the selector’s reset deltas. Precedents on both sides of the fence: PX4’s own IMU/MAG multi-EKF, and ArduPilot’s EKF3 affinity/lane-switching (“lane-switching becomes a mechanism for sensor-switching”) — which is where ArduPilot’s dual-GPS redundancy actually went, not blending. Cost: RAM/CPU per instance and combinatorics with IMU×MAG; config-gated, small boards excluded.
- **In-filter multi-GNSS aiding** (dagar floated this on #25516: fuse individual receivers with their own body-frame offsets directly in the EKF; bengrocholsky sketched per-antenna `EKF2_GPS_POS` arrays, rel-pos-vector fusion, 3-antenna attitude). Honest concern: co-located receivers violate the independence assumption (§6.3.2) — EKF2’s sequential scalar fusion has no off-diagonal R, so fusing both at full weight over-collapses the covariance, and the resulting overconfidence punishes *other* aid sources at their gates. oystub’s caveat points the same direction from practice: “given how bad and non-linear variance estimates and errors from GNSS receivers can behave, one probably has to think pretty hard about how to do this in a good way.” Viable only with deliberate de-weighting or correlation handling; needs its own design doc and log studies before it is more than an idea.

Verdict: Layer 1 is required no matter which (if either) Layer 2 happens — it is the fallback layer even in a multi-source future, and it is what removes blending.

**Migration / deprecation.**

1. Land #27102 first — hard prerequisite. Removing blending before it would re-run the 2022 heading regression that made blending the default in the first place (#19796 → #19907).
2. Implement the Layer-1 selector behind the existing `vehicle_gps_position` topic (all ~30 consumers unaffected); port the failover tests.
3. In the same release: flip `SENS_GPS_MASK` default to 0 and warn once if a non-zero mask is set (“GPS blending is deprecated; using selection”).
4. Docs in the same release: rewrite the four ARK RTK pages (the blending line becomes “heading is automatic”; `SENS_GPS_PRIME` = MB node ID now actually does something), fix `tuning_the_ecl_ekf.md` (already wrong today), release-notes callout for the default-behavior change, drop the redundant SITL dual-GPS airframe param.
5. One release later: delete `GpsBlending`, `SENS_GPS_MASK`, `SENS_GPS_TAU`.

---

## 7. Interference and unequal degradation

Why automatic *and* sticky selection both matter:

| Factor | Effect |
|--------|--------|
| Antenna location / ground plane | One side multipath or blockage |
| Constellation / band config | Different sats, different jamming susceptibility |
| Local radio (telemetry, LTE, video) | Self-interference near one antenna |
| Moving-baseline RTCM path | Rover-only rate degradation (Cases B/C) |

Detection inputs we already have or could use more: `jamming_state`, `spoofing_state`, eph/epv jumps, update gaps, fix_type drops, innovation failures in EKF, and shadow innovations of the standby receiver against the current EKF state (§6.5).

Autoconfigure picks a **good default primary**. In-flight selection only **demotes** on sustained bad health. Multi-EKF per receiver (if pursued — §6.5 Layer 2) isolates a bad receiver entirely without rewriting “who is primary.”

---

## 8. Layered end-state (working mental model)

Not a commitment — a stack of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│  EKF                                                         │
│  - position/vel: ONE selected GNSS aid                       │
│    (later, optional: multi-EKF lane per receiver — §6.5)     │
│  - yaw: vehicle_gnss_heading only (#27102)                   │
└─────────────────────────────────────────────────────────────┘
                              ▲
┌─────────────────────────────────────────────────────────────┐
│  vehicle_gps_position                                        │
│  - classify topology (A/B/C)                                 │
│  - sticky health-gated selection (§6.5) — blending removed   │
│  - publish vehicle_gnss_heading independently                 │
│  - selection status out (who / why switched) → GCS + logs    │
└─────────────────────────────────────────────────────────────┘
                              ▲
┌─────────────────────────────────────────────────────────────┐
│  Drivers / CAN nodes                                         │
│  - role hints (MB vs rover)                                  │
│  - RTCM / MBD                                                │
│  - honest eph, fix, jam flags                                │
└─────────────────────────────────────────────────────────────┘
```

Plus:

- **Config layer:** Manual prime vs Auto once/continuous (`SENS_GPS_SEL` idea).
- **Telemetry:** MAVLink GPS messages follow the same prime resolution (#27868) so GCS matches EKF intent.

---

## 9. Open questions

1. **Case B default:** always MB (rate-first), or rover until rate collapses?
2. **Layer-2 redundancy:** once sticky selection works, is either multi-EKF-per-receiver (`EKF2_MULTI_GPS`) or in-filter multi-GNSS aiding worth its cost? Needs the correlated-error model and instance-count sizing (§6.5). Explicitly *not* a blocker for blending removal.
3. **Deprecation mechanics:** flip the `SENS_GPS_MASK` default and warn in the same release as the selector, or hold one release? Anything owed to intentional blending users beyond a release-notes migration note?
4. **Default params:** stay Manual + prime 0 for zero surprise, or product/airframe defaults for dual-RTK boards?
5. **Role publication:** worth a `sensor_gps` role field early, or heuristics first?
6. **Switch cost:** quantify position step on instance swap in real dual-F9P / dual-CAN logs before designing promote timers (also feeds the Layer-2 go/no-go).
7. **Shadow innovation plumbing:** cleanest source of per-receiver innovation evidence — EKF publishes standby-receiver innovations, or the selector computes coarse residuals against `vehicle_local_position`?

---

## 10. Possible work sequence (when this becomes real work)

Rough, not a PR plan:

1. Land heading/position separation (#27102) and related heading-fallback cleanup — hard prerequisite for touching blending (§6.5).
2. Land MAVLink prime alignment (#27868) so telemetry matches selection.
3. Implement the Layer-1 selector (§6.5): sticky select + health demotion behind `vehicle_gps_position`; selection status output; port `gps_blending_test` failover cases.
4. Same release: deprecate blending — `SENS_GPS_MASK` default → 0 with a warn-once; rewrite ARK RTK pages + `tuning_the_ecl_ekf.md`; release-notes callout.
5. One release later: remove `GpsBlending` + `SENS_GPS_MASK`/`SENS_GPS_TAU`.
6. Document topologies A/B/C and recommended manual prime (MB node ID / instance) without auto yet.
7. Optional (§5 track): `SENS_GPS_SEL` Auto once for serial (`GPS_UBX_MODE`) then CAN heuristics.
8. Later, if wanted: Layer-2 spike — `EKF2_MULTI_GPS` vs in-filter dual GNSS; switch-cost + correlation study from logs first (§9.6).
9. Interference-aware demotion using jam/spoof + rate + shadow innovation (still sticky).

---

## 11. References / breadcrumbs

Code refs pinned to main @ `eddc929db3` (2026-07-16).

**In-tree code:**

- `src/modules/sensors/vehicle_gps_position/` — `GpsBlending` (algorithm), `VehicleGPSPosition` (integration), `gps_blending_test.cpp` (min-eph pinned with `// TODO: should be greater than`, line 213).
- Mechanics: metadata min/max composition in `gps_blend_states()`; weight-invariance + tau slew in `update_gps_offsets()` / `calc_gps_blend_output()`; slowest-rate + same-rate phase-window cadence in `blend_gps_data()`; fallback offset-zeroing in `update()`; PX4’s own instability admission at `gps_blending.hpp:103`.
- EKF2 consumption: `EKF2.cpp:2658-2672` (hacc/vacc/sacc, spoofed/jammed, per-sample antenna offset), `gps_control.cpp:316,354` (noise floors), `gnss_checks.cpp:129-133` (quality gates). Estimator-handover reset deltas: `EKF2Selector.cpp:433-447`. Multi-EKF today is IMU+MAG only (`params_multi.yaml`).
- `lib/gnss/SensorGpsSelector` (branch `pr-27868`) — prime-resolution seed for the Layer-1 selector.

**PX4 PRs / issues (blending record):**

- Origin: [#9431](https://github.com/PX4/PX4-Autopilot/issues/9431) (need: RTK↔GPS handover) → [#9765](https://github.com/PX4/PX4-Autopilot/pull/9765) (priseborough, 2018; merged with “Noise on blended lat/lon in log needs investigating”) → #10570 (dissimilar rates).
- 2020 replay campaign (CarlOlsson): #13953 (timeout → “high innovations”), #14223 (failover didn’t), #14232 (stuck offsets), #14251 (weights double-counted), #14276 (dropped samples); EPIC [#14252](https://github.com/PX4/PX4-Autopilot/issues/14252) **still open** — unfixed: [#13896](https://github.com/PX4/PX4-Autopilot/issues/13896) (lever arms, now largely addressed by #26634) and [#14267](https://github.com/PX4/PX4-Autopilot/issues/14267) (altitude sink from datum-mismatched height averaging).
- Restructuring: #14447 (move to sensors, 2020; broke replay — “a step backwards”), #16682 (class + `SENS_GPS_PRIME`, 2021; “not used very often and has no SITL nor unit test coverage”).
- Default-on saga: [#19796](https://github.com/PX4/PX4-Autopilot/pull/19796) (proper heading fix, closed unmerged) → [#19907](https://github.com/PX4/PX4-Autopilot/pull/19907) (default-on, 2022; bresch: “limited testing … not aware about its corner cases … if we don’t enable it on more vehicles, we don’t have more data”); #21574 (blending as CAN band-aid — “ensures the heading is always used but doesn’t solve this issue”, AlexKlimaj).
- Patch era: [#22575](https://github.com/PX4/PX4-Autopilot/pull/22575) (2023; “We should refactor the GNSS selector completely. This is just a quick patch”), #20335, #17383, #22498, #16531 (min-sacc “doesn’t make full sense”), #21675 (dual-F9P heading recipe includes `SENS_GPS_MASK 0`).
- Direction: [#25516](https://github.com/PX4/PX4-Autopilot/pull/25516) (oystub: “hacky and semantically incorrect”; dev call: “blending mixes concepts that don’t really belong together”; dagar floats per-receiver EKF fusion; bengrocholsky: `EKF2_GPS_POS` array + rel-pos vector + 3-antenna attitude) → #27102. Selection/metadata momentum: #26126 (prime = CAN node ID), #26634/#26660 (per-receiver offsets/delays), #27868 (telemetry follows prime). dagar architecture umbrellas: #15683, #22470.

**ArduPilot (same algorithm, opposite posture):**

- `GPS_AUTO_SWITCH` default = **1 (UseBest)** on stable Copter/Plane; Blend (2) is opt-in. [Blending wiki](https://ardupilot.org/copter/docs/common-gps-blending.html): same-manufacturer, accuracy-reporting receivers only. [GPS-for-yaw wiki](https://ardupilot.org/copter/docs/common-gps-for-yaw.html): “Do not use GPS_AUTO_SWITCH = 2 (Blend) when using Moving Baseline configurations.” — PX4’s ARK pages prescribe the opposite.
- `AP_GPS_Blended.cpp` self-description: the weighted average is “not stable enough for direct use by the autopilot”; code skips blending for MB yaw (“the rover is significant lagged and gives no more information on position or velocity”). Per-receiver antenna offsets (`GPS1_POS_*`) are weight-blended into a fictitious moving antenna, same as PX4 now.
- [ardupilot#13689](https://github.com/ArduPilot/ardupilot/issues/13689): RTK + non-RTK blend → EKF instability/flyaway; triaged as “Blend must be matching GPS’s”, not fixed. Counter-anecdote: identical receivers ping-ponging UseBest selection, advice was Blend — the flapping complaint Layer-1 hysteresis answers.
- Randy Mackay (2021): “varying opinions in the dev team about ‘use best’ vs ‘blending’ but I personally always use blending” — the honest counter-voice; no dev consensus either way, but the shipped default is selection.
- ArduPilot’s actual redundancy direction: [EKF3 affinity / lane-switching](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html) — per-lane GPS, “lane-switching becomes a mechanism for sensor-switching”.

**Engineering basis:**

- [VectorNav GNSS error budget](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-gnsserrorbudget): receiver noise ≈ 0.3 m of an ≈ 11.3 m budget; ionosphere ≈ 5 m dominates.
- [PSU GEOG 862 (Van Sickle)](https://courses.ems.psu.edu/geog862/print/l2.html): atmospheric + ephemeris errors are correlated between nearby receivers (the premise of differential GNSS); multipath and receiver noise are the uncorrelated tail.
- Var(mean of two equal-variance measurements) = σ²(1+ρ)/2 → ≈ no reduction at ρ ≈ 1; independence-assuming weighted averages understate the fused error (Schmelling, [hep-ex/0006004](https://arxiv.org/abs/hep-ex/0006004)).
- Federated-filter / solution-separation literature: pre-mixing destroys fault isolation; parallel estimation + monitored selection preserves it.

**Industry practice (dual GNSS):** NovAtel SPAN/ALIGN, Applanix POS (GAMS), VectorNav VN-300, Garmin G1000 — the second receiver/antenna is for heading and/or health-monitored primary/secondary reversion. None of the surveyed vendors averages two receivers’ position fixes.

**Field reports:** [“RTK Backup GPS Causing Havoc”](https://discuss.px4.io/t/rtk-backup-gps-causing-havoc/48725) (2026: 10 Hz RTK + 1 Hz backup → meters of drift, flyaway; unresolved); PX4 community Q&A 2023-07-19 (with blending off there was no quality-based auto-switch — the gap Layer 1 fills).

**Prior breadcrumbs (kept):**

- u-blox ZED-F9P Moving Base app note (UBX-19009093): rover MB solution rate limited by correction latency; absolute high precision needs corrections on the base for best absolute accuracy.
- PX4 ARK RTK docs: set `SENS_GPS_PRIME` to moving base node ID; rover can see degraded rate/latency when corrections are intermittent.
- PX4 dual F9P heading docs: Main = Rover, Secondary = Moving Base for UART setups.
- In-tree: `VehicleGPSPosition`, `gps_blending`, `GPS_UBX_MODE`, `SENS_GPS_PRIME`, `SENS_GPS_MASK`.

---

## 12. One-paragraph summary

We want automatic, trustworthy dual-GNSS behavior: heading is its own EKF path (#27102); position primary is chosen by topology (metrics for dual independent RTK; moving base when rover+MB, especially with fixed-base RTCM); users can lock manual prime so auto never fights them. Blending is settled (§6): it has been the silent default since 2022, it launders receiver disagreement into sub-gate drift the EKF tracks instead of rejecting, reports best-of-both metadata while carrying worst-of-both mass, drops the pipeline to the slowest receiver’s rate, and survives only as the heading crutch that #27102 obsoletes — averaging two co-located receivers was never statistically sound in the first place. Replace it with sticky, health-gated selection that passes one receiver through honestly and switches only on sustained, evidenced failure (the EKF’s innovation gates and reset accounting absorb the rare step); per-receiver multi-EKF lanes remain the optional later upgrade for true fault isolation.
