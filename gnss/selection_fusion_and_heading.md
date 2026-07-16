# GNSS: heading/position separation, selection, and fusion

Status: **idea / investigation notes** — not a design doc yet.

Goals in one line: easier and more automatic GNSS configuration so PX4 fuses the **best and most reliable** position solution, without mid-flight flapping or coupling heading to the wrong receiver.

Related PX4 work:

- [PR #27102](https://github.com/PX4/PX4-Autopilot/pull/27102) — separate GNSS heading from position (`vehicle_gnss_heading`)
- [PR #27868](https://github.com/PX4/PX4-Autopilot/pull/27868) — MAVLink `GPS_RAW_INT` / `GPS2_RAW` follow `SENS_GPS_PRIME` (boot-order race for CAN dual-GPS)
- Existing `SENS_GPS_PRIME`, `SENS_GPS_MASK` blending, dual u-blox moving-baseline setup

---

## 1. Why this is on my mind

Dual GNSS is common now: dual independent RTK, rover + moving base for heading, sometimes plus a GCS fixed base feeding RTCM. Configuration is still too manual and too easy to get wrong:

- uORB `sensor_gps` instance order is a **boot-order race** for DroneCAN receivers, so “main GPS” is random unless `SENS_GPS_PRIME` is set to a node ID.
- Default `SENS_GPS_PRIME = 0` pins instance 0 — not dual-RTK-aware.
- Heading was historically bundled with position in the same sample/timestamp path, so choosing a position primary could starve or corrupt heading fusion.
- Blending (`SENS_GPS_MASK`) exists as a multi-GPS answer, but there are user complaints; it is not obviously the right long-term architecture.
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

When blending is **on** (`SENS_GPS_MASK != 0`), prime does not drive EKF position selection (weighted blend instead). MAVLink reporting historically ignored prime entirely (hardcoded instance 0/1); #27868 aligns streams with prime.

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

## 6. Fusion: blending vs multi-GPS EKF vs “best one”

### What exists today

- **Single selected GPS** via prime / −1 auto / timeout fallback.
- **Blending** in `vehicle_gps_position` (`SENS_GPS_MASK`, tau, accuracy weights): weighted average of receivers before EKF sees one `vehicle_gps_position`.

### Complaints / skepticism about blending

Blending is a pre-EKF fusion layer. Concerns (to validate with logs/users, not gospel):

- Hides which physical receiver the EKF is actually using.
- Can invent a position that neither receiver reported if offsets/accuracy weights misbehave.
- Tuning surface (`MASK`, `TAU`) is opaque.
- When blending is on, prime’s “prefer this antenna” story is weakened (docs say prime has no EKF effect).
- Dual-heading setups sometimes enable blending mainly so heading still appears on the blended output — a smell that goes away with proper heading separation (#27102).

### Ideal (maybe): EKF fuses two GPS as independent sources

In principle the filter should accept multiple GNSS position/velocity measurements, each with its own:

- timestamp / delay
- innovation gating
- noise / accuracy
- antenna offset (lever arm)

Questions to answer before committing:

1. **State** — Do we need extra states (per-receiver biases, lever arms, clock)? Or only multiple measurements into existing position/velocity?
2. **Derivations / observability** — Same as dual-mag or dual-vision style aiding, or does GNSS need special treatment (common-mode multipath, correlated errors when both see the same constellation geometry)?
3. **Implementation cost** — EKF2 aid source plumbing for a second GNSS vs teaching `vehicle_gps_position` to be smarter.
4. **Discontinuities** — Multi-source fusion can reject a bad receiver without a hard switch; that is the main argument *for* multi-GPS EKF over “pick one.”

If multi-GPS EKF is too large or changes the filter in painful ways, fall back to selection (next section) rather than defending blending as the long-term answer.

### Next best: fuse only the best one (careful selection)

Selection-only is simpler and keeps one GNSS aid source in the EKF.

Hard requirements if we go this way:

- **No flapping.** Switching receivers mid-flight can inject a position step (different antenna, different errors, RTK re-convergence). The cure can be worse than flying on a slightly worse continuous GPS.
- **Hysteresis / stickiness.** Once primary is chosen (or written by Auto once), only switch on sustained failure: timeout, fix collapse, severe eph blow-up, jamming flag — not on a 10 cm eph improvement.
- **Arm freeze.** Optional: lock selection at arm unless primary dies.
- **Antenna offsets.** Switching receivers without correct lever arms is a guaranteed step; per-receiver offsets (`SENS_GPS*_OFF*`) must be part of the story.
- **Interference asymmetry.** Local RFI, different antennas, different constellation masks mean “best” can flip. Prefer demoting a *failed* receiver over chasing the instantaneous winner.

Sketch for a selection state machine:

```
armed? → freeze current primary unless primary unhealthy for T_fail
unhealthy? → candidate secondary must be healthy for T_promote before switch
switch → optional innovation-compatible check / max step size before accepting
```

Metrics for “best” (Case A, and Case B hybrid if used): fix type, eph/epv, rate, continuity, jamming/spoofing — never fix type alone for dual-heading.

---

## 7. Interference and unequal degradation

Why automatic *and* sticky selection both matter:

| Factor | Effect |
|--------|--------|
| Antenna location / ground plane | One side multipath or blockage |
| Constellation / band config | Different sats, different jamming susceptibility |
| Local radio (telemetry, LTE, video) | Self-interference near one antenna |
| Moving-baseline RTCM path | Rover-only rate degradation (Cases B/C) |

Detection inputs we already have or could use more: `jamming_state`, `spoofing_state`, eph/epv jumps, update gaps, fix_type drops, innovation failures in EKF.

Autoconfigure picks a **good default primary**. In-flight selection only **demotes** on sustained bad health. Multi-GPS EKF (if pursued) rejects bad measurements without rewriting “who is primary.”

---

## 8. Layered end-state (working mental model)

Not a commitment — a stack of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│  EKF                                                         │
│  - position/vel: 1..N GNSS aids (ideal) or 1 selected        │
│  - yaw: vehicle_gnss_heading only (#27102)                   │
└─────────────────────────────────────────────────────────────┘
                              ▲
┌─────────────────────────────────────────────────────────────┐
│  vehicle_gps_position                                        │
│  - classify topology (A/B/C)                                 │
│  - select or (maybe later) pass-through multi GPS            │
│  - publish vehicle_gnss_heading independently                │
│  - blending: legacy / maybe deprecate for multi-EKF         │
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
2. **Multi-GPS EKF feasibility:** measurement-only dual GNSS vs extra states; correlated error model.
3. **Blending fate:** keep for transition, hard-deprecate once multi-aid or sticky select is good enough?
4. **Default params:** stay Manual + prime 0 for zero surprise, or product/airframe defaults for dual-RTK boards?
5. **Role publication:** worth a `sensor_gps` role field early, or heuristics first?
6. **Switch cost:** quantify position step on instance swap in real dual-F9P / dual-CAN logs before designing promote timers.
7. **Heading PR dependency:** treat #27102 as merge prerequisite for any “prefer MB” auto behavior.

---

## 10. Possible work sequence (when this becomes real work)

Rough, not a PR plan:

1. Land heading/position separation (#27102) and related heading-fallback cleanup.
2. Land MAVLink prime alignment (#27868) so telemetry matches selection.
3. Document topologies A/B/C and recommended manual prime (MB node ID / instance) without auto yet.
4. Design doc for selection policy: sticky select + health demotion; explicit non-goals on blending.
5. Optional: `SENS_GPS_SEL` Auto once for serial (`GPS_UBX_MODE`) then CAN heuristics.
6. Spike: second GNSS aid source in EKF2 — size the change; go/no-go vs selection-only.
7. Interference-aware demotion using jam/spoof + rate + eph (still sticky).

---

## 11. References / breadcrumbs

- u-blox ZED-F9P Moving Base app note (UBX-19009093): rover MB solution rate limited by correction latency; absolute high precision needs corrections on the base for best absolute accuracy.
- PX4 ARK RTK docs: set `SENS_GPS_PRIME` to moving base node ID; rover can see degraded rate/latency when corrections are intermittent.
- PX4 dual F9P heading docs: Main = Rover, Secondary = Moving Base for UART setups.
- In-tree: `VehicleGPSPosition`, `gps_blending`, `GPS_UBX_MODE`, `SENS_GPS_PRIME`, `SENS_GPS_MASK`.

---

## 12. One-paragraph summary

We want automatic, trustworthy dual-GNSS behavior: heading is its own EKF path (#27102); position primary is chosen by topology (metrics for dual independent RTK; moving base when rover+MB especially with fixed-base RTCM); users can lock manual prime so auto never fights them; we are skeptical of pre-EKF blending as the long-term fusion answer and would prefer either multi-GPS aiding in the EKF or sticky single-GPS selection that only switches on sustained failure — because mid-flight flapping and interference-driven “best receiver” thrash can be worse than a slightly suboptimal but continuous solution.
