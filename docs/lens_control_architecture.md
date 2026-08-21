# Lens Homing and Control Architecture

Target: `firmware/macro-ps` on the Adafruit QT Py RP2040, and the ROS 2 side
running in the `inspection-eoat-docker` container on the Raspberry Pi 5.

This document specifies the control path that lets the lens stepper actually
move. It covers both halves — MCU firmware and host nodes — because the two
are coupled by a deliberate split of responsibility: **the firmware never
decides to home; the host does.**

---

## 1. Where things stand

| Component | State |
|---|---|
| `stepper.cpp` — pin/UART init, step pacing, soft limits, position tracking | Written, compiles, **never executed** |
| `stepper.cpp` — StallGuard homing FSM (`SEEKING_MAX`→`SEEKING_MIN`→`ZEROING`) | Written, **never executed** |
| `microros.cpp` — entity set | One publisher (`pub_imu`). **No executor, no subscriptions** |
| `/lens/command`, `/lens/state`, `/lens/home` | Named in `config.h:90-94`, referenced nowhere else |
| `ps_interfaces` package | **Does not exist** |
| Calibration persistence | **Does not exist** |
| TMC2209 hardware bring-up | Unverified — `driver_ok` is computed but never surfaced |

The driver is complete but unreachable. `state_init()` sets `SYS_UNCALIBRATED`
and nothing ever changes it, so `stepper_tick()` returns at its
`fabsf(velocity) < 1.0f` guard on every single iteration. The motor cannot turn
under any input.

---

## 2. Design decisions

### D1 — Homing is host-triggered, never automatic in firmware

Boot lands in `UNCALIBRATED`. The firmware exposes homing as a goal it
services; it never initiates one.

*Why:* at power-on the micro-ROS agent is not yet connected, so a firmware-
initiated home would run with no way to publish feedback, no way to report a
result, and — per the README §3 priority rules, where `HOMING` locks out
everything except `/lens/home` cancellation — no way to cancel. The board also
resets far more often than it power-cycles (a 1200-baud touch from
`arduino-cli` resets it, as does USB re-enumeration), and StallGuard homing
locates limits *by driving the mechanism into its hard stops*. That is
acceptable occasionally and unacceptable on every reset.

Automatic homing is still available — as **host policy** (§4.2), where it is
observable, cancellable, and skippable.

### D2 — Calibration persists in RP2040 flash

`lens_steps_min`, `lens_steps_max` and the last known `lens_steps` are stored
in the emulated-EEPROM flash sector and restored at boot. A successful restore
boots the system straight to `IDLE`.

*Why:* the lens cannot be back-driven while powered off, so the mechanical
position is still valid across a reset. Re-homing on every boot would be pure
wear for no information gain.

*Consequence to accept:* if power is lost **mid-move**, the stored position
lags reality. §3.7 and D4 handle that.

### D3 — The homing FSM stays on core 1

README §4.4 places homing inside the `/lens/home` action callback on the
micro-ROS task. This implementation keeps it in `stepper_tick()` on core 1,
with the action server on core 0 acting as a supervisor that sets the mode and
observes `homing_phase`.

*Why:* homing is a multi-second operation with hard real-time step pacing.
Running it inside an executor callback means either blocking the executor
(agent pings time out, `/camera_head/imu` stalls) or littering the callback
with yields. Core 1 already owns step timing; the FSM belongs next to it.

### D4 — StallGuard stays armed during normal motion

Today `SG_RESULT` is only consulted while homing. It should also be polled
during `LENS_MOVING`, where a stall means the lens has hit a physical limit the
soft limits did not predict — i.e. the stored calibration is wrong.

*Why:* this is what makes D2 safe. A stale restored position is self-
correcting: the lens runs into a stop, StallGuard fires, the firmware halts,
invalidates the stored calibration, and reports `STALLED`. The host then
re-homes. The `LensState.status` enum in README §5.1 already reserves
`3 = STALLED` for exactly this, so the interface anticipates it.

---

## 3. MCU architecture

### 3.1 Core responsibilities

```
core 0  (setup/loop)                    core 1  (setup1/loop1)
─────────────────────                   ──────────────────────
micro-ROS transport + agent FSM         status_led_tick()
rclc executor spin                      imu_tick()      → imu_queue
  cb_lens_cmd                           led_tick()
  /lens/home action server              stepper_tick()  ← owns step timing
/lens/state publisher (20 Hz)                           ← owns homing FSM
/camera_head/imu drain (200 Hz)         joystick/buttons (stubs)
calibration flash commit (debounced)
```

All cross-core data lives in `SystemState` behind `state_mutex`. Core 1 never
touches micro-ROS; core 0 never generates step pulses.

### 3.2 Mode state machine

```
                    ┌──────────────────┐
   flash restore ──►│      IDLE        │◄── homing complete
   (valid record)   └──┬────────────┬──┘
                       │            │
        /lens/command  │            │  /lens/home goal
                       ▼            ▼
              ┌────────────┐   ┌──────────┐
              │LENS_MOVING │   │  HOMING  │
              └─────┬──────┘   └────┬─────┘
                    │               │
         stall / target reached     │ abort, cancel,
         watchdog timeout           │ or driver_ok == false
                    │               ▼
                    │        ┌───────────────┐
                    └───────►│ UNCALIBRATED  │◄── stall during LENS_MOVING
     (stall only)            └───────────────┘    (invalidates calibration)
```

The single new edge versus README §3 is **flash restore → IDLE** at boot.

### 3.3 micro-ROS entities and executor

`microros.cpp` currently initialises `allocator` and `support` but creates no
executor at all. Add one, sized for the handles below.

| Handle | Type | Notes |
|---|---|---|
| `sub_lens_cmd` | subscription | `/lens/command` |
| `srv_home` / `act_home` | service (phase 1) → action (phase 2) | see §6 |
| `pub_lens_state` | publisher (best-effort) | `/lens/state`, 20 Hz, timer-free — published from `loop()` |
| `pub_imu` | publisher (best-effort) | exists already |

`rclc_executor_init(&executor, &support.context, N, &allocator)` where `N`
counts subscriptions + services + action servers only (publishers are not
executor handles). Spin with
`rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0))` in the
`AGENT_STATE_CONNECTED` branch of `loop()`, immediately before
`microros_drain_and_publish()`.

`microros_destroy_entities()` must fini the executor and every new entity, in
reverse creation order — the existing teardown only handles `pub_imu`.

### 3.4 `/lens/command`

Callback runs on core 0. Gate first, then write under the mutex:

```c
// Rejected outright — README §3 priority rules.
if (mode == SYS_PS_CAPTURING || mode == SYS_HOMING || mode == SYS_UNCALIBRATED)
    return;
```

Two command modes:

- **VELOCITY** — `value` is steps/s. Sets `lens_velocity_cmd`, refreshes
  `last_vel_cmd_us`, sets `mode = SYS_LENS_MOVING` when non-zero. Intended for
  streaming jog commands; the watchdog (§3.6) stops motion if the stream dies.
- **POSITION** — `value` is normalised `[0,1]`. Sets `lens_position_target`,
  sets `lens_velocity_cmd = ±LENS_DEFAULT_VELOCITY` toward the target, sets
  `mode = SYS_LENS_MOVING`.

**A required addition to `SystemState`.** The README's reference callback sets
`last_vel_cmd_us` only in velocity mode. With the watchdog as implemented, a
position move would therefore be killed 150 ms after it starts. The watchdog
must apply to velocity mode only, so core 1 has to know which mode is active:

```c
enum LensCmdMode { LENS_CMD_VELOCITY = 0, LENS_CMD_POSITION = 1 };

struct SystemState {
    ...
    LensCmdMode lens_cmd_mode;
};
```

### 3.5 Position-mode termination

`stepper_tick()` has no notion of `lens_position_target` today — nothing stops
a position move. Add termination in the step domain rather than the normalised
domain, so the stop is exact and free of float drift:

```c
target_steps = steps_min + (int32_t)lroundf(target_norm * (steps_max - steps_min));
```

Stop when `steps == target_steps`, or when the next step would move past it
(overshoot guard for the case where the target shifts mid-move). On arrival:
`lens_velocity_cmd = 0`, `mode = SYS_IDLE`, mark calibration dirty (§3.7).

`LENS_POSITION_TOLERANCE` (0.005) stays relevant only for the host deciding
whether a commanded move is worth issuing.

### 3.6 Watchdog and stall handling during motion

| Condition | Applies to | Action |
|---|---|---|
| `now - last_vel_cmd_us > VEL_WATCHDOG_MS` | VELOCITY mode only | `stop_and_idle()` |
| `steps` reaches `target_steps` | POSITION mode only | stop, `SYS_IDLE`, mark dirty |
| next step violates soft limits | both | `stop_and_idle()` |
| `SG_RESULT() < STALL_THRESHOLD` | both (**new**, D4) | halt, invalidate calibration, `SYS_UNCALIBRATED`, report `STALLED` |

Stall polling reuses the existing rate limit (`STALL_POLL_MS`, 10 ms) and needs
its own settling window after motion starts, for the same reason homing does:
`SG_RESULT` reads low whenever the motor is not turning, so a freshly started
move would otherwise report an instant false stall. Reuse
`HOMING_STALL_GUARD_MS` (150 ms) measured from the first step of the move.

### 3.7 Calibration persistence

**Record** — stored in the emulated-EEPROM sector:

```c
struct CalibrationRecord {
    uint32_t magic;        // 0x4C454E53 'LENS' — distinguishes from erased flash
    uint16_t version;      // bump on any layout change
    uint16_t flags;        // bit0: calibration valid
    int32_t  steps_min;
    int32_t  steps_max;
    int32_t  steps_now;    // position at last clean stop
    uint32_t crc32;        // over all preceding fields
};
```

`magic` + `version` + `crc32` together mean an erased sector, a record written
by older firmware, and a partially written record are all rejected, falling
back to `UNCALIBRATED`.

**Mechanism.** `EEPROM.begin(4096)` / `EEPROM.commit()` from the arduino-pico
core. Two properties of that implementation matter and are already handled by
the library:

- `commit()` wraps the erase/program in `rp2040.idleOtherCore()` /
  `resumeOtherCore()` with interrupts disabled. This is **mandatory** on the
  RP2040 — a flash write while the other core executes from XIP will hang the
  chip — and it means we must not hand-roll the write.
- `write()` only sets the dirty flag when a byte actually changes, and
  `commit()` early-returns when not dirty. Redundant commits therefore cost no
  flash wear.

**Write policy.** Core 0 owns the commit. Core 1 sets a `calibration_dirty`
flag; core 0 commits when *all* of: the flag is set, `mode != SYS_LENS_MOVING`
and `mode != SYS_HOMING`, and 2 s have elapsed since the last motion.

The 2 s debounce means a burst of operator jog commands produces one write, not
one per jog. Commit points in practice:

- homing completion (authoritative — writes min, max and the centred position)
- 2 s after any move stops
- immediately on stall (writing `flags = 0` to invalidate)

**Restore.** In `state_init()` on core 0, before core 1 begins stepping (core 1
already blocks on `mutex_is_initialized(&state_mutex)`):

```
read record → valid?  ─yes─► steps_min/max/steps restored, recompute
                              lens_position_norm, mode = SYS_IDLE
                └─no─► leave SYS_UNCALIBRATED
```

**Cost of a commit.** A sector erase plus program parks core 1 for roughly
40–60 ms. During that window there is no IMU sampling, no LED refresh and no
step generation. This is why commits are debounced to a stopped lens: the
visible effect is a ~50 ms gap in `/camera_head/imu` shortly after motion ends.
Document this rather than chase it.

**Endurance.** The sector is rated ~100k erase cycles. With the debounce, a
realistic operator session produces well under 100 commits/day, giving a
service life measured in years. Worth revisiting only if the lens is ever
driven by a continuous automated sweep.

### 3.8 `/lens/state`

Published from `loop()` at `LENS_STATE_RATE_HZ` (20 Hz), independent of any
callback, per README §6. `status` maps from `SystemMode`:

| `SystemMode` | `LensState.status` |
|---|---|
| `SYS_UNCALIBRATED` | 0 `UNCALIBRATED` |
| `SYS_IDLE` | 1 `IDLE` |
| `SYS_LENS_MOVING` | 2 `MOVING` |
| stall latched | 3 `STALLED` |
| `SYS_HOMING` | 4 `HOMING` |
| `SYS_PS_CAPTURING` | 1 `IDLE` — the lens is held stationary; the capture state is not a lens state |

`STALLED` is a latched sub-state of `UNCALIBRATED`, cleared when a new homing
goal is accepted. It exists so the host can distinguish "never calibrated" from
"calibration was invalidated by hitting something", and apply different policy
(§4.2).

### 3.9 `/lens/home` supervision

The action server on core 0 does not perform homing; it drives and observes it.

```
goal accepted
   └─► clear stall latch; homing_phase = HOMING_NONE; mode = SYS_HOMING
       (core 1 sees mode==HOMING, phase==NONE and enters SEEKING_MAX)

each loop() iteration while goal active:
   read homing_phase
   phase changed?          → publish feedback {phase, description}
   mode became SYS_IDLE?   → success: mark calibration dirty, commit,
                             publish result {success, range_steps}
   mode became SYS_UNCALIBRATED? → abort: publish result {false, message}
   elapsed > home_timeout? → cancel (see below)

cancel requested:
   mode = SYS_UNCALIBRATED   (core 1 sees mode != HOMING, calls homing_reset())
```

Detecting completion by the `SYS_HOMING → SYS_IDLE` edge is unambiguous because
`/lens/command` is rejected while homing (§3.4), so nothing else can move the
mode during a goal.

Aborts to cover: `driver_ok == false` (already implemented — homing without
StallGuard would drive blindly into a stop), and no phase progress within a
timeout.

---

## 4. Host architecture

### 4.1 `ps_interfaces`

The package the README assumes does not exist. It must provide, at minimum for
the lens path:

```
ps_interfaces/msg/LensCommand.msg      uint8 mode, float32 value
ps_interfaces/msg/LensState.msg        float32 position_norm, float32 velocity,
                                       uint8 status, builtin_interfaces/Time stamp
ps_interfaces/action/HomeLens.action   goal: (empty)
                                       result: bool success, float32 range_steps, string message
                                       feedback: uint8 phase, string phase_description
```

**This is the single biggest cost in the plan.** `micro_ros_arduino` ships
*precompiled* — `install.sh` clones it and uses the prebuilt
`cortex-m0plus` archive. Custom message types are not available to the firmware
until that library is **rebuilt** with `ps_interfaces` in its type set, via the
`micro_ros_setup` / `micro_ros_arduino` docker build flow.

> **Correction.** An earlier revision of this document claimed actions worked on
> the stock archive and only custom *types* needed a rebuild, and sequenced the
> work to prove motion on `std_msgs` / `std_srvs` first to defer that cost.
> That is wrong, and the reason matters.
>
> `rcl_action` and `rclc/action_server.h` are indeed compiled in. But upstream
> builds the `cortex_m0` target — the one the QT Py links — with
> `colcon_verylowmem.meta`, which sets `RMW_UXRCE_MAX_SERVICES=0`,
> `MAX_PUBLISHERS=2` and `MAX_SUBSCRIPTIONS=1`. Those size static arrays inside
> the archive, so no sketch-side `#define` can raise them. The stock archive
> cannot create **any** service, so even the interim `std_srvs/Trigger` plan was
> unbuildable; and with `pub_imu` and `/lens/state` the publisher budget is
> already exhausted before an action server asks for its two.
>
> Verified by reading the pool symbols straight out of the shipped archive:
> ```
> ar x src/cortex-m0plus/libmicroros.a
> nm --print-size *.obj | grep -E 'C custom_(publishers|subscriptions|services)'
> ```
> against the same symbols in `cortex-m4` (known 10 / 5 / 1) to get the slot
> sizes: publisher 216 B, subscription 216 B, service 200 B.
>
> The rebuild is therefore unavoidable and unconditional, so it is now Phase 1
> and `ps_interfaces` rides along at no extra cost. `install.sh` performs it
> automatically and caches it; see `firmware/macro-ps/micro_ros/README.md`.

Standard packages already compiled in include `std_msgs`, `std_srvs`,
`sensor_msgs`, `geometry_msgs`, `control_msgs` and `example_interfaces`.

### 4.2 `lens_manager` node

New node in the `inspection_eoat` package. It owns the auto-homing policy that
D1 deliberately keeps out of the firmware.

Responsibilities:

- Subscribe `/lens/state`. Treat the firmware as authoritative about status.
- On first `status == UNCALIBRATED` after the node comes up, if
  `auto_home_on_start`, send a `/lens/home` goal.
- On `status == STALLED`, if `auto_home_on_stall`, re-home — bounded by
  `max_home_retries` and then latch a fault, so a genuinely jammed mechanism
  does not grind against a stop indefinitely.
- Never home while `status == HOMING`, and never send a second goal while one
  is active.
- Expose `/lens/recalibrate` (`std_srvs/Trigger`) so an operator can force a
  home regardless of policy.
- Publish `/lens/ready` (`std_msgs/Bool`) so downstream nodes — anything that
  needs a focused lens — can gate on calibration instead of racing it.

Parameters:

| Parameter | Default | Meaning |
|---|---|---|
| `auto_home_on_start` | `true` | Home once when an uncalibrated lens is seen |
| `auto_home_on_stall` | `true` | Re-home after a stall invalidates calibration |
| `max_home_retries` | `2` | Consecutive failed homes before latching a fault |
| `home_timeout_s` | `60.0` | Goal abandoned past this |
| `state_timeout_s` | `2.0` | `/lens/state` silence treated as firmware lost |

Because calibration now survives reboots (D2), the common startup path is
`status == IDLE` on the first message and `lens_manager` does nothing at all.
Homing becomes the exception, not the rule.

### 4.3 Launch integration

In `bringup.launch.py`, after `micro_ros_agent`:

```
micro_ros_agent  →  lens_manager  →  (downstream nodes gate on /lens/ready)
```

`lens_manager` must tolerate the agent not yet being up — it waits for
`/lens/state` rather than assuming the node exists, since the QT Py's agent
handshake takes a second or two after container start.

---

## 5. Sequences

**Cold boot, valid stored calibration (the normal case)**

```
MCU reset → state_init() reads flash → valid → mode = IDLE
core 0 pings agent → CONNECTED → entities created
/lens/state publishes status=IDLE at 20 Hz
lens_manager sees IDLE → no action → /lens/ready = true
```

**Cold boot, no stored calibration (first flash, or after a stall)**

```
state_init() finds erased/invalid record → mode = UNCALIBRATED
lens_manager sees UNCALIBRATED → sends /lens/home goal
core 0: mode = SYS_HOMING → core 1 runs SEEKING_MAX → SEEKING_MIN → ZEROING
feedback published per phase; NeoKeys blink amber per README §4.4
core 1 centres, mode = SYS_IDLE
core 0 detects edge → commits calibration → result{success, range_steps}
lens_manager → /lens/ready = true
```

**Stall during a normal move**

```
core 1: SG_RESULT < threshold during LENS_MOVING
      → halt, flags = 0 committed, mode = SYS_UNCALIBRATED, stall latched
/lens/state → status = STALLED
lens_manager → re-home (retry budget permitting)
```

**Agent disconnect mid-move**

Already handled in `macro-ps.ino:66-72`: velocity zeroed, `HOMING` demoted to
`UNCALIBRATED` (an interrupted home is invalid), `PS_CAPTURING` to `IDLE`.
`IDLE` is deliberately left alone, so a Pi reboot or agent restart does not
discard calibration. On reconnect `lens_manager` sees `IDLE` and does nothing.

---

## 6. Implementation phases

**Phase 0 — hardware bring-up.** Surface `driver_ok`. Nothing below is
meaningful until the TMC2209 is confirmed answering on `Serial2`; it has never
been exercised. Cheapest form: publish it in `/lens/state`, or a distinct
status-LED pattern at boot. Then drive a fixed test velocity to confirm
direction, `rms_current` and microstepping against the real mechanism.

**Phase 1 — `ps_interfaces` and the library rebuild.** *Done.* See
`firmware/macro-ps/micro_ros/README.md`. This was originally sequenced last, on
the assumption that stock types would let motion be proven first without paying
for a rebuild. That assumption was wrong — see the correction in §4.1 — so it
now comes first, because nothing else can be built until it is done.

**Phase 2 — motion.** `/lens/command` (`LensCommand`), `/lens/state`
(`LensState`), `/lens/home` (`HomeLens` action). Adds the executor, the command
path, position-mode termination, stall-during-motion, and flash persistence.
This is the phase that proves homing end to end.

**Phase 3 — host policy.** `lens_manager` with the retry/latch policy,
`/lens/ready` gating, `/lens/recalibrate`, launch wiring, parameter surface.

There is no longer an interim interface layer to throw away. The earlier plan
built one on `std_msgs/Float32` + `std_srvs/Trigger` to decouple "does the
mechanism work" from "is the message package built"; since the rebuild is
unavoidable either way, that indirection would have cost a throwaway host node
and firmware branch and bought nothing.

---

## 7. Open risks

**Step pacing shares a loop with `strip.show()`.** `led_tick()` and
`stepper_tick()` both run in `loop1()`. With 75 pixels (16+24+32 ring plus 3
NeoKeys) a `show()` blocks ~2.3 ms with interrupts disabled. At
`LENS_DEFAULT_VELOCITY` (500 steps/s) the step interval is 2 ms, so a ring
update swallows an entire step interval and the motion visibly stutters.
README §4.3 in fact specifies a **hardware timer alarm** for STEP generation;
the current implementation uses software pacing in the tick loop. If phase 0/1
shows audible or visible roughness, moving step generation to a PIO state
machine or an alarm ISR is the fix. Worth measuring before designing around.

**`HOMING_VELOCITY` and `STALL_THRESHOLD` are unvalidated guesses.**
StallGuard sensitivity is strongly coupled to speed and current; `SGTHRS` needs
tuning on the real mechanism, and stall detection is unreliable at very low
speeds. Expect to iterate on `HOMING_VELOCITY` (200), `STALL_THRESHOLD` (50)
and `TMC_RMS_CURRENT_MA` (600) together, and expect the value that works for
homing to differ from the one that works for D4's normal-motion detection.

**Mid-move power loss.** Accepted per D2. The stored position lags reality and
the first move afterwards may run into a stop — caught by D4, at the cost of
one stall event and a re-home. If this proves common in practice, the
alternative is a dirty-flag write at move *start*, which doubles flash wear;
not recommended unless the failure is actually observed.

**`test_connection()` semantics.** Returns 0 on success. It reads back a
register over the half-duplex link, so it detects a dead UART but not a
mis-set `TMC_DRIVER_ADDRESS` on a board strapped differently — a wrong address
can still answer. Confirm the BIGTREETECH step-stick's MS1/MS2 address straps
match `TMC_DRIVER_ADDRESS` (`0b00`) during phase 0.
