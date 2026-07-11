# Gravity Compensation

Opt-in mode that makes the ElRobot leader arm hold its pose against gravity
while remaining easy to move by hand, instead of the default fully
torque-disabled ("floppy") leader behavior. Toggled from the "Gravity Comp"
button in station-viewer's `BusCard.tsx`, scoped to whichever bus that's
clicked on — independent of whether that bus is currently mirroring to a
follower.

## The constraint that shapes everything here

Feetech ST3215 servos have **no native torque/current-setpoint register** —
only a position-control loop with a `TorqueLimit` effort clamp
(`software/drivers/st3215/src/protocol/memory.rs`). There is no way to tell a
motor "apply 0.4 Nm"; you can only tell it "go to position X" and "don't push
harder than Y". So gravity compensation is *approximated*: every 20ms, compute
how hard gravity is currently pulling on each joint, then nudge
`GoalPosition` a little further in that same direction. The servo's own
internal position gain then supplies a restoring torque roughly proportional
to that offset — a virtual spring, not a real torque source. This only
approximates *static* holding torque; it is not a substitute for real
closed-loop torque control and will not behave correctly under fast dynamic
motion.

## Module layout

- **`elrobot_dynamics.rs`** — static data tables (joint origins/axes/limits,
  per-link mass and center-of-mass) extracted by hand from
  `hardware/elrobot/simulation/elrobot_follower.urdf`, plus
  `forward_kinematics()` and `gravity_torques()`. Gravity torque is computed
  via the virtual-work / Jacobian-transpose method: for each joint, sum the
  moment every *outboard* link's weight exerts about that joint's axis. No
  inertia tensors or velocity terms — this is a static model only.
- **`kinematics.rs`** — minimal hand-rolled `Vec3`/`Mat3` (Rodrigues rotation).
  No `nalgebra`/`urdf-rs` dependency: the chain is fixed at 7 joints and the
  math is small enough to hand-roll, matching this codebase's existing style
  (see `st3215::protocol::units`'s manual bit-twiddling).
- **`control.rs`** — the control law: raw servo ticks → joint angle (mirrors
  the frontend's `resolveElrobotJointValue` sign convention exactly), and
  torque → clamped `GoalPosition` offset using each motor's *live calibrated*
  range (not a fixed tick-per-radian constant).
- **`mod.rs`** — orchestration: per-bus background task lifecycle
  (`GravityComp::start`/`stop`), the 20ms control loop, and the safety
  cutoffs (below). This is also where `MotorCommand::TorqueLimit` gets sent
  before `Torque(1)` on start, and torque gets disabled + `TorqueLimit`
  restored to the ST3215 default on stop.

## Known approximations (deliberate, not bugs)

- **Follower masses, not leader masses.** The URDF models the *follower*
  arm's servos (12V/30kg). The leader uses physically lighter,
  different-gear-ratio servos (per the hardware BOM) chosen for
  backdrivability. Reusing follower link masses is a first-pass
  approximation — `gain_rad_per_nm` (see Tuning below) is the intended lever
  for correcting the residual error on real hardware, not this mass table.
- **Lumped gripper tip.** Motor 8 (gripper) and its gear/jaws are collapsed
  into one fixed point mass attached at the `ST3215_8_v1_1` frame, rather
  than modeled per-gripper-angle. The URDF's inertial data for the jaw links
  contains physically implausible offsets (larger than the arm's own 430mm
  reach — a known artifact of this URDF's CAD export for prismatic/mimic
  joints), so those numbers aren't usable directly. Motor 8 itself is never
  enabled or commanded by gravity comp in either direction.
- **ElRobot only.** SO101 has no mass/inertia data checked into this repo, so
  it isn't supported. Gating is in the frontend's
  `devices/registry.ts::supportsGravityComp()`.
- **Gravity is assumed along -Z in the URDF's own base frame** — consistent
  with joint 1's axis being ~(0,0,1) (a base "yaw" joint on an arm bolted to
  a table, output shaft vertical). If a future arm variant is mounted
  differently, this assumption needs revisiting.

## Command / mode plumbing

Modeled as an **orthogonal per-bus boolean**, entirely separate from
`BusMode` (`BR_LEADER`/`BR_FOLLOWER`) — a bus can be a mirroring leader *and*
gravity-comp'd at the same time, so this is not a `BusMode` variant. See
`GravityCompState`/`GravityCompCommand`/`GravityCompBusState` in
`protobufs/drivers/motors-mirroring/mirroring.proto`, routed via its own
`StationCommandType::STC_GRAVITY_COMP_COMMAND` (a dedicated command type
rather than overloading `STC_MOTOR_MIRRORING_COMMAND`, to avoid proto3
decode ambiguity between two different message shapes on the same wire
type). Handlers live in `../lib.rs`
(`handle_start_gravity_comp`/`handle_stop_gravity_comp`/
`deactivate_gravity_comp`).

Mode *state* is persisted to its own normfs queue
(`motors_mirroring/gravity_comp_modes`) and restored on backend restart for
**display purposes only** — restoring never calls `start_gravity_comp()`
again. Unlike mirroring pairs (inert until a live loop reads fresh leader
data), gravity comp actively drives motors the instant its task starts, so
silently re-arming it after a restart with no fresh operator confirmation
would be unsafe. An explicit command is always required to (re-)enable it.

## Safety cutoffs (all in `mod.rs::run_control_loop`)

- **Staleness**: if `st3215/inference` data for the bus goes stale
  (`MAX_DATA_AGE_NS`, shared with the mirroring loop) for
  `stale_cutoff_cycles` consecutive iterations, hard torque-off — not "hold
  last goal".
- **Overcurrent**: if any arm motor's current exceeds `current_cutoff` for
  `stale_cutoff_cycles` consecutive cycles, hard torque-off. Below that
  threshold, the whole arm holds position that cycle rather than sending new
  goals.
- **Self-stop notification**: either cutoff calls `on_self_stop`, a callback
  wired in `lib.rs` that reuses the exact same teardown path as an
  operator-issued stop command (`deactivate_gravity_comp`), so the mode
  displayed in the UI never drifts from what's actually running.
- **Hard ceilings independent of config**: `GravityCompConfig::clamped()` in
  `../config.rs` clamps `torque_limit`/`max_offset_ticks` to fixed ceilings
  regardless of what a YAML config supplies.

## Tuning

`gain_rad_per_nm` (in `GravityCompConfig`, default `0.05`) cannot be derived
from the ST3215's internal PID coefficients — those are undocumented
firmware units. It must be tuned empirically on real hardware: start low,
increase until the arm feels weightless without oscillating. Configurable
via `gravity-comp:` under `drivers.st3215` in the station YAML config (see
`software/station/shared/station-iface/src/config.rs::GravityCompConfig`).

## Testing

Unit tests cover the dynamics/control math only (no hardware needed):
`cargo test -p motors-mirroring`. They check things like joint 1's
near-vertical axis producing ~0 gravity torque, and shoulder torque growing
with horizontal reach — sanity checks on the sign/axis conventions, not a
substitute for on-arm validation.

**Not yet tested on real hardware**: a bus simultaneously gravity-comp'd and
mirrored to a follower. This is a genuinely new code path — before this
module existed, `Inference` never wrote `GoalPosition` to a *source* bus's
own motors, only to *targets*.
