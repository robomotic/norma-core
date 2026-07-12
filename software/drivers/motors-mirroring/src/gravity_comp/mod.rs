//! Orchestration for the leader-arm gravity compensation control loop.
//!
//! ST3215 servos have no native torque/current-setpoint register, only a
//! position loop with a `TorqueLimit` effort clamp - so gravity compensation
//! is approximated by continuously nudging `GoalPosition` around the arm's
//! current pose by an amount proportional to the computed gravity torque,
//! using the servo's internal position gain as an implicit spring. See
//! `elrobot_dynamics` for the torque model and `control` for the offset law.

mod control;
mod elrobot_dynamics;
mod kinematics;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use bytes::Bytes;
use normfs::NormFS;
use parking_lot::RwLock;

use crate::config::{clamp_gravity_comp_gain, MotorConfig, GRAVITY_COMP_REFRESH_INTERVAL};
use crate::inference::model::StationState;
use crate::inference::normalize;
use crate::inference::{Inference, MAX_DATA_AGE_NS};
use crate::types::{BusKey, Command, MotorCommand};
use elrobot_dynamics::{JOINT_COUNT, JOINT_LIMITS_RAD};

/// ST3215 motor IDs for the 7 compensated arm joints (rev_motor_01..07).
/// Motor 8 (the gripper) is never enabled or commanded by gravity
/// compensation in either direction.
pub const ARM_MOTOR_IDS: [u8; JOINT_COUNT] = [1, 2, 3, 4, 5, 6, 7];

struct GravityCompTask {
    stop_flag: Arc<AtomicBool>,
    /// Live-adjustable gain, read by the control loop every cycle so it can
    /// be tuned from the UI without restarting gravity comp or the station
    /// process. Everything else in `GravityCompConfig` is fixed for the
    /// lifetime of the task (only the gain was asked to be UI-tunable).
    gain: Arc<RwLock<f64>>,
    handle: tokio::task::JoinHandle<()>,
}

/// Tracks the running gravity-comp control-loop task per bus. Owned by
/// `Inference`, which is the only thing that talks to `normfs`/motor state on
/// behalf of gravity comp - this struct is just task lifecycle management.
#[derive(Default)]
pub struct GravityComp {
    tasks: RwLock<HashMap<BusKey, GravityCompTask>>,
}

impl GravityComp {
    pub fn new() -> Self {
        Self::default()
    }

    /// Starts the control loop for `bus`. No-op if already running (idempotent,
    /// matching `Inference::start`'s style for mirroring).
    ///
    /// `on_self_stop` is invoked (off the control loop's own task) if the loop
    /// terminates itself for safety reasons (stale data or sustained
    /// overcurrent) - the caller uses this to keep displayed mode state
    /// truthful rather than silently drifting from what's actually running.
    /// `initial_gain`, if provided (e.g. from a UI-supplied start command),
    /// overrides `config.gravity_comp.gain_rad_per_nm` as the starting point
    /// - either way it's still hard-clamped and still adjustable afterwards
    /// via `set_gain`.
    pub fn start<F>(&self, bus: BusKey, config: MotorConfig, initial_gain: Option<f64>, normfs: Arc<NormFS>, on_self_stop: F)
    where
        F: Fn(BusKey) + Send + Sync + 'static,
    {
        if self.tasks.read().contains_key(&bus) {
            return;
        }

        // One-shot setup, ahead of the hot loop: cap TorqueLimit, then enable
        // torque, for the 7 arm motors only.
        Self::send_setup_commands(&normfs, &bus.bus_id, config.gravity_comp.torque_limit);

        let gain = Arc::new(RwLock::new(clamp_gravity_comp_gain(
            initial_gain.unwrap_or(config.gravity_comp.gain_rad_per_nm),
        )));

        let stop_flag = Arc::new(AtomicBool::new(false));
        let loop_stop_flag = Arc::clone(&stop_flag);
        let loop_gain = Arc::clone(&gain);
        let loop_bus = bus.clone();
        let loop_normfs = Arc::clone(&normfs);

        let handle = tokio::spawn(async move {
            Self::run_control_loop(loop_bus, loop_stop_flag, loop_gain, loop_normfs, config, on_self_stop).await;
        });

        self.tasks.write().insert(bus, GravityCompTask { stop_flag, gain, handle });
    }

    /// Stops the control loop for `bus` (if running), disables torque, and
    /// restores the default torque limit on the 7 arm motors.
    pub fn stop(&self, bus: &BusKey, normfs: &Arc<NormFS>) {
        if let Some(task) = self.tasks.write().remove(bus) {
            task.stop_flag.store(true, Ordering::SeqCst);
            task.handle.abort();
        }
        Self::send_teardown_commands(normfs, &bus.bus_id);
    }

    /// Live-updates the gain for a running gravity-comp task, without
    /// restarting it. Returns `false` if `bus` has no running task.
    pub fn set_gain(&self, bus: &BusKey, gain: f64) -> bool {
        let tasks = self.tasks.read();
        match tasks.get(bus) {
            Some(task) => {
                *task.gain.write() = clamp_gravity_comp_gain(gain);
                true
            }
            None => false,
        }
    }

    /// Returns the currently active gain for `bus`, or `None` if gravity
    /// comp isn't running on it.
    pub fn get_gain(&self, bus: &BusKey) -> Option<f64> {
        self.tasks.read().get(bus).map(|task| *task.gain.read())
    }

    fn send_setup_commands(normfs: &Arc<NormFS>, bus_id: &str, torque_limit: u16) {
        let mut commands = Vec::new();
        for &motor_id in &ARM_MOTOR_IDS {
            commands.push(Command {
                target_bus_id: bus_id.to_string(),
                motor_id: motor_id as u32,
                command: MotorCommand::TorqueLimit(torque_limit),
            });
        }
        for &motor_id in &ARM_MOTOR_IDS {
            commands.push(Command {
                target_bus_id: bus_id.to_string(),
                motor_id: motor_id as u32,
                command: MotorCommand::Torque(1),
            });
        }
        Inference::send_st3215_commands(normfs, &Bytes::new(), commands);
    }

    fn send_teardown_commands(normfs: &Arc<NormFS>, bus_id: &str) {
        let mut commands = Vec::new();
        for &motor_id in &ARM_MOTOR_IDS {
            commands.push(Command {
                target_bus_id: bus_id.to_string(),
                motor_id: motor_id as u32,
                command: MotorCommand::Torque(0),
            });
        }
        for &motor_id in &ARM_MOTOR_IDS {
            commands.push(Command {
                target_bus_id: bus_id.to_string(),
                motor_id: motor_id as u32,
                command: MotorCommand::TorqueLimit(st3215::presets::DEFAULT_TORQUE_LIMIT),
            });
        }
        Inference::send_st3215_commands(normfs, &Bytes::new(), commands);
    }

    async fn run_control_loop<F>(
        bus: BusKey,
        stop_flag: Arc<AtomicBool>,
        gain: Arc<RwLock<f64>>,
        normfs: Arc<NormFS>,
        config: MotorConfig,
        on_self_stop: F,
    ) where
        F: Fn(BusKey) + Send + Sync + 'static,
    {
        let mut station_state = StationState::default();
        let mut stale_cycles: u32 = 0;
        let mut overcurrent_cycles: u32 = 0;

        loop {
            if stop_flag.load(Ordering::SeqCst) {
                return;
            }

            let loop_start = tokio::time::Instant::now();
            station_state.update_from_st3215_queue(&normfs).await;

            let now_ns = systime::get_monotonic_stamp_ns();
            let bus_state = station_state.buses.get(&bus);

            let fresh = bus_state
                .map(|b| b.monotonic_stamp_ns > 0 && now_ns.saturating_sub(b.monotonic_stamp_ns) < MAX_DATA_AGE_NS)
                .unwrap_or(false);

            if !fresh {
                stale_cycles += 1;
                if stale_cycles >= config.gravity_comp.stale_cutoff_cycles {
                    log::warn!(
                        "Gravity comp on bus {} stopping: stale data for {} consecutive cycles",
                        bus.bus_id,
                        stale_cycles
                    );
                    Self::send_teardown_commands(&normfs, &bus.bus_id);
                    on_self_stop(bus.clone());
                    return;
                }
                Self::sleep_remaining(loop_start).await;
                continue;
            }
            stale_cycles = 0;

            let bus_state = match bus_state {
                Some(bus_state) => bus_state,
                None => {
                    Self::sleep_remaining(loop_start).await;
                    continue;
                }
            };

            let mut theta = [0.0f64; JOINT_COUNT];
            let mut over_current = false;
            let mut have_all_motors = true;

            for (i, &motor_id) in ARM_MOTOR_IDS.iter().enumerate() {
                match bus_state.motors.get(&motor_id) {
                    Some(motor) => {
                        let (lower, upper) = JOINT_LIMITS_RAD[i];
                        theta[i] = control::raw_to_joint_angle(motor.present_position, motor.range_min, motor.range_max, lower, upper, &config);
                        if motor.current >= config.gravity_comp.current_cutoff {
                            over_current = true;
                        }
                    }
                    None => have_all_motors = false,
                }
            }

            if !have_all_motors {
                Self::sleep_remaining(loop_start).await;
                continue;
            }

            if over_current {
                overcurrent_cycles += 1;
                if overcurrent_cycles >= config.gravity_comp.stale_cutoff_cycles {
                    log::warn!(
                        "Gravity comp on bus {} stopping: overcurrent for {} consecutive cycles",
                        bus.bus_id,
                        overcurrent_cycles
                    );
                    Self::send_teardown_commands(&normfs, &bus.bus_id);
                    on_self_stop(bus.clone());
                    return;
                }
                // Hold position this cycle rather than commanding a new goal.
                Self::sleep_remaining(loop_start).await;
                continue;
            }
            overcurrent_cycles = 0;

            let torques = elrobot_dynamics::gravity_torques(&theta);
            let current_gain = *gain.read();
            let mut commands = Vec::new();

            for (i, &motor_id) in ARM_MOTOR_IDS.iter().enumerate() {
                let motor = bus_state.motors.get(&motor_id).expect("checked above");
                let (lower, upper) = JOINT_LIMITS_RAD[i];

                let range_size = normalize::get_steps_range(motor.range_min, motor.range_max, &config);
                if range_size < config.safety_margin * 2 {
                    continue; // not calibrated enough to safely offset this motor
                }

                let tpr = control::ticks_per_radian(motor.range_min, motor.range_max, lower, upper, &config);
                let offset = control::gravity_torque_to_goal_offset_ticks(
                    torques[i],
                    current_gain,
                    config.gravity_comp.max_offset_ticks,
                    tpr,
                );

                let goal = Self::clamp_goal(motor.present_position, offset, motor.range_min, motor.range_max, config.safety_margin, config.max_steps);

                commands.push(Command {
                    target_bus_id: bus.bus_id.clone(),
                    motor_id: motor_id as u32,
                    command: MotorCommand::Goal(goal),
                });
            }

            if !commands.is_empty() {
                Inference::send_st3215_commands(&normfs, &Bytes::new(), commands);
            }

            Self::sleep_remaining(loop_start).await;
        }
    }

    /// Applies the offset to `present`, wrapping into the servo's tick space,
    /// then clamps into the calibrated range minus a safety margin.
    fn clamp_goal(present: u16, offset: i32, range_min: u16, range_max: u16, margin: u16, max_steps: u16) -> u16 {
        let max_steps_i = max_steps as i32;
        let raw = (present as i32 + offset).rem_euclid(max_steps_i);

        if range_max < range_min {
            // Calibrated range wraps across the 0/max_steps boundary - skip
            // clamping in this rarer case and hold position rather than risk
            // an incorrect wraparound comparison.
            return present;
        }

        let lo = range_min as i32 + margin as i32;
        let hi = range_max as i32 - margin as i32;
        if hi <= lo {
            return present;
        }

        raw.clamp(lo, hi) as u16
    }

    async fn sleep_remaining(loop_start: tokio::time::Instant) {
        let elapsed = loop_start.elapsed();
        if elapsed < GRAVITY_COMP_REFRESH_INTERVAL {
            tokio::time::sleep(GRAVITY_COMP_REFRESH_INTERVAL - elapsed).await;
        }
    }
}
