//! This code contains safety-related code, like arming, and lost link procedures.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    flight_ctrls::{
        autopilot::AutopilotStatus,
        common::{AltType, Params},
    },
    // pid::PidGroup,
    ppks::Location,
    state::{SensorStatus, SystemStatus},
};

#[cfg(feature = "fixed-wing")]
use stm32_hal2::{
    gpio::{self, Port},
    pac,
};

#[cfg(feature = "fixed-wing")]
use cfg_if::cfg_if;

// cfg_if! {
//     if #[cfg(feature = "fixed-wing")] {
//     } else {
//     }
// }

use defmt::println;

use num_traits::Float; // abs on float.

// We must receive arm or disarm signals for this many update cycles in a row to perform those actions.
pub const NUM_ARM_DISARM_SIGNALS_REQUIRED: u8 = 5;

// This flag starts false, then is set as soon as we receive a disarm signal with throttle idle.
// Stays set throughout the remaindeer of run. Ensures the device doesn't start in an armed state.
static RECEIVED_INITIAL_DISARM: AtomicBool = AtomicBool::new(false);

// This flag gets set if you command arm from the controller without the throttle in the idle position.
// When this flag is set, the aircraft won't arm until the arm switch is cycled back to safe.
static ARM_COMMANDED_WITHOUT_IDLE: AtomicBool = AtomicBool::new(false);
// static CONTROLLER_PREV_ARMED: AtomicBool = AtomicBool::new(false);

const THROTTLE_MAX_TO_ARM: f32 = 0.005;

// Time in seconds between subsequent data received before we execute lost-link procedures.
pub const LOST_LINK_TIMEOUT: f32 = 1.;

// Altitude to climb to while executing lost link procedure, in meters AGL. This altitude should keep
// it clear of trees, while remaining below most legal drone limits. A higher alt may increase chances
// of req-acquiring the link.
const LOST_LINK_RTB_ALT: f32 = 100.;

// A/C mus be within this altitude of the commanded alt (ie `LOST_LINK_RTB_ALT`) before proceeding
// towards base etc.
const ALT_EPSILON_BEFORE_LATERAL: f32 = 20.;

// If power has been higher than this power level for this time, consider teh craft airborne
// for the purposes of the attitude lock.
const TAKEOFF_POWER_THRESH: f32 = 0.3;
const TAKEOFF_POWER_TIME: f32 = 1.;

// Block RX reception of packets coming in at a faster rate then this. This prevents external
// sources from interfering with other parts of the application by taking too much time.
// Note that we expect a 500hz packet rate for control channel data.
pub const MAX_RF_UPDATE_RATE: f32 = 800.; // Hz

// For abstracting over fixed-wing 3-position vs quad 2-position arm status.
#[cfg(feature = "quad")]
const MOTORS_ARMED: ArmStatus = ArmStatus::Armed;
#[cfg(feature = "fixed-wing")]
const MOTORS_ARMED: ArmStatus = ArmStatus::MotorsControlsArmed;

/// Indicates master motor arm status. Used for both pre arm, and arm. If either is
/// set to `Disarmed`, the motors will not spin (or stop spinning immediately).
/// Repr u8 is for passing over USB serial.
#[cfg(feature = "quad")]
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ArmStatus {
    /// Motors are disarmed
    Disarmed = 0,
    /// Motors are [pre]armed
    Armed = 1,
}

#[cfg(feature = "fixed-wing")]
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ArmStatus {
    /// Motors are and control surfaces aredisarmed
    Disarmed = 0,
    /// Control surfaces armed; motors disarmed
    ControlsArmed = 1,
    /// Motors and control surface are armed.
    MotorsControlsArmed = 2,
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::Disarmed
    }
}

#[cfg(feature = "fixed-wing")]
/// Enable servos, by resetting its pins.
fn enable_servos() {
    // todo: Pass pin args.
    let alt_mode = 0b10;
    cfg_if! {
        if #[cfg(feature = "h7")] {
            unsafe {
                (*pac::GPIOC::ptr()).moder.modify(|_, w| {
                    w.moder8().bits(alt_mode);
                    w.moder9().bits(alt_mode)
                });
            }

        } else {
            unsafe {
                (*pac::GPIOB::ptr()).moder.modify(|_, w| {
                    w.moder0().bits(alt_mode);
                    w.moder1().bits(alt_mode)
                });
            }

        }
    }
}

#[cfg(feature = "fixed-wing")]
/// Disble servos, by setting mode to output, and forcing low. # todo: high?
/// This helps prevent a cutoff pulse from driving servos beyond a control surface's acceptable
/// range
fn disable_servos() {
    // todo: Pass pin args.
    let out_mode = 0b01;

    cfg_if! {
        if #[cfg(feature = "h7")] {
            unsafe {
                (*pac::GPIOC::ptr()).moder.modify(|_, w| {
                    w.moder8().bits(out_mode);
                    w.moder9().bits(out_mode)
                });
            }

            // todo: Set low?
            // Set high, since setting low might cut off a pulse.
           gpio::set_high(Port::C, 8); // Ch 3
           gpio::set_high(Port::C, 9); // Ch 4
        } else {
            unsafe {
                (*pac::GPIOB::ptr()).moder.modify(|_, w| {
                    w.moder0().bits(out_mode);
                    w.moder1().bits(out_mode)
                });
            }

           gpio::set_high(Port::B, 0); // Ch 3
           gpio::set_high(Port::B, 1); // Ch 4
        }
    }
}

/// Arm or disarm the arm state (and therefor the motors, based on arm switch status and throttle.
/// Arm switch must be set while throttle is idle.
pub fn handle_arm_status(
    arm_signals_received: &mut u8,
    disarm_signals_received: &mut u8,
    controller_arm_status: ArmStatus,
    arm_status: &mut ArmStatus,
    throttle: f32,
) {
    match arm_status.clone() {
        MOTORS_ARMED => {
            if controller_arm_status != MOTORS_ARMED {
                *disarm_signals_received += 1;
            } else {
                *disarm_signals_received = 0;
            }

            if *disarm_signals_received >= NUM_ARM_DISARM_SIGNALS_REQUIRED {
                *disarm_signals_received = 0;

                // On fixed, this could be either disarmed, or controls armed.
                *arm_status = controller_arm_status;

                // Reset integrator on rate PIDs, for example so the value from one flight doesn't
                // affect the next.
                // pid_rate.reset_integrator();
                // pid_attitude.reset_integrator();
                // pid_velocity.reset_integrator();

                println!("Aircraft motors disarmed.");
            }

            #[cfg(feature = "fixed-wing")]
            enable_servos();
        }
        ArmStatus::Disarmed => {
            if controller_arm_status == MOTORS_ARMED {
                *arm_signals_received += 1;
            } else {
                RECEIVED_INITIAL_DISARM.store(true, Ordering::Release);
                ARM_COMMANDED_WITHOUT_IDLE.store(false, Ordering::Release);
                *arm_signals_received = 0;
            }


            if *arm_signals_received >= NUM_ARM_DISARM_SIGNALS_REQUIRED {
                *arm_signals_received = 0;

                if !ARM_COMMANDED_WITHOUT_IDLE.load(Ordering::Acquire) {
                    if throttle < THROTTLE_MAX_TO_ARM {
                        if !RECEIVED_INITIAL_DISARM.load(Ordering::Acquire) {
                            println!(
                                "Arm/idle commanded without receiving initial throttle idle and \
                            disarm signal."
                            );
                        } else {
                            *arm_status = MOTORS_ARMED;
                            println!("Aircraft motors armed.");
                        }
                    } else {
                        // Throttle not idle; reset the process, and set the flag requiring
                        // arm switch cycle to arm.
                        ARM_COMMANDED_WITHOUT_IDLE.store(true, Ordering::Release);
                        // println!("Arm attempted without Throttle not idle; set idle and cycle arm switch to arm.");
                    }
                } else {
                    println!("Arm/idle commanded while throttle is not idle");
                }
            }

            #[cfg(feature = "fixed-wing")]
            disable_servos();
        }
        #[cfg(feature = "fixed-wing")]
        ArmStatus::ControlsArmed => {
            enable_servos();
        }
    }
}

/// If we are lost link haven't received a radio signal in a certain amount of time, execute a lost-link
/// procedure. This function behaves differently depending on if we've just entered it, or if
/// we're in a steady-state.
pub fn link_lost(
    system_status: &SystemStatus,
    autopilot_status: &mut AutopilotStatus,
    // entering_lost_link: bool,
    params: &Params,
    base_pt: &Location,
) {
    // todo: Consider how you want to handle this, with and without GPS.

    // todo: To start, command an attitude-mode hover, with baro alt hold.

    // todo: Make sure you resume flight once link is re-acquired.
    // }

    autopilot_status.alt_hold = Some((AltType::Msl, LOST_LINK_RTB_ALT));

    #[cfg(feature = "quad")]
    if system_status.gps == SensorStatus::Pass {
        if (params.baro_alt_msl - LOST_LINK_RTB_ALT).abs() < ALT_EPSILON_BEFORE_LATERAL {
            autopilot_status.direct_to_point = Some(base_pt.clone());
        }
    }

    #[cfg(feature = "fixed-wing")]
    if system_status.gps == SensorStatus::Pass {
    } else if system_status.magnetometer == SensorStatus::Pass {
        if (params.baro_alt_msl - LOST_LINK_RTB_ALT).abs() < ALT_EPSILON_BEFORE_LATERAL {
            autopilot_status.direct_to_point = Some(base_pt.clone());
        }

        // todo: Store lost-link heading somewhere (probably a LostLinkStatus struct etc)
        // Climb with reverse heading if no GPS available.
        // Note that quadcopter movements may be too unstable to attempt this.
    }
}

/// Unlock the takeoff attitude lock if motor power has exceed a certain power level for a
/// certain amount of time. This is done by changing the `has_taken_off` variable.
pub fn handle_takeoff_attitude_lock(
    throttle: f32,
    time_with_high_throttle: &mut f32,
    has_taken_off: &mut bool,
    dt: f32,
) {
    if throttle >= TAKEOFF_POWER_THRESH {
        if *time_with_high_throttle >= TAKEOFF_POWER_TIME {
            *has_taken_off = true;
            *time_with_high_throttle = 0.;
            return;
        }
        *time_with_high_throttle += dt;
    } else {
        *time_with_high_throttle = 0.;
    }
}
