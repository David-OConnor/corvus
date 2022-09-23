//! This code contains safety-related code, like arming, and lost link procedures.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{
    control_interface::ChannelData,
    flight_ctrls::{
        autopilot::AutopilotStatus,
        common::{AltType, Params},
    },
    // pid::PidGroup,
    ppks::{Location, LocationType},
    state::{SensorStatus, SystemStatus},
};

use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
    } else {
        use crate::flight_ctrls::InputMode;
    }
}

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

// Altitude to climb to while executing lost link procedure, in meters AGL. This altitude should keep
// it clear of trees, while remaining below most legal drone limits. A higher alt may increase chances
// of req-acquiring the link.
const LOST_LINK_RTB_ALT: f32 = 100.;

// A/C mus be within this altitude of the commanded alt (ie `LOST_LINK_RTB_ALT`) before proceeding
// towards base etc.
const ALT_EPSILON_BEFORE_LATERAL: f32 = 20.;

/// Indicates master motor arm status. Used for both pre arm, and arm. If either is
/// set to `Disarmed`, the motors will not spin (or stop spinning immediately).
/// Repr u8 is for passing over USB serial.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum ArmStatus {
    /// Motors are [pre]disarmed
    Disarmed = 0,
    /// Motors are [pre]armed
    Armed = 1,
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::Disarmed
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
    // pid_rate: &mut PidGroup,
    // pid_attitude: &mut PidGroup,
    // pid_velocity: &mut PidGroup,
) {
    // println!("arm rec: {:?}",  arm_signals_received);
    match arm_status {
        ArmStatus::Armed => {
            if controller_arm_status == ArmStatus::Disarmed {
                *disarm_signals_received += 1;
            } else {
                *disarm_signals_received = 0;
            }

            if *disarm_signals_received >= NUM_ARM_DISARM_SIGNALS_REQUIRED {
                *arm_status = ArmStatus::Disarmed;
                *disarm_signals_received = 0;

                // Reset integrator on rate PIDs, for example so the value from one flight doesn't
                // affect the next.
                // pid_rate.reset_integrator();
                // pid_attitude.reset_integrator();
                // pid_velocity.reset_integrator();

                println!("Aircraft disarmed.");
            }
        }
        ArmStatus::Disarmed => {
            if controller_arm_status == ArmStatus::Armed {
                *arm_signals_received += 1;
            } else {
                RECEIVED_INITIAL_DISARM.store(true, Ordering::Release);
                ARM_COMMANDED_WITHOUT_IDLE.store(false, Ordering::Release);
                *arm_signals_received = 0;
            }

            if *arm_signals_received >= NUM_ARM_DISARM_SIGNALS_REQUIRED {
                if !ARM_COMMANDED_WITHOUT_IDLE.load(Ordering::Acquire) {
                    if throttle < THROTTLE_MAX_TO_ARM {
                        if !RECEIVED_INITIAL_DISARM.load(Ordering::Acquire) {
                            println!(
                                "Arm/idle commadned without receiving initial throttle idle and \
                            disarm signal."
                            );
                        } else {
                            *arm_status = ArmStatus::Armed;
                            *arm_signals_received = 0;
                            println!("Aircraft armed.");
                        }
                    } else {
                        // Throttle not idle; reset the process, and set the flag requiring
                        // arm switch cycle to arm.
                        ARM_COMMANDED_WITHOUT_IDLE.store(true, Ordering::Release);
                        *arm_signals_received = 0;
                        // println!("Arm attempted without Throttle not idle; set idle and cycle arm switch to arm.");
                    }
                } else {
                    // println!("(Cycle arm switch to arm.)");
                }
            }
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
