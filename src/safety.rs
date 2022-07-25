//! This code contains safety-related code, like arming, and lost link procedures.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{control_interface::ChannelData, flight_ctrls::quad::InputMode, pid::PidGroup};

use defmt::println;

// We must receive arm or disarm signals for this many update cycles in a row to perform those actions.
pub const NUM_ARM_DISARM_SIGNALS_REQUIRED: u8 = 5;

// This flag starts false, then is set as soon as we receive a disarm signal with throttle idle.
// Stays set throughout the remaindeer of run. Ensures the device doesn't start in an armed state.
static RECEIVED_INITIAL_DISARM: AtomicBool = AtomicBool::new(false);

// This flag gets set if you command arm from the controller without the throttle in the idle position.
// When this flag is set, the aircraft won't arm until the arm switch is cycled back to safe.
static ARM_COMMANDED_WITHOUT_IDLE: AtomicBool = AtomicBool::new(false);
// static CONTROLLER_PREV_ARMED: AtomicBool = AtomicBool::new(false);

pub static LINK_LOST: AtomicBool = AtomicBool::new(false);

const THROTTLE_MAX_TO_ARM: f32 = 0.005;

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
    pid_rate: &mut PidGroup,
    pid_attitude: &mut PidGroup,
    pid_velocity: &mut PidGroup,
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
                pid_rate.reset_integrator();
                pid_attitude.reset_integrator();
                pid_velocity.reset_integrator();

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

/// If we haven't received a radio control signal in a while, perform an action.
/// todo: Immediate actions, or each update loop while link is still lost?
pub fn link_lost_steady_state(
    input_mode: &mut InputMode,
    control_ch_data: &mut ChannelData,
    arm_status: &mut ArmStatus,
    arm_signals_receieved: &mut u8,
) {
    // println!("Handling lost link...")
    // if !timer.is_enabled() {
    //     println!("Lost link to Rx control. Recovering...")
    // todo: Consider how you want to handle this, with and without GPS.

    // todo: To start, command an attitude-mode hover, with baro alt hold.

    // todo: Make sure you resume flight once link is re-acquired.
    // }

    // todo: Only execute a climb if GPS is avail, and are a certain distance away from homestation.

    *input_mode = InputMode::Attitude;
    control_ch_data.pitch = 0.;
    control_ch_data.roll = 0.;
    control_ch_data.yaw = 0.;
    // todo temp!
    // todo: The above plus a throttle control, and attitude mode etc

    control_ch_data.throttle = 0.0; // todo: Drops out of sky for now while we test.
    *arm_status = ArmStatus::Disarmed;
    *arm_signals_receieved = 0;
}
