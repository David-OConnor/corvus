#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c (and pid.h)

use core::mem;

const EPS: f32 = 0.000001; // c uses float == 0 comparison?

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


// todo types
const MAX_PID_PROCESS_DENOM: u16 =        16;
const PID_CONTROLLER_BETAFLIGHT: u16 =    1;
const PID_MIXER_SCALING  : f32 =         1000.0;
const PID_SERVO_MIXER_SCALING : f32 =    0.7;
const PIDSUM_LIMIT: u16 =                 500;
const PIDSUM_LIMIT_YAW: u16 =             400;
const PIDSUM_LIMIT_MIN: u16 =             100;
const PIDSUM_LIMIT_MAX: u16 =            1000;

const PID_GAIN_MAX: u16 = 250;
const F_GAIN_MAX: u16 =  1000;
const D_MIN_GAIN_MAX: u16 =  250;

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
const PTERM_SCALE: f32 = 0.032029;
const ITERM_SCALE: f32 = 0.244381;
const DTERM_SCALE: f32= 0.000529;

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
const FEEDFORWARD_SCALE: f32 = 0.013754;

// Full iterm suppression in setpoint mode at high-passed setpoint rate > 40deg/sec
const ITERM_RELAX_SETPOINT_THRESHOLD: f32 = 40.0;
const ITERM_RELAX_CUTOFF_DEFAULT: u8 =  15;

// Anti gravity I constant
const AG_KI: f32 = 21.586988;

const ITERM_ACCELERATOR_GAIN_OFF: u16 =  0;
const ITERM_ACCELERATOR_GAIN_MAX: u16 = 30000;
const PID_ROLL_DEFAULT: [u8; 4] =  [ 45, 80, 40, 120 ];
const PID_PITCH_DEFAULT: [u8; 4] = [ 47, 84, 46, 125 ];
const PID_YAW_DEFAULT: [u8; 4] =   [ 45, 80,  0, 120 ];
const D_MIN_DEFAULT: [u8; 3] =     [ 30, 34, 0 ];

const DTERM_LPF1_DYN_MIN_HZ_DEFAULT: u8 = 75;
const DTERM_LPF1_DYN_MAX_HZ_DEFAULT: u8 = 150;
const DTERM_LPF2_HZ_DEFAULT: u8 = 150;

#[derive(Clone, Copy, PartialEq)]
enum pidIndex {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
}

#[derive(Clone, Copy, PartialEq)]
enum pidSuperExpoYaw {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
}

#[derive(Clone, Copy, PartialEq)]
enum pidStabilisationState {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
}

#[derive(Clone, Copy, PartialEq)]
enum pidCrashRecovery {
    PID_CRASH_RECOVERY_OFF = 0,
    PID_CRASH_RECOVERY_ON,
    PID_CRASH_RECOVERY_BEEP,
    PID_CRASH_RECOVERY_DISARM,
}

struct pidf {
    P: u8,
    I: u8,
    D: u8,
    F: u16,
}

#[derive(Clone, Copy, PartialEq)]
enum antiGravityMode {
    ANTI_GRAVITY_SMOOTH,
    ANTI_GRAVITY_STEP
}

#[derive(Clone, Copy, PartialEq)]
enum itermRelax {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
    ITERM_RELAX_RP_INC,
    ITERM_RELAX_RPY_INC,
    ITERM_RELAX_COUNT,
}

#[derive(Clone, Copy, PartialEq)]
enum itermRelaxType {
    ITERM_RELAX_GYRO,
    ITERM_RELAX_SETPOINT,
    ITERM_RELAX_TYPE_COUNT,
}

#[derive(Clone, Copy, PartialEq)]
enum feedforwardAveraging {
    FEEDFORWARD_AVERAGING_OFF,
    FEEDFORWARD_AVERAGING_2_POINT,
    FEEDFORWARD_AVERAGING_3_POINT,
    FEEDFORWARD_AVERAGING_4_POINT,
}

const MAX_PROFILE_NAME_LENGTH: u8 = 0;

struct pidProfile {
    yaw_lowpass_hz: u16,                // Additional yaw filter when yaw axis too noisy
    dterm_lpf1_static_hz: u16,          // Static Dterm lowpass 1 filter cutoff value in hz
    dterm_notch_hz: u16,                // Biquad dterm notch hz
    dterm_notch_cutoff: u16,             // Biquad dterm notch low cutoff

    pid[pidf; PID_ITEM_COUNT],

    dterm_lpf1_type: u8,                // Filter type for dterm lowpass 1
    itermWindupPointPercent: u8,        // iterm windup threshold, percent motor saturation
    pidSumLimit: u16, 
    pidSumLimitYaw: u16, 
    pidAtMinThrottle: u8,               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    levelAngleLimit: u8,                // Max angle in degrees in level mode

    horizon_tilt_effect: u8,            // inclination factor for Horizon mode
    horizon_tilt_expert_mode: u8,       // OFF or ON

    // Betaflight PID controller parameters
     antiGravityMode: u8,             // type of anti gravity method
    itermThrottleThreshold: u16,         // max allowed throttle delta before iterm accelerated in ms
    itermAcceleratorGain: u16,           // Iterm Accelerator Gain when itermThrottlethreshold is hit
    yawRateAccelLimit: u16,              // yaw accel limiter for deg/sec/ms
    rateAccelLimit: u16,                // accel limiter roll/pitch deg/sec/ms
    crash_dthreshold: u16,               // dterm crash value
    crash_gthreshold: u16,               // gyro crash value
    crash_setpoint_threshold: u16,       // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    crash_time: u16,                   // ms
    crash_delay: u16,                  // ms
    crash_recovery_angle,           // degrees
    crash_recovery_rate,            // degree/second
    crash_limit_yaw: u16,             // limits yaw errorRate, so crashes don't cause huge throttle increase
    itermLimit: u16, 
    dterm_lpf2_static_hz: u16,          // Static Dterm lowpass 2 filter cutoff value in hz
    crash_recovery: u8,                 // off, on, on and beeps when it is in crash recovery mode
    throttle_boost: u8,                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    throttle_boost_cutoff: u8,          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    iterm_rotation: u8,                 // rotates iterm to translate world errors to local coordinate system
    iterm_relax_type: u8,               // Specifies type of relax algorithm
    iterm_relax_cutoff: u8,             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    iterm_relax: u8,                    // Enable iterm suppression during stick input
    acro_trainer_angle_limit: u8,       // Acro trainer roll/pitch angle limit in degrees
    acro_trainer_debug_axis: u8,        // The axis for which record debugging values are captured 0=roll, 1=pitch
    acro_trainer_gain: u8,              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    acro_trainer_lookahead_ms: u16,      // The lookahead window in milliseconds used to reduce overshoot
    abs_control_gain: u8,               // How strongly should the absolute accumulated error be corrected for
    abs_control_limit: u8,              // Limit to the correction
    abs_control_error_limit: u8,        // Limit to the accumulated error
    abs_control_cutoff: u8,             // Cutoff frequency for path estimation in abs control
    dterm_lpf2_type: u8,                // Filter type for 2nd dterm lowpass
    dterm_lpf1_dyn_min_hz: u16,          // Dterm lowpass filter 1 min hz when in dynamic mode
    dterm_lpf1_dyn_max_hz: u16,          // Dterm lowpass filter 1 max hz when in dynamic mode
    launchControlMode: u8,              // Whether launch control is limited to pitch only (launch stand or top-mount) or all axes (on battery)
    launchControlThrottlePercent: u8,   // Throttle percentage to trigger launch for launch control
    launchControlAngleLimit: u8,        // Optional launch control angle limit (requires ACC)
    launchControlGain: u8,              // Iterm gain used while launch control is active
    launchControlAllowTriggerReset: u8, // Controls trigger behavior and whether the trigger can be reset
    use_integrated_yaw: u8,             // Selects whether the yaw pidsum should integrated
    integrated_yaw_relax: u8,           // Specifies how much integrated yaw should be reduced to offset the drag based yaw component
    thrustLinearization,            // Compensation factor for pid linearization
    d_min[XYZ_AXIS_COUNT]: u8,          // Minimum D value on each axis
    d_min_gain: u8,                     // Gain factor for amount of gyro / setpoint activity required to boost D
    d_min_advance: u8,                  // Percentage multiplier for setpoint input to boost algorithm
    motor_output_limit: u8,             // Upper limit of the motor output (percent)
    auto_profile_cell_count: i8,         // Cell count for this profile to be used with if auto PID profile switching is used
    transient_throttle_limit: u8,       // Maximum DC component of throttle change to mix into throttle to prevent airmode mirroring noise
    profileName: [u8; MAX_PROFILE_NAME_LENGTH + 1], // Descriptive name for profile

    dyn_idle_min_rpm: u8,                   // minimum motor speed enforced by the dynamic idle controller
    dyn_idle_p_gain: u8,                // P gain during active control of rpm
    dyn_idle_i_gain: u8,                // I gain during active control of rpm
    dyn_idle_d_gain: u8,                // D gain for corrections around rapid changes in rpm
    dyn_idle_max_increase: u8,          // limit on maximum possible increase in motor idle drive during active control

    feedforward_transition: u8,         // Feedforward attenuation around centre sticks
    feedforward_averaging: u8,          // Number of packets to average when averaging is on
    feedforward_smooth_factor: u8,      // Amount of lowpass type smoothing for feedforward steps
    feedforward_jitter_factor: u8,      // Number of RC steps below which to attenuate feedforward
    feedforward_boost: u8,              // amount of setpoint acceleration to add to feedforward, 10 means 100% added
    feedforward_max_rate_limit: u8,     // Maximum setpoint rate percentage for feedforward

    dterm_lpf1_dyn_expo: u8,            // set the curve for dynamic dterm lowpass filter
    level_race_mode: u8,                // NFE race mode - when true pitch setpoint calculation is gyro based in level mode
    vbat_sag_compensation: u8,          // Reduce motor output by this percentage of the maximum compensation amount

    simplified_pids_mode: u8,
    simplified_master_multiplier: u8,
    simplified_roll_pitch_ratio: u8,
    simplified_i_gain: u8,
    simplified_d_gain: u8,
    simplified_pi_gain: u8,
    simplified_dmin_ratio: u8,
    simplified_feedforward_gain: u8,
    simplified_dterm_filter: u8,
    simplified_dterm_filter_multiplier: u8,
    simplified_pitch_pi_gain: u8,
}

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

struct pidConfig {
    pid_process_denom: u8,             // Processing denominator for PID controller vs gyro sampling rate
    runaway_takeoff_prevention: u8,         // off, on - enables pidsum runaway disarm logic
    runaway_takeoff_deactivate_delay: u16,   // delay in ms for "in-flight" conditions before deactivation (successful flight)
    runaway_takeoff_deactivate_throttle: u8, // minimum throttle percent required during deactivation phase
}

// PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

struct pidAxisData {
    P: f32,
    I: f32,
    D: f32,
    F: f32,

    Sum: f32,
}

// #[repr(C)]
// union dtermLowpass {
//     pt1Filter_t pt1Filter;
//     biquadFilter_t biquadFilter;
//     pt2Filter_t pt2Filter;
//     pt3Filter_t pt3Filter;
// }

struct pidCoefficient {
    Kp: f32,
    Ki: f32,
    Kd: f32,
    Kf: f32,
}

struct pidRuntime {
    dT: f32,
    pidFrequency: f32,
    pidStabilisationEnabled: bool,
    previousPidSetpoint: [f32; XYZ_AXIS_COUNT],
    dtermNotchApplyFn: filterApplyFnPtr,
    dtermNotch: [biquadFilter; XYZ_AXIS_COUNT];
    dtermLowpassApplyFn: filterApplyFnPtr,
    dtermLowpass: [dtermLowpass; XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    dtermLowpass2: [dtermLowpass; XYZ_AXIS_COUNT];
    ptermYawLowpassApplyFn: filterApplyFnPtr,
    ptermYawLowpass: pt1Filter,
    antiGravityEnabled: bool,
    antiGravityMode: u8,
    antiGravityThrottleLpf: pt1Filter,
    antiGravitySmoothLpf: Pt1Filter,
    antiGravityOsdCutoff: f32,
    antiGravityThrottleHpf: f32,
    antiGravityPBoost: f32,
    itermAccelerator: f32,
    itermAcceleratorGain: u16,
    pidCoefficient: [pidCoefficient; XYZ_AXIS_COUNT];
    levelGain: f32,
    horizonGain: f32,
    horizonTransition: f32,
    horizonCutoffDegrees: f32,
    horizonFactorRatio: f32,
    horizonTiltExpertMode: u8,
    maxVelocity: [f32; XYZ_AXIS_COUNT],
    itermWindupPointInv: f32,
    inCrashRecoveryMode: bool,
    crashDetectedAtUs: timeUs,
    crashTimeLimitUs: timeDelta,
    crashTimeDelayUs: timeDelta,
    crashRecoveryAngleDeciDegrees: i32,
    crashRecoveryRate: f32,
    crashGyroThreshold: f32,
    crashDtermThreshold: f32,
    crashSetpointThreshold: f32,
    crashLimitYaw: f32,
    itermLimit: f32,
    itermRotation: bool,
    zeroThrottleItermReset: bool,
    levelRaceMode: bool,
    tpaFactor: f32,

// #ifdef USE_ITERM_RELAX
    windupLpf: [pt1Filter; XYZ_AXIS_COUNT],
    itermRelax: u8,
    itermRelaxType: u8,
    itermRelaxCutoff: u8,
// #endif

// #ifdef USE_ABSOLUTE_CONTROL
    acCutoff: f32,
    acGain: f32,
    acLimit: f32,
    acErrorLimit: f32,
    acLpf: [pt1Filter; XYZ_AXIS_COUNT],
    oldSetpointCorrection: [f32; XYZ_AXIS_COUNT],
// #endif

// #ifdef USE_D_MIN
    dMinRange: [pt2Filter; XYZ_AXIS_COUNT],
    dMinLowpass: [pt2Filter; XYZ_AXIS_COUNT],
    dMinPercent: [f32; XYZ_AXIS_COUNT],
    dMinGyroGain: f32,
    dMinSetpointGain: f32,
// #endif

// #ifdef USE_AIRMODE_LPF
    airmodeThrottleLpf1: pt1Filter,
    airmodeThrottleLpf2: Pt1Filter,
// #endif

// #ifdef USE_RC_SMOOTHING_FILTER
    feedforwardPt3: [pt3Filter; XYZ_AXIS_COUNT],
    feedforwardLpfInitialized: bool,
    rcSmoothingDebugAxis: u8,
    rcSmoothingFilterType: u8,
// #endif // USE_RC_SMOOTHING_FILTER

// #ifdef USE_ACRO_TRAINER
    acroTrainerAngleLimit: f32,
    acroTrainerLookaheadTime: f32,
    acroTrainerDebugAxis: u8,
    acroTrainerGain: f32,
    acroTrainerActive: bool,
    acroTrainerAxisState: [u32; 2],  // only need roll and pitch // todo: int in c. what type?
// #endif

// #ifdef USE_DYN_LPF
    dynLpfFilter: u8,
    dynLpfMin: u16,
    dynLpfMax: u16,
    dynLpfCurveExpo: u8,
// #endif

// #ifdef USE_LAUNCH_CONTROL
    launchControlMode: u8,
    launchControlAngleLimit: u8,
    launchControlKi: f32,
// #endif

// #ifdef USE_INTEGRATED_YAW_CONTROL
    useIntegratedYaw: bool,
    integratedYawRelax: u8,
// #endif

// #ifdef USE_THRUST_LINEARIZATION
    thrustLinearization: f32,
    throttleCompensateAmount: f32,
// #endif

// #ifdef USE_AIRMODE_LPF
    airmodeThrottleOffsetLimit: f32,
// #endif

// #ifdef USE_FEEDFORWARD
    feedforwardTransitionFactor: f32,
    feedforwardAveraging: feedforwardAveraging,
    feedforwardSmoothFactor: f32,
    feedforwardJitterFactor: f32,
    feedforwardBoostFactor: f32,
// #endif
}

// extern pidRuntime_t pidRuntime;
//
// extern const char pidNames[];
//
// extern pidAxisData_t pidData[3];
//
// extern uint32_t targetPidLooptime;
//
// extern float throttleBoost;
// extern pt1Filter_t throttleLpf;

#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
enum levelMode {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
}

const pidNames: [&str; 5] = [
    "ROLL;",
    "PITCH;",
    "YAW;",
    "LEVEL;",
    "MAG;",
];

static mut targetPidLooptime: u32 = 0;
static mut  pidData: [pidAxisData; XYZ_AXIS_COUNT] = [0; XYZ_AXIS_COUNT];
// static mut pidRuntime = mem::zeroed();

// #if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
// #endif

// #if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
// #endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);


const PID_PROCESS_DENOM_DEFAULT: u8 = 1; // todo type

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
const ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
const ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

const CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

const LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 3);

fn resetPidProfile(pidProfile: &pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = [ 50, 50, 75, 0 ],
            [PID_MAG] =   [ 40, 0, 0, 0 ],
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 100,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 85,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 250,
        .itermAcceleratorGain = 3500,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
            // NOTE: dynamic lpf is enabled by default so this setting is actually
            // overridden and the static lowpass 1 is disabled. We can't set this
            // value to 0 otherwise Configurator versions 10.4 and earlier will also
            // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
        .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_min = D_MIN_DEFAULT,
        .d_min_gain = 37,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 25,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .simplified_pids_mode = PID_SIMPLIFIED_TUNING_RPY,
        .simplified_master_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_roll_pitch_ratio = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_i_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dmin_ratio = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_feedforward_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_pitch_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dterm_filter = true,
        .simplified_dterm_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT,
    );

// #ifndef USE_D_MIN
    pidProfile.pid[PID_ROLL].D = 30;
    pidProfile.pid[PID_PITCH].D = 32;
// #endif
}

fn pgResetFn_pidProfiles(pidProfiles: &[pidProfile])
{
    for profile in pidProfiles {
        resetPidProfile(profile);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
const D_LPF_RAW_SCALE: u8 = 25; // todo: type
const D_LPF_FILT_SCALE: u8 = 22;


impl pidRuntime {

fn pidSetItermAccelerator(newItermAccelerator: f32)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

fn pidOsdAntiGravityActive() -> bool
{
    pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff
}

fn pidStabilisationState(pidControllerState: pidStabilisationState)
{
    unsafe {
        pidRuntime.pidStabilisationEnabled = pidControllerState == PID_STABILISATION_ON;
    }
}

const rcAliasToAngleIndexMap: [angle_index; 2] = [ AI_ROLL, AI_PITCH ];

// #ifdef USE_FEEDFORWARD
fn pidGetFeedforwardTransitionFactor(&self) -> f32
{
    self.feedforwardTransitionFactor
}

fn pidGetFeedforwardSmoothFactor(&self) -> f32
{
    self.feedforwardSmoothFactor
}

fn pidGetFeedforwardJitterFactor(&self) -> f32
{
    self.feedforwardJitterFactor
}

fn pidGetFeedforwardBoostFactor(&self) -> f32
{
    self.feedforwardBoostFactor
}
// #endif

fn pidResetIterm()
{
    for axis in 0..3 {
        pidData[axis].I = 0.0;
// #if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0;
// #endif
    }
}

fn pidUpdateTpaFactor(&mut self, throttle: f32)
{
    let tpaBreakpoint: f32 = (currentControlRateProfile.tpa_breakpoint - 1000) / 1000.0;
    let tpaRate: f32 = currentControlRateProfile.tpa_rate / 100.0;
    if throttle > tpaBreakpoint {
        if throttle < 1.0 {
            tpaRate *= (throttle - tpaBreakpoint) / (1.0 - tpaBreakpoint);
        }
    } else {
        tpaRate = 0.0;
    }
    self.tpaFactor = 1.0 - tpaRate;
}

fn pidUpdateAntiGravityThrottleFilter(&mut self, throttle: f32)
{
    if self.antiGravityMode == ANTI_GRAVITY_SMOOTH {
        // calculate a boost factor for P in the same way as for I when throttle changes quickly
        const float antiGravityThrottleLpf = pt1FilterApply(&self.antiGravityThrottleLpf, throttle);
        // focus P boost on low throttle range only
        if throttle < 0.5 {
            self.antiGravityPBoost = 0.5 - throttle;
        } else {
            self.antiGravityPBoost = 0.0;
        }
        // use lowpass to identify start of a throttle up, use this to reduce boost at start by half
        if antiGravityThrottleLpf < throttle {
            self.antiGravityPBoost *= 0.5;
        }
        // high-passed throttle focuses boost on faster throttle changes
        self.antiGravityThrottleHpf = fabsf(throttle - antiGravityThrottleLpf);
        self.antiGravityPBoost = self.antiGravityPBoost * self.antiGravityThrottleHpf;
        // smooth the P boost at 3hz to remove the jagged edges and prolong the effect after throttle stops
        self.antiGravityPBoost = pt1FilterApply(&self.antiGravitySmoothLpf, self.antiGravityPBoost);
    }
}

// #ifdef USE_ACRO_TRAINER
// void pidAcroTrainerInit(void)
// {
//     pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
//     pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
// }
// #endif // USE_ACRO_TRAINER

// #ifdef USE_THRUST_LINEARIZATION
fn pidCompensateThrustLinearization(&self, throttle: f32) -> f32
{
    if self.thrustLinearization != 0.0 {
        // for whoops where a lot of TL is needed, allow more throttle boost
        let throttleReversed = 1.0 - throttle;
        throttle /= 1.0 + pidRuntime.throttleCompensateAmount * powf(throttleReversed, 2);
    }
    return throttle;
}

fn pidApplyThrustLinearization(motorOutput: f32) -> f32
{
    if pidRuntime.thrustLinearization != 0. {
        if motorOutput > 0.0 {
            let motorOutputReversed: f32 = (1.0 - motorOutput);
            motorOutput *= 1.0 + powf(motorOutputReversed, 2) * pidRuntime.thrustLinearization;
        }
    }
    return motorOutput;
}
// #endif

#if defined(USE_ACC)
// calculate the stick deflection while applying level mode expo
static float getLevelModeRcDeflection(uint8_t axis)
{
    const float stickDeflection = getRcDeflection(axis);
    if (axis < FD_YAW) {
        const float expof = currentControlRateProfile->levelExpo[axis] / 100.0f;
        return power3(stickDeflection) * expof + stickDeflection * (1 - expof);
    } else {
        return stickDeflection;
    }
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(fabsf(getLevelModeRcDeflection(FD_ROLL)), fabsf(getLevelModeRcDeflection(FD_PITCH)));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (pidRuntime.horizonTiltExpertMode) {
        if (pidRuntime.horizonTransition > 0 && pidRuntime.horizonCutoffDegrees > 0) {
                    // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((pidRuntime.horizonCutoffDegrees-currentInclination) / pidRuntime.horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / pidRuntime.horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (pidRuntime.horizonFactorRatio < 1.0f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180 - currentInclination) / 180 * (1.0f - pidRuntime.horizonFactorRatio) + pidRuntime.horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = pidRuntime.horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = pidRuntime.horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    float angle = pidProfile->levelAngleLimit * getLevelModeRcDeflection(axis);
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * pidRuntime.levelGain;
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * pidRuntime.horizonGain * horizonLevelStrength);
    }
    return currentPidSetpoint;
}

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -pidRuntime.crashLimitYaw, pidRuntime.crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * pidRuntime.levelGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset iterm, since accumulated error before crash is now meaningless
        // and iterm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < pidRuntime.crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < pidRuntime.crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < pidRuntime.crashRecoveryAngleDeciDegrees) {
                    pidRuntime.inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !pidRuntime.inCrashRecoveryMode
                && fabsf(delta) > pidRuntime.crashDtermThreshold
                && fabsf(errorRate) > pidRuntime.crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm(DISARM_REASON_CRASH_PROTECTION);
                } else {
                    pidRuntime.inCrashRecoveryMode = true;
                    pidRuntime.crashDetectedAtUs = currentTimeUs;
                }
            }
            if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) < pidRuntime.crashTimeDelayUs && (fabsf(errorRate) < pidRuntime.crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > pidRuntime.crashSetpointThreshold)) {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (pidRuntime.inCrashRecoveryMode) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

// todo: These all should probably be methods on pidruntime.

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (pidRuntime.itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingFeedforwardFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == pidRuntime.rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (pidRuntime.feedforwardLpfInitialized) {
        ret = pt3FilterApply(&pidRuntime.feedforwardPt3[axis], pidSetpointDelta);
        if (axis == pidRuntime.rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&pidRuntime.acLpf[axis], *currentPidSetpoint);
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * pidRuntime.dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidRuntime.pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        if (isAirmodeActivated()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * pidRuntime.dT,
                -pidRuntime.acErrorLimit, pidRuntime.acErrorLimit);
            const float acCorrection = constrainf(axisError[axis] * pidRuntime.acGain, -pidRuntime.acLimit, pidRuntime.acLimit);
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }
        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif

// #ifdef USE_AIRMODE_LPF
fn pidUpdateAirmodeLpf(currentOffset: f32)
{
    if (pidRuntime.airmodeThrottleOffsetLimit.abs() < EPS) {
        return;
    }

    float offsetHpf = currentOffset * 2.5f;
    offsetHpf = offsetHpf - pt1FilterApply(&pidRuntime.airmodeThrottleLpf2, offsetHpf);

    // During high frequency oscillation 2 * currentOffset averages to the offset required to avoid mirroring of the waveform
    pt1FilterApply(&pidRuntime.airmodeThrottleLpf1, offsetHpf);
    // Bring offset up immediately so the filter only applies to the decline
    if (currentOffset * pidRuntime.airmodeThrottleLpf1.state >= 0 && fabsf(currentOffset) > pidRuntime.airmodeThrottleLpf1.state) {
        pidRuntime.airmodeThrottleLpf1.state = currentOffset;
    }
    pidRuntime.airmodeThrottleLpf1.state = constrainf(pidRuntime.airmodeThrottleLpf1.state, -pidRuntime.airmodeThrottleOffsetLimit, pidRuntime.airmodeThrottleOffsetLimit);
}

fn pidGetAirmodeThrottleOffset() -> f32
{
    unsfe {
        pidRuntime.airmodeThrottleLpf1.state
    }
}
// #endif

// #ifdef USE_LAUNCH_CONTROL
const LAUNCH_CONTROL_MAX_RATE: f32 = 100.0;
const LAUNCH_CONTROL_MIN_RATE: f32 = 5.0;
const LAUNCH_CONTROL_ANGLE_WINDOW: f32 = 10.0;  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
#endif

#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

#ifdef USE_TPA_MODE
    const float tpaFactorKp = (currentControlRateProfile->tpaMode == TPA_MODE_PD) ? pidRuntime.tpaFactor : 1.0f;
#else
    const float tpaFactorKp = pidRuntime.tpaFactor;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();

#if defined(USE_ACC)
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        if (pidRuntime.levelRaceMode && !gpsRescueIsActive) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
    }

    // Keep track of when we entered a self-level mode so that we can
    // add a guard time before crash recovery can activate.
    // Also reset the guard time whenever GPS Rescue is activated.
    if (levelMode) {
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }
    gpsRescuePreviousState = gpsRescueIsActive;
#endif

    // Dynamic i component,
    if ((pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) && pidRuntime.antiGravityEnabled) {
        // traditional itermAccelerator factor for iTerm
        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleHpf * 0.01f * pidRuntime.itermAcceleratorGain;
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(pidRuntime.itermAccelerator * 1000));
        // users AG Gain changes P boost
        pidRuntime.antiGravityPBoost *= pidRuntime.itermAcceleratorGain;
        // add some percentage of that slower, longer acting P boost factor to prolong AG effect on iTerm
        pidRuntime.itermAccelerator += pidRuntime.antiGravityPBoost * 0.05f;
        // set the final P boost amount
        pidRuntime.antiGravityPBoost *= 0.02f;
    } else {
        pidRuntime.antiGravityPBoost = 0.0f;
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(pidRuntime.itermAccelerator * 1000));

    float agGain = pidRuntime.dT * pidRuntime.itermAccelerator * AG_KI;

    // gradually scale back integration when above windup point
    float dynCi = pidRuntime.dT;
    if (pidRuntime.itermWindupPointInv > 1.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }

    // Precalculate gyro deta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
        // -----calculate raw, unfiltered D component

        // Divide rate change by dT to get differential (ie dr/dt).
        // dT is fixed and calculated from the target PID loop time
        // This is done to avoid DTerm spikes that occur with dynamically
        // calculated deltaT whenever another task causes the PID
        // loop execution to be delayed.
        const float delta =
            - (gyroRateDterm[axis] - previousRawGyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
        previousRawGyroRateDterm[axis] = gyroRateDterm[axis];

        // Log the unfiltered D
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_D_LPF, 0, lrintf(delta));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_D_LPF, 1, lrintf(delta));
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_FEEDFORWARD
    bool newRcFrame = false;
    if (getShouldUpdateFeedforward()) {
        newRcFrame = true;
    }
#endif

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        float currentPidSetpoint = getSetpointRate(axis);
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
#if defined(USE_ACC)
        switch (levelMode) {
        case LEVEL_MODE_OFF:

            break;
        case LEVEL_MODE_R:
            if (axis == FD_PITCH) {
                break;
            }

            FALLTHROUGH;
        case LEVEL_MODE_RP:
            if (axis == FD_YAW) {
                break;
            }
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && pidRuntime.acroTrainerActive && !pidRuntime.inCrashRecoveryMode && !launchControlActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y
#if defined(USE_ACC)
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);
#endif

        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;
#ifdef USE_ABSOLUTE_CONTROL
        float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        if (!launchControlActive && !pidRuntime.inCrashRecoveryMode) {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * tpaFactorKp;
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki;
        float axisDynCi;
#ifdef USE_LAUNCH_CONTROL
        // if launch control is active override the iterm gains and apply iterm windup protection to all axes
        if (launchControlActive) {
            Ki = pidRuntime.launchControlKi;
            axisDynCi = dynCi;
        } else
#endif
        {
            Ki = pidRuntime.pidCoefficient[axis].Ki;
            axisDynCi = (axis == FD_YAW) ? dynCi : pidRuntime.dT; // only apply windup protection to yaw
        }

        pidData[axis].I = constrainf(previousIterm + (Ki * axisDynCi + agGain) * itermErrorRate, -pidRuntime.itermLimit, pidRuntime.itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_FEEDFORWARD
        pidSetpointDelta = feedforwardApply(axis, newRcFrame, pidRuntime.feedforwardAveraging);
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate D component
        // disable D if launch control is active
        if ((pidRuntime.pidCoefficient[axis].Kd > 0) && !launchControlActive) {

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta =
                - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;

#if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif

#if defined(USE_D_MIN)
            float dMinFactor = 1.0f;
            if (pidRuntime.dMinPercent[axis] > 0) {
                float dMinGyroFactor = pt2FilterApply(&pidRuntime.dMinRange[axis], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * pidRuntime.dMinGyroGain;
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * pidRuntime.dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = pidRuntime.dMinPercent[axis] + (1.0f - pidRuntime.dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt2FilterApply(&pidRuntime.dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_MIN, 0, lrintf(dMinGyroFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 1, lrintf(dMinSetpointFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 2, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_MIN, 3, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                }
            }

            // Apply the dMinFactor
            preTpaD *= dMinFactor;
#endif
            pidData[axis].D = preTpaD * pidRuntime.tpaFactor;

            // Log the value of D pre application of TPA
            preTpaD *= D_LPF_FILT_SCALE;

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_LPF, 2, lrintf(preTpaD));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_LPF, 3, lrintf(preTpaD));
            }
        } else {
            pidData[axis].D = 0;

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_LPF, 2, 0);
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_LPF, 3, 0);
            }
        }

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in feedforward
        pidSetpointDelta += setpointCorrection - pidRuntime.oldSetpointCorrection[axis];
        pidRuntime.oldSetpointCorrection[axis] = setpointCorrection;
#endif

        // no feedforward in launch control
        float feedforwardGain = launchControlActive ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // halve feedforward in Level mode since stick sensitivity is weaker by about half
            feedforwardGain *= FLIGHT_MODE(ANGLE_MODE) ? 0.5f : 1.0f;
            // transition now calculated in feedforward.c when new RC data arrives
            float feedForward = feedforwardGain * pidSetpointDelta * pidRuntime.pidFrequency;

#ifdef USE_FEEDFORWARD
            pidData[axis].F = shouldApplyFeedforwardLimits(axis) ?
                applyFeedforwardLimit(axis, feedForward, pidRuntime.pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
#else
            pidData[axis].F = feedForward;
#endif
#ifdef USE_RC_SMOOTHING_FILTER
            pidData[axis].F = applyRcSmoothingFeedforwardFilter(axis, pidData[axis].F);
#endif // USE_RC_SMOOTHING_FILTER
        } else {
            pidData[axis].F = 0;
        }

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
#endif
        // calculating the PID sum

        // P boost at the end of throttle chop
        // attenuate effect if turning more than 50 deg/s, half at 100 deg/s
        float agBoostAttenuator = fabsf(currentPidSetpoint) / 50.0f;
        agBoostAttenuator = MAX(agBoostAttenuator, 1.0f);
        const float agBoost = 1.0f + (pidRuntime.antiGravityPBoost / agBoostAttenuator);
        if (axis != FD_YAW) {
            pidData[axis].P *= agBoost;
            DEBUG_SET(DEBUG_ANTI_GRAVITY, axis + 2, lrintf(agBoost * 1000));
        }

        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && pidRuntime.useIntegratedYaw) {
            pidData[axis].Sum += pidSum * pidRuntime.dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * pidRuntime.integratedYawRelax / 100000.0f * pidRuntime.dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if !pidRuntime.pidStabilisationEnabled || gyroOverflowDetected() {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

fn crashRecoveryModeActive(&self) -> bool
{
    self.inCrashRecoveryMode
}

// #ifdef USE_ACRO_TRAINER
// void pidSetAcroTrainerState(bool newState)
// {
//     if (pidRuntime.acroTrainerActive != newState) {
//         if (newState) {
//             pidAcroTrainerInit();
//         }
//         pidRuntime.acroTrainerActive = newState;
//     }
// }
// #endif // USE_ACRO_TRAINER

fn pidSetAntiGravityState(&mut self, newState: bool)
{
    if newState != pidRuntime.antiGravityEnabled {
        // reset the accelerator on state changes
        self.itermAccelerator = 0.0;
    }
    self.antiGravityEnabled = newState;
}

fn pidAntiGravityEnabled(&self) -> bool
{
   self.antiGravityEnabled
}

// #ifdef USE_DYN_LPF
fn dynLpfDTermUpdate(throttle: f32)
{
    if pidRuntime.dynLpfFilter != DYN_LPF_NONE {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}
// #endif

fn pidSetItermReset(&mut self, enabled: bool)
{
    self.zeroThrottleItermReset = enabled;
}

fn pidGetPreviousSetpoint(&self, axis: u32) -> f32 // todo: int type.
{
    self.previousPidSetpoint[axis]
}

fn pidGetDT(&self) -> f32
{
    self.dT;
}

fn pidGetPidFrequency(&self) -> f32
{
    self.pidFrequency
}
}

static mut curve: f32 = 0;

fn dynLpfCutoffFreq(throttle: f32, dynLpfMin: u16, dynLpfMax: u16, expo: u8) -> f32 {
    let expof = expo as f32 / 10.0;
    unsafe {
        curve = throttle * (1 - throttle) * expof + throttle;

        (dynLpfMax - dynLpfMin) * curve + dynLpfMin
    }
}