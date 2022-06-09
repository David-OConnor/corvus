#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid.c

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
const MAX_PID_PROCESS_DENOM       16;
const PID_CONTROLLER_BETAFLIGHT   1;
const PID_MIXER_SCALING  : f32 =         1000.0;
const PID_SERVO_MIXER_SCALING : f32 =    0.7;
const PIDSUM_LIMIT                500;
const PIDSUM_LIMIT_YAW            400;
const PIDSUM_LIMIT_MIN            100;
const PIDSUM_LIMIT_MAX            1000;

const PID_GAIN_MAX 250;
const F_GAIN_MAX 1000;
const D_MIN_GAIN_MAX 250;

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

const MAX_PROFILE_NAME_LENGTH 8u

struct pidProfile {
    uint16_t yaw_lowpass_hz,                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_lpf1_static_hz,          // Static Dterm lowpass 1 filter cutoff value in hz
    uint16_t dterm_notch_hz,                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff,            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT],

    uint8_t dterm_lpf1_type,                // Filter type for dterm lowpass 1
    uint8_t itermWindupPointPercent,        // iterm windup threshold, percent motor saturation
    uint16_t pidSumLimit,
    uint16_t pidSumLimitYaw,
    uint8_t pidAtMinThrottle,               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit,                // Max angle in degrees in level mode

    uint8_t horizon_tilt_effect,            // inclination factor for Horizon mode
    uint8_t horizon_tilt_expert_mode,       // OFF or ON

    // Betaflight PID controller parameters
    uint8_t  antiGravityMode,             // type of anti gravity method
    uint16_t itermThrottleThreshold,        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain,          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint16_t yawRateAccelLimit,             // yaw accel limiter for deg/sec/ms
    uint16_t rateAccelLimit,                // accel limiter roll/pitch deg/sec/ms
    uint16_t crash_dthreshold,              // dterm crash value
    uint16_t crash_gthreshold,              // gyro crash value
    uint16_t crash_setpoint_threshold,      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint16_t crash_time,                    // ms
    uint16_t crash_delay,                   // ms
    uint8_t crash_recovery_angle,           // degrees
    uint8_t crash_recovery_rate,            // degree/second
    uint16_t crash_limit_yaw,               // limits yaw errorRate, so crashes don't cause huge throttle increase
    uint16_t itermLimit,
    uint16_t dterm_lpf2_static_hz,          // Static Dterm lowpass 2 filter cutoff value in hz
    uint8_t crash_recovery,                 // off, on, on and beeps when it is in crash recovery mode
    uint8_t throttle_boost,                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff,          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t iterm_rotation,                 // rotates iterm to translate world errors to local coordinate system
    uint8_t iterm_relax_type,               // Specifies type of relax algorithm
    uint8_t iterm_relax_cutoff,             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax,                    // Enable iterm suppression during stick input
    uint8_t acro_trainer_angle_limit,       // Acro trainer roll/pitch angle limit in degrees
    uint8_t acro_trainer_debug_axis,        // The axis for which record debugging values are captured 0=roll, 1=pitch
    uint8_t acro_trainer_gain,              // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint16_t acro_trainer_lookahead_ms,     // The lookahead window in milliseconds used to reduce overshoot
    uint8_t abs_control_gain,               // How strongly should the absolute accumulated error be corrected for
    uint8_t abs_control_limit,              // Limit to the correction
    uint8_t abs_control_error_limit,        // Limit to the accumulated error
    uint8_t abs_control_cutoff,             // Cutoff frequency for path estimation in abs control
    uint8_t dterm_lpf2_type,                // Filter type for 2nd dterm lowpass
    uint16_t dterm_lpf1_dyn_min_hz,         // Dterm lowpass filter 1 min hz when in dynamic mode
    uint16_t dterm_lpf1_dyn_max_hz,         // Dterm lowpass filter 1 max hz when in dynamic mode
    uint8_t launchControlMode,              // Whether launch control is limited to pitch only (launch stand or top-mount) or all axes (on battery)
    uint8_t launchControlThrottlePercent,   // Throttle percentage to trigger launch for launch control
    uint8_t launchControlAngleLimit,        // Optional launch control angle limit (requires ACC)
    uint8_t launchControlGain,              // Iterm gain used while launch control is active
    uint8_t launchControlAllowTriggerReset, // Controls trigger behavior and whether the trigger can be reset
    uint8_t use_integrated_yaw,             // Selects whether the yaw pidsum should integrated
    uint8_t integrated_yaw_relax,           // Specifies how much integrated yaw should be reduced to offset the drag based yaw component
    uint8_t thrustLinearization,            // Compensation factor for pid linearization
    uint8_t d_min[XYZ_AXIS_COUNT],          // Minimum D value on each axis
    uint8_t d_min_gain,                     // Gain factor for amount of gyro / setpoint activity required to boost D
    uint8_t d_min_advance,                  // Percentage multiplier for setpoint input to boost algorithm
    uint8_t motor_output_limit,             // Upper limit of the motor output (percent)
    int8_t auto_profile_cell_count,         // Cell count for this profile to be used with if auto PID profile switching is used
    uint8_t transient_throttle_limit,       // Maximum DC component of throttle change to mix into throttle to prevent airmode mirroring noise
    char profileName[MAX_PROFILE_NAME_LENGTH + 1], // Descriptive name for profile

    uint8_t dyn_idle_min_rpm,                   // minimum motor speed enforced by the dynamic idle controller
    uint8_t dyn_idle_p_gain,                // P gain during active control of rpm
    uint8_t dyn_idle_i_gain,                // I gain during active control of rpm
    uint8_t dyn_idle_d_gain,                // D gain for corrections around rapid changes in rpm
    uint8_t dyn_idle_max_increase,          // limit on maximum possible increase in motor idle drive during active control

    uint8_t feedforward_transition,         // Feedforward attenuation around centre sticks
    uint8_t feedforward_averaging,          // Number of packets to average when averaging is on
    uint8_t feedforward_smooth_factor,      // Amount of lowpass type smoothing for feedforward steps
    uint8_t feedforward_jitter_factor,      // Number of RC steps below which to attenuate feedforward
    uint8_t feedforward_boost,              // amount of setpoint acceleration to add to feedforward, 10 means 100% added
    uint8_t feedforward_max_rate_limit,     // Maximum setpoint rate percentage for feedforward

    uint8_t dterm_lpf1_dyn_expo,            // set the curve for dynamic dterm lowpass filter
    uint8_t level_race_mode,                // NFE race mode - when true pitch setpoint calculation is gyro based in level mode
    uint8_t vbat_sag_compensation,          // Reduce motor output by this percentage of the maximum compensation amount

    uint8_t simplified_pids_mode,
    uint8_t simplified_master_multiplier,
    uint8_t simplified_roll_pitch_ratio,
    uint8_t simplified_i_gain,
    uint8_t simplified_d_gain,
    uint8_t simplified_pi_gain,
    uint8_t simplified_dmin_ratio,
    uint8_t simplified_feedforward_gain,
    uint8_t simplified_dterm_filter,
    uint8_t simplified_dterm_filter_multiplier,
    uint8_t simplified_pitch_pi_gain,
}

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
    uint8_t runaway_takeoff_prevention;          // off, on - enables pidsum runaway disarm logic
    uint16_t runaway_takeoff_deactivate_delay;   // delay in ms for "in-flight" conditions before deactivation (successful flight)
    uint8_t runaway_takeoff_deactivate_throttle; // minimum throttle percent required during deactivation phase
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;

    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
} dtermLowpass_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpassApplyFn;
    dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
    filterApplyFnPtr ptermYawLowpassApplyFn;
    pt1Filter_t ptermYawLowpass;
    bool antiGravityEnabled;
    uint8_t antiGravityMode;
    pt1Filter_t antiGravityThrottleLpf;
    pt1Filter_t antiGravitySmoothLpf;
    float antiGravityOsdCutoff;
    float antiGravityThrottleHpf;
    float antiGravityPBoost;
    float itermAccelerator;
    uint16_t itermAcceleratorGain;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
    float levelGain;
    float horizonGain;
    float horizonTransition;
    float horizonCutoffDegrees;
    float horizonFactorRatio;
    uint8_t horizonTiltExpertMode;
    float maxVelocity[XYZ_AXIS_COUNT];
    float itermWindupPointInv;
    bool inCrashRecoveryMode;
    timeUs_t crashDetectedAtUs;
    timeDelta_t crashTimeLimitUs;
    timeDelta_t crashTimeDelayUs;
    int32_t crashRecoveryAngleDeciDegrees;
    float crashRecoveryRate;
    float crashGyroThreshold;
    float crashDtermThreshold;
    float crashSetpointThreshold;
    float crashLimitYaw;
    float itermLimit;
    bool itermRotation;
    bool zeroThrottleItermReset;
    bool levelRaceMode;
    float tpaFactor;

#ifdef USE_ITERM_RELAX
    pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
    uint8_t itermRelax;
    uint8_t itermRelaxType;
    uint8_t itermRelaxCutoff;
#endif

#ifdef USE_ABSOLUTE_CONTROL
    float acCutoff;
    float acGain;
    float acLimit;
    float acErrorLimit;
    pt1Filter_t acLpf[XYZ_AXIS_COUNT];
    float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif

#ifdef USE_D_MIN
    pt2Filter_t dMinRange[XYZ_AXIS_COUNT];
    pt2Filter_t dMinLowpass[XYZ_AXIS_COUNT];
    float dMinPercent[XYZ_AXIS_COUNT];
    float dMinGyroGain;
    float dMinSetpointGain;
#endif

#ifdef USE_AIRMODE_LPF
    pt1Filter_t airmodeThrottleLpf1;
    pt1Filter_t airmodeThrottleLpf2;
#endif

#ifdef USE_RC_SMOOTHING_FILTER
    pt3Filter_t feedforwardPt3[XYZ_AXIS_COUNT];
    bool feedforwardLpfInitialized;
    uint8_t rcSmoothingDebugAxis;
    uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_ACRO_TRAINER
    float acroTrainerAngleLimit;
    float acroTrainerLookaheadTime;
    uint8_t acroTrainerDebugAxis;
    float acroTrainerGain;
    bool acroTrainerActive;
    int acroTrainerAxisState[2];  // only need roll and pitch
#endif

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
#endif

#ifdef USE_LAUNCH_CONTROL
    uint8_t launchControlMode;
    uint8_t launchControlAngleLimit;
    float launchControlKi;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    bool useIntegratedYaw;
    uint8_t integratedYawRelax;
#endif

#ifdef USE_THRUST_LINEARIZATION
    float thrustLinearization;
    float throttleCompensateAmount;
#endif

#ifdef USE_AIRMODE_LPF
    float airmodeThrottleOffsetLimit;
#endif

#ifdef USE_FEEDFORWARD
    float feedforwardTransitionFactor;
    feedforwardAveraging_t feedforwardAveraging;
    float feedforwardSmoothFactor;
    float feedforwardJitterFactor;
    float feedforwardBoostFactor;
#endif
} pidRuntime_t;

extern pidRuntime_t pidRuntime;

extern const char pidNames[];

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern float throttleBoost;
extern pt1Filter_t throttleLpf;

void resetPidProfile(pidProfile_t *profile);

void pidResetIterm(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void pidSetItermAccelerator(float newItermAccelerator);
bool crashRecoveryModeActive(void);
void pidAcroTrainerInit(void);
void pidSetAcroTrainerState(bool newState);
void pidUpdateTpaFactor(float throttle);
void pidUpdateAntiGravityThrottleFilter(float throttle);
bool pidOsdAntiGravityActive(void);
bool pidOsdAntiGravityMode(void);
void pidSetAntiGravityState(bool newState);
bool pidAntiGravityEnabled(void);

#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorValue);
float pidCompensateThrustLinearization(float throttle);
#endif

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset);
float pidGetAirmodeThrottleOffset();
#endif

#ifdef UNIT_TEST
#include "sensors/acceleration.h"
extern float axisError[XYZ_AXIS_COUNT];
void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint);
void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate);
void rotateItermAndAxisError();
float pidLevel(int axis, const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint);
float calcHorizonLevelStrength(void);
#endif
void dynLpfDTermUpdate(float throttle);
void pidSetItermReset(bool enabled);
float pidGetPreviousSetpoint(int axis);
float pidGetDT();
float pidGetPidFrequency();
float pidGetFeedforwardBoostFactor();
float pidGetFeedforwardSmoothFactor();
float pidGetFeedforwardJitterFactor();
float pidGetFeedforwardTransitionFactor();
float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);


Skip to content
Pull requests
Issues
Marketplace
Explore
@David-OConnor
betaflight /
betaflight
Public

Code
Issues 192
Pull requests 75
Discussions
Actions
Wiki
Security

    Insights

betaflight/src/main/flight/pid.c
@ctzsnooze
ctzsnooze use Throttle Setpoint, not rcDATA(throttle), for TPA
Latest commit 2262dfc on Jan 23
History
50 contributors
@borisbstyle
@mikeller
@martinbudden
@ctzsnooze
@etracer65
@MJ666
@hydra
@IllusionFpv
@kmitchel
@Vidalcris
@blckmn
@supiiik
1307 lines (1160 sloc) 52 KB
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"
#include "config/simplified_tuning.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/feedforward.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
#endif

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

#if defined(STM32F1)
const PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
const PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE) || defined(STM32G4) //G4 sometimes cpu overflow when PID rate set to higher than 4k
const PID_PROCESS_DENOM_DEFAULT       2
#else
const PID_PROCESS_DENOM_DEFAULT       1
#endif

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

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
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

#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 30;
    pidProfile->pid[PID_PITCH].D = 32;
#endif
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
const D_LPF_RAW_SCALE 25
const D_LPF_FILT_SCALE 22


void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

#ifdef USE_FEEDFORWARD
float pidGetFeedforwardTransitionFactor()
{
    return pidRuntime.feedforwardTransitionFactor;
}

float pidGetFeedforwardSmoothFactor()
{
    return pidRuntime.feedforwardSmoothFactor;
}

float pidGetFeedforwardJitterFactor()
{
    return pidRuntime.feedforwardJitterFactor;
}

float pidGetFeedforwardBoostFactor()
{
    return pidRuntime.feedforwardBoostFactor;
}
#endif

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

void pidUpdateTpaFactor(float throttle)
{
    const float tpaBreakpoint = (currentControlRateProfile->tpa_breakpoint - 1000) / 1000.0f;
    float tpaRate = currentControlRateProfile->tpa_rate / 100.0f;
    if (throttle > tpaBreakpoint) {
        if (throttle < 1.0f) {
            tpaRate *= (throttle - tpaBreakpoint) / (1.0f - tpaBreakpoint);
        }
    } else {
        tpaRate = 0.0f;
    }
    pidRuntime.tpaFactor = 1.0f - tpaRate;
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    if (pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        // calculate a boost factor for P in the same way as for I when throttle changes quickly
        const float antiGravityThrottleLpf = pt1FilterApply(&pidRuntime.antiGravityThrottleLpf, throttle);
        // focus P boost on low throttle range only
        if (throttle < 0.5f) {
            pidRuntime.antiGravityPBoost = 0.5f - throttle;
        } else {
            pidRuntime.antiGravityPBoost = 0.0f;
        }
        // use lowpass to identify start of a throttle up, use this to reduce boost at start by half
        if (antiGravityThrottleLpf < throttle) {
            pidRuntime.antiGravityPBoost *= 0.5f;
        }
        // high-passed throttle focuses boost on faster throttle changes
        pidRuntime.antiGravityThrottleHpf = fabsf(throttle - antiGravityThrottleLpf);
        pidRuntime.antiGravityPBoost = pidRuntime.antiGravityPBoost * pidRuntime.antiGravityThrottleHpf;
        // smooth the P boost at 3hz to remove the jagged edges and prolong the effect after throttle stops
        pidRuntime.antiGravityPBoost = pt1FilterApply(&pidRuntime.antiGravitySmoothLpf, pidRuntime.antiGravityPBoost);
    }
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        // for whoops where a lot of TL is needed, allow more throttle boost
        const float throttleReversed = (1.0f - throttle);
        throttle /= 1.0f + pidRuntime.throttleCompensateAmount * powf(throttleReversed, 2);
    }
    return throttle;
}

float pidApplyThrustLinearization(float motorOutput)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            const float motorOutputReversed = (1.0f - motorOutput);
            motorOutput *= 1.0f + powf(motorOutputReversed, 2) * pidRuntime.thrustLinearization;
        }
    }
    return motorOutput;
}
#endif

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

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset)
{
    if (pidRuntime.airmodeThrottleOffsetLimit == 0.0f) {
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

float pidGetAirmodeThrottleOffset()
{
    return pidRuntime.airmodeThrottleLpf1.state;
}
#endif

#ifdef USE_LAUNCH_CONTROL
const LAUNCH_CONTROL_MAX_RATE 100.0f
const LAUNCH_CONTROL_MIN_RATE 5.0f
const LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

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
    if (!pidRuntime.pidStabilisationEnabled || gyroOverflowDetected()) {
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

bool crashRecoveryModeActive(void)
{
    return pidRuntime.inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
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
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT()
{
    return pidRuntime.dT;
}

float pidGetPidFrequency()
{
    return pidRuntime.pidFrequency;
}

    © 2022 GitHub, Inc.

    Terms
    Privacy
    Security
    Status
    Docs
    Contact GitHub
    Pricing
    API
    Training
    Blog
    About

