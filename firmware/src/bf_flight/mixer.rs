#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/betaflight/betaflight/blob/master/src/main/flight/mixer.c and .h

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

const QUAD_MOTOR_COUNT: usize = 4;

// Note: this is called MultiType/MULTITYPE_* in baseflight.
enum mixerMode
{
    // MIXER_TRI = 1,
    // MIXER_QUADP = 2,
    MIXER_QUADX = 3,
    // MIXER_BICOPTER = 4,
    // MIXER_GIMBAL = 5,
    // MIXER_Y6 = 6,
    // MIXER_HEX6 = 7,
    // MIXER_FLYING_WING = 8,
    // MIXER_Y4 = 9,
    // MIXER_HEX6X = 10,
    // MIXER_OCTOX8 = 11,
    // MIXER_OCTOFLATP = 12,
    // MIXER_OCTOFLATX = 13,
    // MIXER_AIRPLANE = 14,        // airplane / singlecopter / dualcopter (not yet properly supported)
    // MIXER_HELI_120_CCPM = 15,
    // MIXER_HELI_90_DEG = 16,
    // MIXER_VTAIL4 = 17,
    // MIXER_HEX6H = 18,
    // MIXER_PPM_TO_SERVO = 19,    // PPM -> servo relay
    // MIXER_DUALCOPTER = 20,
    // MIXER_SINGLECOPTER = 21,
    // MIXER_ATAIL4 = 22,
    // MIXER_CUSTOM = 23,
    // MIXER_CUSTOM_AIRPLANE = 24,
    // MIXER_CUSTOM_TRI = 25,
    // MIXER_QUADX_1234 = 26
}

enum mixerType
{
    MIXER_LEGACY = 0,
    MIXER_LINEAR = 1,
    MIXER_DYNAMIC = 2,
}

/// Custom mixer data per motor
struct motorMixer {
    throttle: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

PG_DECLARE_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer);

/// Custom mixer configuration
struct mixer_s {
    uint8_t motorCount;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

struct mixerConfig {
    mixerMode: u8,
    yaw_motors_reversed: bool,
    crashflip_motor_percent: u8,
    crashflip_expo: u8,
    mixer_type: u8,
}

// PG_DECLARE(mixerConfig_t, mixerConfig);

// #define CHANNEL_FORWARDING_DISABLED (uint8_t)0xFF

// extern const mixer_t mixers[];
// extern float motor[MAX_SUPPORTED_MOTORS];
// extern float motor_disarmed[MAX_SUPPORTED_MOTORS];
// struct rxConfig_s;


const DYN_LPF_THROTTLE_STEPS: usize =           100;
const DYN_LPF_THROTTLE_UPDATE_DELAY_US: u32 = 5000; // minimum of 5ms between updates

static FAST_DATA_ZERO_INIT float motorMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

fn getMotorMixRange() -> f32
{
    motorMixRange
}

fn writeMotors()
{
    motorWriteAll(motor);
}

fn writeAllMotors(mc: i16)
{
    // Sends commands to all motors
    for i in 0..mixerRuntime.motorCount {
        motor[i] = mc;
    }
    writeMotors();
}

fn stopMotors()
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT int8_t motorOutputMixSign;

fn calculateThrottleAndCurrentMotorEndpoints(currentTimeUs: timeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    let mut currentThrottleInputRange = 0.;
//     if mixerRuntime.feature3dEnabled {
//         uint16_t rcCommand3dDeadBandLow;
//         uint16_t rcCommand3dDeadBandHigh;
//
//         if (!ARMING_FLAG(ARMED)) {
//             rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
//         }
//
//         if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
//             // The min_check range is halved because the output throttle is scaled to 500us.
//             // So by using half of min_check we maintain the same low-throttle deadband
//             // stick travel as normal non-3D mode.
//             const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
//             rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
//             rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
//         } else {
//             rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
//             rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
//         }
//
//         const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
//         const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;
//
//         if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
//             // INVERTED
//             motorRangeMin = mixerRuntime.motorOutputLow;
//             motorRangeMax = mixerRuntime.deadbandMotor3dLow;
// #ifdef USE_DSHOT
//             if (isMotorProtocolDshot()) {
//                 motorOutputMin = mixerRuntime.motorOutputLow;
//                 motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
//             } else
// #endif
//             {
//                 motorOutputMin = mixerRuntime.deadbandMotor3dLow;
//                 motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
//             }
//
//             if (motorOutputMixSign != -1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = -1;
//
//             rcThrottlePrevious = rcCommand[THROTTLE];
//             throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
//             currentThrottleInputRange = rcCommandThrottleRange3dLow;
//         } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
//             // NORMAL
//             motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
//             motorRangeMax = mixerRuntime.motorOutputHigh;
//             motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
//             motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
//             if (motorOutputMixSign != 1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = 1;
//             rcThrottlePrevious = rcCommand[THROTTLE];
//             throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
//             currentThrottleInputRange = rcCommandThrottleRange3dHigh;
//         } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
//                 !flight3DConfigMutable()->switched_mode3d) ||
//                 isMotorsReversed()) {
//             // INVERTED_TO_DEADBAND
//             motorRangeMin = mixerRuntime.motorOutputLow;
//             motorRangeMax = mixerRuntime.deadbandMotor3dLow;
//
// #ifdef USE_DSHOT
//             if (isMotorProtocolDshot()) {
//                 motorOutputMin = mixerRuntime.motorOutputLow;
//                 motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
//             } else
// #endif
//             {
//                 motorOutputMin = mixerRuntime.deadbandMotor3dLow;
//                 motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
//             }
//
//             if (motorOutputMixSign != -1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = -1;
//
//             throttle = 0;
//             currentThrottleInputRange = rcCommandThrottleRange3dLow;
//         } else {
//             // NORMAL_TO_DEADBAND
//             motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
//             motorRangeMax = mixerRuntime.motorOutputHigh;
//             motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
//             motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
//             if (motorOutputMixSign != 1) {
//                 reversalTimeUs = currentTimeUs;
//             }
//             motorOutputMixSign = 1;
//             throttle = 0;
//             currentThrottleInputRange = rcCommandThrottleRange3dHigh;
//         }
//         if (currentTimeUs - reversalTimeUs < 250000) {
//             // keep iterm zero for 250ms after motor reversal
//             pidResetIterm();
//         }
//     } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;

// #ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0) {
            const float maxIncrease = isAirmodeActivated() ? mixerRuntime.dynIdleMaxIncrease : 0.05;
            float minRps = rpmMinMotorFrequency();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, (minRps * 10));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0, maxIncrease);
            DEBUG_SET(DEBUG_DYN_IDLE, 0, (MAX(-1000.0, dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, (mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, (dynIdleD * 10000));
        } else {
            motorRangeMinIncrease = 0;
        }
// #endif

// #if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        let mut motorRangeAttenuationFactor = 0;;
        // reduce motorRangeMax when battery is full
        if mixerRuntime.vbatSagCompensationFactor > 0.0 {
            let currentCellVoltage = getBatterySagCellVoltage();
            // batteryGoodness = 1 when voltage is above vbatFull, and 0 when voltage is below vbatLow
            float batteryGoodness = 1.0f - constrainf((mixerRuntime.vbatFull - currentCellVoltage) / mixerRuntime.vbatRangeToCompensate, 0.0f, 1.0f);
            motorRangeAttenuationFactor = (mixerRuntime.vbatRangeToCompensate / mixerRuntime.vbatFull) * batteryGoodness * mixerRuntime.vbatSagCompensationFactor;
            DEBUG_SET(DEBUG_BATTERY, 2, batteryGoodness * 100);
            DEBUG_SET(DEBUG_BATTERY, 3, motorRangeAttenuationFactor * 1000);
        }
        motorRangeMax = isFlipOverAfterCrashActive() ? mixerRuntime.motorOutputHigh : mixerRuntime.motorOutputHigh - motorRangeAttenuationFactor * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
// #else
//         motorRangeMax = mixerRuntime.motorOutputHigh;
// #endif

        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        motorOutputMixSign = 1;
    }

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

const CRASH_FLIP_DEADBAND: u32 = 20;
const CRASH_FLIP_STICK_MINF: f32 = 0.15;

fn applyFlipOverAfterCrashModeToMotors()
{
    if (ARMING_FLAG(ARMED)) {
        let flipPowerFactor = 1.0f - mixerConfig()->crashflip_expo / 100.0f;
        let stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        let stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        let stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);

        let stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        let stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        let stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

        let mut signPitch = getRcDeflection(FD_PITCH) < 0. ? 1. : -1.;
        let mut signRoll = getRcDeflection(FD_ROLL) < 0. ? 1. : -1.;
        let mut signYaw = (getRcDeflection(FD_YAW) < 0. ? 1. : -1.) * (mixerConfig()->yaw_motors_reversed ? 1. : -1.);

        let mut stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
        let mut stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        let cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
        let cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        let crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
        let flipStickRange = 1.0 - crashFlipStickMinExpo;
        let flipPower = MAX(0.0, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

        for i in 0..mixerRuntime.motorCount {
            float motorOutputNormalised =
                signPitch * mixerRuntime.currentMixer[i].pitch +
                signRoll * mixerRuntime.currentMixer[i].roll +
                signYaw * mixerRuntime.currentMixer[i].yaw;

            if motorOutputNormalised < 0 {
                if (mixerConfig().crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            }
            motorOutputNormalised = MIN(1.0, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? mixerRuntime.disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for i in 0..mixerRuntime.motorCount {
            motor[i] = motor_disarmed[i];
        }
    }
}

fn applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for i in 0..mixerRuntime.motorCount {
        float motorOutput = motorOutputMixSign * motorMix[i] + throttle * activeMixer[i].throttle;
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if !ARMING_FLAG(ARMED) {
        for i in 0..mixerRuntime.motorCount {
            motor[i] = motor_disarmed[i];
        }
    }
}

fn applyThrottleLimit(throttle: f32) -> f32
{
    if currentControlRateProfile.throttle_limit_percent < 100 {
        let throttleLimitFactor = currentControlRateProfile.throttle_limit_percent / 100.0;
        switch (currentControlRateProfile.throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

fn applyMotorStop()
{
    for i in 0..mixerRuntime.motorCount {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

// #ifdef USE_DYN_LPF
fn updateDynLpfCutoffs(currentTimeUs: timeUs, throttle: f32)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US {
        let quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if quantizedThrottle != dynLpfPreviousQuantizedThrottle {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            let dynLpfThrottle = quantizedThrottle as f32 / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
// #endif

 fn applyMixerAdjustmentLinear(motorMix: &mut f32, airmodeEnabled: bool) {
    let motorMixNormalizationFactor = if motorMixRange > 1.0 { motorMixRange } else { 1.0 };
    let motorMixDelta = 0.5 * motorMixRange;

    for i in 0..mixerRuntime.motorCount {
        if airmodeEnabled || throttle > 0.5 {
            if mixerConfig().mixer_type == MIXER_LINEAR {
                motorMix[i] = scaleRangef(throttle, 0.0, 1.0, motorMix[i] + motorMixDelta, motorMix[i] - motorMixDelta);
            } else {
                motorMix[i] = scaleRangef(throttle, 0.0, 1.0, motorMix[i] + ABS(motorMix[i]), motorMix[i] - ABS(motorMix[i]));
            }
        }
        motorMix[i] /= motorMixNormalizationFactor;
    }
}

fn applyMixerAdjustment(motorMix: &mut f32, motorMixMin: f32, motorMixMax: f32, airmodeEnabled: bool) {
// #ifdef USE_AIRMODE_LPF
    let unadjustedThrottle = throttle;
    throttle += pidGetAirmodeThrottleOffset();
    let mut airmodeThrottleChange = 0.;
// #endif

    if motorMixRange > 1.0 {
        for i in 0..mixerRuntime.motorCount {
            motorMix[i] /= motorMixRange;
        }
        // Get the maximum correction by setting offset to center when airmode enabled
        if airmodeEnabled {
            throttle = 0.5;
        }
    } else {
        if airmodeEnabled || throttle > 0.5 {
            throttle = constrainf(throttle, -motorMixMin, 1.0 - motorMixMax);
// #ifdef USE_AIRMODE_LPF
            airmodeThrottleChange = constrainf(unadjustedThrottle, -motorMixMin, 1.0 - motorMixMax) - unadjustedThrottle;
// #endif
        }
    }

// #ifdef USE_AIRMODE_LPF
    pidUpdateAirmodeLpf(airmodeThrottleChange);
// #endif
}

fn mixTable(currentTimeUs: timeUs)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if isFlipOverAfterCrashActive() {
        applyFlipOverAfterCrashModeToMotors();

        return;
    }

    let launchControlActive = isLaunchControlActive();

    motorMixer_t * activeMixer = &mixerRuntime.currentMixer[0];
// #ifdef USE_LAUNCH_CONTROL
    if launchControlActive && (currentPidProfile.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
        activeMixer = &mixerRuntime.launchControlMixer[0];
    }
// #endif

    // Calculate and Limit the PID sum
    let scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile.pidSumLimit, currentPidProfile.pidSumLimit) / PID_MIXER_SCALING;
    let scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile.pidSumLimit, currentPidProfile.pidSumLimit) / PID_MIXER_SCALING;

    let yawPidSumLimit = currentPidProfile.pidSumLimitYaw;

// #ifdef USE_YAW_SPIN_RECOVERY
    let yawSpinDetected = gyroYawSpinDetected();
    if yawSpinDetected {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
// #endif // USE_YAW_SPIN_RECOVERY

    let mut scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if !mixerConfig().yaw_motors_reversed {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if currentControlRateProfile.throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF {
        throttle = applyThrottleLimit(throttle);
    }

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

    // and for TPA
    pidUpdateTpaFactor(throttle);

// #ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
// #endif

    // apply throttle boost when throttle moves quickly
// #if defined(USE_THROTTLE_BOOST)
    if throttleBoost > 0.0 {
        let throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0, 1.0);
    }
// #endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

// #ifdef USE_DYN_IDLE
    // Set min throttle offset of 1% when stick is at zero and dynamic idle is active
    if mixerRuntime.dynIdleMinRps > 0.0 {
        throttle = MAX(throttle, 0.01);
    }
// #endif

// #ifdef USE_THRUST_LINEARIZATION
    // reduce throttle to offset additional motor output
    throttle = pidCompensateThrustLinearization(throttle);
// #endif

    // Find roll/pitch/yaw desired output
    // ??? Where is the optimal location for this code?
    let mut motorMix = [0.; MAX_SUPPORTED_MOTORS];
    let mut motorMixMax = 0;
    let mut motorMixMin = 0;
    for i in 0..mixerRuntime.motorCount {
        let mut mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        if mix > motorMixMax {
            motorMixMax = mix;
        } else if mix < motorMixMin {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
    let airmodeEnabled = airmodeIsEnabled() || launchControlActive;
// #ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if yawSpinDetected && !airmodeEnabled {
        throttle = 0.5;
    }
// #endif // USE_YAW_SPIN_RECOVERY

// #ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if launchControlActive {
        throttle = 0.0;
    }
// #endif

// #ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if FLIGHT_MODE(GPS_RESCUE_MODE) {
        throttle = gpsRescueGetThrottle();
    }
// #endif

    motorMixRange = motorMixMax - motorMixMin;
    if mixerConfig().mixer_type > MIXER_LEGACY {
        applyMixerAdjustmentLinear(motorMix, airmodeEnabled);
    } else {
        applyMixerAdjustment(motorMix, motorMixMin, motorMixMax, airmodeEnabled);
    }

    if featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig().mincheck) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

fn mixerSetThrottleAngleCorrection(correctionValue: u32)
{
    throttleAngleCorrection = correctionValue;
}

fn mixerGetThrottle() -> f32
{
    mixerThrottle
}
