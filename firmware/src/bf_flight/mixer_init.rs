#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/betaflight/betaflight/blob/master/src/main/flight/mixe_init.c and .h

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

pub struct mixerRuntime {
    motorCount: u8,
    currentMixer: [motorMixer; MAX_SUPPORTED_MOTORS],
    // #ifdef USE_LAUNCH_CONTROL
    launchControlMixer: [motorMixer; MAX_SUPPORTED_MOTORS],
    // #endif
    feature3dEnabled: bool,
    motorOutputLow: f32,
    motorOutputHigh: f32,
    disarmMotorOutput: f32,
    deadbandMotor3dHigh: f32,
    deadbandMotor3dLow: f32,
    // #ifdef USE_DYN_IDLE
    dynIdleMaxIncrease: f32,
    idleThrottleOffset: f32,
    dynIdleMinRps,: f32,
    dynIdlePGain: f32,
    prevMinRps: f32,
    dynIdleIGain: f32,
    dynIdleDGain: f32,
    dynIdleI: f32,
    minRpsDelayK: f32,
    // #endif
// #if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    vbatSagCompensationFactor: f32,
    vbatFull: f32,
    vbatRangeToCompensate: f32,
// #endif
}

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

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,
    .yaw_motors_reversed = false,
    .crashflip_motor_percent = 0,
    .crashflip_expo = 35,
    .mixer_type = MIXER_LEGACY,
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

mixerMode_e currentMixerMode;

const mixerQuadX: [motorMixer; 4] = [
    [ 1.0, -1.0,  1.0, -1.0 ],          // REAR_R
    [ 1.0, -1.0, -1.0,  1.0 ],          // FRONT_R
    [ 1.0,  1.0,  1.0,  1.0 ],          // REAR_L
    [ 1.0,  1.0, -1.0, -1.0 ],          // FRONT_L
];
// #ifndef USE_QUAD_MIXER_ONLY
// static const motorMixer_t mixerTricopter[] = {
//     { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
//     { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
//     { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
// };
// 
// static const motorMixer_t mixerQuadP[] = {
//     { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
//     { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
//     { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
//     { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
// };

// #if defined(USE_UNCOMMON_MIXERS)
// static const motorMixer_t mixerBicopter[] = {
//     { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
//     { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
// };
// #else
// #define mixerBicopter NULL
// #endif
// 
// static const motorMixer_t mixerY4[] = {
//     { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
//     { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
//     { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
//     { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
// };
// 
// 
// #if (MAX_SUPPORTED_MOTORS >= 6)
// static const motorMixer_t mixerHex6X[] = {
//     { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
//     { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
//     { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
//     { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
//     { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
//     { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
// };
// 
// #if defined(USE_UNCOMMON_MIXERS)
// static const motorMixer_t mixerHex6H[] = {
//     { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
//     { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
//     { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
//     { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
//     { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
//     { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
// };
// 
// static const motorMixer_t mixerHex6P[] = {
//     { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
//     { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
//     { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
//     { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
//     { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
//     { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
// };
// static const motorMixer_t mixerY6[] = {
//     { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
//     { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
//     { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
//     { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
//     { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
//     { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
// };
// #else
// #define mixerHex6H NULL
// #define mixerHex6P NULL
// #define mixerY6 NULL
// #endif // USE_UNCOMMON_MIXERS
// #else
// #define mixerHex6X NULL
// #endif // MAX_SUPPORTED_MOTORS >= 6

// #if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
// static const motorMixer_t mixerOctoX8[] = {
//     { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
//     { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
//     { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
//     { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
//     { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
//     { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
//     { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
//     { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
// };
// 
// static const motorMixer_t mixerOctoFlatP[] = {
//     { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
//     { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
//     { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
//     { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
//     { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
//     { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
//     { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
//     { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
// };
// 
// static const motorMixer_t mixerOctoFlatX[] = {
//     { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
//     { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
//     { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
//     { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
//     { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
//     { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
//     { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
//     { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
// };
// #else
// #define mixerOctoX8 NULL
// #define mixerOctoFlatP NULL
// #define mixerOctoFlatX NULL
// #endif

// static const motorMixer_t mixerVtail4[] = {
//     { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
//     { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
//     { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
//     { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
// };
// 
// static const motorMixer_t mixerAtail4[] = {
//     { 1.0f, -0.58f,  0.58f, -1.0f },          // REAR_R
//     { 1.0f, -0.46f, -0.39f,  0.5f },          // FRONT_R
//     { 1.0f,  0.58f,  0.58f,  1.0f },          // REAR_L
//     { 1.0f,  0.46f, -0.39f, -0.5f },          // FRONT_L
// };

// #if defined(USE_UNCOMMON_MIXERS)
// static const motorMixer_t mixerDualcopter[] = {
//     { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
//     { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
// };
// #else
// #define mixerDualcopter NULL
// #endif

// static const motorMixer_t mixerSingleProp[] = {
//     { 1.0f,  0.0f,  0.0f, 0.0f },
// };

// const mixerQuadX1234: [motorMixer; 4] = [
//     [ 1.0,  1.0, -1.0, -1.0 ],          // FRONT_L
//     [ 1.0, -1.0, -1.0,  1.0 ],          // FRONT_R
//     [ 1.0, -1.0,  1.0, -1.0 ],          // REAR_R
//     [ 1.0,  1.0,  1.0,  1.0 ],          // REAR_L
// ];

// Keep synced with mixerMode_e
// Some of these entries are bogus when servos (USE_SERVOS) are not configured,
// but left untouched to keep ordinals synced with mixerMode_e (and configurator).
// const mixers: [mixer; 69] = {
//     // motors, use servo, motor mixer
//     [ 0, false, NULL ],                // entry 0
//     [ 3, true,  mixerTricopter ],      // MIXER_TRI
//     [ 4, false, mixerQuadP ],          // MIXER_QUADP
//     [ 4, false, mixerQuadX ],          // MIXER_QUADX
//     [ 2, true,  mixerBicopter ],       // MIXER_BICOPTER
//     [ 0, true,  NULL ],                // * MIXER_GIMBAL
//     [ 6, false, mixerY6 ],             // MIXER_Y6
//     [ 6, false, mixerHex6P ],          // MIXER_HEX6
//     [ 1, true,  mixerSingleProp ],     // * MIXER_FLYING_WING
//     [ 4, false, mixerY4 ],             // MIXER_Y4
//     [ 6, false, mixerHex6X ],          // MIXER_HEX6X
//     [ 8, false, mixerOctoX8 ],         // MIXER_OCTOX8
//     [ 8, false, mixerOctoFlatP ],      // MIXER_OCTOFLATP
//     [ 8, false, mixerOctoFlatX ],      // MIXER_OCTOFLATX
//     [ 1, true,  mixerSingleProp ],     // * MIXER_AIRPLANE
//     [ 1, true,  mixerSingleProp ],     // * MIXER_HELI_120_CCPM
//     [ 0, true,  NULL ],                // * MIXER_HELI_90_DEG
//     [ 4, false, mixerVtail4 ],         // MIXER_VTAIL4
//     [ 6, false, mixerHex6H ],          // MIXER_HEX6H
//     [ 0, true,  NULL ],                // * MIXER_PPM_TO_SERVO
//     [ 2, true,  mixerDualcopter ],     // MIXER_DUALCOPTER
//     [ 1, true,  NULL ],                // MIXER_SINGLECOPTER
//     [ 4, false, mixerAtail4 ],         // MIXER_ATAIL4
//     [ 0, false, NULL ],                // MIXER_CUSTOM
//     [ 2, true,  NULL ],                // MIXER_CUSTOM_AIRPLANE
//     [ 3, true,  NULL ],                // MIXER_CUSTOM_TRI
//     [ 4, false, mixerQuadX1234 ],
// ];
// #endif // !USE_QUAD_MIXER_ONLY

FAST_DATA_ZERO_INIT mixerRuntime_t mixerRuntime;

fn getMotorCount() -> u8
{
    return mixerRuntime.motorCount;
}

fn areMotorsRunning() -> bool 
{
    let mut motorsRunning = false;
    if ARMING_FLAG(ARMED) {
        motorsRunning = true;
    } else {
        for i in 0..mixerRuntime.motorCount {
            if motor_disarmed[i] != mixerRuntime.disarmMotorOutput {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

// #ifdef USE_SERVOS
// bool mixerIsTricopter(void)
// {
//     return (currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI);
// }
// #endif

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
fn initEscEndpoints()
{
    let mut motorOutputLimit = 1.0;
    if (currentPidProfile.motor_output_limit < 100) {
        motorOutputLimit = currentPidProfile.motor_output_limit / 100.0;
    }
    motorInitEndpoints(motorConfig(), motorOutputLimit, &mixerRuntime.motorOutputLow, &mixerRuntime.motorOutputHigh, &mixerRuntime.disarmMotorOutput, &mixerRuntime.deadbandMotor3dHigh, &mixerRuntime.deadbandMotor3dLow);
}

// Initialize pidProfile related mixer settings

fn mixerInitProfile()
{
// #ifdef USE_DYN_IDLE
    if motorConfigMutable().dev.useDshotTelemetry {
        mixerRuntime.dynIdleMinRps = currentPidProfile.dyn_idle_min_rpm * 100.0 / 60.0;
    } else {
        mixerRuntime.dynIdleMinRps = 0.0;
    }
    mixerRuntime.dynIdlePGain = currentPidProfile.dyn_idle_p_gain * 0.00015;
    mixerRuntime.dynIdleIGain = currentPidProfile.dyn_idle_i_gain * 0.01 * pidGetDT();
    mixerRuntime.dynIdleDGain = currentPidProfile.dyn_idle_d_gain * 0.0000003 * pidGetPidFrequency();
    mixerRuntime.dynIdleMaxIncrease = currentPidProfile.dyn_idle_max_increase * 0.001;
    mixerRuntime.minRpsDelayK = 800 * pidGetDT() / 20.0; //approx 20ms D delay, arbitrarily suits many motors
    if !mixerRuntime.feature3dEnabled && mixerRuntime.dynIdleMinRps {
        mixerRuntime.motorOutputLow = DSHOT_MIN_THROTTLE; // Override value set by initEscEndpoints to allow zero motor drive
    }
// #endif

// #if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    mixerRuntime.vbatSagCompensationFactor = 0.0;
    if currentPidProfile.vbat_sag_compensation > 0 {
        //TODO: Make this voltage user configurable
        mixerRuntime.vbatFull = CELL_VOLTAGE_FULL_CV;
        mixerRuntime.vbatRangeToCompensate = mixerRuntime.vbatFull - batteryConfig().vbatwarningcellvoltage;
        if mixerRuntime.vbatRangeToCompensate > 0 {
            mixerRuntime.vbatSagCompensationFactor = currentPidProfile.vbat_sag_compensation as f32 / 100.0;
        }
    }
// #endif
}

/// #ifdef USE_LAUNCH_CONTROL
/// Create a custom mixer for launch control based on the current settings
/// but disable the front motors. We don't care about roll or yaw because they
/// are limited in the PID controller.
fn loadLaunchControlMixer()
{
    for i in 0..MAX_SUPPORTED_MOTORS {
        mixerRuntime.launchControlMixer[i] = mixerRuntime.currentMixer[i];
        // limit the front motors to minimum output
        if mixerRuntime.launchControlMixer[i].pitch < 0.0 {
            mixerRuntime.launchControlMixer[i].pitch = 0.0;
            mixerRuntime.launchControlMixer[i].throttle = 0.0;
        }
    }
}
// #endif

// #ifndef USE_QUAD_MIXER_ONLY

fn mixerConfigureOutput()
{
    mixerRuntime.motorCount = 0;

    if currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE {
        // load custom mixer into currentMixer
        for i in 0..MAX_SUPPORTED_MOTORS {
            // check if done
            if customMotorMixer(i).throttle == 0.0 {
                break;
            }
            mixerRuntime.currentMixer[i] = customMotorMixer(i);
            mixerRuntime.motorCount += 1;
        }
    } else {
        mixerRuntime.motorCount = mixers[currentMixerMode].motorCount;
        if mixerRuntime.motorCount > MAX_SUPPORTED_MOTORS {
            mixerRuntime.motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if mixers[currentMixerMode].motor {
            for i in 0..mixerRuntime.motorCount {
                mixerRuntime.currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
// #ifdef USE_LAUNCH_CONTROL
    loadLaunchControlMixer();
// #endif
    mixerResetDisarmedMotors();
}

fn mixerLoadMix(index: usize, customMixers: &motorMixer)
{
    // we're 1-based
    index += 1;
    // clear existing
    for i in 0..MAX_SUPPORTED_MOTORS {
        customMixers[i].throttle = 0.0;
    }
    // do we have anything here to begin with?
    if mixers[index].motor != NULL){
        for i in 0..mixers[index].motorCount {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}
// #else
fn mixerConfigureOutput()
{
    mixerRuntime.motorCount = QUAD_MOTOR_COUNT;
    for i in 0..mixers[index].motorCount {
        mixerRuntime.currentMixer[i] = mixerQuadX[i];
    }
// #ifdef USE_LAUNCH_CONTROL
    loadLaunchControlMixer();
// #endif
    mixerResetDisarmedMotors();
}
// #endif // USE_QUAD_MIXER_ONLY

fn mixerInit(mixerMode: mixerMode)
{
    currentMixerMode = mixerMode;

    mixerRuntime.feature3dEnabled = featureIsEnabled(FEATURE_3D);

    initEscEndpoints();
// #ifdef USE_SERVOS
//     if (mixerIsTricopter()) {
//         mixerTricopterInit();
//     }
// #endif

// #ifdef USE_DYN_IDLE
    mixerRuntime.idleThrottleOffset = getDigitalIdleOffset(motorConfig());
    mixerRuntime.dynIdleI = 0.0;
    mixerRuntime.prevMinRps = 0.0;
// #endif

    mixerConfigureOutput();
}

fn mixerResetDisarmedMotors()
{
    // set disarmed motor values
    for i in 0..MAX_SUPPORTED_MOTORS {
        motor_disarmed[i] = mixerRuntime.disarmMotorOutput;
    }
}

fn getMixerMode() -> mixerMode
{
    currentMixerMode
}

fn mixerModeIsFixedWing(mixerMode: mixerMode) -> bool
{
    match mixerMode {
        MIXER_FLYING_WING | MIXER_AIRPLANE | MIXER_CUSTOM_AIRPLANE => true,
        _ => false,
    }
}

fn isFixedWing() -> bool
{
    mixerModeIsFixedWing(currentMixerMode)
}

fn getMotorOutputLow() -> f32
{
    mixerRuntime.motorOutputLow
}

fn getMotorOutputHigh() -> f32
{
    mixerRuntime.motorOutputHigh
}
