#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/betaflight/betaflight/blob/master/src/main/flight/pid_init.c

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

// todo consts: type?
// #if defined(USE_D_MIN)
const D_MIN_RANGE_HZ: u16 = 85;    // PT2 lowpass input cutoff to peak D around propwash frequencies
const D_MIN_LOWPASS_HZ: u16 = 35;  // PT2 lowpass cutoff to smooth the boost effect
const D_MIN_GAIN_FACTOR: f32 = 0.00008;
const D_MIN_SETPOINT_GAIN_FACTOR: f32 = 0.00008;
// #endif

const ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF: u16 = 15;  // The anti gravity throttle highpass filter cutoff
const ANTI_GRAVITY_SMOOTH_FILTER_CUTOFF: u16 = 3;  // The anti gravity P smoothing filter cutoff

fn pidSetTargetLooptime(pidLooptime: u32)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 0.000001;
    pidRuntime.pidFrequency = 1.0 / pidRuntime.dT;
    dshotSetPidLoopTime(targetPidLooptime);
}

fn pidInitFilters(pidProfile: &pidProfile)
{
    // assert!(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if targetPidLooptime == 0 {
        // no looptime set, so set all the filters to null
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
        return;
    }

    let pidFrequencyNyquist = pidRuntime.pidFrequency / 2; // No rounding needed

    let dTermNotchHz = if pidProfile.dterm_notch_hz <= pidFrequencyNyquist {
        pidProfile.dterm_notch_hz
    } else {
        if pidProfile.dterm_notch_cutoff < pidFrequencyNyquist {
            pidFrequencyNyquist
        } else {
            0
        }
    };

    if dTermNotchHz != 0 && pidProfile.dterm_notch_cutoff != 0 {
        pidRuntime.dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        let notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for axis in FD_ROLL..=FD_YAW {
            biquadFilterInit(&pidRuntime.dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH, 1.0);
        }
    } else {
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    let dterm_lpf1_init_hz = pidProfile.dterm_lpf1_static_hz;

// #ifdef USE_DYN_LPF
    if pidProfile.dterm_lpf1_dyn_min_hz {
        dterm_lpf1_init_hz = pidProfile.dterm_lpf1_dyn_min_hz;
    }
// #endif

    if dterm_lpf1_init_hz > 0 {
        match pidProfile.dterm_lpf1_type {
        FILTER_PT1 => {
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)
            pt1FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt1FilterInit(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
        }
        FILTER_BIQUAD => {
            if (pidProfile -> dterm_lpf1_static_hz < pidFrequencyNyquist) {
                // # ifdef
                // USE_DYN_LPF
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)
                biquadFilterApplyDF1;
                // # else
                // pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)
                // biquadFilterApply;
                // # endif
                for axis in FD_ROLL..=FD_YAW {
                    biquadFilterInitLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, dterm_lpf1_init_hz, targetPidLooptime);
                }
            } else {
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
        }
        FILTER_PT2 => {
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)
            pt2FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt2FilterInit(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
        }
        FILTER_PT3 => {
            pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)
            pt3FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt3FilterInit(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(dterm_lpf1_init_hz, pidRuntime.dT));
            }
        }
        _ => {
            pidRuntime.dtermLowpassApplyFn = nullFilterApply;
            }
        }
    } else {
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if pidProfile.dterm_lpf2_static_hz > 0 {
        match pidProfile.dterm_lpf2_type {
        FILTER_PT1 => {
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt1FilterInit(&pidRuntime.dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
        }
        FILTER_BIQUAD => {
                if pidProfile.dterm_lpf2_static_hz < pidFrequencyNyquist {
                    pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)
                    biquadFilterApply;
                    for axis in FD_ROLL..=FD_YAW {
                        biquadFilterInitLPF(&pidRuntime.dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lpf2_static_hz, targetPidLooptime);
                    }
                } else {
                    pidRuntime.dtermLowpassApplyFn = nullFilterApply;
                }
            }
        FILTER_PT2 => {
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)
            pt2FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt2FilterInit(&pidRuntime.dtermLowpass2[axis].pt2Filter, pt2FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
        }
        FILTER_PT3 => {
            pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)
            pt3FilterApply;
            for axis in FD_ROLL..=FD_YAW {
                pt3FilterInit(&pidRuntime.dtermLowpass2[axis].pt3Filter, pt3FilterGain(pidProfile->dterm_lpf2_static_hz, pidRuntime.dT));
            }
        }
        _ => {
            pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
        }
        }
    } else {
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
    }

    if pidProfile.yaw_lowpass_hz == 0 {
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        pidRuntime.ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(pidProfile.yaw_lowpass_hz, pidRuntime.dT));
    }

// #if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile.throttle_boost_cutoff, pidRuntime.dT));
// #endif

// #if defined(USE_ITERM_RELAX)
    if pidRuntime.itermRelax {
        for i in 0..XYZ_AXIS_COUNT {
            pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoff, pidRuntime.dT));
        }
    }
// #endif

#if defined(USE_ABSOLUTE_CONTROL)
    if pidRuntime.itermRelax {
        for i in 0..XYZ_AXIS_COUNT {
            pt1FilterInit(&pidRuntime.acLpf[i], pt1FilterGain(pidRuntime.acCutoff, pidRuntime.dT));
        }
    }
#endif

// #if defined(USE_D_MIN)
    // Initialize the filters for all axis even if the d_min[axis] value is 0
    // Otherwise if the pidProfile->d_min_xxx parameters are ever added to
    // in-flight adjustments and transition from 0 to > 0 in flight the feature
    // won't work because the filter wasn't initialized.
    for axis in FD_ROLL..=FD_YAW {
        pt2FilterInit(&pidRuntime.dMinRange[axis], pt2FilterGain(D_MIN_RANGE_HZ, pidRuntime.dT));
        pt2FilterInit(&pidRuntime.dMinLowpass[axis], pt2FilterGain(D_MIN_LOWPASS_HZ, pidRuntime.dT));
     }
// #endif

// #if defined(USE_AIRMODE_LPF)
    if pidProfile.transient_throttle_limit {
        pt1FilterInit(&pidRuntime.airmodeThrottleLpf1, pt1FilterGain(7.0, pidRuntime.dT));
        pt1FilterInit(&pidRuntime.airmodeThrottleLpf2, pt1FilterGain(20.0, pidRuntime.dT));
    }
// #endif

    pt1FilterInit(&pidRuntime.antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.antiGravitySmoothLpf, pt1FilterGain(ANTI_GRAVITY_SMOOTH_FILTER_CUTOFF, pidRuntime.dT));
}

fn pidInit(pidProfile: &pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
// #ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
// #endif
}

// #ifdef USE_RC_SMOOTHING_FILTER
fn pidInitFeedforwardLpf(filterCutoff: u16, debugAxis: u8)
{
    pidRuntime.rcSmoothingDebugAxis = debugAxis;
    if filterCutoff > 0 {
        pidRuntime.feedforwardLpfInitialized = true;
        for axis in FD_ROLL..=FD_YAW {
            pt3FilterInit(&pidRuntime.feedforwardPt3[axis], pt3FilterGain(filterCutoff, pidRuntime.dT));
        }
    }
}

fn pidUpdateFeedforwardLpf(filterCutoff: u16)
{
    if filterCutoff > 0 {
        for axis in FD_ROLL..=FD_YAW {
            pt3FilterUpdateCutoff(&pidRuntime.feedforwardPt3[axis], pt3FilterGain(filterCutoff, pidRuntime.dT));
        }
    }
}
// #endif // USE_RC_SMOOTHING_FILTER

fn pidInitConfig(pidProfile: &pidProfile)
{
    for axis in FD_ROLL..=FD_YAW {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile.id[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile.pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile.pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile.pid[axis].F / 100.0);
    }
// #ifdef USE_INTEGRATED_YAW_CONTROL
    if !pidProfile.use_integrated_yaw
// #endif
    {
        pidRuntime.pidCoefficient[FD_YAW].Ki *= 2.5;
    }
    pidRuntime.levelGain = pidProfile.pid[PID_LEVEL].P / 10.0;
    pidRuntime.horizonGain = pidProfile.pid[PID_LEVEL].I / 10.0;
    pidRuntime.horizonTransition = (float)pidProfile.pid[PID_LEVEL].D;
    pidRuntime.horizonTiltExpertMode = pidProfile.horizon_tilt_expert_mode;
    pidRuntime.horizonCutoffDegrees = (175 - pidProfile.horizon_tilt_effect) * 1.8;
    pidRuntime.horizonFactorRatio = (100 - pidProfile.horizon_tilt_effect) * 0.01;
    pidRuntime.maxVelocity[FD_ROLL] = pidRuntime.maxVelocity[FD_PITCH] = pidProfile.rateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.maxVelocity[FD_YAW] = pidProfile.yawRateAccelLimit * 100 * pidRuntime.dT;
    pidRuntime.itermWindupPointInv = 1.0;
    if pidProfile.itermWindupPointPercent < 100 {
        let itermWindupPoint = pidProfile.itermWindupPointPercent / 100.0;
        pidRuntime.itermWindupPointInv = 1.0 / (1.0 - itermWindupPoint);
    }
    pidRuntime.itermAcceleratorGain = pidProfile.itermAcceleratorGain;
    pidRuntime.crashTimeLimitUs = pidProfile.crash_time * 1000;
    pidRuntime.crashTimeDelayUs = pidProfile.crash_delay * 1000;
    pidRuntime.crashRecoveryAngleDeciDegrees = pidProfile.crash_recovery_angle * 10;
    pidRuntime.crashRecoveryRate = pidProfile.crash_recovery_rate;
    pidRuntime.crashGyroThreshold = pidProfile.crash_gthreshold;
    pidRuntime.crashDtermThreshold = pidProfile.crash_dthreshold;
    pidRuntime.crashSetpointThreshold = pidProfile.crash_setpoint_threshold;
    pidRuntime.crashLimitYaw = pidProfile.crash_limit_yaw;
    pidRuntime.itermLimit = pidProfile.itermLimit;
// #if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile.throttle_boost * 0.1;
// #endif
    pidRuntime.itermRotation = pidProfile.iterm_rotation;
    pidRuntime.antiGravityMode = pidProfile.antiGravityMode;

    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    pidRuntime.antiGravityOsdCutoff = 0.0;
    if pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH {
        pidRuntime.antiGravityOsdCutoff += (pidRuntime.itermAcceleratorGain / 1000.0) * 0.25;
    }

// #if defined(USE_ITERM_RELAX)
    pidRuntime.itermRelax = pidProfile.iterm_relax;
    pidRuntime.itermRelaxType = pidProfile.iterm_relax_type;
    pidRuntime.itermRelaxCutoff = pidProfile.iterm_relax_cutoff;
// #endif

#if defined(USE_ABSOLUTE_CONTROL)
    pidRuntime.acGain = pidProfile.abs_control_gain as f32;
    pidRuntime.acLimit = pidProfile.abs_control_limit as f32;
    pidRuntime.acErrorLimit = pidProfile.abs_control_error_limit as f32;
    pidRuntime.acCutoff = pidProfile.abs_control_cutoff as f32;
    for axis in FD_ROLL..=FD_YAW {
        let iCorrection = -pidRuntime.acGain * PTERM_SCALE / ITERM_SCALE * pidRuntime.pidCoefficient[axis].Kp;
        pidRuntime.pidCoefficient[axis].Ki = MAX(0.0, pidRuntime.pidCoefficient[axis].Ki + iCorrection);
    }
#endif

// #ifdef USE_DYN_LPF
    if pidProfile.dterm_lpf1_dyn_min_hz > 0 {
        match pidProfile.dterm_lpf1_type {
        FILTER_PT1 => {
            pidRuntime.dynLpfFilter = DYN_LPF_PT1;
            }
        FILTER_BIQUAD => {
            pidRuntime.dynLpfFilter = DYN_LPF_BIQUAD;
            }
        FILTER_PT2 => {
            pidRuntime.dynLpfFilter = DYN_LPF_PT2;
            }
        FILTER_PT3 => {
            pidRuntime.dynLpfFilter = DYN_LPF_PT3;
            }
        _  => {
            pidRuntime.dynLpfFilter = DYN_LPF_NONE;
            }
        }
    } else {
        pidRuntime.dynLpfFilter = DYN_LPF_NONE;
    }
    pidRuntime.dynLpfMin = pidProfile.dterm_lpf1_dyn_min_hz;
    pidRuntime.dynLpfMax = pidProfile.dterm_lpf1_dyn_max_hz;
    pidRuntime.dynLpfCurveExpo = pidProfile.dterm_lpf1_dyn_expo;
// #endif

// #ifdef USE_LAUNCH_CONTROL
    pidRuntime.launchControlMode = pidProfile.launchControlMode;
    if sensors(SENSOR_ACC) {
        pidRuntime.launchControlAngleLimit = pidProfile.launchControlAngleLimit;
    } else {
        pidRuntime.launchControlAngleLimit = 0;
    }
    pidRuntime.launchControlKi = ITERM_SCALE * pidProfile.launchControlGain;
// #endif

// #ifdef USE_INTEGRATED_YAW_CONTROL
    pidRuntime.useIntegratedYaw = pidProfile.use_integrated_yaw;
    pidRuntime.integratedYawRelax = pidProfile.integrated_yaw_relax;
// #endif

// #ifdef USE_THRUST_LINEARIZATION
    pidRuntime.thrustLinearization = pidProfile.thrustLinearization / 100.0;
    pidRuntime.throttleCompensateAmount = pidRuntime.thrustLinearization - 0.5 * powf(pidRuntime.thrustLinearization, 2);
// #endif

// #if defined(USE_D_MIN)
    for axis in FD_ROLL..=FD_YAW {
        let dMin = pidProfile.d_min[axis];
        if (dMin > 0) && (dMin < pidProfile->pid[axis].D) {
            pidRuntime.dMinPercent[axis] = dMin / (float)(pidProfile.pid[axis].D);
        } else {
            pidRuntime.dMinPercent[axis] = 0;
        }
    }
    pidRuntime.dMinGyroGain = pidProfile.d_min_gain * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
    pidRuntime.dMinSetpointGain = pidProfile.d_min_gain * D_MIN_SETPOINT_GAIN_FACTOR * pidProfile->d_min_advance * pidRuntime.pidFrequency / (100 * D_MIN_LOWPASS_HZ);
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
// #endif

// #if defined(USE_AIRMODE_LPF)
    pidRuntime.airmodeThrottleOffsetLimit = pidProfile.transient_throttle_limit / 100.0;
// #endif

// #ifdef USE_FEEDFORWARD
    if pidProfile.feedforward_transition == 0 {
        pidRuntime.feedforwardTransitionFactor = 0;
    } else {
        pidRuntime.feedforwardTransitionFactor = 100.0 / pidProfile.feedforward_transition;
    }
    pidRuntime.feedforwardAveraging = pidProfile.feedforward_averaging;
    pidRuntime.feedforwardSmoothFactor = 1.0;
    if pidProfile.feedforward_smooth_factor {
        pidRuntime.feedforwardSmoothFactor = 1.0 - (pidProfile.feedforward_smooth_factor as f32) / 100.0;
    }
    pidRuntime.feedforwardJitterFactor = pidProfile.feedforward_jitter_factor;
    pidRuntime.feedforwardBoostFactor = pidProfile.feedforward_boost as f32 / 10.0;
    feedforwardInit(pidProfile);
// #endif

    pidRuntime.levelRaceMode = pidProfile->level_race_mode;
}

fn pidCopyProfile(dstPidProfileIndex: u8, srcPidProfileIndex: u8)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

