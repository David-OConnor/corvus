//! Not a module; promising but failing techniques.


/// Represents a torque; ie a rotation with an associated angular velocity
pub struct _Torque {
    /// A unit vector. Uses the right hand rule.
    pub axis: Vec3,
    /// In radians-per-second.
    pub angular_velocity: f32,
}

impl Default for _Torque {
    fn default() -> Self {
        Self {
            axis: Vec3::new(1., 0., 0.),
            angular_velocity: 0.,
        }
    }
}

impl _Torque {
    // todo: Do you need/want this?
    pub fn from_components(pitch: f32, roll: f32, yaw: f32) -> Self {
        // todo: No idea if this is correct
        let vec = UP * yaw + RIGHT * pitch + FORWARD * roll;
        Self {
            axis: vec.to_normalized(),
            angular_velocity: vec.magnitude(),
        }
    }

    /// Construct torque from 2 quaternions, and the time the rotation takes.
    /// This assumes less than a full rotation between updates.
    pub fn from_attitudes(att_prev: Quaternion, att_this: Quaternion, dt: f32) -> Self {
        let rotation = att_this * att_prev.inverse();

        Self {
            axis: rotation.axis(),
            angular_velocity: rotation.angle() / dt,
        }
    }
}

// The motor RPM of each motor will not go below this. We use this for both quad and fixed-wing motors.
// todo: unimplemented
const IDLE_RPM: f32 = 100.;


/// Estimate the linear drag coefficient, given a single data point.
pub fn calc_drag_coeff(ω_meas: f32, α_meas: f32, α_commanded: f32) -> f32 {
    // https://physics.stackexchange.com/questions/304742/angular-drag-on-body
    // This coefficient maps angular velocity to drag acceleration directly,
    // and is measured (and filtered).

    // For "low-speeds", drag is proportionanl to ω. For high speeds, it's
    // prop to ω^2. The distinction is the reynolds number.

    // todo: Low speed for now.
    // drag_accel = -cω
    // α = α_commanded + cω
    // c = (α - α_commanded) / ω
    // α_commanded = α + cω

    (α_meas - α_commanded) / ω_meas
}

/// If we're unable to use our current parameters to determine a linear-jerk trajectory,
/// we specify a time-to-correction based on the params, then using that time, calculate
/// an angular accel and jerk. Returned result is α_0, j
fn α_from_ttc(θ_0: f32, ω_0: f32, θ_tgt: f32, ω_tgt: f32, ttc_per_dθ: f32) -> f32 {
    // Time to correction
    let ttc = ttc_per_dθ * (θ_tgt - θ_0).abs(); // todo: Naive. Take vel and accel into account.

    // static mut i: u32 = 0;
    // unsafe { i += 1 };
    // if unsafe { i } % 3_000 == 0 {
    //     println!("ttc inner: {}", ttc);
    // }

    // Calculate the "initial" target angular acceleration, from the formula we worked out
    // from the kinematic equations for θ(t) and ω(t).
    -2. / ttc.powi(2) * (ttc * (ω_tgt + 2. * ω_0) + 3. * (θ_0 - θ_tgt))
}

// #[cfg(feature = "quad")]
// /// Accel is in m/s^2. Returns an RPM delta between rotor pairs.
// fn accel_to_rpm_delta(α: f32, mapping: asdf) -> f32 {
//
// }
//
// #[cfg(feature = "fixed-wing")]
// /// Accel is in m/s^2. Returns an RPM delta between rotor pairs.
// fn accel_to_rpm_servo_cmds(α: f32, mapping: asdf) -> f32 {
//
// }

// todo: Due to numerical precision issues, should you work in ms?

/// Calculate the time required to perform an angle and angular rate correction, for a given axis.
/// This assumes constant jerk. If this is not possible given these constraints, returns `None`.
/// This function does not check if the time is longer than suitable; we handle that elsewhere.
///
/// We accomplish this by solving the formula
fn find_time_to_correct(
    θ_0: f32, ω_0: f32, ω_dot_0: f32, θ_tgt: f32, ω_tgt: f32
) -> Option<f32> {
    const EPS: f32 = 0.000001;

    if (ω_dot_0).abs() < EPS && (θ_0 - θ_tgt).abs() > EPS && (ω_tgt + 2. * ω_0).abs() > EPS {
        // todo: QC this change that incorporates dω
        Some(3. * (θ_tgt - θ_0) / (ω_tgt + 2. * ω_0))
    } else if (ω_dot_0).abs() < EPS {
        // This happens if there's 0 initial angular acceleration, no change in desired
        // attitude, but we do have a desired change in angular velocity. (todo is this right?)
        None
    } else {
        // If `inner` is negative, there is no solution given our initial conditions, and
        // constant jerk condition. We return `None` here, and conduct an rapid
        // change of acceleration (a discontinuity), at which point we can then find a constant-jerk
        // result after the change has been applied.
        // This would occur if, for example, α_0 and/or θ_0 is high,
        // and/or ω_0 is low.

        let inner = 6. * ω_dot_0 * (θ_tgt - θ_0) + (ω_tgt + 2. * ω_0).powi(2);

        if inner < 0. {
            None
        } else {
            let t_a = -(inner.sqrt() + 2. * ω_0 + ω_tgt) / ω_dot_0;
            let t_b = (inner.sqrt() - 2. * ω_0 - ω_tgt) / ω_dot_0;

            // Choose the lower of these solutions if both are positive; otherwise
            // choose the positive one, should it exist.
            if t_a > 0. && t_b > 0. {
                if t_a < t_b {
                    Some(t_a)
                } else {
                    Some(t_b)
                }
            } else if t_a > 0. {
                Some(t_a)
            } else if t_b > 0. {
                Some(t_b)
            } else {
                None
            }
        }
    }
}

fn find_ctrl_setting(
    θ_0: f32,
    ω_0: f32,
    ω_dot_0: f32,
    // The change in angle desired, along a given axis.
    θ_tgt: f32,
    // The change in angular velocity desired, along a given axis. We can find this by subtracting
    // the current angular velocity from the commanded rate.
    ω_tgt: f32,
    // ctrl_cmd_prev: f32,
    coeffs: &CtrlCoeffs,
    drag_coeff: f32,
    accel_map: &AccelMap,
    // filters: &mut FlightCtrlFilters,
    dt: f32,
) -> f32 {
    // todo: Take time to spin up/down into account?

    // let dθ = θ_tgt - θ_0;

    let time_to_correct = find_time_to_correct(θ_0, ω_0, ω_dot_0, θ_tgt, ω_tgt);

    static mut i: u32 = 0;
    unsafe { i += 1 };

    let mut α_target = match time_to_correct {
        Some(ttc) => {
            // If it would take too longer to perform the correction, calculate a new
            // angular acceleration that fits the criteria.
            if ttc > coeffs.max_ttc_per_dθ * (θ_tgt - θ_0).abs() {
                if unsafe { i } % 4_000 == 0 {
                    println!("unable to correct in time");
                }

                α_from_ttc(θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ)
                // If the time to correction is sufficiently small, apply a constant jerk, which
                // should be roughly the previous jerk if part way through a maneuver.
            } else {
                if unsafe { i } % 4_000 == 0 {
                    println!("Can correct in time");
                }

                // Calculate the (~constant for a given correction) change in angular acceleration.
                // let j = 6. * (2. * θ_0 + ttc * ω_0) / ttc.powi(3);
                let j = 6. / ttc.powi(3) * (2. * θ_0 + ttc * ω_0 - 2. * θ_tgt + ttc * ω_tgt);

                // This is the actual target acceleration, determined by the questions above:
                ω_dot_0 + j * dt
            }
        }
        None => {
            // if unsafe { i } % 7_000 == 0 {
            //     println!(
            //         "theta_0: {}, omega_0: {}, theta_tgt: {}, omega_tgt: {}, coeff: {}",
            //         θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ
            //     );
            // }

            α_from_ttc(θ_0, ω_0, θ_tgt, ω_tgt, coeffs.ttc_per_dθ)
        }
    };

    // The target acceleration needs to include both the correction, and drag compensation.
    let drag_accel = -drag_coeff * ω_0;
    α_target += drag_accel;

    // Calculate how, most recently, the control command is affecting angular accel.
    // A higher constant means a given command has a higher affect on angular accel.
    // todo: Track and/or lowpass effectiveness over recent history, at diff params.
    // todo: Once you have bidir dshot, use RPM instead of power.
    // todo: DO we sill want this?
    // todo: You should probably track ctrl eff in the main loop; not here.
    // let ctrl_effectiveness = α_meas / ctrl_cmd_prev;

    // Apply a lowpass filter to our effectiveness, to reduce noise and fluctuations.
    // let ctrl_effectiveness = filters.apply(ctrl_effectiveness);

    // This distills to: (dω / time_to_correction) / (α / ctrl_cmd_prev) =
    // (dω / time_to_correction) x (ctrl_cmd_prev / α) =
    // (dω x ctrl_cmd_prev) / (time_to_correction x α) =
    //
    // (dθ * coeffs.p_ω - ω x ctrl_cmd_prev) /
    // ((coeffs.time_to_correction_p_ω * dω.abs() + coeffs.time_to_correction_p_θ * dθ.abs()) x α)

    // Units: rad x cmd / (s * rad/s) = rad x cmd / rad = cmd
    // `cmd` is the unit we use for ctrl inputs. Not sure what (if any?) units it has.
    // α_target /= ctrl_effectiveness;

    // static mut i: u32 = 0;
    // unsafe { i += 1 };
    // if unsafe { i } % 6_000 == 0 {
    //     println!("Tgt accel: {}", α_target);
    // }

    if unsafe { i } % 4_000 == 0 {
        println!("TTC: {}, Pitch acc tgt: {}", time_to_correct, α_target);
    }

    accel_map.interpolate(α_target)
}