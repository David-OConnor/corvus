//! This module contains code used to update PID coefficients based on fligth parameters.
//! It includes flight profiles designed to test the paramters, and code used to update it.
//!
//! For a given PID parameter and flight condition, there is an optimal coefficient: The one that
//! achieves critical damping. // todo: BUt how do you combine P, I, and D to do this?

use cmsis_dsp_api as dsp_api;
use cmsis_dsp_sys as dsp_sys;
