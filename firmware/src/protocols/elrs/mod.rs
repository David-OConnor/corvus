//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/tree/master/src
//! There's no formal ELRS spec, so we use the official source code as our guide. For this reason, we don't deviate
//! much in code style; prefer 1:1 mapping to official codebase, to idiomatic Rust.

// todo // Other files to look at: "
// // OTA would be the main thing it sounds like you're missing
// // rx_main gives the top level of the receiver, common defines the modem params, fhss covers the channels
// // oh, and PFD is the code for syncing the rx to the tx, that's fairly critical. Most of this lives in src/lib/*"

mod common;
mod fhss;
mod lqcalc;
mod msp;
mod pfd;
mod lowpassfilter;
mod rx_main;
mod ota;
mod stubborn;
mod sx1280_regs;
mod crsf;
mod sx1280;
mod sx1280_hal;

// todo: QC every time you translated a ++. (Search for +=1, followed by using that index). You need
// todo to more it after the access!

use crate::flight_ctrls::CtrlInputs;

use stm32_hal2::{pac::SPI3, spi::Spi};

pub fn get_inputs(spi: &mut Spi<SPI3>) -> CtrlInputs {
    // todo: fix this.

    CtrlInputs::default()
}

// todo: Should this mod be under drivers, or top level? Or a new top-level mod folder called protocols,
// todo where DSHOT, and our USB comms also is?
