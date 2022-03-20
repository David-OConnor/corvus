//! Adapted from the official ELRS example here: https://github.com/ExpressLRS/ExpressLRS/tree/master/src
//! There's no formal ELRS spec, so we use the official source code as our guide.

// todo // Other files to look at: "
// // OTA would be the main thing it sounds like you're missing
// // rx_main gives the top level of the receiver, common defines the modem params, fhss covers the channels
// // oh, and PFD is the code for syncing the rx to the tx, that's fairly critical. Most of this lives in src/lib/*"

// mod rx_main; // todo
mod common;
mod fhss;
mod ots;
mod sx1280_regs;


// todo: Should this mod be under drivers, or top level? Or a new top-level mod folder called protocols,
// todo where DSHOT, and our USB comms also is?