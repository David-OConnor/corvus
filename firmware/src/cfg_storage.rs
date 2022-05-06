//! Using the internal flash storage to store and load config and setup data.

use stm32_hal2::flash::Flash;

use crate::UserCfg;

// impl From<[u8; 69]> for UserCfg {
//     fn from(v: [u8; 69]) -> Self {
//         Self {
//
//         }
//     }
// }
//
// impl From<UserCfg> for [u8; 69] {
//     fn from(v: UserCfg) -> Self {
//         []
//     }
// }
//
