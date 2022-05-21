//! Using the internal flash storage to store and load config and setup data.

use stm32_hal2::flash::{Bank, Flash};

use crate::{UserCfg, FLASH_CFG_PAGE};

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

impl UserCfg {
    /// Save to flash memory
    pub fn save(&self, flash: &mut Flash) {
        // let  data: [u8; 69] = self.into();
        let mut data = [0; 69];

        flash.erase_write_page(Bank::B1, FLASH_CFG_PAGE, &data).ok();
    }
    //
    // /// Load from flash memory
    // pub fn load(flash: &mut Flash) -> Self {
    //
    // }
}
