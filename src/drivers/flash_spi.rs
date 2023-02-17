//! This module contains setup code for W25 SPI flash.
//! We use normal SPI on G4, and quad spi on H7.

use stm32_hal2::{gpio::Pin, pac, spi};

#[cfg(feature = "h7")]
pub type SpiFlashType = stm32_hal2::qspi::Qspi;
#[cfg(feature = "g4")]
pub type SpiFlashType = crate::drivers::spi2_kludge::Spi2<pac::SPI2>;

#[derive(Clone, Copy)]
pub enum FlashSpiError {
    NotConnected,
}

// todo: This will be diff for H7.
impl From<spi::SpiError> for FlashSpiError {
    fn from(_e: spi::SpiError) -> Self {
        Self::NotConnected
    }
}

/// See Datasheet, Section 13.1 (Note: This doesn't include all regs)
#[allow(dead_code)]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Reg {
    Jedec = 0x9f,
}

/// Initialize the flash peripheral, and verify it's returning the correct device id and metadata.
pub fn setup(spi: &mut SpiFlashType, cs: &mut Pin) -> Result<(), FlashSpiError> {
    let mut buf = [Reg::Jedec as u8, 0, 0, 0];
    cs.set_low();

    #[cfg(feature = "g4")]
    spi.transfer(&mut buf)?;

    cs.set_high();

    // The first val is used by all W25 flash. Second means memory type A. Third means 16mb or less.
    // Given SPI devices may report 0s if not connected properly, this is a good check that
    // we have 2-way communication.
    // todo: Will be different for H7.
    if buf[1] != 0xef || buf[2] != 0x40 || buf[3] != 0x15 {
        return Err(FlashSpiError::NotConnected);
    }

    Ok(())
}
