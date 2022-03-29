//! This module handles mapping control inputs from the ELRS radio controller to program functions.
//! It is not based on the ELRS spec; it's an interface layer between that, and the rest of this program.

/// Represents data from a channel, including what channel it is, and the data passed.
/// [ELRS FAQ](https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support)
/// Assumes "Wide hybrid mode", as described in the FAQ.
enum Channel {
    /// Channels 1-4 are 10-bit channels.
    Ch1(u16),
    Ch2(u16),
    Ch3(u16),
    Ch4(u16),
    /// Aux 1 is 2-positions, and must be used for arming. AKA "Channel 5"
    Aux1(crate::ArmStatus),
    /// Aux 2-8 are 64 or 128-position channels. (6 or 7 bit)
    Aux2(u8),
    Aux3(u8),
    Aux4(u8),
    Aux5(u8),
    Aux6(u8),
    Aux7(u8),
    Aux8(u8),
}
