//! Our own (non-translated) CRSF impl
//! [Detailed protocol info](https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol)
//! [WIP clean driver in C](https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c)
//! https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.h
//!
//! [Addtional standaone ref](https://github.com/CapnBry/CRServoF/tree/master/lib/CrsfSerial)
//! From BF:
//         // CRSF protocol uses a single wire half duplex uart connection.
//         //  * The master sends one frame every 4ms and the slave replies between two frames from the master.
//         //  *
//         //  * 420000 baud
//         //  * not inverted
//         //  * 8 Bit
//         //  * 1 Stop bit
//         //  * Big endian
//         //  * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
//         //  * Max frame size is 64 bytes
//         //  * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.

// Use circular DMA. Idle interrupt.

use stm32_hal2::{
    usart::Usart,
    pac::{USART3},
};