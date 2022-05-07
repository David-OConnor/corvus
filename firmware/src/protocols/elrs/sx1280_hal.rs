#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280_hal.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/SX1280Driver/SX1280_hal.cpp

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project
*/

// todo: These blocking delays could potentially be very bad! QC them!
use cortex_m::{self, delay::Delay};
use stm32_hal2::{gpio::Pin, pac::SPI2, spi::Spi};

use defmt::println;

use super::sx1280_regs::*;

// todo: this blocking delay could be trouble!
pub fn delay_ms(time_ms: u32) {
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, 170_000_000);
    delay.delay_ms(time_ms);
}

// todo: Can't repr bool, so repr u8
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
enum SX1280_BusyState_ {
    SX1280_NOT_BUSY = 1,
    SX1280_BUSY = 0,
}

pub struct SX1280Hal {
    spi: Spi<SPI2>,
    nss: Pin,
    dio: Pin,
    busy_pin: Pin,
}

impl SX1280Hal {
    pub fn end(&mut self) {
        self.TXRXdisable(); // make sure the RX/TX amp pins are disabled
                            // detachInterrupt(GPIO_PIN_DIO1);
                            // SPI.end();
                            // IsrCallback = nullptr; // remove callbacks
    }

    pub fn new(spi: Spi<SPI2>, nss: Pin, dio: Pin, busy_pin: Pin) -> Self {
        // (This functionality is handled in `main.rs`.)
        Self {
            spi,
            nss,
            dio,
            busy_pin,
        }
    }

    pub fn reset(&mut self) {
        println!("SX1280 Reset");

        // todo: Currently aren't using rst pin
        // #if defined(GPIO_PIN_RST) && (GPIO_PIN_RST != UNDEF_PIN)
        //     pinMode(GPIO_PIN_RST, OUTPUT);
        //
        //     delay.delay_ms(50);
        //     digitalWrite(GPIO_PIN_RST, LOW);
        //     delay.delay_ms(50);
        //     digitalWrite(GPIO_PIN_RST, HIGH);
        // #endif

        // #if defined(GPIO_PIN_BUSY) && (GPIO_PIN_BUSY != UNDEF_PIN)
        // (We're using a busy pin)
        while self.busy_pin.is_high() {}
        // #else
        //     delay(10); // typically 2ms observed
        // #endif

        //this->BusyState = SX1280_NOT_BUSY;
        println!("SX1280 Ready!");
    }

    pub fn WriteCommandOne(&mut self, command: RadioCommands, val: u8) {
        self.WriteCommand(command, &[val], 1);
    }

    pub fn WriteCommand(&mut self, command: RadioCommands, buffer: &[u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 1];

        OutBuffer[0] = command as u8;
        for i in 0..size {
            OutBuffer[1 + i] = buffer[i];
        }

        self.WaitOnBusy();
        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        // self.BusyDelay(12);
        delay_ms(12); // todo!
    }

    pub fn ReadCommand(&mut self, command: RadioCommands, buffer: &mut [u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 2];

        self.WaitOnBusy();
        self.nss.set_low();

        if command == RadioCommands::GET_STATUS {
            OutBuffer[0] = command as u8;
            OutBuffer[1] = 0x00;
            OutBuffer[2] = 0x00;

            self.spi.transfer(&mut OutBuffer);
            buffer[0] = OutBuffer[0];
        } else {
            OutBuffer[0] = command as u8;
            OutBuffer[1] = 0x00;

            for i in 0..size {
                OutBuffer[2 + i] = buffer[i];
            }

            self.spi.transfer(&mut OutBuffer);

            for i in 0..size {
                buffer[i] = OutBuffer[2 + i];
            }
        }
        self.nss.set_high();
    }

    pub fn WriteRegister(&mut self, address: Reg, buffer: &[u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 3];

        OutBuffer[0] = RadioCommands::WRITE_REGISTER as u8;
        OutBuffer[1] = ((address as u16 & 0xFF00) >> 8) as u8;
        OutBuffer[2] = (address as u16 & 0x00FF) as u8;

        for i in 0..size {
            OutBuffer[3 + i] = buffer[i];
        }

        self.WaitOnBusy();
        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        delay_ms(12);
    }

    pub fn WriteRegisterOne(&mut self, address: Reg, value: u8) {
        self.WriteRegister(address, &[value], 1);
    }

    pub fn ReadRegister(&mut self, reg: Reg, buffer: &mut [u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 4];

        OutBuffer[0] = RadioCommands::READ_REGISTER as u8;
        OutBuffer[1] = ((reg as u16 & 0xFF00) >> 8) as u8;
        OutBuffer[2] = (reg as u16 & 0x00FF) as u8;
        OutBuffer[3] = 0x00;

        self.WaitOnBusy();
        self.nss.set_low();

        self.spi.transfer(&mut OutBuffer);
        for i in 0..size {
            buffer[i] = OutBuffer[4 + i];
        }

        self.nss.set_high();
    }

    pub fn ReadRegisterOne(&mut self, reg: Reg) -> u8 {
        let mut data: u8 = 0;
        self.ReadRegister(reg, &mut [data], 1);
        data
    }

    pub fn WriteBuffer(&mut self, offset: u8, buffer: &[u8], size: usize) {
        let mut localbuf = [0_u8; size];

        for i in 0..size {
            localbuf[i] = buffer[i];
        }

        let mut OutBuffer = [0_u8; size + 2];

        OutBuffer[0] = RadioCommands::WRITE_BUFFER as u8;
        OutBuffer[1] = offset;

        for i in 0..size {
            localbuf[i] = OutBuffer[2 + i];
        }

        self.WaitOnBusy();

        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        delay_ms(12); // todo!
    }

    pub fn ReadBuffer(&mut self, offset: u8, buffer: &mut [u8], size: usize) {
        let mut OutBuffer: [u8; size + 3] = [0; size + 3];
        let mut localbuf: [u8; size] = [0; size];

        OutBuffer[0] = RadioCommands::READ_BUFFER as u8;
        OutBuffer[1] = offset;
        OutBuffer[2] = 0x00;

        self.WaitOnBusy();
        self.nss.set_low();

        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        for i in 0..size {
            localbuf[i] = OutBuffer[3 + i];
        }

        for i in 0..size {
            {
                buffer[i] = localbuf[i];
            }
        }
    }

    pub fn WaitOnBusy(&self) -> bool {
        let wtimeoutUS: u32 = 1_000;
        let startTime: u32 = micros();

        while self.busy_pin.is_high()
        // wait untill not busy or until wtimeoutUS
        {
            if (micros() - startTime) > wtimeoutUS {
                //println!("TO");
                return false;
            } else {
            }
        }
        // #else
        //     // observed BUSY time for Write* calls are 12-20uS after NSS de-assert
        //     // and state transitions require extra time depending on prior state
        //     if BusyDelayDuration{
        //         while ((micros() - BusyDelayStart) < BusyDelayDuration)
        //             #ifdef PLATFORM_STM32
        //             __NOP();
        //             #elif PLATFORM_ESP32
        //             _NOP();
        //             #elif PLATFORM_ESP8266
        //             _NOP();
        //             #endif
        //         BusyDelayDuration = 0;
        //     }
        //     // delayMicroseconds(80);
        // #endif
        return true;
    }

    pub fn dioISR() {
        // todo?
        // if self.IsrCallback {
        //     instance.IsrCallback();
        // }
    }

    // We don't use these pins.
    pub fn TXenable(&self) {
        // #if defined(GPIO_PIN_PA_ENABLE) && (GPIO_PIN_PA_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_PA_ENABLE, HIGH);
        // #endif
        // #if defined(GPIO_PIN_RX_ENABLE) && (GPIO_PIN_RX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
        // #endif
        // #if defined(GPIO_PIN_TX_ENABLE) && (GPIO_PIN_TX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_TX_ENABLE, HIGH);
        // #endif
        // #if defined(GPIO_PIN_ANT_CTRL_1) && (GPIO_PIN_ANT_CTRL_1 != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_ANT_CTRL_1, HIGH);
        // #endif
        // #if defined(GPIO_PIN_ANT_CTRL_2) && (GPIO_PIN_ANT_CTRL_2 != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_ANT_CTRL_2, LOW);
        // #endif
    }
    //
    pub fn RXenable(&self) {
        // #if defined(GPIO_PIN_PA_ENABLE) && (GPIO_PIN_PA_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_PA_ENABLE, HIGH);
        // #endif
        // #if defined(GPIO_PIN_RX_ENABLE) && (GPIO_PIN_RX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_RX_ENABLE, HIGH);
        // #endif
        // #if defined(GPIO_PIN_TX_ENABLE) && (GPIO_PIN_TX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
        // #endif
        // #if defined(GPIO_PIN_ANT_CTRL_1) && (GPIO_PIN_ANT_CTRL_1 != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_ANT_CTRL_1, LOW);
        // #endif
        // #if defined(GPIO_PIN_ANT_CTRL_2) && (GPIO_PIN_ANT_CTRL_2 != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_ANT_CTRL_2, HIGH);
        // #endif
    }
    //
    pub fn TXRXdisable(&self) {
        // #if defined(GPIO_PIN_RX_ENABLE) && (GPIO_PIN_RX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
        // #endif
        // #if defined(GPIO_PIN_TX_ENABLE) && (GPIO_PIN_TX_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
        // #endif
        // #if defined(GPIO_PIN_PA_ENABLE) && (GPIO_PIN_PA_ENABLE != UNDEF_PIN)
        //     digitalWrite(GPIO_PIN_PA_ENABLE, LOW);
        // #endif
    }
    //
    // #endif // UNIT_TEST
}
