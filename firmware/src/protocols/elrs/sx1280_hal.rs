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

use stm32_hal2::{gpio::Pin, pac::SPI2, spi::Spi};

use defmt::println;

use super::sx1280_regs::*;

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
    fn end() {
        TXRXdisable(); // make sure the RX/TX amp pins are disabled
        detachInterrupt(GPIO_PIN_DIO1);
        SPI.end();
        IsrCallback = nullptr; // remove callbacks
    }

    fn init(spi: Spi<SPI2>, nss: Pin, dio: Pin, busy_pin: Pin) -> Self {
        // (This functionality is handled in `main.rs`.)
        Self {
            spi,
            nss,
            dio,
            busy_pin,
        }
    }

    fn reset(&mut self) {
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
        memcpy(OutBuffer + 1, buffer, size);

        WaitOnBusy();
        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        BusyDelay(12);
    }

    pub fn ReadCommand(&mut self, command: RadioCommands, buffer: &mut [u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 2];

        WaitOnBusy();
        self.nss.set_low();

        if command == RadioCommands::GET_STATUS {
            OutBuffer[0] = command as u8;
            OutBuffer[1] = 0x00;
            OutBuffer[2] = 0x00;
            // todo: What's going on here? Why the sep size?
            SPI.transfer(OutBuffer, RADIO_GET_STATUS_BUF_SIZEOF);
            buffer[0] = OutBuffer[0];
        } else {
            OutBuffer[0] = command as u8;
            OutBuffer[1] = 0x00;
            memcpy(OutBuffer + 2, buffer, size);
            self.spi.transfer(&mut OutBuffer);

            memcpy(buffer, OutBuffer + 2, size);
        }
        self.nss.set_high();
    }

    fn WriteRegister(&mut self, address: u16, buffer: &[u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 3];

        OutBuffer[0] = RadioCommands::WRITE_REGISTER as u8;
        OutBuffer[1] = ((address & 0xFF00) >> 8) as u8;
        OutBuffer[2] = (address & 0x00FF) as u8;

        memcpy(OutBuffer + 3, buffer, size);

        WaitOnBusy();
        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        BusyDelay(12);
    }

    fn WriteRegisterOne(&mut self, address: u16, value: u8) {
        self.WriteRegister(address, &[value], 1);
    }

    fn ReadRegister(&mut self, address: u16, buffer: &[u8], size: usize) {
        let mut OutBuffer = [0_u8; size + 4];

        OutBuffer[0] = RadioCommands::READ_REGISTER as u8;
        OutBuffer[1] = ((address & 0xFF00) >> 8) as u8;
        OutBuffer[2] = (address & 0x00FF) as u8;
        OutBuffer[3] = 0x00;

        WaitOnBusy();
        self.nss.set_low();

        spi.transfer(&mut OutBuffer);
        memcpy(buffer, OutBuffer + 4, size);

        self.nss.set_high();
    }

    fn ReadRegisterOne(&mut self, address: u16) -> u8 {
        let mut data: u8 = 0;
        self.ReadRegister_b(address, &mut data, 1);
        data
    }

    fn WriteBuffer(&mut self, offset: u8, buffer: &[u8], size: usize) {
        let mut localbuf = [0_u8; size];

        for i in 0..size {
            localbuf[i] = buffer[i];
        }

        let mut OutBuffer = [0_u8; size + 2];

        OutBuffer[0] = RadioCommands::WRITE_BUFFER as u8;
        OutBuffer[1] = offset;

        memcpy(OutBuffer + 2, localbuf, size);

        WaitOnBusy();

        self.nss.set_low();
        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        BusyDelay(12);
    }

    fn ReadBuffer(&mut self, offset: u8, buffer: &mut [u8], size: usize) {
        let mut OutBuffer: [u8; size + 3] = [0; size + 3];
        let mut localbuf: [u8; size] = [0; size];

        OutBuffer[0] = RadioCommands::READ_BUFFER as u8;
        OutBuffer[1] = offset;
        OutBuffer[2] = 0x00;

        WaitOnBusy();
        self.nss.set_low();

        self.spi.transfer(&mut OutBuffer);
        self.nss.set_high();

        memcpy(localbuf, OutBuffer + 3, size);

        for i in 0..size {
            {
                buffer[i] = localbuf[i];
            }
        }
    }

    fn WaitOnBusy(&self) -> bool {
        // (We use the busy pin)
        // #if defined(GPIO_PIN_BUSY) && (GPIO_PIN_BUSY != UNDEF_PIN)
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

    fn dioISR() {
        if instance.IsrCallback {
            instance.IsrCallback();
        }
    }

    // We don't use these pins.
    // fn TXenable() {
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
    // }
    //
    // fn RXenable() {
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
    // }
    //
    // fn TXRXdisable() {
    // #if defined(GPIO_PIN_RX_ENABLE) && (GPIO_PIN_RX_ENABLE != UNDEF_PIN)
    //     digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
    // #endif
    // #if defined(GPIO_PIN_TX_ENABLE) && (GPIO_PIN_TX_ENABLE != UNDEF_PIN)
    //     digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    // #endif
    // #if defined(GPIO_PIN_PA_ENABLE) && (GPIO_PIN_PA_ENABLE != UNDEF_PIN)
    //     digitalWrite(GPIO_PIN_PA_ENABLE, LOW);
    // #endif
    // }
    //
    // #endif // UNIT_TEST
}
