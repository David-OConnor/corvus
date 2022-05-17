#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here:
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/StubbornSender/stubborn_sender.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/StubbornSender/stubborn_sender.cpp
// todo: stubborn Receivers?

// The number of times to resend the same package index before going to RESYNC
const SENDER_MAX_MISSED_PACKETS: u32 = 20;

#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum SenderState {
    SENDER_IDLE = 0,
    SENDING,
    WAIT_UNTIL_NEXT_CONFIRM,
    RESYNC,
    RESYNC_THEN_SEND, // perform a RESYNC then go to SENDING
}

pub struct StubbornSender {
    pub data: [u8; 69],
    pub length: u8,
    pub bytesPerCall: u8,
    pub currentOffset: usize,
    pub currentPackage: u8,
    pub waitUntilTelemetryConfirm: bool,
    pub resetState: bool,
    pub waitCount: u16,
    pub maxWaitCount: u16,
    pub maxPackageIndex: u8,
    pub senderState: SenderState,
}

impl StubbornSender {
    pub fn new(maxPackageIndex: u8) -> Self {
        Self {
            // todo: Probably don't want to store data here, or leave it as an upper bound.
            // todo, ie and pass a ref to apt fns.
            data: [0; 69],
            length: 0,
            bytesPerCall: 1,
            currentOffset: 0,
            currentPackage: 0,
            waitUntilTelemetryConfirm: true,
            resetState: true,
            waitCount: 0,
            // 80 corresponds to UpdateTelemetryRate(ANY, 2, 1), which is what the TX uses in boost mode
            maxWaitCount: 80,
            maxPackageIndex,
            senderState: SenderState::SENDER_IDLE,
        }
    }

    /***
     * Queues a message to send, will abort the current message if one is currently being transmitted
     ***/
    pub fn SetDataToTransmit(
        &mut self,
        lengthToTransmit: u8,
        dataToTransmit: [u8; 69],
        bytesPerCall: u8,
    ) {
        if lengthToTransmit / bytesPerCall >= maxPackageIndex {
            return;
        }

        self.length = lengthToTransmit;
        self.data = dataToTransmit;
        self.currentOffset = 0;
        self.currentPackage = 1;
        self.waitCount = 0;
        self.bytesPerCall = bytesPerCall;
        self.senderState = if senderState == SenderState::SENDER_IDLE {
            SenderState::SENDING
        } else {
            SenderState::RESYNC_THEN_SEND
        };
    }

    pub fn IsActive(&self) -> bool {
        self.senderState != SenderState::SENDER_IDLE
    }

    pub fn GetCurrentPayload(
        &mut self,
        packageIndex: &mut u8,
        count: &mut u8,
        currentData: &mut [u8],
        data_len: usize, // not in orig, but I think we need it
    ) {
        match self.senderState {
            SenderState::RESYNC | SenderState::RESYNC_THEN_SEND => {
                *packageIndex = self.maxPackageIndex;
                *count = 0;
                for i in 0..data_len {
                    currentData[i] = 0;
                }
            }
            SenderState::SENDING => {
                for i in 0..data_len {
                    currentData[i] += self.currentOffset as u8;
                }
                *packageIndex = self.currentPackage;
                if bytesPerCall > 1 {
                    if self.currentOffset + self.bytesPerCall as usize <= self.length as usize {
                        *count = self.bytesPerCall;
                    } else {
                        *count = self.length - self.currentOffset as u8;
                    }
                } else {
                    *count = 1;
                }
            }
            _ => {
                *count = 0;
                for i in 0..data_len {
                    currentData[i] = 0;
                }
                *packageIndex = 0;
            }
        }
    }

    pub fn ConfirmCurrentPayload(&mut self, telemetryConfirmValue: bool) {
        let mut nextSenderState = self.senderState;

        match self.senderState {
            SenderState::SENDING => {
                if telemetryConfirmValue != self.waitUntilTelemetryConfirm {
                    self.waitCount += 1;
                    if self.waitCount > self.maxWaitCount {
                        self.waitUntilTelemetryConfirm = !telemetryConfirmValue;
                        nextSenderState = SenderState::RESYNC;
                    }
                }

                self.currentOffset += bytesPerCall;
                self.currentPackage += 1;
                self.waitUntilTelemetryConfirm = !self.waitUntilTelemetryConfirm;
                self.waitCount = 0;

                if self.currentOffset >= length {
                    nextSenderState = SenderState::WAIT_UNTIL_NEXT_CONFIRM;
                }
            }

            SenderState::RESYNC
            | SenderState::RESYNC_THEN_SEND
            | SenderState::WAIT_UNTIL_NEXT_CONFIRM => {
                if telemetryConfirmValue == waitUntilTelemetryConfirm {
                    nextSenderState = if self.senderState == SenderState::RESYNC_THEN_SEND {
                        SenderState::SENDING
                    } else {
                        SenderState::SENDER_IDLE
                    };
                    self.waitUntilTelemetryConfirm = !telemetryConfirmValue;
                }
                // switch to resync if tx does not confirm value fast enough
                else if self.senderState == SenderState::WAIT_UNTIL_NEXT_CONFIRM {
                    self.waitCount += 1;
                    if self.waitCount > maxWaitCount {
                        self.waitUntilTelemetryConfirm = !telemetryConfirmValue;
                        nextSenderState = SenderState::RESYNC;
                    }
                }
            }
            SenderState::SENDER_IDLE => (),
        }

        self.senderState = nextSenderState;
    }

    /// Called when the telemetry ratio or air rate changes, calculate
    /// the new threshold for how many times the telemetryConfirmValue
    /// can be wrong in a row before giving up and going to RESYNC
    pub fn UpdateTelemetryRate(&mut self, airRate: u16, tlmRatio: u8, tlmBurst: u8) {
        // consipicuously unused airRate parameter, the wait count is strictly based on number
        // of packets, not time between the telemetry packets, or a wall clock timeout
        // (void)airRate;
        // The expected number of packet periods between telemetry packets
        let packsBetween: u32 = (tlmRatio * (1 + tlmBurst) / tlmBurst) as u32;
        self.maxWaitCount = (packsBetween * SENDER_MAX_MISSED_PACKETS) as u16;
    }
}

pub struct StubbornReceiver {
    pub data: [u8; 69],
    pub finishedData: bool,
    pub length: u8,
    pub bytesPerCall: u8,
    pub currentOffset: usize,
    pub currentPackage: u8,
    pub telemetryConfirm: bool,
    pub maxPackageIndex: u8,
}

impl StubbornReceiver {
    pub fn new(maxPackageIndex: u8) -> Self {
        Self {
            // todo: Probably don't want to store data here, or leave it as an upper bound.
            // todo, ie and pass a ref to apt fns.
            data: [0; 69],
            finishedData: false,
            bytesPerCall: 1,
            currentOffset: 0,
            currentPackage: 0,
            length: 0,
            telemetryConfirm: false,
            maxPackageIndex,
        }
    }

    pub fn GetCurrentConfirm(&self) -> bool {
        self.telemetryConfirm
    }

    pub fn SetDataToReceive(&mut self, maxLength: u8, dataToReceive: [u8; 69], bytesPerCall: u8) {
        self.length = maxLength;
        self.data = dataToReceive;
        self.currentPackage = 1;
        self.currentOffset = 0;
        self.finishedData = false;
        self.bytesPerCall = bytesPerCall;
    }

    pub fn ReceiveData(&mut self, packageIndex: u8, receiveData: &[u8]) {
        if packageIndex == 0 && self.currentPackage > 1 {
            self.finishedData = true;
            self.telemetryConfirm = !self.telemetryConfirm;
            return;
        }

        if packageIndex == self.maxPackageIndex {
            self.telemetryConfirm = !self.telemetryConfirm;
            self.currentPackage = 1;
            self.currentOffset = 0;
            self.finishedData = false;
            return;
        }

        if self.finishedData {
            return;
        }

        if packageIndex == self.currentPackage {
            for i in 0..self.bytesPerCall {
                self.currentOffset += 1;
                self.data[self.currentOffset] = receiveData[i as usize];
            }

            currentPackage += 1;
            self.telemetryConfirm = !self.telemetryConfirm;
            return;
        }

        return;
    }

    pub fn HasFinishedData(&self) -> bool {
        self.finishedData
    }

    pub fn Unlock(&mut self) {
        if finishedData {
            self.currentPackage = 1;
            self.currentOffset = 0;
            self.finishedData = false;
        }
    }
}