//! This module contains code for the DSHOT digital protocol, using to control motor speed.
//!
//! [Some information on the protocol](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/):
//! Every digital protocol has a structure, also called a frame. It defines which information is at
//! which position in the data stream. And the frame structure of DSHOT is pretty straight forward:
//!
//! 11 bit throttle: 2048 possible values. 0 is reserved for disarmed. 1-47 are reserved for special commands.
//! Leaving 48 to 2047 (2000 steps) for the actual throttle value
//! 1 bit telemetry request - if this is set, telemetry data is sent back via a separate channel
//! 4 bit CRC: (Cyclic Redundancy) Check to validate data (throttle and telemetry request bit)
//! 1 and 0 in the DSHOT frame are distinguished by their high time. This means that every bit has a certain (constant) length,
//! and the length of the high part of the bit dictates if a 1 or 0 is being received.
//!
//! The DSHOT protocol (DSHOT-300, DSHOT-600 etc) is determined by the `DSHOT_ARR_600` and `DSHOT_PSC_600` settings in the
//! main crate; ie set a 600kHz countdown for DSHOT-600.

use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::delay::Delay;

use stm32_hal2::{
    dma::{ChannelCfg, Dma, Priority},
    pac,
    pac::DMA1,
    timer::{CaptureCompare, CountDir, InputSlaveMode, InputTrigger, OutputCompare, Polarity},
};

use crate::flight_ctrls::common::{Motor, MotorTimers};

use defmt::println;

// todo: Bidirectional: Set timers to active low, set GPIO idle to high, and perhaps set down counting
// todo if required. Then figure out input capture, and fix in HAL.

// todo (Probalby in another module) - RPM filtering, once you have bidirectional DSHOT working.
// Article: https://brushlesswhoop.com/betaflight-rpm-filter/
// todo: Basically, you set up a notch filter at rotor RPM. (I think; QC this)

use cfg_if::cfg_if;
use usb_device::device::UsbDeviceState::Default;

// Enable bidirectional DSHOT, which returns RPM data
pub const BIDIR_EN: bool = true;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately for DSHOT.
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period.
// ARR = (TIMclk/Updatefrequency) / (PSC + 1) - 1

pub const DSHOT_PSC_600: u16 = 0;

// ESC telemetry is false except when setting motor direction.
static mut ESC_TELEM: bool = false;

// We use these flags to determine how to handle the TC ISRs, ie when
// a send command is received, set the mode to input and vice versa.
#[cfg(feature = "g4")]
pub static CH_A_REC_MODE: AtomicBool = AtomicBool::new(false);
pub static CH_B_REC_MODE: AtomicBool = AtomicBool::new(false);

// Update frequency: 600kHz
// 170Mhz tim clock on G4.
// 240Mhz tim clock on H743
// 260Mhz tim clock on H723 @ 520Mhz. 275Mhz @ 550Mhz
cfg_if! {
    if #[cfg(feature = "h7")] {
        // pub const DSHOT_ARR_600: u32 = 399;  // 240Mhz tim clock
        pub const DSHOT_ARR_600: u32 = 432;  // 260Mhz tim clock
        // pub const DSHOT_ARR_600: u32 = 457; // 275Mhz tim clock
    } else if #[cfg(feature = "g4")] {
        pub const DSHOT_ARR_600: u32 = 282; // 170Mhz tim clock
    }
}

// Duty cycle values (to be written to CCMRx), based on our ARR value. 0. = 0%. ARR = 100%.
const DUTY_HIGH: u32 = DSHOT_ARR_600 * 3 / 4;
const DUTY_LOW: u32 = DSHOT_ARR_600 * 3 / 8;

// We use this during config that requires multiple signals sent, eg setting. motor direction.
// pub static PAUSE_AFTER_DSHOT: AtomicBool = AtomicBool::new(false);

// Use this pause duration, in ms, when setting up motor dir.
pub const PAUSE_BETWEEN_COMMANDS: u32 = 1;
pub const PAUSE_AFTER_SAVE: u32 = 40; // Must be at least 35ms.
                                      // BLHeli_32 requires you repeat certain commands, like motor direction, 6 times.
pub const REPEAT_COMMAND_COUNT: u32 = 10; // todo: Set back to 6 once sorted out.

// DMA buffers for each rotor. 16-bit data. Note that
// rotors 1/2 and 3/4 share a timer, so we can use the same DMA stream with them. Data for the 2
// channels are interleaved.
// Len 36, since last 2 entries will be 0 per channel. Required to prevent extra pulses. (Not sure exactly why)

cfg_if! {
    if #[cfg(feature = "h7")] {
        static mut PAYLOAD: [u16; 72] = [0; 72];
        // todo: The receive payload may be shorter due to how it's encoded; come back to this.
        static mut PAYLOAD_REC: [u16; 72] = [0; 72];
    } else if #[cfg(feature = "g4")] {
        static mut PAYLOAD_R1_2: [u16; 36] = [0; 36];
        pub static mut PAYLOAD_R1_2_REC: [u16; 36] = [0; 36]; // todo temp pub
        static mut PAYLOAD_R3_4: [u16; 36] = [0; 36];
        static mut PAYLOAD_R3_4_REC: [u16; 36] = [0; 36];
    }
}

/// Possible DSHOT commands (ie, DSHOT values 0 - 47). Does not include power settings.
/// [Special commands section](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
/// [BlHeli command code](https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt)
///
/// Commands are only executed when motors are stopped
/// Note that throttle has to be zero, and the telemetry bit must be set in the command frames.
/// Also note that a significant delay (several hundred ms) may be needed between commands.
#[derive(Copy, Clone)]
#[repr(u16)]
pub enum Command {
    /// Note: Motor Stop is perhaps not yet implemented.
    _MotorStop = 0,
    _Beacon1 = 1,
    _Beacon2 = 2,
    _Beacon3 = 3,
    _Beacon4 = 4,
    _Beacon5 = 5,
    _EscInfo = 6,
    /// SpinDir1 and 2 are forced normal and reversed. If you have the ESC set to reversed in the config,
    /// these will not reverse the motor direction, since it is already operating in reverse.
    SpinDir1 = 7, // 6x
    SpinDir2 = 8,   // 6x
    _3dModeOff = 9, // 6x
    _3dModeOn = 10, // 6x
    _SettingsRequest = 11,
    SaveSettings = 12, // 6x, wait at least 35ms before next command.
    /// Normal and reversed with respect to configuration.
    _SpinDirNormal = 20, // 6x
    _SpinDirReversed = 21, // 6x
    _Led0On = 22,      // BLHeli32 only
    _Led1On = 23,      // BLHeli32 only
    _Led2On = 24,      // BLHeli32 only
    _Led3On = 25,      // BLHeli32 only
    _Led0Off = 26,     // BLHeli32 only
    _Led1Off = 27,     // BLHeli32 only
    _Led2Off = 28,     // BLHeli32 only
    _Led3Off = 29,     // BLHeli32 only
    _AudioStreamModeOnOff = 30, // KISS audio Stream mode on/Off
    _SilendModeOnOff = 31, // KISS silent Mode on/Off
    /// Disables commands 42 to 47
    _TelemetryEnable = 32, // 6x
    /// Enables commands 42 to 47
    _TelemetryDisable = 33, // 6x
    /// Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
    _ContinuousErpmTelemetry = 34, // 6x
    /// Enables commands 42 to 47 and sends erpm period if normal Dshot frame
    _ContinuousErpmPeriodTelemetry = 35, // 6x
    /// 1°C per LSB
    _TemperatureTelemetry = 42,
    /// 10mV per LSB, 40.95V max
    _VoltageTelemetry = 43,
    /// 100mA per LSB, 409.5A max
    _CurrentTelemetry = 44,
    /// 10mAh per LSB, 40.95Ah max
    _ConsumptionTelemetry = 45,
    /// 100erpm per LSB, 409500erpm max
    _ErpmTelemetry = 46,
    /// 16us per LSB, 65520us max TBD
    _ErpmPeriodTelemetry = 47,
    // Max = 47, // todo: From Betaflight, but not consistent with the Brushlesswhoop article
}

pub enum CmdType {
    Command(Command),
    Power(f32),
}

/// Stop all motors, by setting their power to 0. Note that the Motor Stop command may not
/// be implemented, and this approach gets the job done. Run this at program init, so the ESC
/// get its required zero-throttle setting, generally required by ESC firmware to complete
/// initialization.
pub fn stop_all(timers: &mut MotorTimers, dma: &mut Dma<DMA1>) {
    // Note that the stop command (Command 0) is currently not implemented, so set throttles to 0.
    set_power(0., 0., 0., 0., timers, dma);
}

/// Set up the direction for each motor, in accordance with user config. Note: This blocks!
/// (at least for now). The intended use case is to run this only at init, and during Preflight,
/// if adjusting motor mapping.
pub fn setup_motor_dir(
    motors_reversed: (bool, bool, bool, bool),
    timers: &mut MotorTimers,
    dma: &mut Dma<DMA1>,
) {
    // A blocking delay.
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, 170_000_000);

    // Throttle must have been commanded to 0 a certain number of timers,
    // and the telemetry bit must be bit set to use commands.
    // Setting the throttle twice (with 1ms delay) doesn't work; 10x works. The required value is evidently between
    // these 2 bounds.
    for i in 0..30 {
        stop_all(timers, dma);
        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }
    // I've confirmed that setting direction without the telemetry bit set will fail.
    unsafe { ESC_TELEM = true };

    delay.delay_ms(PAUSE_BETWEEN_COMMANDS);

    println!("Setting up motor direction");

    // Spin dir commands need to be sent 6 times. (or 10?) We're using the "forced" spin dir commands,
    // ie not with respect to ESC configuration; although that would be acceptable as well.
    for _ in 0..REPEAT_COMMAND_COUNT {
        let cmd_1 = if motors_reversed.0 {
            Command::SpinDir2
        } else {
            Command::SpinDir1
        };
        let cmd_2 = if motors_reversed.1 {
            Command::SpinDir2
        } else {
            Command::SpinDir1
        };
        let cmd_3 = if motors_reversed.2 {
            Command::SpinDir2
        } else {
            Command::SpinDir1
        };
        let cmd_4 = if motors_reversed.3 {
            Command::SpinDir2
        } else {
            Command::SpinDir1
        };

        setup_payload(Motor::M1, CmdType::Command(cmd_1));
        setup_payload(Motor::M2, CmdType::Command(cmd_2));
        setup_payload(Motor::M3, CmdType::Command(cmd_3));
        setup_payload(Motor::M4, CmdType::Command(cmd_4));

        send_payload(timers, dma);

        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }

    for _ in 0..REPEAT_COMMAND_COUNT {
        setup_payload(Motor::M1, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M2, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M3, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M4, CmdType::Command(Command::SaveSettings));

        send_payload(timers, dma);

        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }
    delay.delay_ms(PAUSE_AFTER_SAVE);

    unsafe { ESC_TELEM = false };
}

/// Update our DSHOT payload for a given rotor, with a given power level. This created a payload
/// of tick values to send to the CCMR register; the output pin is set high or low for each
/// tick duration in succession.
pub fn setup_payload(rotor: Motor, cmd: CmdType) {
    // First 11 (0:10) bits are the throttle settings. 0 means disarmed. 1-47 are reserved
    // for special commands. 48 - 2_047 are throttle value (2_000 possible values)

    // Bit 11 is 1 to request telemetry; 0 otherwise.
    // Bits 12:15 are CRC, to validate data.

    let data_word = match cmd {
        CmdType::Command(c) => c as u16,
        CmdType::Power(pwr) => (pwr * 1_999.) as u16 + 48,
    };

    let packet = (data_word << 1) | (if unsafe { ESC_TELEM } { 1 } else { 0 });

    // Compute the checksum
    let crc = if BIDIR_EN {
        (!(packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0F
    } else {
        (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
    };
    let packet = (packet << 4) | crc;

    cfg_if! {
        if #[cfg(feature = "h7")] {
            let (payload, offset) = unsafe {
                match rotor {
                    Motor::M1 => (&mut PAYLOAD, 0),
                    Motor::M2 => (&mut PAYLOAD, 1),
                    Motor::M3 => (&mut PAYLOAD, 2),
                    Motor::M4 => (&mut PAYLOAD, 3),
                }
            };
        } else {
            let (payload, offset) = unsafe {
                match rotor {
                    Motor::M1 => (&mut PAYLOAD_R1_2, 0),
                    Motor::M2 => (&mut PAYLOAD_R1_2, 1),
                    Motor::M3 => (&mut PAYLOAD_R3_4, 0),
                    Motor::M4 => (&mut PAYLOAD_R3_4, 1),
                }
            };
        }
    }

    // Create a DMA payload of 16 timer CCR (duty) settings, each for one bit of our data word.
    for i in 0..16 {
        let bit = (packet >> i) & 1;
        let val = if bit == 1 { DUTY_HIGH } else { DUTY_LOW };
        // DSHOT uses MSB first alignment.
        // Values alternate in the buffer between the 2 registers we're editing, so
        // we interleave values here. (Each timer and DMA stream is associated with 2 channels).
        payload[(15 - i) * 2 + offset] = val as u16;
    }

    // Note that the end stays 0-padded, since we init with 0s, and never change those values.
}

/// Set a rotor pair's power, using a 16-bit DHOT word, transmitted over DMA via timer CCR (duty)
/// settings. `power` ranges from 0. to 1.
pub fn set_power(
    power1: f32,
    power2: f32,
    power3: f32,
    power4: f32,
    timers: &mut MotorTimers,
    dma: &mut Dma<DMA1>,
) {
    setup_payload(Motor::M1, CmdType::Power(power1));
    setup_payload(Motor::M2, CmdType::Power(power2));
    setup_payload(Motor::M3, CmdType::Power(power3));
    setup_payload(Motor::M4, CmdType::Power(power4));

    send_payload(timers, dma)
}

/// Set a single rotor's power. Used by preflight; not normal operations.
pub fn set_power_single(rotor: Motor, power: f32, timers: &mut MotorTimers, dma: &mut Dma<DMA1>) {
    setup_payload(rotor, CmdType::Power(power));
    send_payload(timers, dma)
}

/// Send the stored payload.
fn send_payload(timers: &mut MotorTimers, dma: &mut Dma<DMA1>) {
    // Stop the receive transaction.
    dma.stop(Motor::M1.dma_channel());
    #[cfg(all(feature = "h7", feature = "quad"))]
    dma.stop(Motor::M3.dma_channel());

    // if BIDIR_EN {
    //     // Was likely in input mode previously; update.
    //     set_to_output(timers);
    // }

    // todo: TS
    // timers.r12.disable();

    // Note that timer enabling is handled by `write_dma_burst`.

    let dma_cfg = ChannelCfg {
        priority: Priority::Low, // todo
        ..ChannelCfg::default()
    };

    unsafe {
        cfg_if! {
            if #[cfg(feature = "h7")] {
                timers.rotors.write_dma_burst(
                    &PAYLOAD,
                    Motor::M1.base_addr_offset(),
                    4, // Burst len of 4, since we're updating 4 channels.
                    Motor::M1.dma_channel(),
                    dma_cfg.clone(),
                    dma,
                    true,
                );
            } else {
                timers.r12.write_dma_burst(
                    &PAYLOAD_R1_2,
                    Motor::M1.base_addr_offset(),
                    2, // Burst len of 2, since we're updating 2 channels.
                    Motor::M1.dma_channel(),
                    dma_cfg.clone(),
                    dma,
                    true,
                );
                #[cfg(feature = "quad")]
                    timers.r34.write_dma_burst(
                    &PAYLOAD_R3_4,
                    Motor::M3.base_addr_offset(),
                    2,
                    Motor::M3.dma_channel(),
                    dma_cfg,
                    dma,
                    false,
                );
            }
        }
    }
}

#[cfg(feature = "g4")]
/// Receive an RPM payload; for bidirectional mode.
pub fn receive_payload_a(timers: &mut MotorTimers, dma: &mut Dma<DMA1>) {
    // The previous transfer should already be complete, but just in case.
    dma.stop(Motor::M1.dma_channel());

    // Note that timer enabling is handled by `read_dma_burst`.
    unsafe {
        timers.r12.read_dma_burst(
            &PAYLOAD_R1_2_REC,
            Motor::M1.base_addr_offset(),
            2,
            Motor::M1.dma_channel(),
            ChannelCfg {
                // priority: Priority::High, // todo
                ..ChannelCfg::default()
            },
            dma,
            true,
        );
    }
}

/// Receive an RPM payload for channel B on G4, or all channels on H7; for bidirectional mode.
pub fn receive_payload_b(timers: &mut MotorTimers, dma: &mut Dma<DMA1>) {
    dma.stop(Motor::M3.dma_channel());

    unsafe {
        cfg_if! {
            if #[cfg(feature = "h7")] {
                timers.rotors.read_dma_burst(
                    &PAYLOAD_REC,
                    Motor::M1.base_addr_offset(),
                    4,
                    Motor::M1.dma_channel(),
                    ChannelCfg::default(),
                    dma,
                    true,
                );
            } else {
                timers.r34.read_dma_burst(
                    &PAYLOAD_R3_4_REC,
                    Motor::M3.base_addr_offset(),
                    2,
                    Motor::M3.dma_channel(),
                    ChannelCfg::default(),
                    dma,
                    false,
                );
            }
        }
    }
}

/// Change timer polarity and count direction, to enable or disable bidirectional DSHOT.
/// This results in the signal being active low for enabled, and active high for disabled.
/// Timer settings default (in HAL and hardware) to disabled.
pub fn set_bidirectional(enabled: bool, timers: &mut MotorTimers) {
    // todo: We need a way to configure channel 2 as a servo, eg for fixed-wing
    // todo with a rudder.

    let mut polarity = Polarity::ActiveHigh;
    let mut count_dir = CountDir::Up;

    if enabled {
        polarity = Polarity::ActiveLow;
        count_dir = CountDir::Down;
    }

    // Set up channels 1 and 2: This is for both quadcopters and fixed-wing.
    cfg_if! {
        if #[cfg(feature = "h7")] {
            timers.rotors.set_polarity(Motor::M1.tim_channel(), polarity);
            timers.rotors.set_polarity(Motor::M2.tim_channel(), polarity);

            // todo: Does setting this direction affect servo use, eg fixed-wing?
            timers.rotors.cfg.direction = count_dir;
            timers.rotors.set_dir();
        } else {
            timers.r12.set_polarity(Motor::M1.tim_channel(), polarity);
            timers.r12.set_polarity(Motor::M2.tim_channel(), polarity);

            timers.r12.cfg.direction = count_dir;
            timers.r12.set_dir();
        }
    }

    // For quadcopters, set up channels 3 and 4 as well.
    cfg_if! {
        if #[cfg(all(feature = "h7", feature = "quad"))] {
            timers.rotors.set_polarity(Motor::M3.tim_channel(), polarity);
            timers.rotors.set_polarity(Motor::M4.tim_channel(), polarity);
        } else if #[cfg(all(feature = "g4", feature = "quad"))] {
            timers.r34.set_polarity(Motor::M3.tim_channel(), polarity);
            timers.r34.set_polarity(Motor::M4.tim_channel(), polarity);

            timers.r34.cfg.direction = count_dir;
            timers.r34.set_dir();
        }
    }
}

/// Set the timer(s) to output mode. Do this in init, and after a receive
/// phase of bidirectional DSHOT.
pub fn set_to_output(timers: &mut MotorTimers, motor_a: Motor, motor_b: Motor, three_four: bool) {
    let oc = OutputCompare::Pwm1;

    // todo: Filter by Quad etc?

    cfg_if! {
        if #[cfg(feature = "h7")] {
            // Arbitrary duty cycle set, since we'll override it with DMA bursts.
            timers.rotors.enable_pwm_output(motor_a.tim_channel(), oc, 0.);
            timers.rotors.enable_pwm_output(motor_b.tim_channel(), oc, 0.);
        } else {
            if three_four {
                timers.r34.enable_pwm_output(motor_a.tim_channel(), oc, 0.);
                timers.r34.enable_pwm_output(motor_b.tim_channel(), oc, 0.);
            } else {
                timers.r12.enable_pwm_output(motor_a.tim_channel(), oc, 0.);
                timers.r12.enable_pwm_output(motor_b.tim_channel(), oc, 0.);
            }
        }
    }
}

/// Set the timer(s) to input mode. Used to receive PWM data in bidirectional mode.
/// We use motor args here, since this will run separately depending on the DSHOT
/// ISR for G4.
pub fn set_to_input(timers: &mut MotorTimers, motor_a: Motor, motor_b: Motor, three_four: bool) {
    let cc = CaptureCompare::InputTi1; // todo?
                                       // let cc = CaptureCompare::InputTrc; // todo?

    let pol_p = Polarity::ActiveLow;
    let pol_n = Polarity::ActiveHigh;

    // todo: Filter by Quad etc?

    cfg_if! {
        if #[cfg(feature = "h7")] {
            timers.rotors.set_input_capture(motor_a.tim_channel(), cc,  pol_p, pol_n);
            timers.rotors.set_input_capture(motor_b.tim_channel(), cc,  pol_p, pol_n);
        } else {
            if three_four {
                timers.r34.set_input_capture(motor_a.tim_channel(), cc, pol_p, pol_n);
                timers.r34.set_input_capture(motor_b.tim_channel(), cc, pol_p, pol_n);
            } else {
                timers.r12.set_input_capture(motor_a.tim_channel(), cc, pol_p, pol_n);
                timers.r12.set_input_capture(motor_b.tim_channel(), cc, pol_p, pol_n);
            }
        }
    }
}
