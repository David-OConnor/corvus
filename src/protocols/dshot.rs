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
//! The DSHOT protocol (DSHOT-300, DSHOT-600 etc) is determined by the `DSHOT_ARR_600` and
//! `DSHOT_PSC_600` settings; ie set a 600kHz countdown for DSHOT-600.

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::delay::Delay;

use stm32_hal2::{
    dma::{self, ChannelCfg, Priority},
    pac,
    timer::{CountDir, OutputCompare, Polarity},
};

use crate::setup::{self, MotorTimer, DSHOT_SPEED, TIM_CLK_SPEED};

// todo: Bidirectional: Set timers to active low, set GPIO idle to high, and perhaps set down counting
// todo if required. Then figure out input capture, and fix in HAL.

// todo (Probalby in another module) - RPM filtering, once you have bidirectional DSHOT working.
// Article: https://brushlesswhoop.com/betaflight-rpm-filter/
// todo: Basically, you set up a notch filter at rotor RPM. (I think; QC this)

use cfg_if::cfg_if;

use defmt::println;

// Enable bidirectional DSHOT, which returns RPM data
pub const BIDIR_EN: bool = false;

// Timer prescaler for rotor PWM. We leave this, and ARR constant, and explicitly defined,
// so we can set duty cycle appropriately for DSHOT.
// (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period.
// ARR = (TIMclk/Updatefrequency) / (PSC + 1) - 1
// With PSC being fixed at 0: ARR = TIMclk/UpdateFrequency - 1

pub const PSC_DSHOT: u16 = 0;

// ESC telemetry is false except when setting motor direction.
static mut ESC_TELEM: bool = false;

// We use these flags to determine how to handle the TC ISRs, ie when
// a send command is received, set the mode to input and vice versa.

// The number of motors here affects our payload interleave logic, and DMA burst length written.
const NUM_MOTORS: usize = 4;

pub const ARR_DSHOT: u32 = TIM_CLK_SPEED / DSHOT_SPEED - 1;
// Duty cycle values (to be written to CCMRx), based on our ARR value. 0. = 0%. ARR = 100%.
const DUTY_HIGH: u32 = ARR_DSHOT * 3 / 4;
const DUTY_LOW: u32 = ARR_DSHOT * 3 / 8;

// We use this during config that requires multiple signals sent, eg setting. motor direction.

// Use this pause duration, in ms, when setting up motor dir.
pub const PAUSE_BETWEEN_COMMANDS: u32 = 1;
pub const PAUSE_AFTER_SAVE: u32 = 40; // Must be at least 35ms.
                                      // BLHeli_32 requires you repeat certain commands, like motor direction, 6 times.
pub const REPEAT_COMMAND_COUNT: u32 = 10; // todo: Set back to 6 once sorted out.

// DMA buffers for each rotor. 16-bit data.
// Last 2 entries will be 0 per channel. Required to prevent extra pulses. (Not sure why)
static mut PAYLOAD: [u16; 18 * NUM_MOTORS] = [0; 18 * NUM_MOTORS];
// todo: The receive payload may be shorter due to how it's encoded; come back to this.

// The position we're reading when updating each motor's RPM read.
pub static M1_RPM_I: AtomicUsize = AtomicUsize::new(0);
pub static M2_RPM_I: AtomicUsize = AtomicUsize::new(0);
pub static M3_RPM_I: AtomicUsize = AtomicUsize::new(0);
pub static M4_RPM_I: AtomicUsize = AtomicUsize::new(0);

pub const REC_BUF_LEN: usize = 26; // More than we need

pub static mut PAYLOAD_REC_1: [u16; REC_BUF_LEN] = [0; REC_BUF_LEN];
pub static mut PAYLOAD_REC_2: [u16; REC_BUF_LEN] = [0; REC_BUF_LEN];
pub static mut PAYLOAD_REC_3: [u16; REC_BUF_LEN] = [0; REC_BUF_LEN];
pub static mut PAYLOAD_REC_4: [u16; REC_BUF_LEN] = [0; REC_BUF_LEN];

/// Specify the motor by its connection to the ESC.
// /// Includes methods that get information regarding timer
// /// and DMA, per specific board setups, in `setup`.
// /// Note that this is more appplicable to quads, but isn't in the `quad` module due to how
// /// we've structured DSHOT code.
#[derive(Clone, Copy)]
pub enum Motor {
    M1,
    M2,
    M3,
    M4,
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
pub fn stop_all(timer: &mut MotorTimer) {
    // Note that the stop command (Command 0) is currently not implemented, so set throttles to 0.
    set_power(0., 0., 0., 0., timer);
}

/// Set up the direction for each motor, in accordance with user config. Note: This blocks!
/// (at least for now). The intended use case is to run this only at init, and during Preflight,
/// if adjusting motor mapping.
pub fn setup_motor_dir(motors_reversed: (bool, bool, bool, bool), timer: &mut MotorTimer) {
    // A blocking delay.
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let mut delay = Delay::new(cp.SYST, 170_000_000);

    // Throttle must have been commanded to 0 a certain number of timers,
    // and the telemetry bit must be bit set to use commands.
    // Setting the throttle twice (with 1ms delay) doesn't work; 10x works. The required value is evidently between
    // these 2 bounds.
    for _ in 0..30 {
        stop_all(timer);
        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }
    // I've confirmed that setting direction without the telemetry bit set will fail.
    unsafe { ESC_TELEM = true };

    delay.delay_ms(PAUSE_BETWEEN_COMMANDS);

    // println!("Setting up motor direction");

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

        send_payload(timer);

        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }

    for _ in 0..REPEAT_COMMAND_COUNT {
        setup_payload(Motor::M1, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M2, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M3, CmdType::Command(Command::SaveSettings));
        setup_payload(Motor::M4, CmdType::Command(Command::SaveSettings));

        send_payload(timer);

        delay.delay_ms(PAUSE_BETWEEN_COMMANDS);
    }
    delay.delay_ms(PAUSE_AFTER_SAVE);

    unsafe { ESC_TELEM = false };
}

/// Calculate CRC. Used for both sending and receiving. `data` here does not include the
/// CRC itself, but contains the other 12 bits, right shifted 4.
pub fn calc_crc(data: u16) -> u16 {
    if BIDIR_EN {
        !(data ^ (data >> 4) ^ (data >> 8)) & 0x0F
    } else {
        (data ^ (data >> 4) ^ (data >> 8)) & 0x0F
    }
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

    let packet = (data_word << 1) | (unsafe { ESC_TELEM } as u16);

    // Compute the checksum
    let packet = (packet << 4) | calc_crc(packet);

    let offset = match rotor {
        Motor::M1 => 0,
        Motor::M2 => 1,
        Motor::M3 => 2,
        Motor::M4 => 3,
    };

    // Create a DMA payload of 16 timer CCR (duty) settings, each for one bit of our data word.
    for i in 0..16 {
        let bit = (packet >> i) & 1;
        let val = if bit == 1 { DUTY_HIGH } else { DUTY_LOW };
        // DSHOT uses MSB first alignment.
        // Values alternate in the buffer between the 4 registers we're editing, so
        // we interleave values here. (Each timer and DMA stream is associated with 2 channels).
        unsafe { PAYLOAD[(15 - i) * NUM_MOTORS + offset] = val as u16 };
    }

    // Note that the end stays 0-padded, since we init with 0s, and never change those values.
}

/// Set a rotor pair's power, using a 16-bit DHOT word, transmitted over DMA via timer CCR (duty)
/// settings. `power` ranges from 0. to 1.
pub fn set_power(power1: f32, power2: f32, power3: f32, power4: f32, timer: &mut MotorTimer) {
    setup_payload(Motor::M1, CmdType::Power(power1));
    setup_payload(Motor::M2, CmdType::Power(power2));
    setup_payload(Motor::M3, CmdType::Power(power3));
    setup_payload(Motor::M4, CmdType::Power(power4));

    send_payload(timer);
}

/// Set a single rotor's power. Used by preflight; not normal operations.
pub fn set_power_single(rotor: Motor, power: f32, timer: &mut MotorTimer) {
    setup_payload(rotor, CmdType::Power(power));
    send_payload(timer)
}

/// Send the stored payload.
fn send_payload(timer: &mut MotorTimer) {
    // Stop any transations in progress.
    dma::stop(setup::MOTORS_DMA_PERIPH, setup::MOTOR_CH);

    unsafe {
        timer.write_dma_burst(
            &PAYLOAD,
            setup::DSHOT_BASE_DIR_OFFSET,
            NUM_MOTORS as u8, // Update a channel per number of motors, up to 4.
            setup::MOTOR_CH,
            ChannelCfg {
                // Take precedence over CRSF and ADCs.
                priority: Priority::High,
                ..ChannelCfg::default()
            },
            true,
            setup::MOTORS_DMA_PERIPH,
        );
    }
}

/// Receive an RPM payload for all channels in bidirectional mode.
/// Note that we configure what won't affect the FC-ESC transmission in the reception timer's
/// ISR on payload-reception-complete. Here, we configure things that would affect transmission.
pub fn receive_payload() {
    // Set all motor pins to input mode.
    let input_mode = 0b00;
    unsafe {
        cfg_if! {
            if #[cfg(feature = "h7")] {
                (*pac::GPIOC::ptr())
                    .moder
                    .modify(|_, w| {
                        w.moder6().bits(input_mode);
                        w.moder7().bits(input_mode);
                        w.moder8().bits(input_mode);
                        w.moder9().bits(input_mode)
                    });

            } else {
                (*pac::GPIOC::ptr())
                    .moder
                    .modify(|_, w| w.moder6().bits(input_mode));
                (*pac::GPIOA::ptr())
                    .moder
                    .modify(|_, w| w.moder4().bits(input_mode));
                (*pac::GPIOB::ptr())
                    .moder
                    .modify(|_, w| {
                    w.moder0().bits(input_mode);
                    w.moder1().bits(input_mode)
                });
            }
        }
    }

    // Enable interrupts on the motor pins.
    let exti = unsafe { &(*pac::EXTI::ptr()) };
    cfg_if! {
        if #[cfg(feature = "h7")] {
            exti.cpuimr1.modify(|_, w| {
                w.mr6().set_bit();
                w.mr7().set_bit();
                w.mr8().set_bit();
                w.mr9().set_bit()
            });
        } else {
            exti.imr1.modify(|_, w| {
                w.im6().set_bit();
                w.im4().set_bit();
                w.im0().set_bit();
                w.im1().set_bit()
            });
        }
    }

    unsafe {
        (*pac::TIM2::ptr()).cr1.modify(|_, w| w.cen().set_bit());
    }
}

/// Change timer polarity and count direction, to enable or disable bidirectional DSHOT.
/// This results in the signal being active low for enabled, and active high for disabled.
/// Timer settings default (in HAL and hardware) to disabled.
pub fn set_bidirectional(enabled: bool, timer: &mut MotorTimer) {
    // todo: We need a way to configure channel 2 as a servo, eg for fixed-wing
    // todo with a rudder.

    let mut polarity = Polarity::ActiveHigh;
    let mut count_dir = CountDir::Up;

    if enabled {
        polarity = Polarity::ActiveLow;
        count_dir = CountDir::Down;
    }

    // Set up channels 1 and 2: This is for both quadcopters and fixed-wing.
    timer.set_polarity(Motor::M1.tim_channel(), polarity);
    timer.set_polarity(Motor::M2.tim_channel(), polarity);

    #[cfg(feature = "quad")]
    timer.set_polarity(Motor::M3.tim_channel(), polarity);
    #[cfg(feature = "quad")]
    timer.set_polarity(Motor::M4.tim_channel(), polarity);

    timer.cfg.direction = count_dir;
    timer.set_dir();
}

/// Set the timer(s) to output mode. This only runs on init.
pub fn set_to_output(timer: &mut MotorTimer) {
    let oc = OutputCompare::Pwm1;

    timer.set_auto_reload(ARR_DSHOT as u32);

    // todo: Here and elsewhere in this module, if you allocate timers/motors differently than 2/2
    // todo for fixed-wing, you'll need to change this logic.

    timer.enable_pwm_output(Motor::M1.tim_channel(), oc, 0.);
    timer.enable_pwm_output(Motor::M2.tim_channel(), oc, 0.);

    #[cfg(feature = "quad")]
    timer.enable_pwm_output(Motor::M3.tim_channel(), oc, 0.);
    #[cfg(feature = "quad")]
    timer.enable_pwm_output(Motor::M4.tim_channel(), oc, 0.);
}

/// This function, called in motor line EXTI ISRs, updates a motor's receive
/// RPM buffer with the current count, from the RPM-receive timer.
pub fn _update_rec_buf(rpm_i: &AtomicUsize, payload_rec: &mut [u16]) {
    let count = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() as u16 };

    let i = rpm_i.fetch_add(1, Ordering::Relaxed);

    // This shouldn't come up, but this ensures it won't overflow if it does for whatever
    // reason.
    if i >= REC_BUF_LEN {
        println!("Error: RPM I overflow");
        rpm_i.store(0, Ordering::Release);
    }

    unsafe {
        // We know the first edge is low, then alternates low, high.
        // payload_rec[i] = count; // todo: What the hell; why doesn't this work?
        PAYLOAD_REC_3[i] = count;
    }
}

// todo: We are having a mysterious problem with updating the receive buf using an argument.
// todo: Hard coded fns for now.

pub fn update_rec_buf_1(rpm_i: &AtomicUsize) {
    let count = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() as u16 };

    let mut i = rpm_i.fetch_add(1, Ordering::Relaxed);

    // This shouldn't come up, but this ensures it won't overflow if it does for whatever
    // reason.
    if i >= REC_BUF_LEN {
        println!("Error: RPM I 1 overflow");
        rpm_i.store(0, Ordering::Release);
        i = 0;
    }

    unsafe {
        // We know the first edge is low, then alternates low, high.
        PAYLOAD_REC_1[i] = count;
    }
}

pub fn update_rec_buf_2(rpm_i: &AtomicUsize) {
    let count = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() as u16 };

    let mut i = rpm_i.fetch_add(1, Ordering::Relaxed);

    if i >= REC_BUF_LEN {
        println!("Error: RPM I 2 overflow");
        rpm_i.store(0, Ordering::Release);
        i = 0;
    }

    unsafe {
        PAYLOAD_REC_2[i] = count;
    }
}

pub fn update_rec_buf_3(rpm_i: &AtomicUsize) {
    let count = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() as u16 };

    let mut i = rpm_i.fetch_add(1, Ordering::Relaxed);

    if i >= REC_BUF_LEN {
        println!("Error: RPM I 3 overflow");
        rpm_i.store(0, Ordering::Release);
        i = 0;
    }

    unsafe {
        PAYLOAD_REC_3[i] = count;
    }
}

pub fn update_rec_buf_4(rpm_i: &AtomicUsize) {
    let count = unsafe { (*pac::TIM2::ptr()).cnt.read().bits() as u16 };

    let mut i = rpm_i.fetch_add(1, Ordering::Relaxed);

    if i >= REC_BUF_LEN {
        println!("Error: RPM I 4 overflow");
        rpm_i.store(0, Ordering::Release);
        i = 0;
    }

    unsafe {
        PAYLOAD_REC_4[i] = count;
    }
}
