#![no_main]
#![no_std]
// Used on USB protocol. Allows adding to the const param buff size
// to make packet size.

// Potential markets:
// - Hobby / racing (duh)
// Tower inspections (Maybe market disruption by using fixed-wing + tpod?)
//
// https://www.youtube.com/watch?v=zOByx3Izf5U
// For state estimation
// https://www.youtube.com/watch?v=RZd6XDx5VXo (Series)
// https://www.youtube.com/watch?v=whSw42XddsU
// https://www.youtube.com/playlist?list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a
// For quadrotor PID control
// https://www.youtube.com/playlist?list=PLn8PRpmsu08oOLBVYYIwwN_nvuyUqEjrj
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y
// https://www.youtube.com/playlist?list=PLn8PRpmsu08pFBqgd_6Bi7msgkWFKL33b
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::{self, asm};
use rtic::app;

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use ahrs::{Ahrs, Fix, FixType, Params};

use stm32_hal2::{
    self,
    adc::Adc,
    dma::{self, ChannelCfg, DmaInterrupt},
    flash::Flash,
    gpio::{self, Pin},
    i2c::I2c,
    iwdg,
    pac::{self, I2C1, I2C2, SPI1, TIM1, TIM2, TIM5},
    spi::Spi,
    timer::{Timer, TimerInterrupt},
    usart::UsartInterrupt,
};

use usb_device::prelude::*;
use usbd_serial::{self, SerialPort};

use cfg_if::cfg_if;

mod atmos_model;
mod can_reception;
mod control_interface;
mod drivers;
mod flight_ctrls;
mod imu_processing;
mod init;
mod main_loop;
mod protocols;
mod safety;
mod sensors_shared;
mod setup;
mod state;
mod system_status;
mod util;

use crate::{
    control_interface::ChannelData,
    drivers::{
        baro_dps310 as baro, gps_ublox as gnss, imu_icm426xx as imu, mag_lis3mdl as mag, osd,
        tof_vl53l1 as tof,
    },
    flight_ctrls::{
        autopilot::AutopilotStatus,
        ctrl_effect_est::AccelMaps,
        filters::FlightCtrlFilters,
        pid::{MotorCoeffs, MotorPidGroup},
    },
    imu_processing::{filter_imu::ImuFilters, imu_shared},
    protocols::{
        crsf::{self, LinkStats},
        dshot, msp, usb_preflight,
    },
    sensors_shared::ExtSensor,
    state::{StateVolatile, UserConfig},
    system_status::SystemStatus,
};

cfg_if! {
    if #[cfg(feature = "h7")] {
        use stm32_hal2::{
            clocks::{PllCfg, VosRange},
            // todo: USB1 on H723; USB2 on H743.
            // usb::{Usb1, UsbBus, Usb1BusType as UsbBusType},
            usb::{Usb2, UsbBus, Usb2BusType as UsbBusType},
            // pac::OCTOSPI1,
            pac::QUADSPI,
            qspi::{Qspi},
        };
        // This USART alias is made pub here, so we don't repeat this line in other modules.
        pub use stm32_hal2::pac::{ADC1 as ADC};
    } else if #[cfg(feature = "g4")] {
        use stm32_hal2::{
            usb::{ UsbBusType},
        };

        pub use stm32_hal2::pac::{UART4, ADC2 as ADC};
    }
}

cfg_if! {
    if #[cfg(feature = "fixed-wing")] {
        // use flight_ctrls::{autopilot::Orbit, ControlPositions, };
    } else {
    }
}

// todo: Can't get startup code working separately since Shared and Local must be private per an RTIC restriction.
// todo: See this GH issue: https://github.com/rtic-rs/cortex-m-rtic/issues/505
// mod startup;

// todo: Cycle flash pages for even wear. Can postpone this.

// If IMU updates at 8kHz and ratio is 4, the flight control loop operates at 2kHz.

cfg_if! {
    if #[cfg(feature = "h7")] {
        // H723: 1Mb of flash, in one bank.
        // 8 sectors of 128kb each.
        // (H743 is similar, but may have 2 banks, each with those properties)
        const FLASH_CFG_PAGE: usize = 6; // called sector on H7.
        const FLASH_WAYPOINT_SECTOR: usize = 7;
    } else {
        // G47x/G48x: 512k flash.
        // Assumes configured as a single bank: 128 pages of 4kb each.
        // (If using G4 dual bank mode: 128 pages of pages of 2kb each, per bank)
        const FLASH_CFG_PAGE: usize = 126;
        const FLASH_WAYPOINT_PAGE: usize = 127;
    }
}

#[cfg(feature = "h7")]
static mut USB_EP_MEMORY: [u32; 1024] = [0; 1024];

// todo: Temp as we switch from PID to other controls; we still will have
// todo params that can be adjusetd in flight.
const CTRL_COEFF_ADJ_TIMEOUT: f32 = 0.3; // seconds
const CTRL_COEFF_ADJ_AMT: f32 = 0.01; // seconds

// We use a hardware counter to measure relative system time. This is the number of times
// it has overflowed. (timer expired)
const TICK_TIMER_PERIOD: f32 = 0.5; // in seconds. Decrease for higher measurement precision.
pub static TICK_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

static mut CAN_BUF_RX: [u8; 64] = [0; 64];

//// The time, in ms, to wait during initializing to allow the ESC and RX to power up and initialize.
// const WARMUP_TIME: u32 = 100;

// todo: Bit flags that display as diff colored LEDs, and OSD items

#[rtic::app(device = pac, peripherals = false)]
mod app {
    use super::*;
    use ahrs::ppks::PositInertial;
    use stm32_hal2::dma::DmaPeriph;

    #[shared]
    pub struct Shared {
        pub user_cfg: UserConfig,
        pub state_volatile: StateVolatile,
        pub system_status: SystemStatus,
        pub autopilot_status: AutopilotStatus,
        pub current_params: Params,
        // None if the data is stale. eg lost link, no link established.
        pub control_channel_data: Option<ChannelData>,
        /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
        pub link_stats: LinkStats,
        pub spi1: Spi<SPI1>,
        pub i2c1: I2c<I2C1>,
        pub i2c2: I2c<I2C2>,
        pub uart_osd: setup::UartOsd, // for our DJI OSD, via MSP protocol
        pub altimeter: baro::Altimeter,
        pub flash_onboard: Flash,
        pub motor_timer: setup::MotorTimer,
        pub servo_timer: setup::ServoTimer,
        pub usb_dev: UsbDevice<'static, UsbBusType>,
        pub usb_serial: SerialPort<'static, UsbBusType>,
        /// `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        pub power_used: f32,
        pub imu_filters: ImuFilters,
        pub flight_ctrl_filters: FlightCtrlFilters,
        // Note: We don't currently haveh PID filters, since we're not using a D term for the
        // RPM PID.
        pub ext_sensor_active: ExtSensor,
        pub pwr_maps: AccelMaps,
        // /// Store rotor RPM: (M1, M2, M3, M4). Quad only, but we can't feature gate
        // /// shared fields.
        // rpm_readings: RpmReadings,
        // rpms_commanded: MotorRpm,
        pub motor_pid_state: MotorPidGroup,
        /// PID motor coefficients
        pub motor_pid_coeffs: MotorCoeffs,
        pub tick_timer: Timer<TIM5>,
        pub can: setup::Can_,
        pub fix: Fix,
        pub ahrs: Ahrs,
        pub posit_inertial: PositInertial,
    }

    #[local]
    pub struct Local {
        // update_timer: Timer<TIM15>,
        pub uart_crsf: setup::UartCrsf, // for ELRS over CRSF.
        pub uart_gnss: setup::UartGnss, // for ELRS over CRSF.
        // spi_flash: SpiFlash,  // todo: Fix flash in HAL, then do this.
        pub arm_signals_received: u8, // todo: Put sharedin state volatile.
        pub disarm_signals_received: u8,
        /// We use this counter to subdivide the main loop into longer intervals,
        /// for various tasks like logging, and outer loops.
        // update_isr_loop_i: usize,
        pub imu_isr_loop_i: u32,
        // aux_loop_i: usize, // todo temp
        pub ctrl_coeff_adj_timer: Timer<TIM1>,
        pub time_with_high_throttle: f32,
        pub dshot_read_timer: Timer<TIM2>,
        pub cs_imu: Pin,
        // todo: `params_prev` is an experimental var used in our alternative/experimental
        // todo flight controls code as a derivative.
        pub params_prev: Params,
        pub batt_curr_adc: Adc<ADC>,
        /// In seconds. Used to track main loop task durations. The 0 index is for the
        /// part of the main loop that runs every time.
        pub task_durations: main_loop::TaskDurations,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        crate::init::run(cx)
    }

    #[idle(shared = [], local = [])]
    /// In this function, we perform setup code that must occur with interrupts enabled.
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    /// Runs when new IMU data is ready. Trigger a DMA read.
    /// High priority since it's important, and quick-to-execute
    #[task(binds = EXTI15_10,
    shared = [spi1], local = [], priority = 7)]
    fn imu_data_isr(mut cx: imu_data_isr::Context) {
        #[cfg(feature = "h7")]
        gpio::clear_exti_interrupt(12); // PB12
        #[cfg(feature = "g4")]
        gpio::clear_exti_interrupt(13); // PC13

        // println!("IMU r");

        cx.shared.spi1.lock(|spi| {
            imu_shared::read_imu(imu::READINGS_START_ADDR, spi, setup::IMU_DMA_PERIPH);
        });
    }

    /// This ISR Handles received data from the IMU, after DMA transfer is complete. This occurs whenever
    /// we receive IMU data; it nominally (and according to our measurements so far) runs at 8kHz.
    /// Note that on the H7 FC with the dedicated IMU LSE, it may run slightly faster.
    ///
    /// Certain tasks, like reading IMU measurements and filtering are run each time this function runs.
    /// Flight control logic is run once every several runs. Other tasks are run even less,
    /// sequenced among each other.
    // #[task(binds = DMA1_STR2,
    #[task(binds = DMA1_CH2,
    shared = [altimeter, ahrs, spi1, i2c1, i2c2, current_params, control_channel_data, link_stats,
    autopilot_status, imu_filters, flight_ctrl_filters, user_cfg, motor_pid_state, motor_pid_coeffs,
    motor_timer, servo_timer, state_volatile, system_status, tick_timer, uart_osd],
    local = [imu_isr_loop_i, cs_imu, params_prev, time_with_high_throttle,
    arm_signals_received, disarm_signals_received, batt_curr_adc, task_durations], priority = 4)]
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        dma::clear_interrupt(
            setup::IMU_DMA_PERIPH,
            setup::IMU_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        cx.local.cs_imu.set_high();

        cx.shared.spi1.lock(|spi1| {
            // Note that this step is mandatory, per STM32 RM.
            spi1.stop_dma(
                setup::IMU_TX_CH,
                Some(setup::IMU_RX_CH),
                setup::IMU_DMA_PERIPH,
            );
        });

        iwdg::refresh();

        main_loop::run(cx);
    }

    // todo H735 issue on GH: https://github.com/stm32-rs/stm32-rs/issues/743 (works on H743)
    // todo: NVIC interrupts missing here for H723 etc!
    // #[task(binds = OTG_HS,
    // #[task(binds = OTG_FS,
    #[task(binds = USB_LP,
    shared = [usb_dev, usb_serial, current_params, control_channel_data, flash_onboard,
    link_stats, user_cfg, state_volatile, system_status, autopilot_status, motor_timer, servo_timer],
    local = [], priority = 10)]
    /// This ISR handles interaction over the USB serial port, eg for configuring using a desktop
    /// application. It should be a high priority, or the host may disconnect the device for not responding
    /// quickly enough. If the priority is too low, the PC interface software will behave strangely,
    /// so this is somewhat self-critiquing.
    /// *It appears we need to set this to be a lower priority than IMU data, but higher than IMU TC.
    fn usb_isr(cx: usb_isr::Context) {
        // todo: Do we want to use an approach where we push stats, or this approach where
        // todo respond only?
        (
            cx.shared.usb_dev,
            cx.shared.usb_serial,
            cx.shared.current_params,
            cx.shared.control_channel_data,
            cx.shared.link_stats,
            cx.shared.user_cfg,
            cx.shared.state_volatile,
            cx.shared.system_status,
            cx.shared.autopilot_status,
            cx.shared.motor_timer,
            cx.shared.servo_timer,
            cx.shared.flash_onboard,
            // cx.shared.rpm_readings,
        )
            .lock(
                |usb_dev,
                 usb_serial,
                 params,
                 ch_data,
                 link_stats,
                 user_cfg,
                 state_volatile,
                 system_status,
                 autopilot_status,
                 motor_timer,
                 servo_timer,
                 flash,
                 // rpm_readings
                | {
                    if !usb_dev.poll(&mut [usb_serial]) {
                        return;
                    }

                    let mut buf = [0u8; 60]; // todo: Adjust this A/R!!!
                    match usb_serial.read(&mut buf) {
                        Ok(_count) => {
                            usb_preflight::handle_rx(
                                usb_serial,
                                &buf,
                                params.attitude,
                                &state_volatile.attitude_commanded,
                                params.alt_msl_baro,
                                state_volatile.pressure_static,
                                state_volatile.temp_baro,
                                params.alt_tof,
                                state_volatile.batt_v,
                                state_volatile.esc_current,
                                ch_data,
                                &link_stats,
                                user_cfg,
                                system_status,
                                autopilot_status,
                                &mut state_volatile.arm_status,
                                // &mut user_cfg.control_mapping,
                                &mut state_volatile.op_mode,
                                motor_timer,
                                servo_timer,
                                &mut state_volatile.motor_servo_state,
                                &mut state_volatile.preflight_motors_running,
                                flash,
                            );
                        }
                        Err(_) => {
                            // println!("Error reading USB signal from PC");
                        }
                    }
                },
            )
    }

    // #[task(binds = DMA1_STR3,
    #[task(binds = DMA1_CH3,
    shared = [motor_timer], priority = 6)]
    /// We use this ISR to initialize the RPM reception procedures upon completion of the dshot
    /// power setting transmission to the ESC.
    fn dshot_isr(mut _cx: dshot_isr::Context) {
        dma::clear_interrupt(
            setup::MOTORS_DMA_PERIPH,
            setup::MOTOR_CH,
            DmaInterrupt::TransferComplete,
        );

        // (From testing) We must stop this transaction manually before future transactions will work.
        dma::stop(setup::MOTORS_DMA_PERIPH, setup::MOTOR_CH);

        _cx.shared.motor_timer.lock(|motor_timer| {
            motor_timer.disable();

            if dshot::BIDIR_EN {
                dshot::M1_RPM_I.store(0, Ordering::Release);
                dshot::M2_RPM_I.store(0, Ordering::Release);
                dshot::M3_RPM_I.store(0, Ordering::Release);
                dshot::M4_RPM_I.store(0, Ordering::Release);

                // Make sure to clear these buffers at reception start, not after completion; if we do it after,
                // they will be blanked before we can process them.
                unsafe {
                    dshot::PAYLOAD_REC_1 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_2 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_3 = [0; dshot::REC_BUF_LEN];
                    dshot::PAYLOAD_REC_4 = [0; dshot::REC_BUF_LEN];
                }

                dshot::receive_payload();
            }
        });
    }

    #[task(binds = EXTI9_5, priority = 10)]
    /// We use this to read RPM status on motor 1 on G4, or any motor on H7.
    /// Triggers on rising and falling edges.
    /// High priority since this is time-sensitive, and fast-to-execute.
    fn rpm_read_m1(_cx: rpm_read_m1::Context) {
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        // Determine which line fired. I'm not sure if it's going to be generally (always?)
        // a single line, or multiple ones.
        cfg_if! {
            if #[cfg(feature = "h7")] {
                let pr = exti.cpupr1.read();
                // Don't use if/else, in case multiple fire simultaneously, which seems likely.
                if pr.pr6().bit_is_set() {
                    gpio::clear_exti_interrupt(6);
                    dshot::update_rec_buf_1(&dshot::M1_RPM_I);
                }

                if pr.pr7().bit_is_set() {
                    gpio::clear_exti_interrupt(7);
                    dshot::update_rec_buf_2(&dshot::M2_RPM_I);
                }

                if pr.pr8().bit_is_set() {
                    gpio::clear_exti_interrupt(8);
                    dshot::update_rec_buf_3(&dshot::M3_RPM_I);
                }

                if pr.pr9().bit_is_set() {
                    gpio::clear_exti_interrupt(9);
                    dshot::update_rec_buf_4(&dshot::M4_RPM_I);
                }
            } else {
                // On G4, this is only for Motor 1.
                gpio::clear_exti_interrupt(6);
                dshot::update_rec_buf_1(&dshot::M1_RPM_I);
            }
        }
    }

    #[task(binds = EXTI4, priority = 10)]
    /// Similar to `rpm_read_m1`, but for M2, on G4 only.
    fn rpm_read_m2(_cx: rpm_read_m2::Context) {
        gpio::clear_exti_interrupt(4);
        // println!("2"); // todo: This is on rapid fire. Why?

        dshot::update_rec_buf_2(&dshot::M2_RPM_I);
    }

    #[task(binds = EXTI0, priority = 10)]
    /// Similar to `rpm_read_m1`, but for M3, on G4 only.
    fn rpm_read_m3(_cx: rpm_read_m3::Context) {
        gpio::clear_exti_interrupt(0);

        dshot::update_rec_buf_3(&dshot::M3_RPM_I);
    }

    #[task(binds = EXTI1, priority = 10)]
    /// Similar to `rpm_read_m1`, but for M4, on G4 only.
    fn rpm_read_m4(_cx: rpm_read_m4::Context) {
        gpio::clear_exti_interrupt(1);

        dshot::update_rec_buf_4(&dshot::M4_RPM_I);
    }

    #[task(binds = TIM2, shared = [], local = [dshot_read_timer], priority = 9)]
    /// This interrupt fires slightly after the last bit of RPM data is received.
    /// Its timer is started once power setting is transmitted.
    /// In this ISR, we disable reception, and return the DSHOT lines to an output
    /// state.
    fn dshot_read_isr(mut cx: dshot_read_isr::Context) {
        let timer = &mut cx.local.dshot_read_timer; // code shortener.
        timer.clear_interrupt(TimerInterrupt::Update);
        timer.disable();

        // Disable interrupts on the motor pins.
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        cfg_if! {
            if #[cfg(feature = "h7")] {
                exti.cpuimr1.modify(|_, w| {
                    w.mr6().clear_bit();
                    w.mr7().clear_bit();
                    w.mr8().clear_bit();
                    w.mr9().clear_bit()
                });
            } else {
                exti.imr1.modify(|_, w| {
                    w.im6().clear_bit();
                    w.im4().clear_bit();
                    w.im0().clear_bit();
                    w.im1().clear_bit()
                });
            }
        }

        // Set motor pins back to their timer alt fn.
        let alt_mode = 0b10;
        unsafe {
            cfg_if! {
                if #[cfg(feature = "h7")] {
                    (*pac::GPIOC::ptr())
                        .moder
                        .modify(|_, w| {
                            w.moder6().bits(alt_mode);
                            w.moder7().bits(alt_mode);
                            w.moder8().bits(alt_mode);
                            w.moder9().bits(alt_mode)
                        });

                } else {
                    (*pac::GPIOC::ptr())
                        .moder
                        .modify(|_, w| w.moder6().bits(alt_mode));
                    (*pac::GPIOA::ptr())
                        .moder
                        .modify(|_, w| w.moder4().bits(alt_mode));
                    (*pac::GPIOB::ptr())
                        .moder
                        .modify(|_, w| {
                        w.moder0().bits(alt_mode);
                        w.moder1().bits(alt_mode)
                    });
                }
            }
        }
        // We interpret data in the main loop; hot here.
    }

    // todo: Evaluate priority.
    // #[task(binds = UART7,
    #[task(binds = USART2,
// shared = [control_channel_data, link_stats, system_status,
//], local = [uart_crsf], priority = 8)]
    shared = [], local = [uart_crsf], priority = 8)]
    /// This ISR handles CRSF reception. It handles, in an alternating fashion, message starts,
    /// and message ends. For message starts, it begins a DMA transfer. For message ends, it
    /// processes the radio data, passing it into shared resources for control channel data,
    /// and link stats.
    ///
    /// Ideally, the only locks we have here are things used in lower-priority ISRs,
    /// like link timer etc.
    ///
    /// Must be a higher priority than the IMU TC isr.
    fn crsf_isr(mut cx: crsf_isr::Context) {
        let uart = &mut cx.local.uart_crsf; // Code shortener

        let start_of_message = uart.regs.isr.read().cmf().bit_is_set();

        uart.clear_interrupt(UsartInterrupt::CharDetect(None));
        uart.clear_interrupt(UsartInterrupt::Idle);

        // todo: Store link stats and control channel data in an intermediate variable.
        // todo: Don't lock it. At least, you don't want any delay when starting the read,
        // todo although a delay on finishing the read is fine.

        // Stop the DMA read, since it will likely not have filled the buffer, due
        // to the variable message sizies.
        dma::stop(setup::CRSF_DMA_PERIPH, setup::CRSF_RX_CH);

        let transfer_in_prog = crsf::TRANSFER_IN_PROG.load(Ordering::Acquire);

        // Not sure why we need the additional message start check here.
        if transfer_in_prog == false && start_of_message {
            crsf::TRANSFER_IN_PROG.store(true, Ordering::Release);

            // Don't allow the starting char, as used in the middle of a message,
            // to trigger an interrupt.
            uart.disable_interrupt(UsartInterrupt::CharDetect(None));

            unsafe {
                uart.read_dma(
                    &mut crsf::RX_BUFFER,
                    setup::CRSF_RX_CH,
                    ChannelCfg {
                        // Take precedence over the ADC, but not motors.
                        priority: dma::Priority::Medium,
                        ..Default::default()
                    },
                    setup::CRSF_DMA_PERIPH,
                );
            }
        } else if transfer_in_prog == true {
            crsf::TRANSFER_IN_PROG.store(false, Ordering::Release);
            // Line is idle.

            // A `None` value here re-enables the interrupt without changing the char to match.
            uart.enable_interrupt(UsartInterrupt::CharDetect(None));

            crsf::NEW_PACKET_RECEIVED.store(true, Ordering::Release);
        } else {
            // println!("Spurious IDLE on CRSF reception");
        }
    }

    // todo: Diff UART on H7
    #[task(binds = UART4, shared = [uart_osd, state_volatile, system_status, tick_timer], local = [], priority = 2)]
    fn osd_rec_isr(mut cx: osd_rec_isr::Context) {
        cx.shared.uart_osd.lock(|uart| {
            uart.clear_interrupt(UsartInterrupt::CharDetect(None));

            let timestamp = cx
                .shared
                .tick_timer
                .lock(|timer| util::get_timestamp(timer));

            cx.shared.system_status.lock(|status| {
                status.update_timestamps.osd = Some(timestamp);
            });

            // Messages BF sends:
            // 182: MSP_DISPLAYPORT, payload = 0 (heartbeat)

            if osd::OSD_INTERRUPT_CYCLE.load(Ordering::Acquire) {
                osd::OSD_INTERRUPT_CYCLE.store(false, Ordering::Release);
                uart.disable_interrupt(UsartInterrupt::CharDetect(None));
                uart.enable_interrupt(UsartInterrupt::CharDetect(Some(msp::MSG_ID_STATUS)));
            } else {
                osd::OSD_INTERRUPT_CYCLE.store(true, Ordering::Release);
                uart.disable_interrupt(UsartInterrupt::CharDetect(None));
                uart.enable_interrupt(UsartInterrupt::CharDetect(Some(msp::MSG_ID_FC_TYPE)));
            }

            // This interrupt triggered by matching the status function;
            // send a status indicating the device is armed.

            if osd::OSD_WRITE_IN_PROGRESS.load(Ordering::Acquire) {
                return;
            }

            osd::OSD_WRITE_IN_PROGRESS.store(true, Ordering::Release);

            cx.shared.state_volatile.lock(|state_volatile| {
                // todo: This can probably be set up once on init, since it never changes.
                osd::make_arm_status_buf(state_volatile.arm_status == safety::MOTORS_ARMED);
            });

            // todo: Put back
            unsafe {
                uart.write_dma(
                    &osd::OSD_ARM_BUF,
                    setup::OSD_TX_CH,
                    Default::default(),
                    setup::OSD_DMA_PERIPH,
                )
            };
        });
    }

    #[task(binds = DMA2_CH3, shared = [], priority = 2)]
    /// Baro write complete; start baro read.
    fn osd_tx_isr(_cx: osd_tx_isr::Context) {
        dma::clear_interrupt(
            setup::OSD_DMA_PERIPH,
            setup::OSD_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        // This appears to be a required step between UART DMA transmission.
        dma::stop(setup::OSD_DMA_PERIPH, setup::OSD_TX_CH);
        osd::OSD_WRITE_IN_PROGRESS.store(false, Ordering::Release);
    }

    #[task(binds = TIM5, shared = [tick_timer], local = [], priority = 1)]
    /// Increments the tick overflow.
    fn tick_isr(mut cx: tick_isr::Context) {
        cx.shared.tick_timer.lock(|timer| {
            // todo: Do this without locking.
            timer.clear_interrupt(TimerInterrupt::Update);
        });

        TICK_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    // #[task(binds = DMA2_STR1,
    #[task(binds = DMA2_CH1,
    shared = [i2c2], priority = 5)]
    /// Baro write complete; start baro read.
    fn baro_write_tc_isr(mut cx: baro_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_TX_CH);

        cx.shared.i2c2.lock(|i2c| unsafe {
            i2c.read_dma(
                baro::ADDR,
                &mut sensors_shared::READ_BUF_BARO,
                setup::BARO_RX_CH,
                Default::default(),
                setup::BARO_DMA_PERIPH,
            );
        });
    }

    // todo: For now, we start new transfers in the main loop.

    // #[task(binds = DMA2_STR2,
    #[task(binds = DMA2_CH2,
    shared = [altimeter, current_params, state_volatile, system_status, tick_timer], priority = 2)]
    /// Baro read complete; handle data, and start next write.
    fn baro_read_tc_isr(mut cx: baro_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_RX_CH);

        let buf = unsafe { &sensors_shared::READ_BUF_BARO };

        // todo: This is fragile, esp re the 11. figure, and if this figure still makes sense
        // todo if we change the values.
        const DT_BARO: f32 = main_loop::DT_IMU
            * (main_loop::FLIGHT_CTRL_IMU_RATIO as f32
                * main_loop::NUM_IMU_LOOP_TASKS as f32
                * main_loop::BARO_RATIO as f32);

        (
            cx.shared.altimeter,
            cx.shared.current_params,
            cx.shared.state_volatile,
        )
            .lock(|altimeter, params, state_volatile| {
                // todo: Process your baro reading here.
                let (pressure, temp) = altimeter.pressure_temp_from_readings(buf);

                state_volatile.pressure_static = pressure;
                state_volatile.temp_baro = temp;

                let altitude =
                    atmos_model::estimate_altitude_msl(pressure, temp, &altimeter.ground_cal);

                // todo: We must low-pass this, or take a wider window.
                params.v_z_baro = (altitude - params.alt_msl_baro) / DT_BARO;

                params.alt_msl_baro = altitude;
            });

        let timestamp = cx.shared.tick_timer.lock(util::get_timestamp);

        cx.shared.system_status.lock(|status| {
            status.update_timestamps.baro = Some(timestamp);
        });
    }

    #[task(binds = USART1, shared = [tick_timer, fix, current_params, posit_inertial, ahrs, system_status],
    local = [uart_gnss], priority = 10)]
    fn gnss_isr(mut cx: gnss_isr::Context) {
        let uart = cx.local.uart_gnss;

        uart.clear_interrupt(UsartInterrupt::CharDetect(None));
        uart.clear_interrupt(UsartInterrupt::Idle);

        // Stop the DMA read, since it will likely not have filled the buffer, due
        // to the variable message sizes.
        dma::stop(setup::GNSS_DMA_PERIPH, setup::GNSS_RX_CH);

        if !gnss::TRANSFER_IN_PROG.load(Ordering::Acquire) {
            // println!("GPS A");
            gnss::TRANSFER_IN_PROG.store(true, Ordering::Release);

            uart.disable_interrupt(UsartInterrupt::CharDetect(None));

            let mut buf = [0; 40];
            // uart.read(&mut buf);
            // println!("BUF: {:?}", buf);
            // return;

            unsafe {
                uart.read_dma(
                    &mut gnss::RX_BUFFER,
                    setup::GNSS_RX_CH,
                    ChannelCfg {
                        ..Default::default()
                    },
                    setup::GNSS_DMA_PERIPH,
                );
            }
        } else {
            // println!("GPS B");
            gnss::TRANSFER_IN_PROG.store(false, Ordering::Release);

            // A `None` value here re-enables the interrupt without changing the char to match.
            uart.enable_interrupt(UsartInterrupt::CharDetect(None));

            // Re-enable the char match interrupt.
            let uart_pac = unsafe { &(*pac::USART1::ptr()) };
            uart_pac.cr1.modify(|_, w| w.cmie().set_bit());

            // todo: TIck timer here appears broken.
            let timestamp = cx.shared.tick_timer.lock(util::get_timestamp);

            cx.shared.system_status.lock(|status| {
                status.update_timestamps.gnss = Some(timestamp);
            });

            static mut I: u32 = 0;
            let i = unsafe { I };

            match gnss::Message::from_buf(unsafe { &gnss::RX_BUFFER }) {
                Ok(m) => {
                    println!("OK GPS");
                    match m.class_id {
                        gnss::MsgClassId::NavPvt => {
                            match gnss::fix_from_payload(m.payload, timestamp) {
                                Ok(fix) => {
                                    println!("Fix received");
                                    unsafe { I += 1 };

                                    match fix.type_ {
                                        FixType::Fix3d | FixType::Combined => {
                                            cx.shared.posit_inertial.lock(|posit_inertial| {
                                                posit_inertial.update_anchor(&fix);
                                            });

                                            cx.shared.fix.lock(|fix_resource| {
                                                // todo: Maybe update method on Params for this? here for now.
                                                // todo while we test it.
                                                let dt_fix =
                                                    fix.timestamp_s - fix_resource.timestamp_s;
                                                //
                                                // let gnss_acc_nse =
                                                //     (ahrs::ppks::ned_vel_to_xyz(
                                                //         fix.ned_velocity,
                                                //     ) - ahrs::ppks::ned_vel_to_xyz(
                                                //         fix_resource.ned_velocity,
                                                //     )) / dt_fix;

                                                cx.shared.ahrs.lock(|ahrs| {
                                                    ahrs.update_from_fix(&fix);

                                                    // ahrs::attitude::heading_from_gnss_acc(
                                                    //     gnss_acc_nse,
                                                    //     ahrs.linear_acc_estimate,
                                                    // );
                                                });

                                                *fix_resource = fix;
                                            });
                                        }
                                        _ => {}
                                    }
                                }
                                // Ie, a fault etc.
                                Err(_) => {
                                    // todo: PUt back until solved. Decluttered for now.
                                    // println!("GNSS error while reading a fix");
                                }
                            }
                        }
                        gnss::MsgClassId::NavDop => {
                            if let Ok(dop_) = gnss::DilutionOfPrecision::from_payload(m.payload) {
                                // cx.shared.dilution_of_precision.lock(|dop| {
                                //     *dop = dop_;
                                // });
                            }
                        }
                        gnss::MsgClassId::NavCov => {
                            // println!("Nav covariance");
                            if let Ok(cov) = gnss::Covariance::from_payload(m.payload) {
                                // *covariance = cov;
                            }
                        }
                        gnss::MsgClassId::AckAck => {
                            println!("\n\n\nAck");
                        }
                        gnss::MsgClassId::AckNak => {
                            println!("\n\n\nNACK");
                        }
                        _ => {
                            println!(
                                "Unrecognized GNSS ID received. Class: {}. Id: {}",
                                m.class_id.to_vals().0,
                                m.class_id.to_vals().1
                            );
                        }
                    }
                }
                Err(_e) => {
                    // println!("ERror GPS");
                    // todo: Put this back and troubleshoot it. Getting it a lot.
                    // todo: Jitter?
                    // println!("Error parsing GNSS message");
                }
            }
        }
    }

    // #[task(binds = FDCAN1_IT0,
    #[task(binds = FDCAN1_INTR0_IT,
    shared = [can], priority = 4)] // todo: Temp high prio
    /// Ext sensors write complete; start read of the next sensor in sequence.
    fn can_isr(cx: can_isr::Context) {
        can_reception::run(cx);
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
