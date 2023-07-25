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

use cfg_if::cfg_if;

use cortex_m::{self, asm, delay::Delay};

use defmt::println;
use defmt_rtt as _;
use panic_probe as _;

use ahrs::{Ahrs, DeviceOrientation, Params};

use stm32_hal2::{
    self,
    adc::{self, Adc, AdcConfig, AdcDevice},
    clocks::{self, Clocks, CrsSyncSrc, InputSrc, PllSrc},
    dma::{self, ChannelCfg, Dma, DmaInterrupt, DmaPeriph},
    flash::{Bank, Flash},
    gpio::{self, Pin},
    i2c::I2c,
    pac::{self, I2C1, I2C2, SPI1, TIM1, TIM16, TIM17, TIM2, TIM5},
    spi::Spi,
    timer::{Timer, TimerConfig, TimerInterrupt},
    usart::UsartInterrupt,
};

use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{self, SerialPort};

use packed_struct::PackedStruct;

use fdcan::{id::Id, interrupt::Interrupt};

use dronecan::{self, f16};

mod atmos_model;
mod cfg_storage;
mod control_interface;
mod drivers;
mod flight_ctrls;
mod imu_processing;
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
        dshot, usb_preflight,
    },
    sensors_shared::{ExtSensor, V_A_ADC_READ_BUF},
    state::{StateVolatile, UserCfg},
    system_status::{SensorStatus, SystemStatus},
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
            usb::{self, UsbBus, UsbBusType},
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

// Due to the way the USB serial lib is set up, the USB bus must have a static lifetime.
// In practice, we only mutate it at initialization.
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

// todo: Can't get startup code working separately since Shared and Local must be private per an RTIC restriction.
// todo: See this GH issue: https://github.com/rtic-rs/cortex-m-rtic/issues/505
// mod startup;

// todo: Cycle flash pages for even wear. Can postpone this.

// If IMU updates at 8kHz and ratio is 4, the flight control loop operates at 2kHz.

// todo: Move this A/R if you end up using it.
static mut RX_BUF_CAN: [u8; 64] = [0; 64];

cfg_if! {
    if #[cfg(feature = "h7")] {
        // H723: 1Mb of flash, in one bank.
        // 8 sectors of 128kb each.
        // (H743 is similar, but may have 2 banks, each with those properties)
        const FLASH_CFG_SECTOR: usize = 6;
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

    #[shared]
    struct Shared {
        user_cfg: UserCfg,
        state_volatile: StateVolatile,
        system_status: SystemStatus,
        autopilot_status: AutopilotStatus,
        current_params: Params,
        // None if the data is stale. eg lost link, no link established.
        control_channel_data: Option<ChannelData>,
        /// Link statistics, including Received Signal Strength Indicator (RSSI) from the controller's radio.
        link_stats: LinkStats,
        spi1: Spi<SPI1>,
        i2c1: I2c<I2C1>,
        i2c2: I2c<I2C2>,
        altimeter: baro::Altimeter,
        flash_onboard: Flash,
        lost_link_timer: Timer<TIM17>,
        motor_timer: setup::MotorTimer,
        servo_timer: setup::ServoTimer,
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: SerialPort<'static, UsbBusType>,
        /// `power_used` is in rotor power (0. to 1. scale), summed for each rotor x milliseconds.
        power_used: f32,
        imu_filters: ImuFilters,
        flight_ctrl_filters: FlightCtrlFilters,
        // Note: We don't currently haveh PID filters, since we're not using a D term for the
        // RPM PID.
        ext_sensor_active: ExtSensor,
        pwr_maps: AccelMaps,
        // /// Store rotor RPM: (M1, M2, M3, M4). Quad only, but we can't feature gate
        // /// shared fields.
        // rpm_readings: RpmReadings,
        // rpms_commanded: MotorRpm,
        motor_pid_state: MotorPidGroup,
        /// PID motor coefficients
        motor_pid_coeffs: MotorCoeffs,
        tick_timer: Timer<TIM5>,
        can: setup::Can_,
    }

    #[local]
    struct Local {
        // update_timer: Timer<TIM15>,
        uart_crsf: setup::UartCrsf, // for ELRS over CRSF.
        // spi_flash: SpiFlash,  // todo: Fix flash in HAL, then do this.
        arm_signals_received: u8, // todo: Put sharedin state volatile.
        disarm_signals_received: u8,
        /// We use this counter to subdivide the main loop into longer intervals,
        /// for various tasks like logging, and outer loops.
        // update_isr_loop_i: usize,
        imu_isr_loop_i: u32,
        // aux_loop_i: usize, // todo temp
        ctrl_coeff_adj_timer: Timer<TIM1>,
        uart_osd: setup::UartOsd, // for our DJI OSD, via MSP protocol
        time_with_high_throttle: f32,
        ahrs: Ahrs,
        dshot_read_timer: Timer<TIM2>,
        cs_imu: Pin,
        // todo: `params_prev` is an experimental var used in our alternative/experimental
        // todo flight controls code as a derivative.
        params_prev: Params,
        batt_curr_adc: Adc<ADC>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        init::run(cx)
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
    shared = [spi1, i2c1, i2c2, current_params, control_channel_data, link_stats, lost_link_timer,
    autopilot_status, imu_filters, flight_ctrl_filters, user_cfg, motor_pid_state, motor_pid_coeffs,
    motor_timer, servo_timer, state_volatile, system_status, tick_timer],
    local = [ahrs, imu_isr_loop_i, cs_imu, params_prev, time_with_high_throttle,
    arm_signals_received, disarm_signals_received, batt_curr_adc], priority = 4)]
    fn imu_tc_isr(mut cx: imu_tc_isr::Context) {
        main_loop::run(cx);
    }

    // todo H735 issue on GH: https://github.com/stm32-rs/stm32-rs/issues/743 (works on H743)
    // todo: NVIC interrupts missing here for H723 etc!
    // #[task(binds = OTG_HS,
    // #[task(binds = OTG_FS,
    #[task(binds = USB_LP,
    shared = [usb_dev, usb_serial, current_params, control_channel_data,
    link_stats, user_cfg, state_volatile, system_status, motor_timer, servo_timer],
    local = [], priority = 6)]
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
            cx.shared.motor_timer,
            cx.shared.servo_timer,
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
                 motor_timer,
                 servo_timer,
                 // rpm_readings
                | {
                    if !usb_dev.poll(&mut [usb_serial]) {
                        return;
                    }

                    let mut buf = [0u8; 8];
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
                                &user_cfg.waypoints,
                                system_status,
                                &mut state_volatile.arm_status,
                                // &mut user_cfg.control_mapping,
                                &mut state_volatile.op_mode,
                                motor_timer,
                                servo_timer,
                                &mut state_volatile.motor_servo_state,
                                &mut state_volatile.preflight_motors_running,
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
    #[task(binds = USART3,
    // #[task(binds = USART2,
// shared = [control_channel_data, link_stats, system_status,
// lost_link_timer], local = [uart_crsf], priority = 8)]
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

        let start_of_message = unsafe { uart.regs.isr.read().cmf().bit_is_set() };

        uart.clear_interrupt(UsartInterrupt::CharDetect(None));
        uart.clear_interrupt(UsartInterrupt::Idle);

        // todo: Store link stats and control channel data in an intermediate variable.
        // todo: Don't lock it. At least, you don't want any delay when starting the read,
        // todo although a delay on finishing the read is fine.

        // Stop the DMA read, since it will likely not have filled the buffer, due
        // to the variable message sizies.
        dma::stop(setup::CRSF_DMA_PERIPH, setup::CRSF_RX_CH);

        // if crsf::TRANSFER_IN_PROG
        //     .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
        //     .is_ok()
        // {

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
            println!("Spurious IDLE on CRSF reception");
        }
    }

    /// If this triggers, it means we've received no radio control signals for a significant
    ///period of time; we treat this as a lost-link situation.
    /// (Note that this is for TIM17 on both variants)
    // #[task(binds = TIM17,
    #[task(binds = TIM1_TRG_COM,
    shared = [lost_link_timer, state_volatile, autopilot_status,
    current_params, system_status, control_channel_data], priority = 2)]
    fn lost_link_isr(mut cx: lost_link_isr::Context) {
        println!("Lost the link!");

        cx.shared.lost_link_timer.lock(|timer| {
            timer.clear_interrupt(TimerInterrupt::Update);
            timer.disable();
            timer.reset_count();
        });

        safety::LINK_LOST.store(true, Ordering::Release);

        cx.shared.control_channel_data.lock(|ch_data| {
            *ch_data = None;
        });

        (
            cx.shared.state_volatile,
            cx.shared.autopilot_status,
            cx.shared.current_params,
            cx.shared.system_status,
        )
            .lock(|state_volatile, autopilot_status, params, system_status| {
                // We run this during the main loop, but here the `entering` flag is set to true,
                // to initialize setup steps.
                safety::link_lost(
                    system_status,
                    autopilot_status,
                    params,
                    &state_volatile.base_point,
                );
            });
    }

    #[task(binds = TIM5, shared = [tick_timer], local = [], priority = 1)]
    /// Increments the tick overflow.
    fn tick_isr(mut cx: tick_isr::Context) {
        cx.shared.tick_timer.lock(|timer| {
            // todo: Do this without locking.
            timer.clear_interrupt(TimerInterrupt::Update);
        });
        // (*pac::TIM5::ptr())

        TICK_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
    }

    // #[task(binds = DMA2_STR1,
    #[task(binds = DMA2_CH1,
    shared = [i2c2], priority = 2)]
    /// Baro write complete; start baro read.
    fn baro_write_tc_isr(mut cx: baro_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::BARO_DMA_PERIPH, setup::BARO_RX_CH);

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
    shared = [altimeter, current_params, state_volatile], priority = 3)]
    /// Baro read complete; handle data, and start next write.
    fn baro_read_tc_isr(cx: baro_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::BARO_DMA_PERIPH,
            setup::BARO_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        let buf = unsafe { &sensors_shared::READ_BUF_BARO };

        (
            cx.shared.altimeter,
            cx.shared.current_params,
            cx.shared.state_volatile,
        )
            .lock(|altimeter, params, state_volatile| {
                // code shortener.

                // todo: Process your baro reading here.
                let (pressure, temp) = altimeter.pressure_temp_from_readings(buf);

                state_volatile.pressure_static = pressure;
                state_volatile.temp_baro = temp;
                params.alt_msl_baro =
                    atmos_model::estimate_altitude_msl(pressure, temp, &altimeter.ground_cal)
            });
    }

    // #[task(binds = DMA2_STR3,
    #[task(binds = DMA2_CH3,
    shared = [i2c1, ext_sensor_active], priority = 2)]
    /// External sensors write complete; start external sensors read.
    fn ext_sensors_write_tc_isr(cx: ext_sensors_write_tc_isr::Context) {
        dma::clear_interrupt(
            setup::EXT_SENSORS_DMA_PERIPH,
            setup::EXT_SENSORS_TX_CH,
            DmaInterrupt::TransferComplete,
        );

        dma::stop(setup::EXT_SENSORS_DMA_PERIPH, setup::EXT_SENSORS_RX_CH);

        println!("Ext sensors B");
        (cx.shared.i2c1, cx.shared.ext_sensor_active).lock(|i2c1, ext_sensor_active| {
            // todo: Skip sensors if marked as not connected?

            // unsafe {
            //     match ext_sensor_active {
            //         ExtSensor::Mag => {
            //             i2c1.read_dma(
            //                 mag::ADDR,
            //                 &mut sensors_shared::MAG_READINGS,
            //                 setup::EXT_SENSORS_RX_CH,
            //                 Default::default(),
            //                 setup::EXT_SENSORS_DMA_PERIPH,
            //             );
            //         }
            //         ExtSensor::Gps => {
            //             i2c1.read_dma(
            //                 gps::ADDR,
            //                 &mut sensors_shared::GPS_READINGS,
            //                 setup::EXT_SENSORS_RX_CH,
            //                 Default::default(),
            //                 setup::EXT_SENSORS_DMA_PERIPH,
            //             );
            //         }
            //         ExtSensor::Tof => {
            //             i2c1.read_dma(
            //                 tof::ADDR,
            //                 &mut sensors_shared::TOF_READINGS,
            //                 setup::EXT_SENSORS_RX_CH,
            //                 Default::default(),
            //                 setup::EXT_SENSORS_DMA_PERIPH,
            //             );
            //         }
            //     }
            // }
        });
    }

    // #[task(binds = DMA2_STR4,
    #[task(binds = DMA2_CH4,
    shared = [i2c1, ext_sensor_active], priority = 2)]
    /// Ext sensors write complete; start read of the next sensor in sequence.
    fn ext_sensors_read_tc_isr(cx: ext_sensors_read_tc_isr::Context) {
        dma::clear_interrupt(
            setup::EXT_SENSORS_DMA_PERIPH,
            setup::EXT_SENSORS_RX_CH,
            DmaInterrupt::TransferComplete,
        );

        println!("Ext sensors A");
        (cx.shared.i2c1, cx.shared.ext_sensor_active).lock(|i2c1, ext_sensor_active| {
            // todo: Skip sensors if marked as not connected?

            // todo: Interp data, and place data into its apt struct here.

            // unsafe {
            // match ext_sensor_active {
            // ExtSensor::Mag => {
            //     i2c1.write_dma(
            //         gps::ADDR,
            //         &mut sensors_shared::WRITE_BUF_GPS,
            //         false,
            //         setup::EXT_SENSORS_RX_CH,
            //         Default::default(),
            //         setup::EXT_SENSORS_DMA_PERIPH,
            //     );
            //     *ext_sensor_active = ExtSensor::Gps;
            // }
            // ExtSensor::Gps => {
            //     i2c1.write_dma(
            //         tof::ADDR,
            //         &mut sensors_shared::WRITE_BUF_TOF,
            //         false,
            //         setup::EXT_SENSORS_RX_CH,
            //         Default::default(),
            //         setup::EXT_SENSORS_DMA_PERIPH,
            //     );
            //     *ext_sensor_active = ExtSensor::Tof;
            // }
            // ExtSensor::Tof => {
            //     *ext_sensor_active = ExtSensor::Mag;
            //     // End of sequence; don't start a new transfer.
            // }
            // }
            // }
        });
    }

    // #[task(binds = FDCAN1_IT0,
    #[task(binds = FDCAN1_INTR0_IT,
    shared = [can], priority = 4)] // todo: Temp high prio
    /// Ext sensors write complete; start read of the next sensor in sequence.
    fn can_isr(mut cx: can_isr::Context) {
        // todo: Set up appropriate hardware filters, like Fix2, AHRS, IMU etc.
        // println!("\nCAN ISR");

        cx.shared.can.lock(|can| {
            can.clear_interrupt(Interrupt::RxFifo0NewMsg);

            let rx_buf = unsafe { &mut RX_BUF_CAN }; // todo

            let rx_result = can.receive0(rx_buf);

            match dronecan::get_frame_info(rx_result) {
                Ok(frame_info) => {
                    let id = match frame_info.id {
                        Id::Standard(id) => id.as_raw() as u32,
                        Id::Extended(id) => id.as_raw(),
                    };

                    let can_id = dronecan::CanId::from_value(id);
                    // println!(
                    //     "Frame info. Len: {}, ts: {}, pri: {}, type_id: {}, source id: {}",
                    //     frame_info.len,
                    //     frame_info.time_stamp,
                    //     can_id.priority.val(),
                    //     can_id.type_id,
                    //     can_id.source_node_id
                    // );

                    // let tail_byte = dronecan::get_tail_byte(&rx_buf, frame_info.len).ok();
                    let tail_byte = dronecan::get_tail_byte(rx_buf, frame_info.len).ok();

                    if let Some(tail_byte) = tail_byte {
                        //     println!(
                        //         "Start of xfer: {}, end: {}, toggle: {}, transfer_id: {}",
                        //         tail_byte.start_of_transfer,
                        //         tail_byte.end_of_transfer,
                        //         tail_byte.toggle,
                        //         tail_byte.transfer_id
                        //     );

                        if can_id.type_id != 20007 {
                            println!("Id: {}", can_id.type_id);
                        }

                        // todo: See notes on GNSS CAN firmware about why we hard-code these
                        // todo match arm vals.

                        match can_id.type_id {
                            1_063 => {
                                let fix = dronecan::gnss::FixDronecan::unpack(
                                    rx_buf[0..dronecan::MsgType::Fix2.payload_size() as usize]
                                        .try_into()
                                        .unwrap(),
                                );
                                match fix {
                                    Ok(f) => {
                                        println!(
                                            "Fix. Time: {}. Lat: {}. Lon: {}. Msl: {}",
                                            f.gnss_timestamp,
                                            f.latitude_deg_1e8 as f32 / 10_000_000.,
                                            f.longitude_deg_1e8 as f32 / 10_000_000.,
                                            f.height_msl_mm as f32 / 1_000.,
                                        );
                                    }
                                    Err(_) => {
                                        println!("Error unpacking fix");
                                    }
                                }
                                // println!("Test broadcast");
                                // // todo temp: Testing config get/set.
                                // dronecan::broadcast(
                                //     can,
                                //     dronecan::MsgPriority::Slow,
                                //     2_110,
                                //     1,
                                //     0,
                                //     &mut [5, 6, 7, 8, 9, 10, 11, 12],
                                //     8, //
                                //     true,
                                // ).ok();

                                // let mut cfg_to_set = &[0; 8]
                                // dronecan::broadcast(
                                //     can,
                                //     dronecan::MsgPriority::Slow,
                                //     2_111,
                                //     0,
                                //     0,
                                //     &mut cfg_to_set,
                                //     0,
                                //     true,
                                // ).ok();
                            }
                            1_028 => {
                                let pressure = f32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
                                println!("Pressure: {} kPa", pressure / 1_000.);
                            }
                            1_029 => {
                                // let temp =
                                //     f32::from(f16::from_le_bytes(rx_buf[0..2].try_into().unwrap()));
                                // println!("Temp: {} K", temp);
                            }
                            1_002 => {
                                // todo
                                // let x =
                                //     f32::from(f16::from_le_bytes(rx_buf[1..3].try_into().unwrap()));
                                // let y =
                                //     f32::from(f16::from_le_bytes(rx_buf[3..5].try_into().unwrap()));
                                // let z =
                                //     f32::from(f16::from_le_bytes(rx_buf[5..7].try_into().unwrap()));
                                // println!("Mag. x: {}, y: {}, z: {}", x, y, z);
                            }
                            341 => {
                                let uptime = u32::from_le_bytes(rx_buf[0..4].try_into().unwrap());
                                println!(
                                    "Node status. Uptime sec: {}, health: {}, mode; {}",
                                    uptime, rx_buf[4], rx_buf[5]
                                );
                            }
                            3_115 => {
                                println!("Position fused");
                            }
                            _ => {
                                println!("Unknown message type received: {}", can_id.type_id);
                                println!("Rx buf: {:?}", rx_buf);
                            }
                        }
                    }
                }
                Err(_) => {
                    println!("Error getting frame info")
                }
            }
        });
    }
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}
