#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]


//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/HWTIMER/STM32_hwTimer.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/HWTIMER/STM32_hwTimer.cpp

// todo: Implement! And review all code that calls the timer, and make sure it's accurate!

const TimerIntervalUSDefault: u32 = 20_000;

use stm32_hal2::{timer::{Timer, TimerInterrupt}, pac::TIM4};

pub struct hwTimer {
    pub MyTim: Timer<TIM4>,
    pub HWtimerInterval: u32,
    pub  isTick: bool,
    pub PhaseShift: i32,
    pub FreqOffset: i32,
    pub PauseDuration: u32,
    pub running: bool,
    pub alreadyInit: bool,
}

impl hwTimer {
    pub fn new(timer: Timer<TIM4>) {
        Self {
            MyTim: timer,
            HWtimerInterval: TimerIntervalUSDefault,
            isTick: false,
            PhaseShift: 0,
            FreqOffset: 0,
            PauseDuration: 0,
            running: false,
            alreadyInit: false,
        }
    }

pub fn init(&mut self) {
    if !self.alreadyInit    {
        self.MyTim.enable_interrupt(TimerInterrupt::Update);
        // self.MyTim.setMode(1, TIMER_OUTPUT_COMPARE); // todo?
        // self.MyTim.set_output_compare(TimeChannel1, ); // todo?
        // MyTim->setOverflow(hwTimer::HWtimerInterval >> 1, MICROSEC_FORMAT); // 22(50Hz) to 3(500Hz) scaler
        self.MyTime.set_period(sdf);
        // MyTim->setPreloadEnable(false); Handled in `main.rs`
        self.alreadyInit = true;
    }
}

pub fn stop(&mut self) {
    self.running = false;
    self.MyTim.disable();
    self.MyTim.reset_counter();
}

 /// Schedule a pause of the specified duration, in us -- TX Only
 /// Will pause until the TICK interrupt, then the next timer will
 /// fire Duration - interval/2 after that
 /// 65535us max!
pub fn pause(&self, duration: u32) {}

pub fn resume(&mut self,) {
    self.isTick = false;
    self.running = true;
    // self.MyTim->setOverflow((hwTimer::HWtimerInterval >> 1), MICROSEC_FORMAT);
    // todo:  Hardcode ARR and PSC?
    self.MyTim.set_period(sdf);
    self.MyTim.reset_counter();
    self.MyTim.enable();
    self.MyTim.reinitialize(); // will trigger the interrupt immediately, but will update the prescaler shadow reg
}

pub fn updateInterval(newTimerInterval: u32) {
    // timer should not be running when updateInterval() is called
    // todo?
    hwTimer::HWtimerInterval = newTimerInterval;
}

pub fn resetFreqOffset(&mut self) {
    self.FreqOffset = 0;
}

pub fn incFreqOffset(&mut self) {
    self.FreqOffset += 1;
}

pub fn decFreqOffset(&mut self) {
    self.FreqOffset -= 1;
}

pub fn phaseShift(&mut self, newPhaseShift: i32) {
    let minVal: i32 = -(hwTimer::HWtimerInterval >> 2);
    let maxVal: i32 = hwTimer::HWtimerInterval >> 2;
    self.PhaseShift = constrain(newPhaseShift, minVal, maxVal);
}

pub fn callback(&mut self) {
    if self.isTick {

        self.MyTim->setOverflow((hwTimer::HWtimerInterval >> 1), MICROSEC_FORMAT);
        let adjustedInterval: u32 = self.MyTimgetOverflow(TICK_FORMAT) + FreqOffset;
        self.MyTim->setOverflow(adjustedInterval, TICK_FORMAT);
        self.callbackTick();
    }
    else
    {

        self.MyTim->setOverflow((hwTimer::HWtimerInterval >> 1) + hwTimer::PhaseShift, MICROSEC_FORMAT);
        uint32_t adjustedInterval = self.MyTim->getOverflow(TICK_FORMAT) + FreqOffset;
        self.MyTim->setOverflow(adjustedInterval, TICK_FORMAT);
        self.PhaseShift = 0;
        self.callbackTock();
    }
    self.isTick = !hwTimer::isTick;
}

}