#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/PFD/PFD.h

#[derive(Default)]
pub struct PFD {
    pub intEventTime: u32,
    pub extEventTime: u32,
    pub result: i32,
    pub gotExtEvent: bool,
    pub gotIntEvent: bool,
}

impl PFD {
    #[inline(always)]
    /// reference (external osc)
    pub fn extEvent(&mut self, time: u32) {
        self.extEventTime = time;
        self.gotExtEvent = true;
    }

    #[inline(always)]
    /// internal osc event
    pub fn intEvent(&mut self, time: u32) {
        self.intEventTime = time;
        self.gotIntEvent = true;
    }

    #[inline(always)]
    pub fn reset(&mut self) {
        self.gotExtEvent = false;
        self.gotIntEvent = false;
    }

    #[inline(always)]
    pub fn calcResult(&mut self) {
        self.result = if gotExtEvent && gotIntEvent {
            (extEventTime - intEventTime) as i32
        } else {
            0
        };
    }
}
