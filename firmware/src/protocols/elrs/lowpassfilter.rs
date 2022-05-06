#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! From https://raw.githubusercontent.com/ExpressLRS/ExpressLRS/master/src/include/LowPassFilter.h
//!
//! todo: Should you use CMSIS-DSP instead? Probably. This is fine to start.

/////////// Super Simple Fixed Point Lowpass ////////////////

pub struct LPF {
    pub SmoothDataINT: i32,
    pub SmoothDataFP: i32,
    pub Beta: i32,       // Length = 16
    pub FP_Shift: i32,   //Number of fractional bits
    pub NeedReset: bool, // wait for the first data to upcoming.
}

impl LPF {
    pub fn new_a(Beta: i32, FP_Shift: i32) -> Self {
        Self {
            SmoothDataINT: 0, // ??
            SmoothDataFP: 0,  // ??
            Beta,
            FP_Shift,
            NeedReset: true,
        }
    }

    pub fn new_b(Beta: i32) -> Self {
        Self {
            SmoothDataINT: 0, // ??
            SmoothDataFP: 0,  // ??
            Beta,
            FP_Shift: 5, // default to 5
            NeedReset: true,
        }
    }

    pub fn new_c() -> Self {
        Self {
            SmoothDataINT: 0, // ??
            SmoothDataFP: 0,  // ??
            Beta: 3,
            FP_Shift: 5,
            NeedReset: true,
        }
    }

    pub fn update(&mut self, Indata: i32) -> i32 {
        if self.NeedReset {
            self.init(Indata);
            self.SmoothDataINT
        }

        let mut RawData: i32 = 0;
        RawData = Indata;
        RawData <<= self.FP_Shift; // Shift to fixed point
        self.SmoothDataFP = (self.SmoothDataFP << self.Beta) - self.SmoothDataFP;
        self.SmoothDataFP += RawData;
        self.SmoothDataFP >>= self.Beta;
        // Don't do the following shift if you want to do further
        // calculations in fixed-point using SmoothData
        self.SmoothDataINT = self.SmoothDataFP >> self.FP_Shift;
        self.SmoothDataINT
    }

    pub fn reset(&mut self) {
        self.NeedReset = true;
    }

    pub fn init(&mut self, Indata: i32) {
        self.NeedReset = false;

        self.SmoothDataINT = Indata;
        self.SmoothDataFP = self.SmoothDataINT << self.FP_Shift;
    }

    pub fn value(&self) -> i32 {
        self.SmoothDataINT
    }
}
