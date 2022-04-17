#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Link-quality calculation. From https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/LQCALC/LQCALC.h

// From RxMain; CPP template.
const N: u8 = 100;

#[derive(Default)]
pub struct LQCALC {
    pub LQ: u8,
    pub index: usize, // current position in LQArray
    pub count: u8,
    pub LQmask: u32,
    pub LQArray: [u32; (N as usize + 31) / 32],
}

impl LQCALC {
    pub fn new() -> Self {
        let mut result = Result::default();

        result.reset();
        // count is reset here only once on construction to start LQ counting
        // at 100% on first connect, but 0 out of N after a failsafe
        result.count = 1;

        result
    }

    /* Set the bit for the current period to true and update the running LQ */
    pub fn add(&mut self) {
        if currentIsSet() {
            return;
        }
        self.LQArray[&self.index] |= self.LQmask;
        self.LQ += 1;
    }

    /* Start a new period */
    pub fn inc(&mut self) {
        // Increment the counter by shifting one bit higher
        // If we've shifted out all the bits, move to next idx
        LQmask = LQmask << 1;
        if LQmask == 0 {
            LQmask = (1 << 0);
            index += 1;
        }

        // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
        if (index == (N / 32)) && (LQmask & (1 << (N % 32))) {
            self.index = 0;
            self.LQmask = (1 << 0);
        }

        if LQArray[index] & LQmask != 0 {
            self.LQArray[index] &= !self.LQmask;
            self.LQ -= 1;
        }

        if count < N {
            count += 1;
        }
    }

    /* Return the current running total of bits set, in percent */
    pub fn getLQ(&self) -> u8 {
        self.LQ as u32 * 100 / count
    }

    /* Return the current running total of bits set, up to N */
    pub fn getLQRaw(&self) -> u8 {
        self.LQ
    }

    /* Return the number of periods recorded so far, up to N */
    pub fn getCount(&self) -> u8 {
        self.count
    }

    /* Return N, the size of the LQ history */
    pub fn getSize(&self) -> u8 {
        self.N
    }

    /* Initialize and zero the history */
    pub fn reset(&mut self) {
        // count is intentonally not zeroed here to start LQ counting up from 0
        // after a failsafe, instead of down from 100
        self.LQ = 0;
        self.index = 0;
        self.LQmask = (1 << 0);

        for i in 0..sizeof(LQArray) / sizeof(LQArray[0]) {
            self.LQArray[i] = 0;
        }
    }

    /*  Return true if the current period was add()ed */
    pub fn currentIsSet(&self) -> bool {
        self.LQArray[index] & self.LQmask
    }
}
