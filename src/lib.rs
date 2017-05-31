#![no_std]
#![feature(const_fn)]

//! Stepper motor speed ramp generator.
//!
//! Given acceleration, target speed and target step to stop
//! at, generates acceleration or deceleration profile for the stepper motor, in the form of delays
//! between steps.
//!
//! Uses algorithm from "[Generate stepper-motor speed pro les in real time][1]" paper by David Austin.
//!
//! # Examples
//! ```
//! use stepgen::Stepgen;
//!
//! let mut stepper = Stepgen::new(1_000_000);
//!
//! stepper.set_acceleration(1000 << 8).unwrap(); // 1200 steps per second per second
//! stepper.set_target_step(1000); // stop at step 1000
//! stepper.set_target_speed(800 << 8).unwrap(); // 240RPM (4 turns per second)
//!
//! // Take 99 steps
//! for _ in 0..99 {
//!     stepper.next();
//! }
//!
//! assert_eq!(99, stepper.current_step());
//! assert_eq!(113621, stepper.current_speed());
//! assert_eq!(2242, (stepper.next() + 128) >> 8); // delay for 100th step, rounded to nearest integer
//! ```
//! ## Note on numbers
//!
//! In few APIs, stepgen keeps numbers as fixed-point numbers, using least significant 8 bits
//! for the fractional part and the remaining bits for the integral part.
//!
//!
//! ## Links
//! [1]: http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time


#[derive(Debug, PartialEq)]
pub enum Error {
    TooSlow,
    TooFast,
}

pub type Result = core::result::Result<(), Error>;

// How many timer ticks it would take for one update (rough estimate), to make sure we are not
// running too fast so we cannot update the ticker
const TICKS_PER_UPDATE: u32 = 10;

#[derive(Debug)]
pub struct Stepgen {
    // Current step
    current_step: u32,

    // Amount of acceleration steps we've taken so far
    speed: u32,
    // Previously calculated delay, in 16.16 format
    delay: u32,

    // If slewing, this will be the slewing delay. Switched to this mode once
    // we overshoot target speed. 16.16 format.
    slewing_delay: u32,

    // Timer frequency
    ticks_per_second: u32,
    // First step delay, in 16.16 format
    first_delay: u32,
    // Target step
    target_step: u32,
    // Target speed delay, in 16.16 format
    target_delay: u32,
}

/// This function computes square root of an `u64` number.
fn u64sqrt(x0: u64) -> u64 {
    let mut x = x0;
    let mut xr = 0; // result register
    let mut q2 = 0x4000_0000_0000_0000u64; // scan-bit register, set to highest possible result bit
    while q2 != 0 {
        if (xr + q2) <= x {
            x -= xr + q2;
            xr >>= 1;
            xr += q2; // test flag
        } else {
            xr >>= 1;
        }
        q2 >>= 2; // shift twice
    }

    // add for rounding, if necessary
    if xr < x { xr + 1 } else { xr }
}

impl Stepgen {
    pub const fn new(ticks_per_second: u32) -> Stepgen {
        Stepgen {
            current_step: 0,
            speed: 0,
            delay: 0,
            slewing_delay: 0,
            ticks_per_second: ticks_per_second,
            first_delay: 0,
            target_step: 0,
            target_delay: 0,
        }
    }


    // Configuration methods. Should not be called while motor is running.

    /// Set stepper motor acceleration, in steps per second per second (in 16.8 format).
    /// Note that this method is computation intensive, so it's best to set acceleration
    /// once and never change it.
    ///
    /// # Examples
    ///
    /// ```
    /// use stepgen::Stepgen;
    /// let mut stepgen = Stepgen::new(1_000_000);
    ///
    /// stepgen.set_acceleration(1200 << 8).unwrap();
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if acceleration is too slow (first delay does not fit into 16.8).
    ///
    /// Too slow:
    ///
    /// ```
    /// use stepgen::{Stepgen, Error};
    ///
    /// let mut stepper = Stepgen::new(1_000_000);
    ///
    /// // 1 step per second per second -- too slow!
    /// assert_eq!(Error::TooSlow, stepper.set_acceleration(1 << 8).unwrap_err());
    /// ```
    pub fn set_acceleration(&mut self, acceleration: u32) -> Result {
        // c0 = F*sqrt(2/a)*.676 = F*sqrt(2/a)*676/1000 =
        //      F*sqrt(2*676*676/a)/1000 =
        //      F*sqrt(2*676*676*1^16)/(1000*1^8)
        // We bring as much as we can under square root, to increase accuracy of division
        // sqrt(1 << 16) is (1 << 8), which is to convert to 24.8
        // We shift 24 bits to the left to adjust for acceleration in 24.8 format plus to convert
        // result into 24.8 format, so the resulting shift is 40 bits.
        // 676 is used to correct for the first step (see the linked paper)
        let c0long: u64 = ((2u64 * 676 * 676) << 40) / (acceleration as u64);
        let c0: u64 = ((self.ticks_per_second as u64) * u64sqrt(c0long) / 1000) >> 8;
        if (c0 >> 24) != 0 {
            // Doesn't fit in 16.8 format, our timer is only 16 bit.
            return Err(Error::TooSlow);
        }
        // Convert to 16.16 format. We only need this precision during intermediate calculations.
        self.first_delay = (c0 as u32) << 8;
        Ok(())
    }

    /// Set destination step for the stepper motor pulse generator. This is one of the two methods
    /// to control the step generation (target step and target speed). If current step > target
    /// step, stepper motor would slow down until stop if running or stay stopped if not running.
    ///
    /// # Notes
    /// 1. Steps could only go in positive direction. Therefore, setting target step to 0 wil
    /// always force step generation to decelerate and stop.
    pub fn set_target_step(&mut self, target_step: u32) {
        self.target_step = target_step;
    }

    /// Set slew speed (maximum speed stepper motor would run). Note that stepper
    /// motor would only reach this speed if target step is far enough, so there is
    /// enough space for acceleration/deceleration.
    ///
    /// # Errors
    ///
    /// Returns an error if target speed is either too slow (first delay does not fit into 16.8) or
    /// too fast (first delay is shorter than `TICKS_PER_UPDATE`).
    ///
    /// Too slow:
    ///
    /// ```
    /// use stepgen::{Stepgen, Error};
    ///
    /// let mut stepper = Stepgen::new(1_000_000);
    ///
    /// // 1 step per second per second -- too slow!
    /// assert_eq!(Error::TooSlow, stepper.set_target_speed(1 << 8).unwrap_err());
    /// ```
    ///
    /// Too fast:
    ///
    /// ```
    /// use stepgen::{Stepgen, Error};
    ///
    /// let mut stepper = Stepgen::new(1_000_000);
    ///
    /// // 1_000_000 step per second per second -- too slow!
    /// assert_eq!(Error::TooFast, stepper.set_target_speed(1_000_000 << 8).unwrap_err());
    /// ```
    pub fn set_target_speed(&mut self, target_speed: u32) -> Result {
        let delay = ((self.ticks_per_second as u64) << 16) / (target_speed as u64);
        if (delay >> 24) != 0 {
            // Too slow, doesn't fit in in 16.8 format, our timer is only 16 bit.
            return Err(Error::TooSlow);
        }
        if delay <= (TICKS_PER_UPDATE as u64) * (1 << 8) {
            // Too fast, less than 10 ticks of a timer. 10 is an arbitrary number,
            // just to make sure we have enough time to calculate next delay.
            return Err(Error::TooFast);
        }
        // Convert to 16.16 format. We only need this precision during intermediate calculations.
        self.target_delay = (delay as u32) << 8;
        Ok(())
    }

    pub fn current_step(&self) -> u32 {
        self.current_step
    }

    pub fn target_step(&self) -> u32 {
        self.target_step
    }

    /// Get estimated current speed, in 24.8 format
    pub fn current_speed(&self) -> u32 {
        let delay = if self.slewing_delay != 0 { self.slewing_delay } else { self.delay };
        let delay = delay >> 8; // Convert to 16.8 format
        if delay != 0 {
            let speed = ((self.ticks_per_second as u64) << 16) / (delay as u64);
            return speed as u32;
        } else {
            0
        }
    }

    /// Returns '0' if should stop. Otherwise, returns timer delay in 24.8 format
    pub fn next(&mut self) -> u32 {
        let target_step = self.target_step;
        let target_delay = self.target_delay;
        let st = self.current_step;

        // We are at the stop point and speed is zero -- return "stopped" (delay of 0)
        if st >= target_step && self.speed <= 1 {
            self.speed = 0;
            return 0;
        }

        // Stop slewing if target delay was changed
        if self.slewing_delay != 0 && self.slewing_delay != target_delay {
            self.slewing_delay = 0;
        }

        // Steps made so far
        self.current_step += 1;

        if self.speed == 0 {
            // First step: load first delay, take the slower one
            self.delay = if self.first_delay < target_delay { target_delay } else { self.first_delay };
            self.speed = 1;
            return self.delay >> 8; // Convert to 16.8 format
        }

        // Calculate the projected step we would stop at if we start decelerating right now
        let est_stop = st + self.speed;
        if est_stop == target_step {
            // We would stop one step earlier than we want, so let's just
            // return the same delay as the current one and start deceleration
            // on the next step.
        } else if est_stop > target_step {
            // We need to stop at target step, slow down
            self.slowdown();

            // We are not slewing even though we could have slowed down below the slewing speed
            self.slewing_delay = 0;
        } else if self.slewing_delay == 0 && self.delay < target_delay {
            // Not slewing and running too fast, slow down
            self.slowdown();

            // Switch to slewing if we slowed down enough
            if self.delay >= target_delay {
                self.slewing_delay = target_delay;
            }
        } else if self.slewing_delay == 0 && self.delay > target_delay {
            // Not slewing and running too slow, speed up
            self.speedup();

            // Switch to slewing if we have accelerated enough
            if self.delay <= target_delay {
                self.slewing_delay = target_delay;
            }
        }

        // If slewing, return slew delay. delay should be close enough, but could
        // be different due to the accumulated rounding errors
        let d = if self.slewing_delay != 0 { self.slewing_delay } else { self.delay };
        return d >> 8; // Convert to 16.8 format
    }


    fn speedup(&mut self) {
        let denom = 4 * self.speed + 1;
        self.delay -= (2 * self.delay + denom / 2) / denom;
        self.speed += 1;
    }

    fn slowdown(&mut self) {
        self.speed -= 1;
        let denom = 4 * self.speed - 1;
        self.delay += (2 * self.delay + denom / 2) / denom;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sqrt_works() {
        assert_eq!(0, u64sqrt(0));
        assert_eq!(1, u64sqrt(1));
        assert_eq!(2, u64sqrt(4));
        assert_eq!(3, u64sqrt(10));
        assert_eq!(4, u64sqrt(15));
        assert_eq!(0x80_00_00_00u64, u64sqrt(0x4000_0000_0000_0000u64));
        assert_eq!(0x1_00_00_00_00u64, u64sqrt(0xffff_ffff_ffff_ffffu64));
    }
}
