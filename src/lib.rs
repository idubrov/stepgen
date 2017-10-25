#![no_std]
#![feature(const_fn)]
#![warn(missing_docs)]
#![deny(warnings)]

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
//! stepper.set_acceleration(1000 << 8).unwrap(); // 1000 steps per second per second
//! stepper.set_target_speed(800 << 8).unwrap(); // 800 steps per second (4 turns per second)
//! stepper.set_target_step(1000).unwrap(); // stop at step 1000
//!
//! // Take 99 steps
//! for _ in 0..99 {
//!     println!("{}", stepper.next().unwrap());
//! }
//!
//! assert_eq!(99, stepper.current_step());
//! assert_eq!(113621, stepper.current_speed());
//! assert_eq!(2242, (stepper.next().unwrap() + 128) >> 8); // delay for 100th step, rounded to the nearest integer
//! ```
//! ## Note on numbers
//!
//! In few APIs, stepgen keeps numbers as fixed-point numbers, using least significant 8 bits
//! for the fractional part and the remaining bits for the integral part.
//!
//! [1]: http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time


/// Error code for some of the stepgen operations.
#[derive(Debug, PartialEq)]
pub enum Error {
    /// Requested parameter (acceleration or speed) is too slow -- delay is too long and does not
    /// fit in 16.8 format.
    TooSlow,

    /// Requested speed is too fast -- delay is to short for the MCU to process it timely.
    TooFast,

    /// Speed or acceleration was not configured when step is set.
    SpeedAccelerationNotSet
}

/// Result type for some of the stepgen operations.
pub type Result = core::result::Result<(), Error>;

// Smallest delay we can handle without significant rounding errors
const FASTEST_DELAY: u32 = 30;

/// State of the stepgen.
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
    /// Create new copy of stepgen. `ticks_per_second` defines size of each tick stepgen operates.
    /// All settings (acceleration, speed) and current parameters (speed) are defined in terms of
    /// these ticks.
    pub const fn new(ticks_per_second: u32) -> Stepgen {
        Stepgen {
            current_step: 0,
            speed: 0,
            delay: 0,
            slewing_delay: 0,
            ticks_per_second,
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
        let c0long: u64 = ((2u64 * 676 * 676) << 40) / u64::from(acceleration);
        let c0: u64 = (u64::from(self.ticks_per_second) * u64sqrt(c0long) / 1000) >> 8;
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
    /// # Errors
    /// If speed or acceleration are not set, returns an error `Error::SpeedAccelerationNotSet`.
    ///
    /// # Notes
    /// 1. Steps could only go in positive direction. Therefore, setting target step to 0 wil
    /// always force step generation to decelerate and stop.
    pub fn set_target_step(&mut self, target_step: u32) -> Result {
        if self.target_delay == 0 || self.first_delay == 0 {
            return Err(Error::SpeedAccelerationNotSet);
        }
        self.target_step = target_step;
        Ok(())
    }

    /// Set slew speed (maximum speed stepper motor would run), in steps per second. Note that
    /// stepper motor would only reach this speed if target step is far enough, so there is
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
    /// // 1 step per second -- too slow!
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
    /// // 1_000_000 step per second per second -- too fast!
    /// assert_eq!(Error::TooFast, stepper.set_target_speed(1_000_000 << 8).unwrap_err());
    /// ```
    pub fn set_target_speed(&mut self, target_speed: u32) -> Result {
        if target_speed == 0 {
            // Too slow, speed is zero
            return Err(Error::TooSlow);
        }
        let delay = (u64::from(self.ticks_per_second) << 16) / u64::from(target_speed);
        if (delay >> 24) != 0 {
            // Too slow, doesn't fit in in 16.8 format, our timer is only 16 bit.
            return Err(Error::TooSlow);
        }
        if delay <= u64::from(FASTEST_DELAY) * (1 << 8) {
            // Too fast, less than 10 ticks of a timer. 10 is an arbitrary number,
            // just to make sure we have enough time to calculate next delay.
            return Err(Error::TooFast);
        }
        // Convert to 16.16 format. We only need this precision during intermediate calculations.
        self.target_delay = (delay as u32) << 8;
        Ok(())
    }

    /// Current step stepgen is at.
    pub fn current_step(&self) -> u32 {
        self.current_step
    }

    /// Target step stepgen should stop at. Note that if stepper is running too fast, it might not
    /// be able to stop exactly at this step. This could happen when target step is updated after
    /// stepper motor accelerated to certain speed.
    pub fn target_step(&self) -> u32 {
        self.target_step
    }

    /// Get estimated current speed, in 24.8 format
    pub fn current_speed(&self) -> u32 {
        let delay = if self.slewing_delay != 0 { self.slewing_delay } else { self.delay };
        let delay = delay >> 8; // Convert to 16.8 format
        if delay != 0 {
            let speed = (u64::from(self.ticks_per_second) << 16) / u64::from(delay);
            speed as u32
        } else {
            0
        }
    }

    /// Returns '0' if should stop. Otherwise, returns timer delay in 24.8 format
    fn next_delay(&mut self) -> u32 {
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
            let d = if target_delay > self.first_delay {
                // No acceleration is necessary -- just return the target delay
                target_delay
            } else {
                // First step: load first delay, count as one acceleration step
                self.delay = self.first_delay;
                self.speed = 1;
                self.delay
            };
            return d >> 8; // Convert to 16.8 format
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
        d >> 8 // Convert to 16.8 format
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

impl Iterator for Stepgen {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match Stepgen::next_delay(self) {
            0 => None,
            v => Some(v)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FREQUENCY: u32 = 1_000_000; // Tests assume timer ticking at 1us (1Mhz)

    fn round(delay: u32) -> u32 {
        (delay + 128) >> 8
    }

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

    #[test]
    fn acceleration_too_slow() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        assert_eq!(Err(Error::TooSlow), stepgen.set_acceleration(1 << 8));
    }

    #[test]
    fn too_slow() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        assert_eq!(Err(Error::TooSlow), stepgen.set_target_speed(1 << 8));
    }

    #[test]
    fn too_slow_2() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_target_speed(3907).unwrap();
        assert_eq!(Err(Error::TooSlow), stepgen.set_target_speed(3906));
    }

    #[test]
    fn too_slow_zero() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        assert_eq!(Err(Error::TooSlow), stepgen.set_target_speed(0));
    }

    #[test]
    fn slower_than_first_step_after_accel() {
        // Setting very slow speed after acceleration is OK
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_acceleration(1000 << 8).unwrap();
        stepgen.set_target_speed(5120).unwrap(); // 20 pulses per second = 50_000 delay
        stepgen.set_target_step(3).unwrap();
        assert!(stepgen.first_delay < stepgen.target_delay);

        // Walk three steps
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert!(stepgen.next().is_none());
    }

    #[test]
    fn slower_than_first_step_before_accel() {
        // Setting acceleration after setting slow speed is also OK
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_target_speed(5120).unwrap();
        stepgen.set_acceleration(1000 << 8).unwrap();
        stepgen.set_target_step(3).unwrap();
        assert!(stepgen.first_delay < stepgen.target_delay);

        // Walk three steps
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert_eq!(50_000 << 8, stepgen.next().unwrap());
        assert_eq!(0, stepgen.speed);
        assert!(stepgen.next().is_none());
    }

    #[test]
    fn too_fast() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        assert_eq!(Err(Error::TooFast), stepgen.set_target_speed(1_000_000 << 8));
    }

    #[test]
    fn slow_during_acceleration() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_target_speed(800 << 8).unwrap();
        stepgen.set_acceleration(1000 << 8).unwrap();
        stepgen.set_target_step(core::u32::MAX).unwrap();

        assert_eq!(0, stepgen.current_speed());

        assert_eq!(30232, round(stepgen.next().unwrap()));
        assert_eq!(18139, round(stepgen.next().unwrap()));
        assert_eq!(14108, round(stepgen.next().unwrap()));
        assert_eq!(11938, round(stepgen.next().unwrap()));

        // Update target speed, want to run slower
        stepgen.set_target_speed(50 << 8).unwrap();
        assert_eq!(14108, round(stepgen.next().unwrap()));
        assert_eq!(18139, round(stepgen.next().unwrap()));
        assert_eq!(20000, round(stepgen.next().unwrap())); // 20000 = 1_000_000 / 50
        assert_eq!(50 << 8, stepgen.current_speed());

        // Slow a little bit more
        stepgen.set_target_speed(40 << 8).unwrap();
        assert_eq!(25000, round(stepgen.next().unwrap())); // 25000 = 1_000_000 / 40
        assert_eq!(40 << 8, stepgen.current_speed());
    }

    #[test]
    fn no_speed_set() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_acceleration(1000 << 8).unwrap();
        assert_eq!(Err(Error::SpeedAccelerationNotSet), stepgen.set_target_step(1000_000_000));
    }

    #[test]
    fn no_acceleration_set() {
        let mut stepgen = Stepgen::new(FREQUENCY);
        stepgen.set_target_speed(800 << 8).unwrap();
        assert_eq!(Err(Error::SpeedAccelerationNotSet), stepgen.set_target_step(1000_000_000));
    }
}
