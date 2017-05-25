# stepgen

Stepper motor speed ramp generator. Given acceleration, target speed and target step to stop
at, generates acceleration or deceleration profile for the stepper motor, in the form of delays
between steps.

## Examples
```rust
use stepgen::{new};
let mut stepper = new(1_000_000);

stepper.set_acceleration(1000 << 8); // 1200 steps per second per second
stepper.set_target_step(1000); // stop at step 1000
stepper.set_target_speed(800 << 8); // 240RPM (4 turns per second)
println!("First delay {}", stepper.first_delay);

// Take 99 steps
for _ in 0..99 {
    stepper.next();
}

assert_eq!(99, stepper.current_step());
assert_eq!(113621, stepper.current_speed());
assert_eq!(2242, (stepper.next() + 128) >> 8); // delay for 100th step, rounded to nearest integer
```
### Note on numbers

In few APIs, stepgen keeps numbers as fixed-point numbers, using least significant 8 bits
for the fractional part and the remaining bits for the integral part.


### Links
[1] [Generate stepper-motor speed profiles in real time](http://www.embedded.com/design/mcus-processors-and-socs/4006438/Generate-stepper-motor-speed-profiles-in-real-time)
