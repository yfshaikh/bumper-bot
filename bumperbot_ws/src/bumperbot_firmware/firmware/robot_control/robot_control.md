# `robot_control.ino` explained

This firmware controls two DC motors (right and left) using:
- L298N H-bridge outputs (direction + PWM)
- Quadrature encoder feedback
- Two independent PID loops (one per wheel)

It receives wheel velocity setpoints over serial, computes measured wheel speed from encoders, runs PID, and writes PWM commands back to the motor driver.

## 1) Hardware mapping

- Motor driver PWM pins:
  - Right wheel: `L298N_enA` (pin 9)
  - Left wheel: `L298N_enB` (pin 11)
- Motor direction pins:
  - Right wheel: `L298N_in1`, `L298N_in2`
  - Left wheel: `L298N_in3`, `L298N_in4`
- Encoders:
  - Right A/B: pins `3` / `5` (`3` is interrupt pin)
  - Left A/B: pins `2` / `4` (`2` is interrupt pin)

## 2) Runtime flow

Main loop does two things:

1. **Parse serial command stream** continuously (byte-by-byte).
2. Every `interval = 100 ms`:
   - Convert encoder counts to measured wheel speeds (`rad/s`)
   - Run PID (`Compute()`)
   - Apply PWM outputs (`analogWrite`)
   - Publish measured wheel speeds over serial

## 3) Serial protocol

The parser expects wheel commands in this style:

- `r` -> following direction/value apply to right wheel
- `l` -> following direction/value apply to left wheel
- `p` / `n` -> wheel direction (positive / negative)
- numeric text (up to 5 chars, like `12.34`)
- `,` -> end of value and commit it

Typical combined command sequence:
- `rp12.34,ln08.50,`

Meaning:
- right wheel: +12.34 rad/s
- left wheel: -8.50 rad/s

The code flips H-bridge direction pins only when direction changes, then stores magnitude setpoints in:
- `right_wheel_cmd_vel`
- `left_wheel_cmd_vel`

## 4) Encoder math (why that formula works)

Measured velocity in code:

`wheel_meas_vel = (10 * encoder_counter * (60.0/385.0)) * 0.10472`

Where:
- `encoder_counter` = pulses counted in the last `100 ms`
- `* 10` -> pulses/second
- `/ 385` -> revolutions/second (assuming 385 pulses/revolution)
- `* 60` -> RPM
- `* 0.10472` -> RPM to rad/s (`2*pi/60`)

So this is:
- pulses in 0.1 s -> pulses/s -> rev/s -> RPM -> rad/s

## 5) PID control logic

Each wheel has its own PID object:
- Input: measured speed (`right_wheel_meas_vel`, `left_wheel_meas_vel`)
- Setpoint: commanded speed (`right_wheel_cmd_vel`, `left_wheel_cmd_vel`)
- Output: PWM duty (`right_wheel_cmd`, `left_wheel_cmd`, 0..255)

After `Compute()`:
- if setpoint is exactly zero, output is forced to `0.0` to prevent tiny PID outputs from trying to move the motor.

## 6) Encoder direction signs

ISRs count pulses on phase-A rising edges and sample phase-B to infer direction sign:
- right ISR writes `"p"` or `"n"` to `right_wheel_sign`
- left ISR writes `"p"` or `"n"` to `left_wheel_sign`

Note: left sign mapping is intentionally opposite of right in this code, usually due to mirrored encoder/motor mounting.

## 7) Telemetry output

Every 100 ms the firmware prints:

- `"r" + right_sign + right_meas + ",l" + left_sign + left_meas + ","`

Example:
- `rp7.83,ln7.79,`

This gives host software both wheel measured speeds and directions for odometry/control debugging.
