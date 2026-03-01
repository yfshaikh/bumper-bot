# `mpu6050_driver.py` explained

This node reads IMU data from an MPU6050 over I2C and publishes `sensor_msgs/msg/Imu` on `/imu/out`.

## 1) High-level behavior

- Creates a ROS 2 node named `mpu6050_driver`
- Initializes the MPU6050 registers over I2C bus 1
- Runs a periodic timer callback every `0.01 s` (100 Hz)
- Reads raw accelerometer and gyroscope registers
- Converts raw counts to SI-like units used by the node
- Publishes the `Imu` message with a fresh timestamp
- If I2C fails (`OSError`), marks sensor disconnected and retries init on next timer cycle

## 2) Register map used

Important register addresses in the file:

- `PWR_MGMT_1 (0x6B)` -> wake and clock source config
- `SMPLRT_DIV (0x19)` -> sample-rate divider
- `CONFIG (0x1A)` -> digital low-pass / general config
- `GYRO_CONFIG (0x1B)` -> gyro full-scale range
- `INT_ENABLE (0x38)` -> enable interrupt source(s)
- Sensor data starts at:
  - accel: `0x3B`, `0x3D`, `0x3F`
  - gyro: `0x43`, `0x45`, `0x47`

Device address is `0x68`.

## 3) Data path in the timer callback

Every timer tick:

1. Ensure I2C is connected (re-init if needed).
2. Read six 16-bit signed values:
   - `acc_x`, `acc_y`, `acc_z`
   - `gyro_x`, `gyro_y`, `gyro_z`
3. Convert raw values to published values:
   - `linear_acceleration.{x,y,z} = raw_acc / 1670.13`
   - `angular_velocity.{x,y,z} = raw_gyro / 7509.55`
4. Stamp and publish the `Imu` message.

## 4) 16-bit signed conversion math

MPU6050 outputs each axis as two bytes:

- `value = (high << 8) | low` gives unsigned 16-bit value in `[0, 65535]`.
- Two's complement conversion:
  - if value is in upper half, subtract `65536` to map into signed range `[-32768, 32767]`.

In code:

- if `value > 32768`: `value = value - 65536`

This converts raw register bytes to signed integer sensor counts.

## 5) Unit scaling math

The file uses fixed divisors:

- `1670.13` for accelerometer
- `7509.55` for gyroscope

So published values are:

- `acc_scaled = raw_acc / 1670.13`
- `gyro_scaled = raw_gyro / 7509.55`

These divisors encode both sensor sensitivity and unit conversion chosen by this project. In many MPU6050 examples, conversion is often done in two stages (LSB-to-g, then g-to-m/s^2; and LSB-to-deg/s, then deg/s-to-rad/s). Here those steps are pre-combined into single constants.

## 6) Fault tolerance

Any I2C access failure raises `OSError`. The code:

- sets `is_connected_ = False`
- skips publishing for that cycle
- retries initialization on the next timer callback

This provides simple automatic recovery after temporary disconnects.

## 7) ROS interface

- Topic: `/imu/out`
- Message type: `sensor_msgs/msg/Imu`
- QoS: `qos_profile_sensor_data`
- Frame ID: `base_footprint`

Orientation is not computed in this node; only angular velocity and linear acceleration fields are filled.
