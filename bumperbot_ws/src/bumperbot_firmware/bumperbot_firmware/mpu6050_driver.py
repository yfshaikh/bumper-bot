#!/usr/bin/env python3
# =============================================================================
# mpu6050_driver.py  —  bumperbot_firmware
# =============================================================================
# ROS 2 Python node that reads the MPU6050 6-DOF IMU sensor over I2C and
# publishes the data as sensor_msgs/Imu on /imu/out at 100 Hz.
#
# Role in the system:
#   - On the real robot this is started by real_robot.launch.py as a standalone
#     node alongside the hardware interface.
#   - Its output (/imu/out) is consumed by both bumperbot_localization nodes:
#     the kalman_filter (angular velocity fusion) and imu_republisher (for EKF).
#   - In simulation, this node is NOT used — the Gazebo bridge republishes a
#     simulated IMU on the same /imu/out topic instead.
#
# Hardware: MPU6050 connected via I2C bus 1 (standard Raspberry Pi bus).
#   I2C address: 0x68 (AD0 low)
# =============================================================================

import rclpy.time
import smbus                    # Python I2C library for Linux (SMBus protocol)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data   # best-effort QoS for sensors
from sensor_msgs.msg import Imu

# --- MPU6050 register addresses ---
PWR_MGMT_1   = 0x6B  # Power management register; write 1 to wake from sleep
SMPLRT_DIV   = 0x19  # Sample rate divider (sample rate = 8kHz / (1+SMPLRT_DIV))
CONFIG       = 0x1A  # Low-pass filter config
GYRO_CONFIG  = 0x1B  # Gyroscope full-scale range (24 = ±2000°/s)
INT_ENABLE   = 0x38  # Interrupt enable register
ACCEL_XOUT_H = 0x3B  # High byte of X-axis accelerometer reading
ACCEL_YOUT_H = 0x3D  # High byte of Y-axis accelerometer reading
ACCEL_ZOUT_H = 0x3F  # High byte of Z-axis accelerometer reading
GYRO_XOUT_H  = 0x43  # High byte of X-axis gyroscope reading
GYRO_YOUT_H  = 0x45  # High byte of Y-axis gyroscope reading
GYRO_ZOUT_H  = 0x47  # High byte of Z-axis gyroscope reading
DEVICE_ADDRESS = 0x68  # MPU6050 default I2C address (AD0 pin LOW)


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # Track I2C connection state — used to auto-reconnect on failures
        self.is_connected_ = False
        self.init_i2c()

        # Publish IMU messages with sensor-data QoS (best-effort, no retries)
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        # Reuse a single Imu message object to avoid per-callback allocations
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"  # IMU is rigidly attached to the chassis

        # Timer period: 0.01 s = 100 Hz publish rate
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            # If the I2C bus failed previously, attempt to reinitialise before reading
            if not self.is_connected_:
                self.init_i2c()
            
            # Read raw 16-bit signed counts from the accelerometer registers
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read raw 16-bit signed counts from the gyroscope registers
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Convert raw counts to m/s² and rad/s.
            # Scale factors (1670.13 for accel, 7509.55 for gyro) combine:
            #   - Sensor sensitivity from the datasheet (LSB/g and LSB/(°/s))
            #   - Unit conversion (g → m/s², °/s → rad/s)
            self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
            self.imu_msg_.angular_velocity.x = gyro_x / 7509.55
            self.imu_msg_.angular_velocity.y = gyro_y / 7509.55
            self.imu_msg_.angular_velocity.z = gyro_z / 7509.55

            # Stamp with current ROS clock and publish
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            # I2C error (e.g. sensor unplugged) — mark disconnected and retry
            # on the next timer tick rather than crashing the node
            self.is_connected_ = False

    def init_i2c(self):
        """Open the I2C bus and configure the MPU6050 startup registers."""
        try:
            self.bus_ = smbus.SMBus(1)  # bus 1 = /dev/i2c-1 on Raspberry Pi
            # Write MPU6050 configuration registers in order:
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)   # sample rate = 8kHz/(1+7) = 1kHz
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)   # wake sensor, use internal 8MHz osc
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)       # no digital low-pass filter
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24) # full-scale ±2000°/s
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)   # enable data-ready interrupt
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
        
    def read_raw_data(self, addr):
        """Read a 16-bit two's-complement value from two consecutive registers.

        The MPU6050 stores each axis measurement as a high byte + low byte.
        We combine them into an unsigned 16-bit integer and then convert to
        signed range so negative accelerations/rotations are represented correctly.
        """
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)    # most significant byte
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)   # least significant byte
        
        # Combine high and low bytes into a 16-bit unsigned integer
        value = ((high << 8) | low)
            
        # Convert unsigned [0, 65535] to signed [-32768, 32767] (two's complement)
        if(value > 32768):
            value = value - 65536
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()