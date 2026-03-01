# Bumperbot — Real Robot Guide

How to build, flash, and run the physical Bumperbot robot, plus a reference for common issues and how to fix them.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Hardware Overview](#2-hardware-overview)
3. [Step 1 — Flash the Arduino Firmware](#3-step-1--flash-the-arduino-firmware)
4. [Step 2 — Pair the Bluetooth Controller](#4-step-2--pair-the-bluetooth-controller)
5. [Step 3 — Build the ROS 2 Workspace](#5-step-3--build-the-ros-2-workspace)
6. [Step 4 — Launch the Robot](#6-step-4--launch-the-robot)
7. [Step 5 — Drive with the Controller](#7-step-5--drive-with-the-controller)
8. [Verifying the System is Working](#8-verifying-the-system-is-working)
9. [Common Issues and Fixes](#9-common-issues-and-fixes)

---

## 1. Prerequisites

- Raspberry Pi running Ubuntu 22.04 with ROS 2 Humble installed
- Arduino Nano connected via USB
- 8BitDo Ultimate 2C Wireless Controller (or compatible Xbox-layout gamepad)
- L298N motor driver, two DC motors with quadrature encoders, MPU-6050 IMU

---

## 2. Hardware Overview

### Arduino Pin Mapping

| Signal | Arduino Pin |
|---|---|
| Right motor PWM (`enA`) | 9 |
| Right motor direction `in1` | 12 |
| Right motor direction `in2` | 13 |
| Left motor PWM (`enB`) | 11 |
| Left motor direction `in3` | 7 |
| Left motor direction `in4` | 8 |
| Right encoder phase A (interrupt) | 3 |
| Right encoder phase B | 5 |
| Left encoder phase A (interrupt) | 2 |
| Left encoder phase B | 4 |

### Controller Button/Axis Map (8BitDo Ultimate 2C — XInput mode)

| Control | Index | Function |
|---|---|---|
| Left stick Y | Axis 1 | Forward / backward |
| Right stick X | Axis 2 | Turn left / right |
| RB (right bumper) | Button 7 | Deadman — must hold to move |

---

## 3. Step 1 — Flash the Arduino Firmware

Open the sketch in the Arduino IDE and upload it to the Nano:

```
src/bumperbot_firmware/firmware/robot_control/robot_control.ino
```

Verify the serial port (usually `/dev/ttyUSB0` or `/dev/ttyACM0`) and baud rate (57600) match the hardware interface configuration before uploading.

---

## 4. Step 2 — Pair the Bluetooth Controller

On the Raspberry Pi, open the Bluetooth shell:

```bash
bluetoothctl
```

Inside the shell, run these commands in order. Put the controller into pairing mode first (hold the pairing button until the light flashes rapidly — use **XInput "X" mode** on the 8BitDo):

```
scan on
# Wait until you see "8BitDo Ultimate 2C Wireless" appear with its MAC address
pair <MAC_ADDRESS>
trust <MAC_ADDRESS>
connect <MAC_ADDRESS>
exit
```

Confirm the device is present:

```bash
ls /dev/input/js*
# Should show /dev/input/js0
```

### Make the device accessible to ROS 2 (one-time setup)

The ROS 2 `joy` node uses the `evdev` API (`/dev/input/event*`), which requires read permission. Find the event device for the controller:

```bash
cat /proc/bus/input/devices | grep -A6 "8BitDo"
# Look for: H: Handlers=event5 js0  (event number may differ)
```

**Temporary fix** (resets on reboot):

```bash
sudo chmod a+r /dev/input/event5   # replace 5 with your event number
```

**Permanent fix** — create a udev rule:

```bash
sudo nano /etc/udev/rules.d/99-8bitdo.rules
```

Paste:
```
SUBSYSTEM=="input", ATTRS{name}=="8BitDo Ultimate 2C Wireless", GROUP="input", MODE="0660"
```

Apply it:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 5. Step 3 — Build the ROS 2 Workspace

On the Raspberry Pi (inside `~/bumperbot_ws`):

```bash
cd ~/bumperbot_ws
colcon build --symlink-install
source install/setup.bash
```

Add the source line to `~/.bashrc` so you don't have to repeat it:

```bash
echo "source ~/bumperbot_ws/install/setup.bash" >> ~/.bashrc
```

---

## 6. Step 4 — Launch the Robot

```bash
ros2 launch bumperbot_bringup real_robot.launch.py
```

This single command starts:

- `robot_state_publisher` — publishes the robot URDF/TF tree
- `ros2_control_node` — loads the hardware interface (serial comms to Arduino)
- `joint_state_broadcaster` — publishes wheel joint states
- `bumperbot_controller` (diff drive) — converts velocity commands to per-wheel commands
- `joy_node` — reads the Bluetooth gamepad
- `joy_teleop` — maps gamepad axes/buttons to `cmd_vel`
- `mpu6050_driver` — reads the IMU

All nodes should appear in `ros2 node list` within a few seconds.

---

## 7. Step 5 — Drive with the Controller

**Hold RB (right bumper) and move the left stick** to drive.

| Stick | Effect |
|---|---|
| Left stick forward/back | Linear velocity |
| Right stick left/right | Angular velocity (turning) |

If RB is not held, no motion commands are sent (deadman switch safety feature).

---

## 8. Verifying the System is Working

Check that `/joy` is publishing (controller input is reaching ROS):
```bash
ros2 topic hz /joy
```

Check that velocity commands are reaching the controller:
```bash
ros2 topic echo /bumperbot_controller/cmd_vel
```

Check wheel encoder feedback:
```bash
ros2 topic echo /joint_states
```

While driving, both `wheel_left_joint` and `wheel_right_joint` `velocity` values should be non-zero.

Send a manual drive command to test both wheels independently:
```bash
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"
```

---

## 9. Common Issues and Fixes

---

### `/joy` topic not publishing — `WARNING: topic [/joy] does not appear to be published yet`

**Cause:** The `joy_node` cannot open the controller's event device, usually a permissions issue on `/dev/input/event*`.

**Check:**
```bash
ls -la /dev/input/event*
cat /proc/bus/input/devices | grep -A6 "8BitDo"
```

If the event file is `crw-rw---- 1 root root` (no group/world read), the node silently fails.

**Fix:** See [Step 2 — Pair the Bluetooth Controller](#4-step-2--pair-the-bluetooth-controller) for the temporary (`chmod`) and permanent (`udev` rule) fixes.

---

### Controller connected but robot does not move

**Check in order:**

1. Confirm `/joy` is publishing: `ros2 topic hz /joy`
2. Confirm you are **holding RB** (button 7) while moving the stick — this is the deadman button.
3. Confirm `cmd_vel` is being published: `ros2 topic echo /bumperbot_controller/cmd_vel`
4. Confirm the diff drive controller is active: `ros2 control list_controllers`

---

### Only one wheel moves

**Symptom:** One motor spins normally; the other does not spin at all even though the encoder reads correctly when spun by hand.

**Root cause:** The motor that is not spinning is receiving a PID setpoint but the drive circuit is not delivering current. The encoder working on manual spin confirms the encoder itself is fine — the issue is upstream in the motor drive path.

**Diagnosis steps:**

1. Run `ros2 topic echo /joint_states` while driving. Both `wheel_left_joint` and `wheel_right_joint` should show non-zero velocities. If one stays at `0.0`, that motor's PID is commanding PWM but the motor is not responding.

2. Send a straight-line command to isolate the problem:
   ```bash
   ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \
     "{header: {stamp: {sec: 0}}, twist: {linear: {x: 0.3}, angular: {z: 0.0}}}"
   ```
   Both wheels should spin. If only one spins, it is a hardware/wiring issue.

**Physical checks for the non-spinning motor (e.g. right motor = L298N channel A):**

| Check | What to look for |
|---|---|
| Arduino pin 9 → L298N `enA` | Wire seated firmly at both ends |
| Arduino pins 12, 13 → L298N `in1`, `in2` | Not swapped or loose |
| L298N `OUT1` / `OUT2` → motor terminals | Connector not pulled out |
| L298N channel not burnt | Swap `enA`/`enB` jumpers temporarily — if the previously working side stops and the dead side starts, the Arduino wiring for that channel is the issue |

---

### Joystick axes or buttons feel wrong / wrong axis controls direction

The `joy` package axis/button indices for the 8BitDo Ultimate 2C in XInput mode:

```
jstest /dev/input/js0
```

| Physical control | jstest axis/button |
|---|---|
| Left stick X | Axis 0 |
| Left stick Y | Axis 1 |
| Right stick X | Axis 2 (use for turning) |
| Right stick Y | Axis 3 |
| LT (trigger) | Axis 4 |
| RT (trigger) | Axis 5 |
| LB | Button 6 |
| RB (deadman) | Button 7 |

Adjust `src/bumperbot_controller/config/joy_teleop.yaml` to remap axes/buttons, then rebuild and re-copy the file to the Pi.

---

### Encoders not counting / position stuck at 0

**Cause:** Encoder wires are loose or the interrupt pins are not connected.

- Right encoder phase A must connect to Arduino **pin 3** (hardware interrupt).
- Left encoder phase A must connect to Arduino **pin 2** (hardware interrupt).
- Phase B pins (5 and 4 respectively) determine direction — if swapped, the wheel will report the wrong sign.

Verify by running `ros2 topic echo /joint_states` and manually spinning each wheel by hand. The position value for the corresponding joint should change.

---

### IMU not publishing

**Check:**
```bash
ros2 topic echo /imu/out
```

If nothing appears, the MPU-6050 I2C connection may be loose. Verify SDA/SCL wiring and that the I2C bus is enabled on the Raspberry Pi (`sudo raspi-config` → Interface Options → I2C).
