// =============================================================================
// robot_control.ino  —  bumperbot_firmware/firmware/robot_control
// =============================================================================
// Arduino firmware that runs on the microcontroller connected to the robot's
// motors and wheel encoders. This sketch is the lowest layer in the control
// stack — it receives velocity commands from BumperbotInterface (ROS 2) over
// serial, drives the motors using PID control, and sends encoder feedback back.
//
// Serial protocol (115200 baud, both directions):
//   Commands  ← ROS:     "r<sign><value>,l<sign><value>,"
//                          e.g. "rp12.34,ln05.67,"
//   Feedback  → ROS:     "r<sign><value>,l<sign><value>"
//                          e.g. "rp11.20,lp04.90"
//   Where <sign> is 'p' (positive) or 'n' (negative), <value> is rad/s.
//
// Hardware connections:
//   L298N H-Bridge:
//     enA (PWM)  → pin 9   (right motor speed)
//     enB (PWM)  → pin 11  (left motor speed)
//     in1/in2    → 12/13   (right motor direction)
//     in3/in4    → 7/8     (left motor direction)
//   Wheel encoders (quadrature, using only phase A for speed):
//     Right encoder phase A → pin 3 (interrupt)
//     Right encoder phase B → pin 5 (direction sense)
//     Left  encoder phase A → pin 2 (interrupt)
//     Left  encoder phase B → pin 4 (direction sense)
// =============================================================================

#include <PID_v1.h> // Brett Beauregard's Arduino PID library

// --- L298N H-Bridge pin definitions ---
#define L298N_enA 9  // PWM: right motor speed (0–255)
#define L298N_enB 11 // PWM: left motor speed  (0–255)
#define L298N_in4 8  // Direction control Motor B (left)
#define L298N_in3 7  // Direction control Motor B (left)
#define L298N_in2 13 // Direction control Motor A (right)
#define L298N_in1 12 // Direction control Motor A (right)

// --- Wheel encoder pin definitions ---
#define right_encoder_phaseA 3 // Phase A triggers an interrupt on RISING edge
#define right_encoder_phaseB 5 // Phase B level when A rises encodes direction
#define left_encoder_phaseA 2  // Phase A triggers an interrupt on RISING edge
#define left_encoder_phaseB 4  // Phase B level when A rises encodes direction

// --- Encoder pulse counters ---
// Counts pulses in the current 100 ms window; reset after each control update.
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
// Direction sign reported in telemetry ('p' = forward, 'n' = backward)
String right_wheel_sign = "p";
String left_wheel_sign = "p";
unsigned long last_millis = 0;
const unsigned long interval =
    100; // control loop period in milliseconds (10 Hz)

// --- Serial command parser state ---
// Commands arrive byte-by-byte; these flags track which field is being parsed.
bool is_right_wheel_cmd = false; // currently accumulating the right wheel value
bool is_left_wheel_cmd = false;  // currently accumulating the left wheel value
bool is_right_wheel_forward = true; // current direction of right motor
bool is_left_wheel_forward = true;  // current direction of left motor
// Numeric payload buffer — fixed size "XX.XX\0" (5 chars + null terminator)
char value[] = "00.00";
uint8_t value_idx = 0;        // write cursor into value[]
bool is_cmd_complete = false; // set true after both wheel values received

// --- PID variables ---
// Setpoints (desired speed from ROS), measurements (from encoders), outputs
// (PWM)
double right_wheel_cmd_vel = 0.0;  // desired right wheel speed (rad/s)
double left_wheel_cmd_vel = 0.0;   // desired left wheel speed (rad/s)
double right_wheel_meas_vel = 0.0; // measured right wheel speed (rad/s)
double left_wheel_meas_vel = 0.0;  // measured left wheel speed (rad/s)
double right_wheel_cmd = 0.0;      // PID output → right motor PWM (0–255)
double left_wheel_cmd = 0.0;       // PID output → left motor PWM (0–255)

// --- PID tuning gains (tuned empirically) ---
double Kp_r = 11.5, Ki_r = 7.5, Kd_r = 0.1; // right motor gains
double Kp_l = 12.8, Ki_l = 8.3, Kd_l = 0.1; // left motor gains

// Instantiate PID controllers — DIRECT mode: output increases when setpoint >
// input
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel,
               Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l,
              Ki_l, Kd_l, DIRECT);

void setup() {
  // Configure all L298N direction pins as outputs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set initial forward direction for both motors (HIGH/LOW on in1/in2,
  // in3/in4)
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // Enable PID controllers in automatic mode (they'll compute every Compute()
  // call)
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(
      115200); // must match BumperbotInterface::on_activate() baud rate

  // Configure encoder phase-B sense pins as inputs (no pull-up needed — wired
  // externally)
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Attach ISRs to phase-A pins — triggered on RISING edge of encoder pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA),
                  rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA),
                  leftEncoderCallback, RISING);
}

void loop() {
  // --- Serial command parsing (non-blocking, byte at a time) ---
  if (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') // start of right-wheel token
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0; // reset numeric buffer
      is_cmd_complete = false;
    } else if (chr == 'l') // start of left-wheel token
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    } else if (chr == 'p') // positive direction
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward) {
        // Toggle both direction pins to reverse the H-bridge (XOR with HIGH)
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      } else if (is_left_wheel_cmd && !is_left_wheel_forward) {
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    } else if (chr == 'n') // negative direction
    {
      if (is_right_wheel_cmd && is_right_wheel_forward) {
        // Reverse H-bridge for right motor
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      } else if (is_left_wheel_cmd && is_left_wheel_forward) {
        // Reverse H-bridge for left motor
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    } else if (chr == ',') // comma terminates a numeric field
    {
      if (is_right_wheel_cmd) {
        // Convert the accumulated ASCII string to a float and store as setpoint
        right_wheel_cmd_vel = atof(value);
      } else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true; // both wheels received — full command is ready
      }
      // Reset the numeric buffer for the next field
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    } else // digit or decimal point — accumulate into the numeric buffer
    {
      if (value_idx < 5) // guard against buffer overflow
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // --- Control loop (runs every 100 ms) ---
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    // Convert pulse count to rad/s:
    //   pulses / 0.1s → pulses/s  (×10)
    //   pulses/s / 385 pulses/rev → rev/s  (385 = encoder CPR for this motor)
    //   rev/s × 60 → RPM
    //   RPM × 2π/60 (= 0.10472) → rad/s
    right_wheel_meas_vel =
        (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel =
        (10 * left_encoder_counter * (60.0 / 385.0)) * 0.10472;

    // Run both PID controllers — updates right_wheel_cmd and left_wheel_cmd
    rightMotor.Compute();
    leftMotor.Compute();

    // If the commanded speed is zero, hard-stop the motor (override residual
    // PID output)
    if (right_wheel_cmd_vel == 0.0) {
      right_wheel_cmd = 0.0;
    }
    if (left_wheel_cmd_vel == 0.0) {
      left_wheel_cmd = 0.0;
    }

    // Send encoder feedback to ROS (BumperbotInterface::read() parses this)
    String encoder_read = "r" + right_wheel_sign +
                          String(right_wheel_meas_vel) + ",l" +
                          left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    // Reset for the next 100 ms window
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    // Apply computed PWM duty cycle to the H-bridge enable pins
    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

// ISR: triggered on every rising edge of the right encoder phase A
void rightEncoderCallback() {
  // Read phase B at the moment phase A rises to determine rotation direction.
  // HIGH → phase B leads A → forward (positive) rotation
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_wheel_sign = "p";
  } else {
    right_wheel_sign = "n";
  }
  right_encoder_counter++; // count this pulse
}

// ISR: triggered on every rising edge of the left encoder phase A
void leftEncoderCallback() {
  // Left wheel is mounted mirrored relative to right, so direction logic is
  // inverted. HIGH phase B → negative (backward) when viewed from outside the
  // robot
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_wheel_sign = "n";
  } else {
    left_wheel_sign = "p";
  }
  left_encoder_counter++; // count this pulse
}
