#include <PID_v1.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11 // PWM
#define L298N_in4 8  // Dir Motor B
#define L298N_in3 7  // Dir Motor B
#define L298N_in2 13 // Dir Motor A
#define L298N_in1 12 // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 3 // Interrupt
#define right_encoder_phaseB 5
#define left_encoder_phaseA 2 // Interrupt
#define left_encoder_phaseB 4

// Encoder counters are sampled every 100 ms and reset after each control update.
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
// Direction signs used in outbound telemetry ("p" positive, "n" negative).
String right_wheel_sign = "p"; // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Serial parser state for command stream such as: rp12.34,ln08.90,
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
// Holds ASCII numeric payload before conversion with atof(), e.g. "12.34".
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0; // rad/s
double left_wheel_cmd_vel = 0.0;  // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0; // rad/s
double left_wheel_meas_vel = 0.0;  // rad/s
// Output - Command
double right_wheel_cmd = 0.0; // PWM duty 0-255
double left_wheel_cmd = 0.0;  // PWM duty 0-255
// Tuning
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup()
{
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Initial forward direction for both motors.
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // Enable PID controllers.
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoder phase-B pins (phase-A pins are interrupt sources).
  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  // Count pulses on phase-A rising edges.
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop()
{
  // Parse wheel velocity commands from serial byte-by-byte.
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if (chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if (chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if (chr == 'p')
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if (is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if (chr == 'n')
    {
      if (is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if (is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    // ',' terminates one numeric field and commits it.
    else if (chr == ',')
    {
      if (is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if (is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset numeric parser buffer for next field.
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Collect numeric characters, e.g. "08.75".
    else
    {
      if (value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Run velocity estimation + PID control at fixed 100 ms intervals.
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval)
  {
    // Conversion chain:
    // ticks/0.1s -> ticks/s (*10) -> RPM (*(60/385)) -> rad/s (*2*pi/60 = 0.10472)
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0 / 385.0)) * 0.10472;

    // Compute PWM commands from measured speed vs setpoint.
    rightMotor.Compute();
    leftMotor.Compute();

    // If target speed is zero, force PWM to zero (avoid tiny residual outputs).
    if (right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if (left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    // Publish measured wheel velocities and signs.
    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);

    // Prepare next control window.
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    // Apply PWM to both motors.
    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  // Direction inferred from phase-B level when phase-A rises.
  if (digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "p";
  }
  else
  {
    right_wheel_sign = "n";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  // Left wheel sign mapping is mirrored relative to right wheel mounting.
  if (digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "n";
  }
  else
  {
    left_wheel_sign = "p";
  }
  left_encoder_counter++;
}
