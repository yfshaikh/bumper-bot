// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13 // Dir Motor A
#define L298N_in1 12 // Dir Motor A

#define right_encoder_phaseA 3 // Interrupt
#define right_encoder_phaseB 5

// Incremented in the ISR on each rising edge of encoder phase A.
unsigned int right_encoder_counter = 0;
// "p" for positive direction, "n" for negative direction.
String right_encoder_sign = "p";
// Measured wheel angular velocity (rad/s).
double right_wheel_meas_vel = 0.0;

void setup()
{
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);

  // Phase B is sampled in the ISR to infer direction.
  pinMode(right_encoder_phaseB, INPUT);
  // Count encoder ticks on each rising edge of phase A.
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
}

void loop()
{
  // Velocity math (every 100 ms loop):
  // 1) right_encoder_counter = ticks per 0.1 s
  // 2) *10 -> ticks/s
  // 3) *(60/385) -> RPM  (385 ticks/rev)
  // 4) *0.10472 -> rad/s (2*pi/60)
  right_wheel_meas_vel = (10 * right_encoder_counter * (60.0 / 385.0)) * 0.10472;

  // Prefix format: "r" + direction sign + velocity value.
  String encoder_read = "r" + right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_read);

  // Reset windowed tick count after publishing this sample.
  right_encoder_counter = 0;

  // Apply a fixed PWM command to keep motor spinning.
  analogWrite(L298N_enA, 100);
  // Sampling period = 100 ms.
  delay(100);
}

void rightEncoderCallback()
{
  // Quadrature direction check:
  // If phase B is HIGH at phase A rising edge, mark positive; else negative.
  if (digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p";
  }
  else
  {
    right_encoder_sign = "n";
  }
  // Count one encoder tick.
  right_encoder_counter++;
}
