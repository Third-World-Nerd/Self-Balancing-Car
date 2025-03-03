#include "mpu_madgwick.h"

// Motor control pins
#define ENA 6  // Enable pin for Motor A
#define IN1 4  // Input 1 for Motor A
#define IN2 5  // Input 2 for Motor A
#define ENB 9  // Enable pin for Motor B
#define IN3 7  // Input 1 for Motor B
#define IN4 8  // Input 2 for Motor B


// Variables for logging
float current_angle = 0.0;
int angle_limit = 1;


void setup() {
  imu_setup();
  Serial.begin(2000000);

  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //IMU calibration
  unsigned long Time = millis();
  while (millis() - Time < 40000) {
    unsigned long Time1 = millis();
    ins_update_madgwick();
    while (millis() - Time1 < 10);
  }
}

void loop() {
  unsigned long Time = millis();

    // Read the current angle from the IMU
    ins_update_madgwick();
    current_angle = ypr.roll; // Assuming roll is the tilt angle
    float input_signal;
    if (current_angle<-angle_limit){
      input_signal = 110;
    } 
    if (current_angle>angle_limit){
      input_signal= -110;
    }
    if (current_angle>-angle_limit && current_angle<angle_limit)
    {
      input_signal = 0;
    }
    input_signal = 110;

    // Apply PRBS input to motors
    setMotorSpeed(input_signal);

    // Log data for MATLAB
    Serial.print(input_signal);
    Serial.print(",");
    Serial.println(current_angle);

    while(millis()-Time<10);
}

void setMotorSpeed(float speed) {
  int pwmA = constrain(abs(speed), 0, 255);
  int pwmB = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    // Forward motion
    analogWrite(ENA, pwmA);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    analogWrite(ENB, pwmB);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    // Backward motion
    analogWrite(ENA, pwmA);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    analogWrite(ENB, pwmB);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Stop
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}
