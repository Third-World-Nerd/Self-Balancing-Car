#include "mpu_madgwick.h"

#include <SoftwareSerial.h>

// Motor control pins
#define ENA 6  // Enable pin for Motor A
#define IN1 5  // Input 1 for Motor A
#define IN2 4  // Input 2 for Motor A
#define ENB 9  // Enable pin for Motor B
#define IN3 7  // Input 1 for Motor B
#define IN4 8  // Input 2 for Motor B

// Potentiometer pins
#define POT_P A3  // Potentiometer for kp
#define POT_D A2  // Potentiometer for kd

// Define Bluetooth RX & TX pins
SoftwareSerial btSerial(10, 11); // RX, TX for HC-05

// PID constants
float kp = 6.5;
float ki = 0;//9;
float kd = 0.27;
float integral_limit = 10; // Adjust as needed

// Variables
float steadyState = 0.0;
float setpoint = 0.0;
float current_angle = 0.0;
float previous_error = 0.0;
float integral = 0.0;
unsigned long previous_time = 0;
float motor_speed = 0.0;
float acceleration_factor = 3.5; // Adjust acceleration factor

float Loffset = 0;
float Roffset = 0;
int speed = 50;



void setup() {
  imu_setup();
  btSerial.begin(9600);    // HC-05 Bluetooth module
  Serial.begin(2000000);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  unsigned long Time = millis();
  while (millis() - Time < 40000) {
    unsigned long Time1 = millis();
    ins_update_madgwick();
    while (millis() - Time1 < 10);
  }

  Time = millis();
  double totalAngle = 0;
  int count = 0;
  while (millis() - Time < 5000) {
    unsigned long Time2 = millis();
    ins_update_madgwick();
    totalAngle += ypr.roll;
    count++;
    while (millis() - Time2 < 10);
  }
  steadyState = totalAngle / count;
}

void loop() {
  unsigned long Time = millis();

  if (btSerial.available()) {
    char command = btSerial.read();
    Serial.print("Received: ");
    Serial.println(command);

    
    // Add motor control based on command
    if (command == 'F') {
      setpoint = steadyState - 8;
      Serial.println("Moving Forward");
    } else if (command == 'B') {
      setpoint = steadyState + 8;
      Serial.println("Moving Backward");
    } else if (command == 'L') {
      Loffset = -speed;
      Roffset = speed;
      Serial.println("Turning Left");
    } else if (command == 'R') {
      Loffset = speed;
      Roffset = -speed;
      Serial.println("Turning Right");
    } else {
      setpoint = steadyState;
      Loffset = 0;
      Roffset = 0;
      Serial.println("Stopping");
    }
  }

  kp = map(analogRead(POT_P), 0, 1023, 100, 2000) / 100.0;
  kd = map(analogRead(POT_D), 0, 1023, 1, 100) / 100.0;

  ins_update_madgwick();
  current_angle = ypr.roll;

  float error = setpoint - current_angle;
  unsigned long current_time = millis();
  float delta_time = (current_time - previous_time) / 1000.0;

  integral += error * delta_time;
  if (ki * integral > integral_limit) integral = integral_limit / ki;
  else if (ki * integral < -integral_limit) integral = -integral_limit / ki;

  float derivative = (error - previous_error) / delta_time;
  float output = kp * error + ki * integral + kd * derivative;

  previous_error = error;
  previous_time = current_time;

  float motor_speed = output * acceleration_factor;
  // motor_speed += acceleration * delta_time;


  float Lspeed = motor_speed;
  float Rspeed = motor_speed;

  setMotorSpeed(Lspeed+Loffset, Rspeed+Roffset);

  Serial.print(setpoint);
  Serial.print("     "); 
  Serial.print(current_angle);
  Serial.print("     "); 
  Serial.print(kp);
  Serial.print("     "); 
  Serial.print(kd);
  Serial.print("     "); 
  Serial.print(ki * integral);
  Serial.println("    "); 

  while (millis() - Time < 10);
}

void setMotorSpeed(float Lspeed, float Rspeed) {
  int Lpwm = abs(Lspeed);//+15;
  Lpwm = constrain(Lpwm, 0, 255);
  int Rpwm = abs(Rspeed);
  Rpwm = constrain(Rpwm, 0, 255);


  // Left motor control
  if (Lspeed > 0) {
    analogWrite(ENA, Lpwm);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (Lspeed < 0) {
    analogWrite(ENA, Lpwm);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Right motor control
  if (Rspeed > 0) {
    analogWrite(ENB, Rpwm);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (Rspeed < 0) {
    analogWrite(ENB, Rpwm);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

