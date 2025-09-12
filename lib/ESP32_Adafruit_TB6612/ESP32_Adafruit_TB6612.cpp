/******************************************************************************
TB6612.cpp
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Modified by David Eldridge
12/09/2025
https//github.com/MrEldridge22

Resources:
TB6612 SparkFun Library

Original Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457

Mofified Development environment specifics:
Developed on VS Code with PlatformIO
Developed with ESP32-S3

Changes from original:
- Removed Standby pin functionality (not needed for this project as the Adafruit driver board
    ties Standby to Vcc)
- Changed PWM functionality to use ESP32 LEDC functions (ledcWrite, ledcAttachPin, ledcSetup)


******************************************************************************/

#include "ESP32_Adafruit_TB6612.h"
#include <Arduino.h>

Motor::Motor(int In1pin, int In2pin, int PWMpin, int offset, int PWM_CHANNEL_PARAM, int PWM_FREQ_PARAM, int PWM_RESOLUTION_PARAM) {
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Offset = offset;
  PWM_CHANNEL = PWM_CHANNEL_PARAM;
  PWM_FREQ = PWM_FREQ_PARAM;
  PWM_RESOLUTION = PWM_RESOLUTION_PARAM;

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMpin, PWM_CHANNEL);
}

void Motor::drive(int speed) {
  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
}

void Motor::drive(int speed, int duration) {
  drive(speed);
  delay(duration);
}

void Motor::fwd(int speed) {
   digitalWrite(In1, HIGH);
   digitalWrite(In2, LOW);
   ledcWrite(PWM_CHANNEL, speed);

}

void Motor::rev(int speed) {
   digitalWrite(In1, LOW);
   digitalWrite(In2, HIGH);
   ledcWrite(PWM_CHANNEL, speed);
}

void Motor::brake() {
   digitalWrite(In1, HIGH);
   digitalWrite(In2, HIGH);
   ledcWrite(PWM_CHANNEL, 0);
}

void forward(Motor motor1, Motor motor2, int speed) {
	motor1.drive(speed);
	motor2.drive(speed);
}

void forward(Motor motor1, Motor motor2) {
	motor1.drive(DEFAULTSPEED);
	motor2.drive(DEFAULTSPEED);
}


void back(Motor motor1, Motor motor2, int speed) {
	int temp = abs(speed);
	motor1.drive(-temp);
	motor2.drive(-temp);
}

void back(Motor motor1, Motor motor2) {
	motor1.drive(-DEFAULTSPEED);
	motor2.drive(-DEFAULTSPEED);
}

void left(Motor left, Motor right, int speed) {
	int temp = abs(speed)/2;
	left.drive(-temp);
	right.drive(temp);
	
}

void right(Motor left, Motor right, int speed) {
	int temp = abs(speed)/2;
	left.drive(temp);
	right.drive(-temp);
	
}

void brake(Motor motor1, Motor motor2) {
	motor1.brake();
	motor2.brake();
}