/******************************************************************************
ESP32_Adafruit_TB6612.h

Original SparkFun Library
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Modified by David Eldridge
https://github.com/MrEldridge22
12/09/2025

Original Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457

Modified Development environment specifics:
Developed on VS Code with PlatformIO
Developed with ESP32-S3
******************************************************************************/


#ifndef ESP32_ADAFRUIT_TB6612_h
#define ESP32_ADAFRUIT_TB6612_h

#include <Arduino.h>

//used in some functions so you don't have to send a speed
#define DEFAULTSPEED 2048

class Motor {
  public:
    // Constructor. Mainly sets up pins.
    Motor(int In1pin, int In2pin, int PWMpin, int offset, int PWM_CHANNEL_PARAM, int PWM_FREQ_PARAM, int PWM_RESOLUTION_PARAM);      

    // Drive in direction given by sign, at speed given by magnitude of the 
	//parameter.
    void drive(int speed);  
	
	// drive(), but with a delay(duration)
    void drive(int speed, int duration);  
	
	//Stops motor by setting both input pins high
    void brake(); 

	
  private:
    //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
	int In1, In2, PWM, Offset, PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION;
	
	//private functions that spin the motor CC and CCW
	void fwd(int speed);
	void rev(int speed);


};

//Takes 2 motors and goes forward, if it does not go forward adjust offset 
//values until it does.  These will also take a negative number and go backwards
//There is also an optional speed input, if speed is not used, the function will
//use the DEFAULTSPEED constant.
void forward(Motor motor1, Motor motor2, int speed);
void forward(Motor motor1, Motor motor2);

//Similar to forward, will take 2 motors and go backwards.  This will take either
//a positive or negative number and will go backwards either way.  Once again the
//speed input is optional and will use DEFAULTSPEED if it is not defined.
void back(Motor motor1, Motor motor2, int speed);
void back(Motor motor1, Motor motor2);

//Left and right take 2 motors, and it is important the order they are sent.
//The left motor should be on the left side of the bot.  These functions
//also take a speed value
void left(Motor left, Motor right, int speed);
void right(Motor left, Motor right, int speed);

//This function takes 2 motors and and brakes them
void brake(Motor motor1, Motor motor2);

#endif