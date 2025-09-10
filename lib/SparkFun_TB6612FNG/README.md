SparkFun SparkFun Motor Driver - Dual TB6612FNG (1A) Arduino Library (ESP32 Modified)
===================================================================================

![SparkFun Motor Driver - Dual TB6612FNG (1A)](https://cdn.sparkfun.com//assets/parts/3/1/5/7/09457-01b.jpg)

[*SparkFun Motor Driver - Dual TB6612FNG (1A) (ROB-0457)*](https://www.sparkfun.com/products/9457)

**This is a modified version of the original SparkFun TB6612FNG library, optimized for ESP32 microcontrollers.**

The TB6612FNG motor driver can control up to two DC motors at a constant current of 1.2A (3.2A peak). 
Two input signals (IN1 and IN2) can be used to control the motor in one of four function modes - CW, CCW, short-brake, and stop. 
The two motor outputs (A and B) can be separately controlled, the speed of each motor is controlled via a PWM input signal with a frequency up to 100kHz. 
The STBY pin should be pulled high to take the motor out of standby mode.

## ESP32 Modifications

This modified version includes the following changes for ESP32 compatibility:

### Key Changes:
1. **Hardware PWM Support**: Uses ESP32's LEDC (LED Controller) peripheral instead of analogWrite()
2. **Individual PWM Channels**: Each motor gets its own PWM channel for independent control
3. **Configurable PWM Parameters**: Frequency and resolution can be set per motor instance
4. **Proper Channel Management**: Fixed constructor to properly store PWM channel parameters

### Constructor Changes:
```cpp
Motor(int In1pin, int In2pin, int PWMpin, int offset, int STBYpin, 
      int PWM_CHANNEL, int PWM_FREQ, int PWM_RESOLUTION);
```

### Usage Example for ESP32:
```cpp
#include <SparkFun_TB6612.h>

// Motor pin definitions
#define AIN1 9
#define AIN2 11
#define PWMA 10
#define BIN1 13
#define BIN2 14
#define PWMB 18
#define STBY 7

// PWM configuration
#define PWMA_CHANNEL 0
#define PWMB_CHANNEL 1
#define PWM_FREQ 1000        // 1 kHz frequency
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)

// Motor offsets (1 or -1 to correct direction)
const int offsetA = 1;
const int offsetB = 1;

// Create motor instances
Motor motorA(AIN1, AIN2, PWMA, offsetA, STBY, PWMA_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
Motor motorB(BIN1, BIN2, PWMB, offsetB, STBY, PWMB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

void setup() {
  // Motors are automatically initialized in constructor
}

void loop() {
  // Drive motors with values from -1023 to 1023
  motorA.drive(512);   // 50% speed forward
  motorB.drive(-256);  // 25% speed reverse
  delay(1000);
}
```

### Important ESP32 Considerations:
- **PWM Channels**: ESP32 has 16 PWM channels (0-15). Use different channels for each motor.
- **PWM Frequency**: Recommended 1000-5000 Hz for motor control. Higher frequencies may not work reliably.
- **PWM Resolution**: 10-bit (0-1023) is recommended. Higher resolutions require lower frequencies.
- **GPIO Limitations**: Avoid input-only pins (35-39) for PWM output. Use standard GPIO pins.
- **Channel Conflicts**: Ensure no other code uses the same PWM channels.

### Tested GPIO Pins for ESP32-S3:
- **Safe PWM Pins**: 2, 4, 5, 9, 10, 11, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
- **Avoid**: GPIO 12 (flash voltage), GPIO 35-39 (input-only)

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. 
* **/extras** - Additional documentation for the user. These files are ignored by the IDE. 
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE. 
* **library.properties** - General library properties for the Arduino package manager. 

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Product Repository](https://github.com/sparkfun/Motor_Driver-Dual_TB6612FNG/tree/V_1.1)** - Main repository (including hardware files) for the TB6612FNG Motor Driver.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide)** - Basic hookup guide for the TB6612FNG Motor Driver.

Products that use this Library 
---------------------------------

* [ROB-09457](https://www.sparkfun.com/products/9457)- Motor Driver
* [ROB-13845](https://www.sparkfun.com/products/13845)- Motor Driver with headers installed


License Information
-------------------

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

-Your friends at SparkFun.


