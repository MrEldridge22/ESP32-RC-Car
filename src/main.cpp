#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32_Adafruit_TB6612.h>

// Joystick analog pins
#define JOY_LEFT_Y 5
#define JOY_LEFT_X 6
#define JOY_LEFT_BTN 4
#define JOY_RIGHT_Y 1
#define JOY_RIGHT_X 2
#define JOY_RIGHT_BTN 38

//Adafruit Motor Driver Pins
#define AIN1 10
#define AIN2 9
#define PWMA 3
#define BIN1 21
#define BIN2 45
#define PWMB 0

const int offsetA = 1;
const int offsetB = 1;


// PWM channels for ESP32 LEDC
#define PWMA_CHANNEL 0
#define PWMB_CHANNEL 1
#define PWM_FREQ 1000 // 1 kHz
#define PWM_RESOLUTION 10 // 10 bits (0-1023)

// Variables to hold motor values
int motAValue;
int motBValue;

Motor motorA(AIN1, AIN2, PWMA, offsetA, PWMA_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
Motor motorB(BIN1, BIN2, PWMB, offsetB, PWMB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

bool isReceiver = true; // Set to 'true' for Receiver, 'false' for Transmitter
bool debug = true;  // Set to 'true' to enable debug output on Serial Monitor
bool getMac = false; // Set to 'true' to get the MAC address of the ESP32-S3, false for normal operation

// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0xE4, 0xB0, 0x63, 0xAA, 0x9E, 0x78};

// REPLACE WITH THE MAC Address of your transmitter
uint8_t broadcastAddress[] = {0xE4, 0xB0, 0x63, 0xAA, 0x9E, 0x74};

// Variable to store if sending data was successful
String success;

// Structure to send and receive data
typedef struct struct_message {
    int motA;
    int motB;
} struct_message;

// Create a peer info structure
esp_now_peer_info_t peerInfo;

// Incoming message structure
struct_message incomingValues;

// Function prototypes
void receiverSetup();
void receiverLoop();
void transmitterSetup();
void transmitterLoop();
int readAxis(byte pin);

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingValues, incomingData, sizeof(incomingValues));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  motAValue = incomingValues.motA;
  motBValue = incomingValues.motB;
}


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if (getMac){
    // Get MAC Address
    Serial.print("ESP32-S3 MAC Address: ");
    Serial.println(WiFi.macAddress());
    return; // Stop setup here to avoid ESP-NOW init and related messages
  }
  else if (isReceiver){
    if (debug){
      Serial.println("Starting Receiver Setup");
    }
    receiverSetup();
  }
  else if (!isReceiver){
    if (debug){
      Serial.println("Starting Transmitter Setup");
    }
    transmitterSetup();
  }

}

void loop() {
  if (isReceiver && !getMac) {
    receiverLoop();
  }
  else if (!isReceiver && !getMac){
    transmitterLoop();
  }

}

// Receiver setup function, this is called in the main setup() and replaces the setup() function for the receiver
void receiverSetup() {
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("ESP-NOW Receiver Initialized");

  Serial.println("Receiver Setup Complete");
}

// Receiver loop function, this is called in the main loop() and replaces the loop() function for the receiver
void receiverLoop() {
  // Debugging Output.
  if(debug){
    Serial.print("Motor A: ");
    Serial.print(motAValue);
    Serial.print(" | Motor B: ");
    Serial.println(motBValue);
  }

  // Drive motors
  motorA.drive(motAValue);
  motorB.drive(motBValue);

  delay(10); // Adjust as needed for update rate

}

// Transmitter setup function, this is called in the main setup() and replaces the setup() function
void transmitterSetup() {
  analogSetAttenuation(ADC_11db); // Full 0-3.3V range for ADC
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

// Transmitter loop function, this is called in the main loop() and replaces the loop() function for the transmitter
void transmitterLoop() {
  // Get joystick values and adjust for deadzone
  int throttleMapped = readAxis(JOY_LEFT_Y);
  int turnMapped     = -readAxis(JOY_RIGHT_X);

  // Differential drive mixing
  int leftMotor  = throttleMapped + turnMapped;
  int rightMotor = throttleMapped - turnMapped;

  // Constrain to -2048 to 2047
  leftMotor  = constrain(leftMotor,  -2048, 2047);
  rightMotor = constrain(rightMotor, -2048, 2047);

  // Debugging Output.
  if (debug){
    Serial.print("Throttle Mapped: ");
    Serial.print(throttleMapped);
    Serial.print(" | Turn Mapped: ");
    Serial.print(turnMapped);
    Serial.print(" | Left Motor: ");
    Serial.print(leftMotor);
    Serial.print(" | Right Motor: ");
    Serial.print(rightMotor);
    Serial.print(" | Send Status: ");
    Serial.println(success);
  }

  // Fill struct (motA = left, motB = right)
  struct_message outgoingData;
  outgoingData.motA = leftMotor;
  outgoingData.motB = rightMotor;

  // Send struct via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));

  delay(10); // Adjust as needed for update rate

}

// Adjust for Deadzone (12-bit ADC: 0-4095)
int readAxis(byte pin) {
  int v = analogRead(pin);
  const int ADC_MIN = 0;
  const int ADC_MAX = 4095;
  const int CENTER = 2048;
  const int DEADZONE = 170; // ~4% deadzone (4095 * 0.04)
  const int DEADZONE_LOW = CENTER - DEADZONE;
  const int DEADZONE_HIGH = CENTER + DEADZONE;
  // Map to -2047 to 2047 output
  if (v < DEADZONE_LOW)
    return map(v, ADC_MIN, DEADZONE_LOW, -2048, 0);
  if (v > DEADZONE_HIGH)
    return map(v, DEADZONE_HIGH, ADC_MAX, 0, 2048);
  return 0;
}