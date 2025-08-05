/* 
 * Automatic Hand-Washing Machine with Motorized Tap 
 * Uses L293D for both soap dispenser and tap control 
 * 
 * Wiring: 
 *   IR Sensor: GPIO13 (Active-LOW) 
 *   Soap Motor: IN1=GPIO14, IN2=GPIO27 (L293D Channel 1) 
 *   Tap Motor:  IN3=GPIO26, IN4=GPIO25 (L293D Channel 2) 
 * 
 * States: 
 *   IDLE → SOAP_DISPENSE → TAP_OPEN → WATER_ON → TAP_CLOSE → WAIT_REMOVAL 
 */ 

#include <Arduino.h> 

// Pins 
const int IR_SENSOR_PIN = 13; 
const int SOAP_MOTOR_IN1 = 14; 
const int SOAP_MOTOR_IN2 = 27; 
const int TAP_MOTOR_IN3 = 26; 
const int TAP_MOTOR_IN4 = 25;
const int SOAP_MOTOR_EN12 = 19;
const int TAP_MOTOR_EN34 = 21;

// Timing (ms) 
const unsigned long SOAP_TIME = 500; 
const unsigned long TAP_OPEN_TIME = 2000; 
const unsigned long TAP_CLOSE_TIME = 2000; 
const unsigned long WATER_TIME = 20000; 
const unsigned long DEBOUNCE_DELAY = 50; 

// State Machine 
enum State { 
  STATE_IDLE, 
  STATE_SOAP_DISPENSE, 
  STATE_TAP_OPEN, 
  STATE_WATER_ON, 
  STATE_TAP_CLOSE, 
  STATE_WAIT_REMOVAL 
}; 
State currentState = STATE_IDLE; 

// Timers 
unsigned long stateStartTime = 0; 
int debouncedSensorValue = HIGH; 
int lastSensorValue = HIGH; 
unsigned long lastDebounceTime = 0; 

void setup() { 
  Serial.begin(115200); 
  
  // Initialize pins 
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP); 
  pinMode(SOAP_MOTOR_IN1, OUTPUT); 
  pinMode(SOAP_MOTOR_IN2, OUTPUT); 
  pinMode(TAP_MOTOR_IN3, OUTPUT); 
  pinMode(TAP_MOTOR_IN4, OUTPUT);
  pinMode(SOAP_MOTOR_EN12, OUTPUT);
  pinMode(TAP_MOTOR_EN34, OUTPUT);
  
  // Ensure motors are off 
  stopSoapMotor(); 
  stopTapMotor(); 
  
  Serial.println("System Ready. Tap control via motors."); 
} 

void loop() { 
  int sensorReading = digitalRead(IR_SENSOR_PIN); 
  debounceSensor(sensorReading); 
  
  switch(currentState) { 
    case STATE_IDLE: 
      handleIdle(); 
      break; 
    case STATE_SOAP_DISPENSE: 
      handleSoapDispense(); 
      break; 
    case STATE_TAP_OPEN: 
      handleTapOpen(); 
      break; 
    case STATE_WATER_ON: 
      handleWaterOn(); 
      break; 
    case STATE_TAP_CLOSE: 
      handleTapClose(); 
      break; 
    case STATE_WAIT_REMOVAL: 
      handleWaitRemoval(); 
      break; 
  } 
  delay(10); 
} 

// Debounce IR sensor 
void debounceSensor(int currentReading) { 
  if (currentReading != lastSensorValue) { 
    lastDebounceTime = millis(); 
  } 
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) { 
    debouncedSensorValue = currentReading; 
  } 
  lastSensorValue = currentReading; 
} 

bool isHandPresent() { 
  return (debouncedSensorValue == LOW); // Active-LOW 
} 

void stopSoapMotor() { 
  digitalWrite(SOAP_MOTOR_IN1, LOW); 
  digitalWrite(SOAP_MOTOR_IN2, LOW); 
} 

void dispenseSoap() { 
  digitalWrite(SOAP_MOTOR_IN1, HIGH); // Forward 
  digitalWrite(SOAP_MOTOR_IN2, LOW); 
} 

void stopTapMotor() { 
  digitalWrite(TAP_MOTOR_IN3, LOW); 
  digitalWrite(TAP_MOTOR_IN4, LOW); 
} 

void openTap() { 
  digitalWrite(TAP_MOTOR_IN3, HIGH); // Open direction 
  digitalWrite(TAP_MOTOR_IN4, LOW); 
} 

void closeTap() { 
  digitalWrite(TAP_MOTOR_IN3, LOW); 
  digitalWrite(TAP_MOTOR_IN4, HIGH); // Close direction 
} 

//--- State Handlers ---// 
void handleIdle() { 
  if (isHandPresent()) { 
    Serial.println("HAND DETECTED: Dispensing soap"); 
    currentState = STATE_SOAP_DISPENSE; 
    stateStartTime = millis(); 
    dispenseSoap(); 
  } 
} 

void handleSoapDispense() { 
  if (millis() - stateStartTime >= SOAP_TIME) { 
    stopSoapMotor(); 
    if (isHandPresent()) { 
      Serial.println("OPENING TAP"); 
      currentState = STATE_TAP_OPEN; 
      stateStartTime = millis(); 
      openTap(); 
    } else { 
      Serial.println("HAND REMOVED EARLY: Skipping water"); 
      currentState = STATE_WAIT_REMOVAL; 
    } 
  } 
} 

void handleTapOpen() { 
  if (millis() - stateStartTime >= TAP_OPEN_TIME) { 
    stopTapMotor(); 
    Serial.println("WATER FLOWING"); 
    currentState = STATE_WATER_ON; 
    stateStartTime = millis(); 
  } 
} 

void handleWaterOn() { 
  // Emergency stop if hand removed 
  if (!isHandPresent()) { 
    Serial.println("HAND REMOVED: Closing tap early"); 
    currentState = STATE_TAP_CLOSE; 
    stateStartTime = millis(); 
    closeTap(); 
  } 
  // Normal timeout 
  else if (millis() - stateStartTime >= WATER_TIME) { 
    Serial.println("WATER TIME COMPLETE: Closing tap"); 
    currentState = STATE_TAP_CLOSE; 
    stateStartTime = millis(); 
    closeTap(); 
  } 
} 

void handleTapClose() { 
  if (millis() - stateStartTime >= TAP_CLOSE_TIME) { 
    stopTapMotor(); 
    Serial.println("TAP CLOSED: Wait for hand removal");
    currentState = STATE_WAIT_REMOVAL; 
  } 
} 

void handleWaitRemoval() { 
  if (!isHandPresent()) { 
    Serial.println("HAND REMOVED: Returning to IDLE"); 
    currentState = STATE_IDLE; 
  } 
} 