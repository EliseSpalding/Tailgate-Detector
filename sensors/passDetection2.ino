//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Wire Library for SCCB communication
#include <Wire.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

#define LOX1_ADDRESS 0x29
#define LOX2_ADDRESS 0x2A

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Pin definitions
#define rxPin 7 // Teensy pin 7 <--> HC-05 Tx
#define txPin 8 // Teensy pin 8 <--> HC-05 Rx

//TOF SENSOR
#define IRQ_LOX2 0
#define IRQ_LOX1 1
#define SHT_LOX2 2
#define SHT_LOX1 3
#include "Adafruit_VL53L1X.h"

// Sensor objects
Adafruit_VL53L1X lox1(SHT_LOX1, IRQ_LOX1);
Adafruit_VL53L1X lox2(SHT_LOX2, IRQ_LOX2);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Variable definitions

volatile int peopleCount = 0;
volatile int entryCount = 0;
volatile int exitCount = 0;

// Sensor state tracking
bool sensor1Triggered = false;
bool sensor2Triggered = false;
unsigned long lastTriggerTime = 0;
const unsigned long debounceTime = 100; // ms

// Detection parameters
const int DETECTION_THRESHOLD = 500; // mm (distance to trigger)
const int DOORWAY_WIDTH = 2000;      // mm (approximate doorway width)

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(500);
  Wire.begin();

  //////////////////////////////////////////////////////////////////////////////////////////////

  // Initialize TOF shutdown pins FIRST
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  //////////////////////////////////////////////////////////////////////////////////////////////

  Serial.println(F("Initializing people counter..."));
  
  initializeSensors();
  startRanging();
  
  Serial.println(F("Person-detection system ready"));
  printStatus();
}
////////////////////////////////////////////////////////////////
// Sensor setup functions

void initializeSensors() {
  // Reset both sensors
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  
  // Power on both sensors
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // Deactivate LOX2
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  Wire.begin();
  
  // Initialize LOX1 at default address (0x29)
  if(!lox1.begin(0x29, &Wire)) {
    Serial.println(F("Failed to boot first VL53L1X"));
    while(1);
  }

  // Power off LOX1
  digitalWrite(SHT_LOX1, LOW);
  delay(50);

  // Power on LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);
  
  // Initialize LOX2 at 0x2A
  if(!lox2.begin(0x2A, &Wire)) {
    Serial.println(F("Failed to boot second VL53L1X"));
    while(1);
  }

  // Reinitialize LOX1 at 0x29
  if(!lox1.begin(0x29, &Wire)) {
    Serial.println(F("Failed to reinitialize first VL53L1X"));
    while(1);
  }

  // Power on LOX1
  digitalWrite(SHT_LOX1, HIGH);
  delay(50);
}

void startRanging() {
  // Set timing budgets (15-500ms valid)
  lox1.setTimingBudget(15);
  lox2.setTimingBudget(15);
  
  // Start ranging
  if (!lox1.startRanging() || !lox2.startRanging()) {
    Serial.println(F("Ranging failed"));
    while(1);
  }
}

////////////////////////////////////////////////////////////////

void loop() {
  static int16_t distance1, distance2;
  
  //////////////////////////////////////////////////////////////////////////////////////////////

  // Read sensor 1 if data is ready
  if (lox1.dataReady()) {
    distance1 = lox1.distance();
    if (distance1 != -1) {
      checkSensorTrigger(1, distance1);
    }
    lox1.clearInterrupt();
  }

  // Read sensor 2 if data is ready
  if (lox2.dataReady()) {
    distance2 = lox2.distance();
    if (distance2 != -1) {
      checkSensorTrigger(2, distance2);
    }
    lox2.clearInterrupt();
  }

  // Signal the total number of entries, exits and net change at a constant rate
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink heartbeat LED
    printStatus();
    lastUpdate = millis();
  }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void checkSensorTrigger(int sensorNum, int distance) {
  // Ignore triggers during debounce period
  if (millis() - lastTriggerTime < debounceTime) {
    return;
  }

  // Check if object is within detection range
  if (distance < DETECTION_THRESHOLD && distance > 0) {
    lastTriggerTime = millis();
    
    if (sensorNum == 1) {
      if (!sensor1Triggered) {
        sensor1Triggered = true;
        
        // If sensor 2 was already triggered, it's an exit
        if (sensor2Triggered) {
          handleExit();
        }
      }
    } 
    else if (sensorNum == 2) {
      if (!sensor2Triggered) {
        sensor2Triggered = true;
        
        // If sensor 1 was already triggered, it's an entry
        if (sensor1Triggered) {
          handleEntry();
        }
      }
    }
  } 
  else {
    // Object moved out of detection range
    if (sensorNum == 1 && sensor1Triggered) {
      sensor1Triggered = false;
    } 
    else if (sensorNum == 2 && sensor2Triggered) {
      sensor2Triggered = false;
    }
  }
}

void handleEntry() {
  peopleCount++;
  entryCount++;
  
  // Reset both sensors
  sensor1Triggered = false;
  sensor2Triggered = false;
}

void handleExit() {
  if (peopleCount > 0) {
    peopleCount--;
    exitCount++;
  }
  
  // Reset both sensors
  sensor1Triggered = false;
  sensor2Triggered = false;
}

void printStatus() {
  Serial.printf("People: %d Entries: %d Exits: %d\n", peopleCount, entryCount, exitCount);
  Serial1.printf("%d,%d,%d\n", peopleCount, entryCount, exitCount);
}
