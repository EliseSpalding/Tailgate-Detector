//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Wire Library for SCCB communication
#include <Wire.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Addresses
#define OV7670_ADDR 0x42
#define LOX1_ADDRESS 0x29
#define LOX2_ADDRESS 0x2A

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Pin definitions
#define VSYNC_PIN 4
#define HREF_PIN 5
#define PCLK_PIN 6
#define XCLK_PIN 3
#define rxPin 7 // Teensy pin 7 <--> HC-05 Tx
#define txPin 8 // Teensy pin 8 <--> HC-05 Rx
#define D7_PIN 20
#define D6_PIN 23
#define D5_PIN 22
#define D4_PIN 16
#define D3_PIN 17
#define D2_PIN 15
#define D1_PIN 14
#define D0_PIN 0

#define RESET 21

//TOF SENSOR
#define IRQ_LOX2 9
#define IRQ_LOX1 10
#define SHT_LOX2 11
#define SHT_LOX1 12
#include "Adafruit_VL53L1X.h"

// Sensor objects
Adafruit_VL53L1X lox1(SHT_LOX1, IRQ_LOX1);
Adafruit_VL53L1X lox2(SHT_LOX2, IRQ_LOX2);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// Variable definitions
volatile bool frameAvailable = false;
volatile bool frameCapture = false;


// Format settings
const int windoRow = 120; // upto 120
const int windoCol = 160; // upto 160
const int startRow = (120 - windoRow) / 2;
const int startCol = (160 - windoCol) / 2;

byte imgF[windoRow * windoCol];

// People counter variables
volatile int peopleCount = 0;
volatile int entryCount = 0;
volatile int exitCount = 0;

// Sensor state tracking
bool sensor1Triggered = false;
bool sensor2Triggered = false;
unsigned long lastTriggerTime = 0;
const unsigned long debounceTime = 300; // ms

// Detection parameters
const int DETECTION_THRESHOLD = 500; // mm (distance to trigger)
const int DOORWAY_WIDTH = 2000;      // mm (approximate doorway width)

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// High-speed function to read the data registers
// The pins are wired to GPIO6, enabling simultaenous access of the data pins
// The wiring below is with a Teensy 4.0
inline byte readPixel() {
  uint32_t gpioState = GPIO6_PSR;
  byte pixD = 0;
  pixD |= ((gpioState >> 3) & 0x01) << 0;   // Pin 0
  pixD |= ((gpioState >> 18) & 0x01) << 1;  // Pin 14
  pixD |= ((gpioState >> 19) & 0x01) << 2;  // Pin 15
  pixD |= ((gpioState >> 22) & 0x01) << 3;  // Pin 17
  pixD |= ((gpioState >> 23) & 0x01) << 4;  // Pin 16
  pixD |= ((gpioState >> 24) & 0x01) << 5;  // Pin 22
  pixD |= ((gpioState >> 25) & 0x01) << 6;  // Pin 23
  pixD |= ((gpioState >> 26) & 0x01) << 7;  // Pin 20
  return pixD;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// The function to write the SCCB settings of the camera
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(OV7670_ADDR >> 1);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void setupCamera() {
  writeReg(0x12, 0x80);  // Reset to default values
  delay(100);

  /*
    writeReg(0x12, 0x80); // Reset to default values
    writeReg(0x11, 0x80); // Set some reserved bit to 1!
    writeReg(0x3B, 0x0A); // Banding filter value uses BD50ST 0x9D as value. some reserved stuff + exposure timing can be less than limit on strong light
    writeReg(0x3A, 0x04); // output sequense elesection. Doc too lmited to be sure
    writeReg(0x3A, 0x04);
    writeReg(0x12, 0x04); // Output format: rgb
    writeReg(0x8C, 0x00); // Disable RGB444
    writeReg(0x40, 0xD0); // Set RGB565
    writeReg(0x11, 0x01); // CLKRC: Prescaler to adjust frame rate
    writeReg(0x17, 0x16); // Y window start msb (3-11) I think
    writeReg(0x18, 0x04); // Y window end msb (3-11)
    writeReg(0x32, 0x24); // Y window lsb end= 100b start= 100b
    writeReg(0x19, 0x02); // X window start msb (2-10) I think
    writeReg(0x1A, 0x7A); // X window end msb (2-10) I think
    writeReg(0x03, 0x0A); // X window lsb (10 and 10)
    writeReg(0x15, 0x02); // VSync negative
    writeReg(0x0C, 0x04); // DCW enable?
    writeReg(0x3E, 0x1A); // Divide by 4
    writeReg(0x1E, 0x27); // mirror image black sun enabled and more reserved!
    writeReg(0x72, 0x22); // Downsample by 4
    writeReg(0x73, 0xF2); // Divide by 4
    writeReg(0x4F, 0x80); // matrix coef
    writeReg(0x50, 0x80);
    writeReg(0x51, 0x00);
    writeReg(0x52, 0x22);
    writeReg(0x53, 0x5E);
    writeReg(0x54, 0x80);
    writeReg(0x56, 0x40); // contracts
    writeReg(0x58, 0x9E); // matrix
    writeReg(0x59, 0x88); // AWB
    writeReg(0x5A, 0x88);
    writeReg(0x5B, 0x44);
    writeReg(0x5C, 0x67);
    writeReg(0x5D, 0x49);
    writeReg(0x5E, 0x0E);
    writeReg(0x69, 0x00); // gain per channel
    writeReg(0x6A, 0x40); // more gain
    writeReg(0x6B, 0x0A); // pll reserved stuff!
    writeReg(0x6C, 0x0A); // AWB
    writeReg(0x6D, 0x55);
    writeReg(0x6E, 0x11);
    writeReg(0x6F, 0x9F);
    writeReg(0xB0, 0x84); // reserved!
    writeReg(0xFF, 0xFF);  // End marker
  */
  writeReg(0x6B, 0x80);
  writeReg(0x11, 0x00);

  writeReg(0x3B, 0x0A);
  writeReg(0x3A, 0x04);
  writeReg(0x12, 0x04);  // Output format: rgb
  writeReg(0x8C, 0x00);  // Disable RGB444
  writeReg(0x40, 0xD0);  // Set RGB565

  writeReg(0x17, 0x16);
  writeReg(0x18, 0x04);
  writeReg(0x32, 0x24);
  writeReg(0x19, 0x02);
  writeReg(0x1A, 0x7A);
  writeReg(0x03, 0x0A);
  writeReg(0x15, 0x02);
  writeReg(0x0C, 0x04);
  writeReg(0x3E, 0x1A);  // Divide by 4
  //writeReg(0x1E, 0x27);
  writeReg(0x72, 0x22);  // Downsample by 4
  writeReg(0x73, 0xF2);  // Divide by 4

  writeReg(0x4F, 0x80);
  writeReg(0x50, 0x80);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x22);
  writeReg(0x53, 0x5E);
  writeReg(0x54, 0x80);
  writeReg(0x56, 0x40);
  writeReg(0x58, 0x9E);
  writeReg(0x59, 0x88);
  writeReg(0x5A, 0x88);
  writeReg(0x5B, 0x44);
  writeReg(0x5C, 0x67);
  writeReg(0x5D, 0x49);
  writeReg(0x5E, 0x0E);
  writeReg(0x69, 0x00);
  writeReg(0x6A, 0x40);

  writeReg(0x6C, 0x0A);
  writeReg(0x6D, 0x55);
  writeReg(0x6E, 0x11);
  writeReg(0x6F, 0x9F);
  writeReg(0xB0, 0x84);
  writeReg(0x55, 0x00);  // Brightness
  writeReg(0x56, 0x40);  // Contrast
  writeReg(0xFF, 0xFF);  // End marker
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
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

  // Pin configurations
  pinMode(PCLK_PIN, INPUT);
  pinMode(VSYNC_PIN, INPUT);
  pinMode(HREF_PIN, INPUT);

  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  pinMode(RESET, OUTPUT);

  //////////////////////////////////////////////////////////////////////////////////////////////

  // Enabling the camera reset
  digitalWrite(RESET, LOW);
  delay(500);

  // The external clock supplied to the OV7670
  // should support upto 48MHz, based on the wiring
  // advised upto 8MHz on breadboards
  analogWriteFrequency(XCLK_PIN, 6000000);
  analogWrite(XCLK_PIN, 128);  // 50% duty cycle
  delay(100);

  /*Serial.println(F("Initializing people counter..."));
  
  initializeSensors();
  startRanging();
  
  Serial.println(F("Person-detection system ready"));
  printStatus();
  */
  // Releasing reset on the camera
  digitalWrite(RESET, HIGH);
  delay(100);

  //////////////////////////////////////////////////////////////////////////////////////////////

  setupCamera();

  //////////////////////////////////////////////////////////////////////////////////////////////

  // Interrupt to indicate the start of a new frame
  attachInterrupt(digitalPinToInterrupt(VSYNC_PIN), doThis, RISING);
  delay(100);

  //////////////////////////////////////////////////////////////////////////////////////////////

  // showTime.exe
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
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
  lox1.setTimingBudget(50);
  lox2.setTimingBudget(50);
  
  // Start ranging
  if (!lox1.startRanging() || !lox2.startRanging()) {
    Serial.println(F("Ranging failed"));
    while(1);
  }
}

////////////////////////////////////////////////////////////////

void loop() {
  
  //////////////////////////////////////////////////////////////////////////////////////////////

  // the marker that indicates the start of a new frame
  // and enters capture mode if currently nothing is being captured

  if (!frameAvailable || frameCapture) return;
  frameCapture = true;
  int i = 0;

  // RGB image buffer
  uint16_t img[19200];

  // clock start for FPS calculations
  unsigned long time0 = micros();

  //////////////////////////////////////////////////////////////////////////////////////////////

  // The main working principle is that the VSYNC pind goes HIGH
  // at the start of a new frame, HREF goes HIGH in the start of a new Line
  // and PCLK goes HIGH when D0-D7 are valid and hold the pixel data
  // Here, when capturing in RGB565, each pixel is 2-Bytes long,
  // Hence each pixel takes two PCLK cycles

  while (digitalReadFast(VSYNC_PIN)) {
    while (!digitalReadFast(HREF_PIN)) {
      if (!digitalReadFast(VSYNC_PIN)) break;
    }

    while (digitalReadFast(HREF_PIN))
    {
      if (!digitalReadFast(VSYNC_PIN) || i >= 19200) break;

      while (!digitalReadFast(PCLK_PIN));

      img[i] = (readPixel()) << 8;
      while (digitalReadFast(PCLK_PIN));

      while (!digitalReadFast(PCLK_PIN));

      img[i++] |= readPixel();
      while (digitalReadFast(PCLK_PIN));

    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////

  unsigned long time1 = micros();

  unsigned long time = time1 - time0;
  float fps = 1000000.0f / (float)(time); // FPS calculated

  // SoftWare-Subscaling paraeters
  int k = 0;
  int startRow = (120 - windoRow) / 2;
  int startCol = (160 - windoCol) / 2;

  // this function caters to further software downscaling if required
  for (int row  = startRow; row < (startRow + windoRow); row++) {
    for (int col = startCol; col < (startCol + windoCol); col++) {
      int index = (row * 160 + col);
      int r = ((img[index] >> 11) & 0x1F) * 255 / 31;
      int g = ((img[index] >> 5) & 0x3F) * 255 / 63;
      int b = (img[index] & 0x1F) * 255 / 31;
      int gray = (int)(0.299f * r + 0.587f * g + 0.114f * b);
      imgF[k++] = (uint8_t)(constrain(gray, 0, 255));
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////

  // Print the new frme marker
  Serial.write(0xAA);
  Serial.write(0x00);
  Serial.write(0xBB);
  //Serial2.write(0xFF);
  //Serial.write(0xFF);

  /* Either Print the FPS or end out the image data */
  //int windoSize = windoRow * windoCol;
  Serial.write(imgF, windoRow * windoCol);
  //sendFrame(imgF);
  
  // Send debug text via Bluetooth
  /*static int frameCounter = 0;
  String debugMsg = "Frame #" + String(frameCounter++) + " sent! ";
  debugMsg += "Time: " + String(millis()) + "ms ";
  debugMsg += "FPS: " + String(fps, 1) + "\n";
  Serial2.print(debugMsg);*/

  // Reset to default
  frameAvailable = false;
  frameCapture = false;

  // Add after frame transmission in loop():
  float distance_value = 4.20;
  Serial.print("DIST:");
  Serial.println(distance_value);
  // Read sensor 1 if data is ready
  /*if (lox1.dataReady()) {
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
  }*/
  //delay(2000);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>//

// THe interrupt function
inline void doThis() {
  frameAvailable = true;
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
  Serial.print(F("Current Occupancy: "));
  Serial.println(peopleCount);
  Serial.print(F("Total Entries: "));
  Serial.println(entryCount);
  Serial.print(F("Total Exits: "));
  Serial.println(exitCount);
  Serial2.print(F("Current Occupancy: "));
  Serial2.println(peopleCount);
  Serial2.print(F("Total Entries: "));
  Serial2.println(entryCount);
  Serial2.print(F("Total Exits: "));
  Serial2.println(exitCount);
}

void sendFrame(uint8_t* frame) {
  const int CHUNK_SIZE = 128;
  const int FRAME_SIZE = windoRow * windoCol;
  Serial2.write(0xAA);
  Serial2.write(0xBB);
  for (int i = 0; i < FRAME_SIZE; i+=CHUNK_SIZE) {
    int bytesToSend = min(CHUNK_SIZE, (FRAME_SIZE)-i);
    Serial2.write(frame + i, bytesToSend);
    delayMicroseconds(500);
  }
  Serial2.write(0xCC);
  Serial2.write(0xDD);
  
  byte checksum = 0;
  for (int i = 0; i < FRAME_SIZE; i++) {
    checksum ^= frame[i];
  }
  Serial2.write(checksum);
  
  // Visual indicator
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
