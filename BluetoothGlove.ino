/*
 The Bluetooth LE Glove Listener is part of the VCoS NSF project, and is intended to listen for updates to a BLE characteristic. 
 This characteristic contains a byte array that indicates which motor(s) should vibrate to provide haptic feedback to the user.
 The current interface is as follows:
 [Motor (1 - 6 or 7). Each number corresponds to a motor, or 7 for all motors)]
 [Intensity (0..100 percent)]
 [Duration (1..100 percent of a second)]
 These values are sent in a byte array in the following format:
 [motor, intensity, duration]
 
 Every time the characteristic is updated, it will overwrite the duration of the previous run, terminating it immediately.
 */

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLEGatt.h"

#include "BluefruitConfig.h"

// Play with these (INTENSITY_SCALE has minimum of 3)
#define INTENSITY_SCALE 70
#define INTENSITY_MIN 25
#define INTENSITY_MAX 100

// Pin Mapping for IST Glove: MOT0 - 3 MOT1 - 8 MOT2 - 12 MOT3 - 23
int motorpins[7] = {
  0,13,12,10,6,5,3 }; // motor driver pins 0 through 5, ideally spaced 60 degrees apart 

/* Pin Mapping for Dev Glove: MOT0 - 3 MOT1 - 8 MOT2 - 12 MOT4 - 19
int motorpins[6] = {
    3,8,12,19,23,14 };
*/

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEGatt gatt(ble);

<<<<<<< HEAD
=======
// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

>>>>>>> origin/master
int32_t gattServiceId;
int32_t gattCharId;
uint8_t motor = 0;
uint8_t intensity = 0;
long duration = 0;

void setup(void) {
  Serial.begin(115200);

<<<<<<< HEAD
  for(int i = 0; i < 7; i++) {
=======
  for(int i = 0; i < 5; i++) {
>>>>>>> origin/master
    pinMode(motorpins[i], OUTPUT);
  }
  digitalWriteAll(LOW);
  
<<<<<<< HEAD
=======
  pinMode(LED_PIN, OUTPUT);
  
>>>>>>> origin/master
  if (!ble.begin(VERBOSE_MODE))
  {
    Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
  }

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  ble.factoryReset();
  
  ble.sendCommandCheckOK(F("AT+GAPDEVNAME=HapticGloveB"));
  ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=15-DB-5D-20-50-D4-43-70-A4-39-75-4E-71-82-CB-54"), &gattServiceId);  
  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=15-DB-5D-21-50-D4-43-70-A4-39-75-4E-71-82-CB-54,PROPERTIES=0x08,MIN_LEN=1, MAX_LEN=3, VALUE=0x000000"), &gattCharId);
  ble.reset();

  /* Disable command echo from Bluefruit */
  ble.verbose(false);
<<<<<<< HEAD
  ble.echo(false); 
=======
  ble.echo(false);
    
>>>>>>> origin/master
}

// Write to every motor
void digitalWriteAll(uint8_t state) {
<<<<<<< HEAD
  for (int m = 1; m < 7; m++)
=======
  for (int m = 1; m < 6; m++)
>>>>>>> origin/master
  {
    digitalWrite(motorpins[m], state);
  }
}

void parsePacket(String packet) {
  String motorBuffer = packet.substring(0, 2);
  String intensityBuffer = packet.substring(3, 5);
  String durationBuffer = packet.substring(6); 

  const char* motorStr = motorBuffer.c_str();
  const char* intensityStr = intensityBuffer.c_str();
  const char* durationStr = durationBuffer.c_str();
  
  motor = constrain(strtoul(motorStr, NULL, 16), 0, 7);
  intensity = constrain(strtoul(intensityStr, NULL, 16), 0, 100);
  intensity = map(intensity, 0, 100, INTENSITY_MIN, INTENSITY_MAX);
  duration = constrain(strtoul(durationStr, NULL, 16), 1, 100);
  duration *= 10 * 1000;
}

bool checkCharacteristic() {
  // Read the buffer
  ble.println("AT+GATTCHAR=1");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }

  // Parse our data
  parsePacket(String(ble.buffer));

  if (motor != 0) {
    return true;
  }
}

void runMotor() {
  // Reset the characteristic
  String command = "AT+GATTCHAR=1,0x000000";
  ble.println(command);
  
  Serial.print("Running motor: "); Serial.println(motor);
  Serial.print("    Duration: "); Serial.println(duration);
  Serial.print("    Intensity: "); Serial.println(intensity);
  
  unsigned long start_time = micros();
  
  // Calculate length of the duty cycle
  word cycle_time = INTENSITY_SCALE * intensity;

  // Make sure every motor is off when running a new motor
  digitalWriteAll(LOW);
  
  
  // Keep running until the full duration has passed
  while(start_time + duration > micros()) { 
    // Pulse the motors based on our duty cycle
    if(motor == 7) {
      digitalWriteAll(HIGH);
    }
    else {
      digitalWrite(motorpins[motor], HIGH);
    }
    delayMicroseconds(cycle_time);

    // If there's another byte waiting, read it right away
    if(checkCharacteristic()) {
      break;
    }

    // Pulse the motors based on our duty cycle
    if(motor == 7) {
      digitalWriteAll(LOW);
    }
    else {
      digitalWrite(motorpins[motor], LOW);
    }

    // This algorithm breaks up the delays into chunks to keep checking
    // for the right time to stop the loop.
    // The shorter our delay, the less we have to break up the delays
    int iterations = map(intensity, INTENSITY_MIN, INTENSITY_MAX, 
                         INTENSITY_MIN, INTENSITY_MAX * 2);
    for(uint8_t i = 0; i < iterations; i++)
    {
      delayMicroseconds(((100 * INTENSITY_SCALE) - cycle_time) / iterations);
      
      if(start_time + duration < micros()) {
        break;
      }
    }

    // If the above breaks, just use this:
    // delayMicroseconds(((100 * INTENSITY_SCALE) - cycle_time));
  }
}

void loop(void) {
  motor = 0;
  intensity = 0;
  duration = 0;

  checkCharacteristic();
  // Check our motor characteristic to see if we need to run a motor
  if (motor != 0) {
    runMotor();
  }

  // Make sure every motor is off
  digitalWriteAll(LOW);
}
