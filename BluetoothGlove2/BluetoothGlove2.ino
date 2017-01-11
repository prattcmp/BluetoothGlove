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
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"

#include "BluefruitConfig.h"

// Play with these (INTENSITY_SCALE has minimum of 3)
#define INTENSITY_SCALE 70
#define INTENSITY_MIN 25
#define INTENSITY_MAX 100

// Pin Mapping for IST Glove: MOT0 - 3 MOT1 - 8 MOT2 - 12 MOT3 - 23
int motorpins[6] = {
  0,12,10,6,5,3 }; // motor driver pins 0 through 5, ideally spaced 60 degrees apart 

/* Pin Mapping for Dev Glove: MOT0 - 3 MOT1 - 8 MOT2 - 12 MOT4 - 19
int motorpins[6] = {
    3,8,12,19,23,14 };
*/

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEGatt gatt(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

uint8_t motors = 0;
uint8_t intensity = 0;
word duration = 0;

void setup(void)
{
  while (!Serial); // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  /* Disable command echo from Bluefruit */
  ble.verbose(false);
  ble.echo(false);
}

// Write to every motor
void digitalWriteAll(uint8_t state)
{
  for (int m = 0; m < 5; m++)
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
  
  motors = constrain(strtoul(motorStr, NULL, 16), 0, 7);
  intensity = constrain(strtoul(intensityStr, NULL, 16), 0, 100);
  intensity = map(intensity, 0, 100, INTENSITY_MIN, INTENSITY_MAX);
  duration = constrain(strtoul(durationStr, NULL, 16), 1, 100);
  duration *= 10 * 1000;
}

void checkCharacteristic() {
  // Read the buffer
  ble.println("AT+GATTCHAR=1");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }

  // Parse our data
  parsePacket(String(ble.buffer));
}

void runMotors() {
  // Reset the characteristic
  String command = "AT+GATTCHAR=1,0x000000";
  ble.println(command);
  
  unsigned long start_time = micros();
  
  // Calculate length of the duty cycle
  word cycle_time = INTENSITY_SCALE * intensity;

  // Make sure every motor is off when running a new motor
  digitalWriteAll(LOW);

  // Keep running until the full duration has passed
  while(start_time + duration > micros()) { 
    // Pulse the motors based on our duty cycle
    for(int i = 1;i <= 6;i++) {
      unsigned int live = (motors >> i) & 1;
      if(live == 1) {
        digitalWrite(i, HIGH);
      }
    }

    delayMicroseconds(cycle_time);

    // If there's another byte waiting, read it right away
    checkCharacteristic();
    if(motors != 0) {
      break;
    }

    for(int i = 1;i <= 6;i++) {
      unsigned int live = (motors >> i) & 1;
      if(live == 1) {
        digitalWrite(i, LOW);
      }
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

void loop(void)
{
  // Check our motor characteristic to see if we need to run a motor
  checkCharacteristic();

  if (motors != 0) {
    runMotors();
  }
  
  // Make sure every motor is off
  digitalWriteAll(LOW);
}
