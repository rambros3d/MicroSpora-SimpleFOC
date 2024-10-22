/*
  MicroSpora - MT6701 Sensor Example
  
  This code initializes and reads angle and velocity data from the
  MT6701 magnetic encoder. SPI2 is used since SPI1  will be
  utilized for the DRV8316 driver.

  Pin Mapping:
  - ENC_NC  -> PA7  ; Not connected (dummy MOSI)
  - ENC_SDO -> PA6  ; configured as MISO
  - ENC_CLK -> PA5
  - ENC_CS  -> PA4

  Serial output format:
  - Angle in radians and degrees
  - Velocity in rad/s and RPM
*/

#include <arduino_pin_def.h>

#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

//  Use SPI_2 for sensor since default SPI will be used by DRV8316C
SPIClass SPI_2(ENC_NC, ENC_SDO, ENC_CLK);
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(ENC_CS);

void setup() {

  Serial.begin(250000);
  Serial.println("MT6701 Sensor ready");

  sensor.init(&SPI_2);  //  Initialize sensor on SPI_2 bus

  _delay(1000);
}


void loop() {
  sensor.update();

  float radians = sensor.getAngle();
  float degrees = radians * (180.0 / PI);

  Serial.print("Angle: rad = ");
  Serial.print(radians, 4);

  Serial.print("\tdeg = ");
  Serial.print(degrees, 4);

  float velocity = sensor.getVelocity();
  float rpm = velocity * (60.0 / (2.0 * PI));

  Serial.print("\t  Velocity: rad/s = ");
  Serial.print(velocity,1);

  Serial.print("\t  RPM = ");
  Serial.println(rpm,2);

  _delay(100);
}