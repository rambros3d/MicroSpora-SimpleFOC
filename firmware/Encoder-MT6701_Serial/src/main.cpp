#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// MT6701 pin mapping
// ENC_NC - PA7
// ENC_SDO - PA6
// ENC_CLK - PA5
// ENC_CS - PA4

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
  Serial.print(degrees, 2);

  float velocity = sensor.getVelocity();
  float rpm = velocity * (60.0 / (2.0 * PI));

  Serial.print("\t  Velocity: rad/s = ");
  Serial.print(velocity);

  Serial.print("\t  RPM = ");
  Serial.println(rpm);

  _delay(100);
}