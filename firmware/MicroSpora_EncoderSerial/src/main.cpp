#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

// SPISettings myMT6701SSISettings(100000, MT6701_BITORDER, SPI_MODE1);  // try SPI mode 1 and slower speed
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(ENC_CS);
// , myMT6701SSISettings);


void setup() {
  SPI.setMOSI(LED_BUILTIN);
  SPI.setMISO(ENC_SDO);
  SPI.setSCLK(ENC_CLK);
  SPI.begin();

  Serial.begin(250000);

  sensor.init();
  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  sensor.update();
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
  _delay(100);
}