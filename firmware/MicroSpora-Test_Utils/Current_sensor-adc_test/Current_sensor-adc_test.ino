/**
 * Testing example code for the Inline current sensing class
*/
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

//target variable
float target_velocity = 2.0;
float voltage_limit = 1.0;

// current sensor
// shunt resistor value
// gain value
// pins phase A,B, (C optional)
LowsideCurrentSense current_sense = LowsideCurrentSense(150.0, ISENS_A, ISENS_B, ISENS_C);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // use monitoring with serial
  Serial.begin(250000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  driver.voltage_power_supply = 8.0;
  driver.voltage_limit = voltage_limit;
  driver.init();
  driver.setSlew(Slew_200Vus);
  driver.setPWMMode(PWM6_CurrentLimit_Mode);
  driver.setBuckVoltage(VB_5V);
  driver.setOCPLevel(Curr_24A);
  driver.setOCPMode(AutoRetry_Fault);
  driver.setOCPRetryTime(Retry5ms);

  //current_sense.linkDriver(&driver);

  delay(1000);
  // initialise the current sensing
  if (!current_sense.init()) {
    Serial.println("Current sense init failed.");
    return;
  }

  // for SimpleFOCShield v2.01/v2.0.2
  current_sense.gain_b *= -1;

  Serial.println("Current sense ready.");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

  Serial.print(currents.a * 1000);  // milli Amps
  Serial.print("\t");
  Serial.print(currents.b * 1000);  // milli Amps
  Serial.print("\t");
  Serial.print(currents.c * 1000);  // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude * 1000);  // milli Amps
}
