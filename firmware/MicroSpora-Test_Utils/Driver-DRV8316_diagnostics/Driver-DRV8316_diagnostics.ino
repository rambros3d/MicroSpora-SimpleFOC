
#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"

BLDCMotor motor = BLDCMotor(14);

DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, true);

void printDRV8316Status() {
  DRV8316Status status = driver.getStatus();
  Serial.println("DRV8316 Status:");
  Serial.print("Fault: ");
  Serial.println(status.isFault());
  Serial.print("Buck Error: ");
  Serial.print(status.isBuckError());
  Serial.print("  Undervoltage: ");
  Serial.print(status.isBuckUnderVoltage());
  Serial.print("  OverCurrent: ");
  Serial.println(status.isBuckOverCurrent());
  Serial.print("Charge Pump UnderVoltage: ");
  Serial.println(status.isChargePumpUnderVoltage());
  Serial.print("OTP Error: ");
  Serial.println(status.isOneTimeProgrammingError());
  Serial.print("OverCurrent: ");
  Serial.print(status.isOverCurrent());
  Serial.print("  Ah: ");
  Serial.print(status.isOverCurrent_Ah());
  Serial.print("  Al: ");
  Serial.print(status.isOverCurrent_Al());
  Serial.print("  Bh: ");
  Serial.print(status.isOverCurrent_Bh());
  Serial.print("  Bl: ");
  Serial.print(status.isOverCurrent_Bl());
  Serial.print("  Ch: ");
  Serial.print(status.isOverCurrent_Ch());
  Serial.print("  Cl: ");
  Serial.println(status.isOverCurrent_Cl());
  Serial.print("OverTemperature: ");
  Serial.print(status.isOverTemperature());
  Serial.print("  Shutdown: ");
  Serial.print(status.isOverTemperatureShutdown());
  Serial.print("  Warning: ");
  Serial.println(status.isOverTemperatureWarning());
  Serial.print("OverVoltage: ");
  Serial.println(status.isOverVoltage());
  Serial.print("PowerOnReset: ");
  Serial.println(status.isPowerOnReset());
  Serial.print("SPI Error: ");
  Serial.print(status.isSPIError());
  Serial.print("  Address: ");
  Serial.print(status.isSPIAddressError());
  Serial.print("  Clock: ");
  Serial.print(status.isSPIClockFramingError());
  Serial.print("  Parity: ");
  Serial.println(status.isSPIParityError());
  if (status.isFault())
    driver.clearFault();
  delayMicroseconds(1);  // ensure 400ns delay
  DRV8316_PWMMode val = driver.getPWMMode();
  Serial.print("PWM Mode: ");
  Serial.println(val);
  delayMicroseconds(1);  // ensure 400ns delay
  bool lock = driver.isRegistersLocked();
  Serial.print("Lock: ");
  Serial.println(lock);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(100);
  Serial.begin(250000);
  Serial.println("Initializing...");

  driver.voltage_power_supply = 8.0;
  driver.voltage_limit = 0.5;
  driver.pwm_frequency = 2000;
  driver.init();
  driver.setSlew(Slew_200Vus);
  driver.setBuckVoltage(VB_5V);
  driver.enable();

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 0.5;
  motor.velocity_limit = 20.0;
  motor.init();
  motor.enable();
  Serial.println("Init complete...");

  delay(100);
  printDRV8316Status();

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Done");
  SimpleFOCDebug::enable(&Serial);
}


// velocity set point variable
float target_velocity = 1.0;


void loop() {
  motor.move(target_velocity);
}
