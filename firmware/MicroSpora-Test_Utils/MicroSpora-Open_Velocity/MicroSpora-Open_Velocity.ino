/*
  MicroSpora - Open Loop Velocity Control Example
  
  Control a BLDC motor using the DRV8316 driver in open loop velocity mode.
  The target velocity and voltage limit can be set via serial commands.
  The code initializes the motor and driver, prints monitors DRV8316 status,
  and moves the motor according to set velocity.

  Key Features:
  - Uses default SPI bus for communication with the DRV8316 driver.
  - Prints DRV8316 status including fault, overcurrent, and temperature info.
  - Commands for setting target velocity and voltage limit via serial input.
  
  Driver Pin Mapping:
  - PHA_H (High-side PWM for phase A): PA10
  - PHA_L (Low-side PWM for phase A): PB15
  - PHB_H (High-side PWM for phase B): PA9
  - PHB_L (Low-side PWM for phase B): PB14
  - PHC_H (High-side PWM for phase C): PA8
  - PHC_L (Low-side PWM for phase C): PB13

  - DRV_CS: PC4     ; SPI Chip Select (SS)

  Serial output format:
  - DRV8316 status including faults and warnings
  - Target velocity updates
*/

#include "Arduino.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"

#include <arduino_pin_def.h>

BLDCMotor motor = BLDCMotor(4);

DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

//target variable
float target_velocity = 2.0;
float voltage_limit = 1.0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

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

  // use monitoring with serial
  Serial.begin(250000);
  //SimpleFOCDebug::enable(&Serial);

  Serial.println("Initializing...");

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

  // link the motor and the driver
  motor.linkDriver(&driver);
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = voltage_limit / 2.0;
  motor.velocity_limit = 20.0;
  // init motor hardware
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }
  motor.enable();
  Serial.println("Init complete...");

  _delay(100);
  printDRV8316Status();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("MicroSpora- OpenLoopVelocity example");
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(200);
}

void loop() {
  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motor.move(target_velocity);
  // user communication
  command.run();
}
