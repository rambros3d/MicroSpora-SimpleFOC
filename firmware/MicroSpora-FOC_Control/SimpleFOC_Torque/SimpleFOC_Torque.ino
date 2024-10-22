/*
  MicroSpora - Closed loop Torque Control Example
*/

#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

#include <arduino_pin_def.h>

BLDCMotor motor = BLDCMotor(4);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

SPIClass SPI_2(ENC_NC, ENC_SDO, ENC_CLK);
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(ENC_CS);


LowsideCurrentSense current_sense = LowsideCurrentSense(600.0, ISENSA, ISENSB, ISENSB);

// current set point variable
float target_current = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_current, cmd);
}

float voltage_limit = 5;  // driver voltage limit

void setup() {
  // use monitoring with serial
  Serial.begin(250000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Initializing...");

  //motor.sensor_direction = Direction::CCW;
  //motor.zero_electric_angle = 3.6938;

  sensor.init(&SPI_2);        //  Initialize sensor on SPI_2 bus
  motor.linkSensor(&sensor);  // link the motor to the sensor


  current_sense.init();
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);

  // driver config
  driver.voltage_power_supply = 8.0;
  driver.voltage_limit = voltage_limit;
  driver.init();
  driver.setSlew(Slew_200Vus);
  driver.setCurrentSenseGain(Gain_0V375);
  driver.setPWMMode(PWM6_CurrentLimit_Mode);
  driver.setBuckVoltage(VB_5V);

  motor.linkDriver(&driver);                          // link the motor and the driver
  motor.foc_modulation = FOCModulationType::SinePWM;  // set FOC modulation type
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::foc_current;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // foc currnet control parameters (stm/esp/due/teensy)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 1000;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 1000;
  motor.LPF_current_q.Tf = 0.002f;  // 1ms default
  motor.LPF_current_d.Tf = 0.002f;  // 1ms default

  motor.useMonitoring(Serial);  // comment out if not needed

  // init motor hardware
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  motor.initFOC();
  Serial.println("Init complete...");

  // add target command T
  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));

  digitalWrite(LED_BUILTIN, LOW);
  _delay(200);
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  // velocity, position or torque (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_current);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}
