/*
 * Velocity motion control example modified for MicroSpora
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
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

float target_velocity = 0;     // angle set point variable
float voltage_limit = 5;  // driver voltage limit

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

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

  // driver config
  driver.voltage_power_supply = 8.0;
  driver.voltage_limit = voltage_limit;
  driver.init();
  driver.setSlew(Slew_200Vus);
  //driver.setPWMMode(PWM6_CurrentLimit_Mode);
  driver.setBuckVoltage(VB_5V);

  motor.linkDriver(&driver);                          // link the motor and the driver
  motor.foc_modulation = FOCModulationType::SinePWM;  // set FOC modulation type
  motor.controller = MotionControlType::velocity;        // position control mode
  motor.voltage_limit = voltage_limit;                          // maximal voltage to be set to the motor

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  motor.LPF_velocity.Tf = 0.01f;  // velocity low pass filtering time constant, the lower the less filtered
  //motor.velocity_limit = 100;      // maximal velocity of the position control

  motor.useMonitoring(Serial);  // comment out if not needed

  // init motor hardware
  if (!motor.init()) {
    Serial.println("Motor init failed!");
    return;
  }

  motor.initFOC();
  Serial.println("Init complete...");

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

  digitalWrite(LED_BUILTIN, LOW);
  _delay(200);
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  // this function can be run at much lower frequency than loopFOC() function
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();

}
