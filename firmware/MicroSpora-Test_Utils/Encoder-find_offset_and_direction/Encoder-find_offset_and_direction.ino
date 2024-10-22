/*

  finds sensor offset and direction code modified for MicroSpora

 * Simple example intended to help users find the zero offset and natural direction of the sensor. 
 * 
 * These values can further be used to avoid motor and sensor alignment procedure. 
 * To use these values add them to the code:");
 *    motor.sensor_direction=Direction::CW; // or Direction::CCW
 *    motor.zero_electric_angle=1.2345;     // use the real value!
 * 
 * This will only work for abosolute value sensors - magnetic sensors. 
 * Bypassing the alignment procedure is not possible for the encoders and for the current implementation of the Hall sensors. 
 * library version 1.4.2.
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

float target_voltage = 4;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // use monitoring with serial
  Serial.begin(250000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  sensor.init(&SPI_2);        //  Initialize sensor on SPI_2 bus
  motor.linkSensor(&sensor);  // link the motor to the sensor

  // driver config
  driver.voltage_power_supply = 8.0;
  driver.voltage_limit = 4;
  driver.pwm_frequency = 40000;
  driver.init();
  driver.setSlew(Slew_200Vus);
  //driver.setPWMMode(PWM6_CurrentLimit_Mode);
  driver.setBuckVoltage(VB_5V);

  motor.linkDriver(&driver); 

  // aligning voltage 
  motor.voltage_sensor_align = 0.8;
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // force direction search - because default is CW
  motor.sensor_direction = Direction::UNKNOWN;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println("Sensor zero offset is:");
  Serial.println(motor.zero_electric_angle, 4);
  Serial.println("Sensor natural direction is: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");

  Serial.println("To use these values add them to the code:");
  Serial.print("   motor.sensor_direction=");
  Serial.print(motor.sensor_direction == Direction::CW ? "Direction::CW" : "Direction::CCW");
  Serial.println(";");
  Serial.print("   motor.zero_electric_angle=");
  Serial.print(motor.zero_electric_angle, 4);
  Serial.println(";");

  _delay(1000);
  Serial.println("If motor is not moving the alignment procedure was not successfull!!");


  digitalWrite(LED_BUILTIN, LOW);
  _delay(200);
}


void loop() {
    
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move(2);
}