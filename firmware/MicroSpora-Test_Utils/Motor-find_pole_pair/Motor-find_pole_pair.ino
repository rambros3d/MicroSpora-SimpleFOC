/*

  finds pole pair number code modified for MicroSpora

 * Utility arduino sketch which finds pole pair number of the motor
 *
 * To run it just set the correct pin numbers for the BLDC driver and sensor CPR value and chip select pin.
 *
 * The program will rotate your motor a specific amount and check how much it moved, and by doing a simple calculation calculate your pole pair number.
 * The pole pair number will be outputted to the serial terminal.
 *
 * If the pole pair number is well estimated your motor will start to spin in voltage mode with 2V target.
 *
 * If the code calculates negative pole pair number please invert your motor connector.
 *
 * Try running this code several times to avoid statistical errors.
 * > But in general if your motor spins, you have a good pole pairs number.
*/

#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "drivers/drv8316/drv8316.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

#include <arduino_pin_def.h>

// its important to put pole pairs number as 1!!!
BLDCMotor motor = BLDCMotor(1);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(PHA_H, PHA_L, PHB_H, PHB_L, PHC_H, PHC_L, DRV_CS, false);

SPIClass SPI_2(ENC_NC, ENC_SDO, ENC_CLK);
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(ENC_CS);

float target_voltage = 0.6;

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
  driver.voltage_limit = 0.8;
  driver.pwm_frequency = 20000;
  driver.init();
  driver.setSlew(Slew_200Vus);
  //driver.setPWMMode(PWM6_CurrentLimit_Mode);
  driver.setBuckVoltage(VB_5V);

  motor.linkDriver(&driver);  // link the motor and the driver

  // initialize motor hardware
  motor.init();

  // pole pairs calculation routine
  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");

  float pp_search_voltage = 0.6;      // maximum power_supply_voltage/2
  float pp_search_angle = 6 * _PI;  // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = pp_search_voltage;
  motor.move(0);
  _delay(1000);
  // read the sensor angle
  sensor.update();
  float angle_begin = sensor.getAngle();
  _delay(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while (motor_angle <= pp_search_angle) {
    motor_angle += 0.01f;
    sensor.update();  // keep track of the overflow
    motor.move(motor_angle);
    _delay(1);
  }
  _delay(1000);
  // read the sensor value for 180
  sensor.update();
  float angle_end = sensor.getAngle();
  _delay(50);
  // turn off the motor
  motor.move(0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle) / (angle_end - angle_begin));

  Serial.print(F("Estimated PP : "));
  Serial.println(pp);
  Serial.println(F("PP = Electrical angle / Encoder angle "));
  Serial.print(pp_search_angle * 180 / _PI);
  Serial.print(F("/"));
  Serial.print((angle_end - angle_begin) * 180 / _PI);
  Serial.print(F(" = "));
  Serial.println((pp_search_angle) / (angle_end - angle_begin));
  Serial.println();


  // a bit of monitoring the result
  if (pp <= 0) {
    Serial.println(F("PP number cannot be negative"));
    Serial.println(F(" - Try changing the search_voltage value or motor/sensor configuration."));
    return;
  } else if (pp > 30) {
    Serial.println(F("PP number very high, possible error."));
  } else {
    Serial.println(F("If PP is estimated well your motor should turn now!"));
    Serial.println(F(" - If it is not moving try to relaunch the program!"));
    Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }


  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align sensor and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));


  digitalWrite(LED_BUILTIN, LOW);
  _delay(200);
}


void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      // reset the command buffer
      received_chars = "";
    }
  }
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // communicate with the user
  serialReceiveUserCommand();
}