/* main file for the Arduino sketch
    This file includes the main loop and setup functions for the Arduino sketch.
    It initializes the sensors and actuators, reads sensor data, processes it,
    and controls the actuators based on the processed data.
    Authors: Yohann Godinho, Harry Hartley , Oliver Lotter and 
*/
{
#include "Arduino.h"
#include "accelerometer.h"
#include "ir.h"
#include "servo.h"
#include "dc.h"
#include "ultrasonic.h"
#include "stepper.h"
#include "sharp.h"
}

void void setup(){
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Initialize the sensors and actuators
  initSensors();
  initActuators();
}

void loop(){
  // Read the sensor data
  readSensors();
  
  // Process the sensor data
  processSensorData();
  
  // Control the actuators based on the processed data
  controlActuators();
  
}

initSensors()
{
    // Initialize the accelerometer
    int x,y,z = 0;
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

  
  // Initialize the IR sensor
 
  
  // Initialize the ultrasonic sensor

  
  // Initialize the sharp sensor

  // Initialize the stepper motor

}

init Actuators()
{
  // Initialize the servo motor

  
  // Initialize the DC motor
 
}