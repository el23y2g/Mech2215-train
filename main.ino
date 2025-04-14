/* main file for the Arduino sketch
    This file includes the main loop and setup functions for the Arduino sketch.
    It initializes the sensors and actuators, reads sensor data, processes it,
    and controls the actuators based on the processed data.
    Authors: Yohann Godinho, Harry Hartley , Oliver Lotter and Kris Wang
*/



{// includes
#include <Servo.h>
}

void void setup(){
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Initialize the sensors and actuators
  initSensors();
  initActuators();
  initKalman();
  unsigned long previousMillis = 0;
  const unsigned long interval = 50;
}

void loop(){
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    readSensors();
  
    processSensorData(
      accelerometerData,
      irData,
      ultrasonicData,
      sharpData
    );
    
    controlActuators(
      updateKalman(
          speed,
          accel,
          checkpoint
        )
      );
  
}

void initSensors()
{
    // Initialize the accelerometer
    int x,y,z = 0;
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);

  
  // Initialize the IR sensor
    pinMode(A5, INPUT);


  
  // Initialize the ultrasonic sensor
    pinMode(13, OUTPUT); //trigger pin
    pinMode(A0, INPUT);  //echo pin
  
  // Initialize the sharp sensor
    pinMode(A1, INPUT);



}

void initActuators()
{
  // Initialize the servo motor
    Servo servo;
    int servoAngle;
    servo.attach(3); 

  // Initialize the stepper motor
  #define Ha1a 1
  #define Ha2a 2
  #define Ha3a 3
  #define Ha4a 4
  #define Ha12En 5
  #define Ha34En 6

    pinMode(Ha1a, OUTPUT); 
    pinMode(Ha2a, OUTPUT);
    pinMode(Ha3a, OUTPUT);
    pinMode(Ha4a, OUTPUT);
    pinMode(Ha12En, OUTPUT);
    pinMode(Ha34En, OUTPUT);
    
  // Initialize the DC motor
  #define Hb1A 7
  #define Hb2A 8
  #define Hb3A 9
  #define Hb4A 10
  #define Hb12EN 11
  #define Hb34EN 12

    pinMode(Hb1A, OUTPUT); 
    pinMode(Hb2A, OUTPUT);
    pinMode(Hb3A, OUTPUT);
    pinMode(Hb4A, OUTPUT);
    pinMode(Hb12EN, OUTPUT);
    pinMode(Hb34EN, OUTPUT);
   
}

void readSensors()
{
  // Read the accelerometer data
    int x = analogRead(A2);
    int y = analogRead(A3);
    int z = analogRead(A4);
  
  // Read the IR sensor data
    int irValue = analogRead(A5);
  
  // Read the ultrasonic sensor data
    digitalWrite(13, LOW); // Set trigger pin to low
    delayMicroseconds(2); // Wait for 2 microseconds
    digitalWrite(13, HIGH); // Set trigger pin to high
    delayMicroseconds(10); // Wait for 10 microseconds
    digitalWrite(13, LOW); // Set trigger pin to low
    long duration = pulseIn(A0, HIGH); // Read the echo pin
    float distance = (duration * 0.034) / 2; // Calculate distance in cm

  // Read the sharp sensor data
    int sharpValue = analogRead(A1);
}

void processSensorData(int x, int y, int z, 
                       int irValue, float distance, int sharpValue){
  // Process the accelerometer data  maybe move to read data
    float xAccel = map(x, 0, 1023, -3*9.80665, 3*9.80665);
    float yAccel = map(y, 0, 1023, -3*9.80665, 3*9.80665);
    float zAccel = map(z, 0, 1023, -3*9.80665, 3*9.80665);
    float accel = sqrt(xAccel*xAccel + yAccel*yAccel + zAccel*zAccel);



    // Process the IR sensor data

    // Process the ultrasonic sensor data

    // Process the sharp sensor data


    
}

void initKalman() {


  float train[2] = {0, 0};    // [position, velocity]
  float P[2][2] = {{1, 0}, {0, 1}};  // Covariance matrix

  const float dt = 0.05;  // Time step in seconds
  const float accel_noise = 0.2; 
  const float speed_noise = 0.5;
  const float pos_noise = 0.01; 
  
  const float loopLength = 14.88;
  const float checkpointPositions[3] = {7.14, 7.74, 14.50875};
}

float updateKalman(float speed, float accel, int checkpoint) {
  // Prediction
  train[0] += train[1] * dt + 0.5 * accel * dt * dt; // Update position
  train[1] += accel * dt; // Update velocity

  float F[2][2] = { // State transition matrix
    {1, dt},
    {0, 1}
  };

  float Q[2][2] = { // Noise covariance matrix
    {0.25 * dt * dt * dt * dt * accel_noise, 0.5 * dt * dt * dt * accel_noise},
    {0.5 * dt * dt * dt * accel_noise, dt * dt * accel_noise}
  };

  // P = F * P * F^T + Q      Covariance update
  // 
  float P_temp[2][2];
  float P_new[2][2];
  for (int i = 0; i < 2; i++) //matrix multiplication
    for (int j = 0; j < 2; j++) {
      P_temp[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
    }

  for (int i = 0; i < 2; i++) // dot product + noise
    for (int j = 0; j < 2; j++) {
      P_new[i][j] = P_temp[i][0] * F[j][0] + P_temp[i][1] * F[j][1] + Q[i][j];
      P[i][j] = P_new[i][j];
    }

  // Update from speed
  float H_vel[2] = {0, 1};
  float y = speed - train[1];
  float S = P[1][1] + speed_noise;
  float K[2] = {P[0][1] / S, P[1][1] / S};

  train[0] += K[0] * y;
  train[1] += K[1] * y;

  float P00 = P[0][0], P01 = P[0][1], P10 = P[1][0], P11 = P[1][1];
  P[0][0] = P00 - K[0] * P01;
  P[0][1] = P01 - K[0] * P11;
  P[1][0] = P10 - K[1] * P01;
  P[1][1] = P11 - K[1] * P11;

  // ==== Update from checkpoint ====
  if (checkpoint >= 0 && checkpoint < 3) {
    float pos_meas = checkpointPositions[checkpoint];
    float y_pos = pos_meas - train[0];
    float S_pos = P[0][0] + pos_noise;
    float Kp[2] = {P[0][0] / S_pos, P[1][0] / S_pos};

    train[0] += Kp[0] * y_pos;
    train[1] += Kp[1] * y_pos;

    P00 = P[0][0]; P01 = P[0][1]; P10 = P[1][0]; P11 = P[1][1];
    P[0][0] = P00 - Kp[0] * P00;
    P[0][1] = P01 - Kp[0] * P01;
    P[1][0] = P10 - Kp[1] * P00;
    P[1][1] = P11 - Kp[1] * P01;
  }

  // Keep position wrapped within track length
  train[0] = fmod(train[0], loopLength);
  if (train[0] < 0) train[0] += loopLength;

  return train[0]; // Estimated position in meters
}

void controlActuators(float position) {
  int drop = 0;

  if (position < 0) {
    //arm up

  } else if (position > 8 && checkpoint >= 2) {
    //arm down
    drop = 1;
  }
  if (drop == 1 && position > 14.880) { // assuming instant drop.
                                       // since track wraps, stops accidentally dropping when rolling back
    //drop the object    
  } else {
    //hold the object
  }
}