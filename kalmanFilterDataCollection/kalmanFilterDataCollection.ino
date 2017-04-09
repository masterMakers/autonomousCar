#include <Encoder.h>
#include <Wire.h>
#include "NAxisMotion.h"

//IMU object
NAxisMotion IMU;

//distance sensors
const int numDistSensors = 4;
const int trigPins[numDistSensors] = {24,26,28,32};
const int echoPins[numDistSensors] = {25,27,29,33};
long durations[numDistSensors];
float distances[numDistSensors];
const int LEFT_SIDE = 1, RIGHT_SIDE = 2, FRONT = 3, BACK = 4;

//motor speeds
Encoder motorEncL(2, 3);
Encoder motorEncR(18, 19);
float leftMotorRevsPrevious;
float rightMotorRevsPrevious;
float loopTime;
float lastTime;

int currentDistSensor = 0;
bool humanReadable = false;
String cc = String(",");
String imu = String("i");
String sonic = String("u");
String motor = String("m");

void setup() 
{
  //start serial communication with computer
  Serial.begin(115200);
    
  //Initialize I2C communication to the let the library communicate with the sensor.
  I2C.begin();
  
  //IMU Sensor Initialization
  IMU.initSensor();          //The I2C Address can be changed here inside this function in the library
  IMU.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  IMU.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  
  //initialize distance sensor pins
  for(int i=0; i<numDistSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() 
{    
    //read all sensor data and print to serial//

    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus();  //Update the Calibration Status

    //send ultrasonic pulses
    digitalWrite(trigPins[currentDistSensor], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[currentDistSensor], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPins[currentDistSensor], LOW);
    
    //read time till pulse returns
    durations[currentDistSensor] = pulseIn(echoPins[currentDistSensor], HIGH, 10000); //(echopin, pulse read, timeout in microseconds)
    distances[currentDistSensor] = double(durations[currentDistSensor]) / 29.0 / 2.0;
    if (distances[currentDistSensor]>150.0)
    {
      distances[currentDistSensor] = 150.0;
    }
    
    //update motor positions
    long leftMotorPosition = motorEncL.read();
    long rightMotorPosition = motorEncR.read();

    float leftMotorRevs = float(leftMotorPosition) / 240; //240 is the number of encoder ticks per revolution of output shaft
    float rightMotorRevs = float(rightMotorPosition) / 240;
    
    //approximate wheel velocities
    loopTime = (millis()-lastTime)/1000.0;
    float leftMotorVel =  ((leftMotorRevs - leftMotorRevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
    float rightMotorVel = ((rightMotorRevs - rightMotorRevsPrevious)/ loopTime) * 0.1524 * 3.14159;
    lastTime = millis();
    
    if(humanReadable) {
      //print everything to serial
      Serial.print(" Yaw: ");
      Serial.print(IMU.readEulerHeading()); //Heading data
      Serial.print(" deg ");
      
      Serial.print(" Motor L Vel: ");
      Serial.print(leftMotorVel);
      Serial.print(" m/s ");
      
      Serial.print(" Motor R Vel: ");
      Serial.print(rightMotorVel);
      Serial.print(" m/s ");
      
      for(int i=0; i<numDistSensors; i++) {
        Serial.print(" D");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(distances[i]);
        Serial.print(" cm ");
      }
      Serial.println();
      
    } else {
      Serial.println(sonic + currentDistSensor + cc + distances[currentDistSensor]);
      Serial.println(imu + IMU.readEulerHeading() + cc + IMU.readEulerRoll() + cc + IMU.readEulerPitch());
      Serial.println(motor + leftMotorRevs + cc + rightMotorRevs + cc + leftMotorVel + cc + rightMotorVel);
    }
    
    leftMotorRevsPrevious = leftMotorRevs; //save last wheel position
    rightMotorRevsPrevious = rightMotorRevs;
    currentDistSensor = (currentDistSensor + 1) % numDistSensors;
    
    delay(50);
}

