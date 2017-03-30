#include <Encoder.h>
#include <Wire.h>
#include "NAxisMotion.h"

//IMU object
NAxisMotion IMU;

//loop time
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const double streamPeriod = 50;          //To stream at 20Hz without using additional timers (time period(ms) =1000/frequency(Hz))

//distance sensors
const int numDistSensors = 8;
const int trigPins[numDistSensors] = {22,24,26,28,30,32,34,36};
const int echoPins[numDistSensors] = {23,25,27,29,31,33,35,37};
long durations[numDistSensors];
float distances[numDistSensors];

int currentDistSensor = 0;
long loopCounter = 0;
bool humanReadable = false;
String cc = String(",");
String imu = String("i");
String sonic = String("u");

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
  //keep loop at set rate
  if ((millis() - lastStreamTime) >= streamPeriod) {    
    //keep track of loop time, right now at 20hz
    lastStreamTime = millis(); 
    
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
    durations[currentDistSensor] = pulseIn(echoPins[currentDistSensor], HIGH, 15000); //(echopin, pulse read, timeout in microseconds)
    //convert time to centimeters
    distances[currentDistSensor] = double(durations[currentDistSensor]) / 29.0 / 2.0;
    
    if(humanReadable) {
  //    //print everything to serial
  //    Serial.print(" Yaw: ");
  //    Serial.print(IMU.readEulerHeading()); //Heading data
  //    Serial.print(" deg ");
  //
  //    Serial.print(" Roll: ");
  //    Serial.print(IMU.readEulerRoll()); //Roll data
  //    Serial.print(" deg ");
  //
  //    Serial.print(" Pitch: ");
  //    Serial.print(IMU.readEulerPitch()); //Pitch data
  //    Serial.print(" deg ");
      
      Serial.print(" Current Sensor: ");
      Serial.print(currentDistSensor+1);
      for(int i=0; i<numDistSensors; i++) {
        Serial.print(" D");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(distances[i]);
        Serial.print(" cm ");
      }
      Serial.println();
      
    } else {
      Serial.println(sonic + currentDistSensor + cc + durations[currentDistSensor] + cc + distances[currentDistSensor]);
      // Yaw, Roll, Pitch
      Serial.println(imu + IMU.readEulerHeading() + cc + IMU.readEulerRoll() + cc + IMU.readEulerPitch());
    }
    
    currentDistSensor = (currentDistSensor + 1) % numDistSensors;
    loopCounter++; //keep track of current loop
  }
}

