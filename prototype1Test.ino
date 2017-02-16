#include <Encoder.h>
#include <Wire.h>
#include "DualVNH5019MotorShield.h"
#include "NAxisMotion.h"

//motor object
DualVNH5019MotorShield motorDriver(11,4,13,A0,7,8,12,A1);

//encoder object
Encoder motor1Enc(18, 19);
Encoder motor2Enc(2, 3);

//IMU object
NAxisMotion IMU;

//loop time
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const double streamPeriod = 50;          //To stream at 20Hz without using additional timers (time period(ms) =1000/frequency(Hz))
const double loopTime = streamPeriod/1000; //seconds

//distance sensors
//const int numDistSensors = 6;
//const int trigPins[numDistSensors] = {22,26,28,30,34,36};
//const int echoPins[numDistSensors] = {23,27,29,31,35,37};
//long durations[numDistSensors];
//long distances[numDistSensors];

int currentDistSensor = 0;
long loopCounter = 0;

//motor control
double motor1RevsPrevious;
double motor2RevsPrevious;
int motor1CommandPrevious;
int motor2CommandPrevious;
int kpVelocity = 100;

//make sure motors connected
void stopIfFault()
{
  if (motorDriver.getM1Fault())
  {
    Serial.println("M1 motor connection fault");
    while(1);
  }
  if (motorDriver.getM2Fault())
  {
    Serial.println("M2 motor connection fault");
    while(1);
  }
}

void setup() 
{
  //start serial communication with computer
  Serial.begin(115200);
    
  //Initialize I2C communication to the let the library communicate with the sensor.
  I2C.begin();
  
  //motor driver initialization
  motorDriver.init();
  
  //IMU Sensor Initialization
  IMU.initSensor();          //The I2C Address can be changed here inside this function in the library
  IMU.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  IMU.setUpdateMode(MANUAL);	//The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  
  //initialize distance sensor pins
//  for(int i=0; i<numDistSensors; i++)
//  {
//    pinMode(trigPins[i], OUTPUT);
//    pinMode(echoPins[i], INPUT);
//  }
}

void loop() 
{
  //keep loop at set rate
  if ((millis() - lastStreamTime) >= streamPeriod)
  {    
    //keep track of loop time, right now at 20hz
    lastStreamTime = millis(); 
    
    //read all sensor data and print to serial//
 
    //keep track of which distance sensor we are pinging, only ping one sensor per loop
//    if (currentDistSensor>=numDistSensors)
//    {
//      currentDistSensor = 0;
//    } 
    
    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus();  //Update the Calibration Status

    
//    //update encoder position
    long motor1Position = motor1Enc.read();
    long motor2Position = motor2Enc.read();

    double motor1Revs = double(motor1Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    double motor2Revs = double(motor2Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    
    //approximate wheel velocities
    double motor1Vel = ((motor1Revs - motor1RevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
    double motor2Vel = ((motor2Revs - motor2RevsPrevious)/ loopTime) * 0.1524 * 3.14159;
    
    //send ultrasonic pulses
//    digitalWrite(trigPins[currentDistSensor], LOW);
//    delayMicroseconds(2);
//    digitalWrite(trigPins[currentDistSensor], HIGH);
//    delayMicroseconds(5);
//    digitalWrite(trigPins[currentDistSensor], LOW);
//    
//    //read time till pulse returns
//    durations[currentDistSensor] = pulseIn(echoPins[currentDistSensor], HIGH);
//    
//    //convert time to centimeters
//    distances[currentDistSensor] = double(durations[currentDistSensor]) / 29 / 2;
    
    //print everything to serial
    Serial.print(" Yaw: ");
    Serial.print(IMU.readEulerHeading()); //Heading data
    Serial.print(" deg ");

    Serial.print(" Roll: ");
    Serial.print(IMU.readEulerRoll()); //Roll data
    Serial.print(" deg ");

    Serial.print(" Pitch: ");
    Serial.print(IMU.readEulerPitch()); //Pitch data
    Serial.print(" deg ");
    
    Serial.print(" Motor 1 Position: ");
    Serial.print(motor1Revs);
    Serial.print(" revs ");
    
    Serial.print(" Motor 2 Position: ");
    Serial.print(motor2Revs);
    Serial.print(" revs ");
    
//    for(int i=0; i<numDistSensors; i++)
//    {
//      Serial.print(" Distance ");
//      Serial.print(i+1);
//      Serial.print(": ");
//      Serial.print(distances[i]);
//      Serial.print(" cm ");
//    }

    Serial.println();
    
    //do controls and calculations
    double desiredM1Vel = 0.0;
    double desiredM2Vel = 0.0;
    
    int motor1Command = motor1CommandPrevious + kpVelocity * (desiredM1Vel - motor1Vel); //Proportional velocity control
    int motor2Command = motor2CommandPrevious + kpVelocity * (desiredM2Vel - motor2Vel);
    
    //command motors
    motorDriver.setM1Speed(motor1Command); //speed is between -400 and 400
//    motorDriver.setM2Speed(motor2Command); //speed is between -400 and 400
    stopIfFault();
    
    motor1CommandPrevious = motor1Command;
    motor2CommandPrevious = motor2Command;
    motor1RevsPrevious = motor1Revs; //save last wheel position
    motor2RevsPrevious = motor2Revs;

    currentDistSensor++;
    loopCounter++; //keep track of current loop
  }
}
