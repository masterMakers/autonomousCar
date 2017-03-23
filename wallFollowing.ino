#include <Encoder.h>
#include <Wire.h>
#include "DualVNH5019MotorShield.h"
#include "NAxisMotion.h"

//motor object
DualVNH5019MotorShield motorDriver(11,5,13,A0,7,8,12,A1);

//encoder object
Encoder motor1Enc(18, 19);
Encoder motor2Enc(2, 3);

//IMU object
NAxisMotion IMU;

//loop time
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const double streamPeriod = 20;          //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
const double loopTime = streamPeriod/1000; //seconds

//distance sensors
const int numDistSensors = 5;
const int trigPins[numDistSensors] = {22,24,32,34,36};
const int echoPins[numDistSensors] = {23,25,33,35,37};
long durations[numDistSensors];
float distances[numDistSensors];
int currentDistSensor = 0;

//motor control
double motor1RevsPrevious;
double motor2RevsPrevious;
int motor1CommandPrevious;
int motor2CommandPrevious;
double kpVelocity = 5.0; //10
double kdVelocity = 30.0; //50
int motor1Command = 0;
int motor2Command = 0;
double desiredM1Vel = 1.0;
double desiredM2Vel = 1.0;
double lastM1Error;
double lastM2Error;

//high level control
int gait = 1; //1 is go straight fast, 2 means turn 90 degrees left
double initialYaw;
double yawChange = 0.0;
int counter = 0;

//wall following
double nominalForwardSpeed = 0.5;
double desWallDistance = 40.0;
double rightWallDistance = 0.0;
double previousWallError = 0.0;
double wallError = 0.0;
double wallPIDTerm = 0.0;
double kpWall = 0.00005;
double kdWall = 0.00001;

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
  for(int i=0; i<numDistSensors; i++)
  {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() 
{
  //keep loop at set rate
  if ((millis() - lastStreamTime) >= streamPeriod)
  {    
    //keep track of loop time, right now at 20hz
    lastStreamTime = millis(); 
    
    //read all sensor data//
    
     //send ultrasonic pulses
    digitalWrite(trigPins[currentDistSensor], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPins[currentDistSensor], LOW);
    
    //read time till pulse returns
    durations[currentDistSensor] = pulseIn(echoPins[currentDistSensor], HIGH, 5000);
    
    //convert time to distance (cm)
    distances[currentDistSensor] = double(durations[currentDistSensor]) / 29.0 / 2.0;
    if (distances[currentDistSensor] == 0.0)
    {
      distances[currentDistSensor] = 150.0;
    }
   
    
    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus();  //Update the Calibration Status

    //update motor positions
    long motor1Position = motor1Enc.read();
    long motor2Position = motor2Enc.read();

    double motor1Revs = double(motor1Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    double motor2Revs = double(motor2Position) / 240;
    
    //approximate wheel velocities
    double motor1Vel = ((motor1Revs - motor1RevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
    double motor2Vel = ((motor2Revs - motor2RevsPrevious)/ loopTime) * 0.1524 * 3.14159;
    
    //right wall following
    rightWallDistance = distances[2];
    wallError = rightWallDistance - desWallDistance;
    wallPIDTerm = (kpWall * wallError) + (kdWall * (wallError - previousWallError));
    
    desiredM1Vel = nominalForwardSpeed;
    desiredM2Vel = nominalForwardSpeed;
    
    if (wallError>0)
    {
      desiredM2Vel = nominalForwardSpeed - abs(wallPIDTerm);
    }
    else if (wallError<0)
    {
      desiredM1Vel = nominalForwardSpeed - abs(wallPIDTerm);
    }
       
    
    
//    //behavioral control
//    if ((distances[4]<20.0) && (counter == 0))
//    {
//      gait = 2;
//      initialYaw = IMU.readEulerHeading();
//      counter++;
//    }
//    
//    switch(gait)
//    {
//      case 1: //go straight
//        desiredM1Vel = 1;
//        desiredM2Vel = 1;
//        break;
//      case 2: //turn 90 degrees left
//        desiredM1Vel = 0.5;
//        desiredM2Vel = -0.5;
//        if (abs(IMU.readEulerHeading()-initialYaw) > 89.5)
//        {
//          gait = 1;
//          counter = 0;
//        }
//        break;
//    }
    
    //low level control
    double M1Error = desiredM1Vel - motor1Vel;
    double M2Error = desiredM2Vel - motor2Vel;
    motor1Command = motor1CommandPrevious + (kpVelocity * M1Error) + (kdVelocity * (M1Error - lastM1Error)); //PD control around velocity
    motor2Command = motor2CommandPrevious + (kpVelocity * M2Error) + (kdVelocity * (M2Error - lastM2Error));
    
    //command motors
    motorDriver.setM2Speed(motor1Command); //speed is between -400 and 400
    motorDriver.setM1Speed(motor2Command); 
    stopIfFault();
    
    //print everything to serial
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
    
//    Serial.print(" M1 Pos: ");
//    Serial.print(motor1Revs);
//    Serial.print(" revs ");
//    
//    Serial.print(" M2 Pos: ");
//    Serial.print(motor2Revs);
//    Serial.print(" revs ");
    
//    Serial.print(" M1 Vel: ");
//    Serial.print(motor1Vel);
//    Serial.print(" m/s ");
//    
//    Serial.print(" M2 Vel: ");
//    Serial.print(motor2Vel);
//    Serial.print(" m/s ");
//    
//    Serial.print(" M1 Error: ");
//    Serial.print(M1Error);
//    Serial.print(" m/s ");
//    
//    Serial.print(" M2 Error: ");
//    Serial.print(M2Error);
//    Serial.print(" m/s ");
//
//    Serial.print(" M1 Command: ");
//    Serial.print(motor1Command);
//    
//    Serial.print(" M2 Command: ");
//    Serial.print(motor2Command);
    
//    for(int i=0; i<numDistSensors; i++)
//    {
//      Serial.print(" D");
//      Serial.print(i+1);
//      Serial.print(": ");
//      Serial.print(distances[i]);
//      Serial.print(" cm ");
//    }
    
    
    Serial.print(" M1 Des Vel: ");
    Serial.print(desiredM1Vel);
    
    Serial.print(" M2 Des Vel: ");
    Serial.print(desiredM2Vel);
    
    Serial.print(" PID: ");
    Serial.print(wallPIDTerm);
    
    Serial.print(" Wall Error: ");
    Serial.print(wallError);
    
//    Serial.print(" D5: ");
//    Serial.print(distances[4]);
//    Serial.print(" cm ");
//    
//    Serial.print(" Yaw Change:");
//    Serial.print(abs(IMU.readEulerHeading()-initialYaw));
//    
//    Serial.print(" Gait:");
//    Serial.print(gait);
    
    Serial.println();
    
    //loop cleanup
    lastM1Error = M1Error; //save last error
    lastM2Error = M2Error;
    previousWallError = wallError;
    motor1CommandPrevious = motor1Command; //save last motor command
    motor2CommandPrevious = motor2Command;
    motor1RevsPrevious = motor1Revs; //save last wheel position
    motor2RevsPrevious = motor2Revs;
    currentDistSensor = (currentDistSensor + 1) % numDistSensors; //toggle ultrasonic sensors
  }
}


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
