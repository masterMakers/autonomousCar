#include <Encoder.h>
#include <Wire.h>
#include "DualVNH5019MotorShield.h"
#include "NAxisMotion.h"


//motor object
DualVNH5019MotorShield motorDriver(11,4,6,A0,7,8,12,A1);
//DualVNH5019MotorShield motorDriver;

//encoder object
Encoder motor1Enc(18, 19);
Encoder motor2Enc(2, 3);

//IMU object
NAxisMotion IMU;

//loop time
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 50;          //To stream at 20Hz without using additional timers (time period(ms) =1000/frequency(Hz))

//distance sensors
const int numDistSensors = 8;
const int trigPins[numDistSensors] = {22,24,26,28,30,32,34,36};
const int echoPins[numDistSensors] = {23,25,27,29,31,33,35,37};
long durations[numDistSensors];
long distances[numDistSensors];
int currentDistSensor = 0;
long loopCounter = 0;

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
    
    //read all sensor data and print to serial//
 
    //keep track of which distance sensor we are pinging, only ping one sensor per loop
    if (currentDistSensor>=numDistSensors)
    {
      currentDistSensor = 0;
    } 
    
    //update IMU information
//    IMU.updateEuler();     
    
    //update encoder position
    long motor1Position = motor1Enc.read();
    long motor2Position = motor2Enc.read();

    double motor1Revs = double(motor1Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    double motor2Revs = double(motor2Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    
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
    
//    Serial.print(" Motor 1 Encoder Ticks: ");
//    Serial.print(motor1Position);
//    Serial.print(" ticks ");
    
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
    currentDistSensor++;
    
    //do controls and calculations
    
        //to do....
    
    //command motors
    if (loopCounter<100)
    {
      motorDriver.setM1Speed(loopCounter); //speed is between -400 and 400
      motorDriver.setM2Speed(loopCounter);
    }
    else if ((loopCounter >=100) && (loopCounter < 200))
    {
      motorDriver.setM1Speed(100);
      motorDriver.setM2Speed(100);
    }
    else if ((loopCounter >200) && (loopCounter < 300))
    {
      motorDriver.setM1Brake(100); //0 is full coast, 400 is full brake
      motorDriver.setM2Brake(100); 
    }
    else if ((loopCounter >= 300) && (loopCounter < 400))
    {
      motorDriver.setM1Speed(-loopCounter + 300);
      motorDriver.setM2Speed(-loopCounter + 300);
    }
    else if ((loopCounter >= 300) && (loopCounter < 500))
    {
      motorDriver.setM1Speed(-100);
      motorDriver.setM2Speed(-100);
    }
    else 
    {
      motorDriver.setM1Brake(400);
      motorDriver.setM2Brake(400);
    }
    stopIfFault();
    
    
    loopCounter++; //keep track of current loop
  }
}
