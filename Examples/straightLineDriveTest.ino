#include <Encoder.h>
#include <Wire.h>
#include "DualVNH5019MotorShield.h"

//motor object
DualVNH5019MotorShield motorDriver(11,4,13,A0,7,8,12,A1);

//encoder object
Encoder motor1Enc(18, 19);
Encoder motor2Enc(2, 3);

//loop time
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const double streamPeriod = 50;          //To stream at 20Hz without using additional timers (time period(ms) =1000/frequency(Hz))
const double loopTime = streamPeriod/1000; //seconds

int currentDistSensor = 0;
long loopCounter = 0;

//motor control
double motor1RevsPrevious;
double motor2RevsPrevious;
double motor1CommandPrevious = 0;
double motor2CommandPrevious = 0;
double motor1VelPrevious = 0;
double motor2VelPrevious = 0;
double motor1Command = 0;
double motor2Command = 0;
int kpVelocity = 1;

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
  
  //motor driver initialization
  motorDriver.init();
}

void loop() 
{
  //keep loop at set rate
  if ((millis() - lastStreamTime) >= streamPeriod)
  {    
    //keep track of loop time, right now at 20hz
    lastStreamTime = millis(); 
    
    //update encoder position
    long motor1Position = motor1Enc.read();
    long motor2Position = motor2Enc.read();

    double motor1Revs = double(motor1Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    double motor2Revs = double(motor2Position) / 240; //240 is the number of encoder ticks per revolution of output shaft
    
    //approximate wheel velocities
    double motor1Vel = ((motor1Revs - motor1RevsPrevious)/ loopTime) * 0.1524 * 3.14159; // [m/s]
    double motor2Vel = ((motor2Revs - motor2RevsPrevious)/ loopTime) * 0.1524 * 3.14159;
    
    //approximate wheel accelerations
    double motor1Accel = ((motor1Vel - motor1VelPrevious)/ loopTime); //[m/s^2]
    double motor2Accel = ((motor2Vel - motor2VelPrevious)/ loopTime);

    
    //do controls and calculations
    double desiredM1Vel = 0.1;
    double desiredM2Vel = 0.1;
    
    motor1Command = 200 + kpVelocity * (desiredM1Vel - motor1Vel); //Proportional velocity control
    motor2Command = 200 + kpVelocity * (desiredM2Vel - motor2Vel);
    
    //command motors
    motorDriver.setM1Speed((int)motor1Command); //speed is between -400 and 400
    motorDriver.setM2Speed((int)motor2Command); //speed is between -400 and 400
    stopIfFault();

  //print everything to serial    
    Serial.print(" M1 Position: ");
    Serial.print(motor1Revs);
    Serial.print(" revs ");
    
    Serial.print(" M2 Position: ");
    Serial.print(motor2Revs);
    Serial.print(" revs ");
    
    Serial.print(" M1 Velocity: ");
    Serial.print(motor1Vel);
    Serial.print(" m/s ");
    
    Serial.print(" M12 Velocity: ");
    Serial.print(motor2Vel);
    Serial.print(" m/s ");
    
    Serial.print(" M1 Command: ");
    Serial.print(motor1Command);
    
    Serial.print(" M2 Command: ");
    Serial.print(motor2Command);
    
    Serial.println();

    motor1CommandPrevious = motor1Command;
    motor2CommandPrevious = motor2Command;
    motor1RevsPrevious = motor1Revs; //save last wheel position
    motor2RevsPrevious = motor2Revs;
    motor1VelPrevious = motor1Vel; //save last wheel velocity
    motor2VelPrevious = motor2Vel; //save last wheel velocity
    loopCounter++; //keep track of current loop
  }
}
