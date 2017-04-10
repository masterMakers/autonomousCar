#include <Wire.h>
#include "NAxisMotion.h"

//IMU object
NAxisMotion IMU;

//Fall Detection
int state = 1;
double z_accel;


void setup() 
{
  //start serial communication with computer
  Serial.begin(115200);
    
  //Initialize I2C communication to the let the library communicate with the sensor.
  I2C.begin();
  
  //IMU Sensor Initialization
  IMU.initSensor();          
  IMU.setOperationMode(OPERATION_MODE_NDOF);   
  IMU.setUpdateMode(MANUAL);	
}

void loop() 
{    
    //update IMU information
    IMU.updateEuler();
    IMU.updateAccel();    
    IMU.updateCalibStatus();  //Update the Calibration Status
    
    //read z accel
    z_accel = IMU.readAccelZ();
    
    //where in the jump am I?
    switch(state)
    {
       case 1: //everything is normal
         Serial.print("normal  ");
         if (z_accel>13)
         {
           state = state+1;
         }
         break;
       case 2: //jumping
         Serial.print("jumping  ");
         if (z_accel<2)
         {
           state = state +1; 
         }
         break;
       case 3: //falling
         Serial.print("falling  ");
         if (z_accel>15)
           state = state+1;
         break;
       case 4: //jump over
         Serial.print("jump over  ");
         if (z_accel<13)
         {
           delay(5000); //wait for all the bouncing and vibrations to stop
           state = 1;
         }
         break;
    }
    Serial.print("Z Accel: ");
    Serial.print(z_accel); //z accel data
    Serial.println();
   
    delay(10);
}
