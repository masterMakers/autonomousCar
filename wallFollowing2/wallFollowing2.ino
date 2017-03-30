#include <Encoder.h>
#include <Wire.h>
#include "DualVNH5019MotorShield.h"
#include "NAxisMotion.h"
#include <PID_v1.h>

//motor object
DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
//encoder object
Encoder motorEncL(18, 19);
Encoder motorEncR(2, 3);
//IMU object
NAxisMotion IMU;

//distance sensors
const int numDistSensors = 5;
const int trigPins[numDistSensors] = {22,24,32,34,36};
const int echoPins[numDistSensors] = {23,25,33,35,37};
const int LEFT_CORNER = 0;
const int LEFT_SIDE = 1;
const int RIGHT_SIDE = 2;
const int RIGHT_CORNER = 3;
const int FRONT = 4;
// sides, front then corners
double distances[numDistSensors];
const double M = 1 / 29.0 / 2.0; // distance calibration

//motor control
long motorTicksPrevL;
long motorTicksPrevR;
unsigned long lastEncoderTime = 0;
int cycleCounter = 0;
double motorVelL, motorCmdL, motorVelR, motorCmdR;
double desiredVelL = 1.0;
double desiredVelR = 1.0;
PID pidL(&motorVelL, &motorCmdL, &desiredVelL, 5.0, 0.0, 30.0, DIRECT);
PID pidR(&motorVelR, &motorCmdR, &desiredVelR, 5.0, 0.0, 30.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode
const double K = 1000.0 / (240 / (2 * 0.1524 * 3.14159)); // (ms/s) / (ticksPerMeter)
                                        // ticksPerMeter = ticksPerRev / MetersPerRev

//wall following
double nominalForwardSpeed = 0.5;
double rightWallDistance, wallSteeringAngle;
double desWallDistance = 30.0;
PID pidW(&rightWallDistance, &wallSteeringAngle, &desWallDistance, 0.00005, 0.0, 0.00001, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//high level control
int gait = 1; //1 is go straight fast, 2 means turn 90 degrees left
double initialYaw = 0;
double desiredYaw = 0;
double currentYaw;
double yawChange = 0.0;
int counter = 0;
bool wallDetected = false;

bool debug = true;

void setup() 
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    //Initialize I2C communication to the let the library communicate with the sensor.
    I2C.begin();
    //motor driver initialization
    motorDriver.init();
    
    //IMU Sensor Initialization
    IMU.initSensor();          //The I2C Address can be changed here inside this function in the library
    IMU.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
    IMU.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant 
                                //update functions prior to calling the read functions
    
    //initialize distance sensor pins
    for(int i = 0; i < numDistSensors; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
    initialYaw = IMU.readEulerHeading();

    pidL.SetMode(AUTOMATIC);
    pidR.SetMode(AUTOMATIC);
    pidW.SetMode(AUTOMATIC);

    motorTicksPrevL = motorEncL.read();
    motorTicksPrevR = motorEncR.read();
    lastEncoderTime = millis();
    lastMessage = lastEncoderTime;
}

void loop() 
{
    //update motor speed
    long motorTicksL = motorEncL.read();
    long motorTicksR = motorEncR.read();
    
    unsigned long now = millis();
    double dt = double(now - lastEncoderTime);
    lastEncoderTime = now;

    motorVelL = K * double(motorTicksL - motorTicksPrevL) / dt; // m/s
    motorVelR = K * double(motorTicksR - motorTicksPrevR) / dt;
    motorTicksPrevL = motorTicksL;
    motorTicksPrevR = motorTicksR;

    //low level motor control
    pidL.Compute();
    pidR.Compute();
    motorDriver.setM2Speed(motorCmdL); //speed is between -400 and 400
    motorDriver.setM1Speed(motorCmdR); 
    stopIfFault();
    

    if(cycleCounter >= 4) { 
        // inner motorVel PID loop occurs 5x faster than outer steeringAngle PID loop
        cycleCounter = 0;
        
        for(int i = 0; i < numDistSensors; i++) {
            //send ultrasonic pulses
            digitalWrite(trigPins[i], HIGH);
            delayMicroseconds(5);
            digitalWrite(trigPins[i], LOW);
            //read time till pulse returns, convert time to distance (cm)
            distances[i] = double(pulseIn(echoPins[i], HIGH, 5000)) * M;
            if (distances[i] == 0.0) { // error state, ping never returned
                distances[i] = 150.0;
            }
        }
            
        //update IMU information
        IMU.updateEuler();     
        IMU.updateCalibStatus();  //Update the Calibration Status

        //right wall following
        rightWallDistance = distances[RIGHT_SIDE];
        pidW.Compute();
        if(rightWallDistance < 120.0) {
            wallDetected = true;
        }

        //behavioral control
        if(IMU.readEulerHeading() - desiredYaw >= 180) {
            currentYaw = IMU.readEulerHeading() - desiredYaw - 360;
        } else {
            currentYaw = IMU.readEulerHeading() - desiredYaw;
        }

        desiredVelL = nominalForwardSpeed;
        desiredVelR = nominalForwardSpeed;
        // finite state automaton
        switch(gait) {
        case 1: //go straight
            if (abs(currentYaw - initialYaw) <= 10) {
                if (wallError > 0) {
                    desiredVelR = nominalForwardSpeed - abs(wallSteeringAngle);
                } else if (wallError < 0) {
                    desiredVelL = nominalForwardSpeed - abs(wallSteeringAngle);
                }
            } else if(currentYaw > initialYaw) {
                desiredVelL = 0.25;
                desiredVelR = 0.35;
            } else if(currentYaw < initialYaw) {
                desiredVelL = 0.35;
                desiredVelR = 0.25;
            }

            if (rightWallDistance > 120.0 && wallDetected) {
                gait = 2;
    //          initialYaw = IMU.readEulerHeading();
                counter++;
            }
            break;
        case 2: //turn 90 degrees left
            desiredVelL = 0.5;
            desiredVelR = -0.5;
            wallDetected = false;
            if (abs(currentYaw - initialYaw) > 90) {
                gait = 3;
                counter = 0;
                desiredYaw = 90;
                motorDriver.setM1Brake(400);
                motorDriver.setM2Brake(400);
                delay(1000);
            }
            break;
            case 3:
            if(currentYaw > initialYaw) {
                desiredVelL = 0.4;
                desiredVelR = 0.5;
            } else if(currentYaw < initialYaw) {
                desiredVelL = 0.5;
                desiredVelR = 0.4;
            }
            
    //        desiredVelL = 0.25;
    //        desiredVelR = 0.35;

            if (wallDetected) {
    //          delayMicroseconds(500);
                gait = 1;
            }
            break;
        }

        if(debug) {
            Serial.print(" Yaw: ");
            Serial.print(currentYaw); //Heading data
            Serial.print(" deg ");
            Serial.print(" M1 Des Vel: ");
            Serial.print(desiredVelL);
            Serial.print(" M2 Des Vel: ");
            Serial.print(desiredVelR);
            Serial.print(" Wall Detected: ");
            Serial.print(wallDetected);
            Serial.print(" Gait:");
            Serial.print(gait);    
            Serial.print(" Right sensor reading:");
            Serial.print(rightWallDistance);
            Serial.println();
        }
    }

    cycleCounter++;
}


//make sure motors connected
void stopIfFault()
{
    if (motorDriver.getM1Fault()) {
        Serial.println("M1 motor connection fault");
        while(1);
    }
    if (motorDriver.getM2Fault()) {
        Serial.println("M2 motor connection fault");
        while(1);
    }
}
