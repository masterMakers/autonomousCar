
#include "DualVNH5019MotorShield.h"
#include <Encoder.h>
#include <PID_v1.h>

double mInput = 0.0, mOutput = 0.0, mSetpoint = 1.0;
PID mPID(&mInput, &mOutput, &mSetpoint, 150.0, 80.0, 2.0, DIRECT);
                                  // Kp, Ki, Kd

unsigned int serialPing = 500; // ping interval in ms
unsigned long lastMessage = 0;

DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
Encoder motorEnc(2, 3);
//Encoder motorEnc(18, 19);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    motorDriver.init();

    mPID.SetOutputLimits(-400, 400); // -400 for backwards motion
    mPID.SetSampleTime(50);
    double K = 1000.0 / (240 / (0.1524 * 3.14159)); 
                  // (ms/s) / (ticksPerMeter)
                  // ticksPerMeter = ticksPerRev / MetersPerRev
    mPID.SetWheelParam(K);
    mPID.SetMode(AUTOMATIC);
    unsigned long now = millis();
    lastMessage = now;
    delay(10);
}

void loop()
{
    mPID.ComputeVelocity(motorEnc.read());
    motorDriver.setM2Speed((int)mOutput);
//    motorDriver.setM1Speed((int)mOutput);
    stopIfFault();
    
    unsigned long now = millis();
    if((now - lastMessage) > serialPing) {
        Serial.print(mInput);
        Serial.print(" ");
        Serial.println((int)mOutput);

        lastMessage = now;
    }
    
    if (Serial.available() > 0) { // check for new commands
        char inByte = (char)Serial.read();
        double val = (double)Serial.parseFloat();

        switch(inByte) {
        case 's' :
            mSetpoint = val;
            break;
        case 'p' :
            mPID.SetTunings(val, mPID.GetKi(), mPID.GetKd());
            break;
        case 'd' :
            mPID.SetTunings(mPID.GetKp(), mPID.GetKi(), val);
            break;
        case 'i' :
            mPID.SetTunings(mPID.GetKp(), val, mPID.GetKd());
            break;
        }
    }
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

