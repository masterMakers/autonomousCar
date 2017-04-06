
#include "DualVNH5019MotorShield.h"
#include <Encoder.h>
#include <PID_v1.h>

double mInput = 0.0, mOutput = 0.0, mSetpoint = 1.0;
PID mPID(&mInput, &mOutput, &mSetpoint, 40, 0, 10, DIRECT);
                                  // Kp, Ki, Kd

const double K = 1000.0 / (240 / (0.1524 * 3.14159)); // (ms/s) / (ticksPerMeter)
                           // ticksPerMeter = ticksPerRev / MetersPerRev

unsigned int serialPing = 500; // ping interval in ms
unsigned long lastMessage = 0;
unsigned long lastCompute = 0;

DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
Encoder motorEnc(2, 3);
long motorTicksPrev;

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    motorDriver.init();
    motorTicksPrev = motorEnc.read();

    mPID.SetOutputLimits(-400, 400);
    mPID.SetSampleTime(10);
    mPID.SetMode(AUTOMATIC);
    unsigned long now = millis();
    lastCompute = now;
    lastMessage = now;
    delay(10);
}

void loop()
{
    // velocity calculation
    long motorTicks = motorEnc.read();
    unsigned long now = millis();
    double dt = (double)(now - lastCompute);
    lastCompute = now;

    mInput = (double)(motorTicks - motorTicksPrev) * K / dt;
    mOutput = mPID.Compute(mInput, mOutput, mSetpoint);
    motorTicksPrev = motorTicks;

    motorDriver.setM2Speed((int)mOutput); //speed is between -400 and 400
    stopIfFault();
    
    if((now - lastMessage) > serialPing) {
        // Serial.write('s');
        // Serial.write(Input);

        Serial.print(mInput);
        Serial.print(" ");
        Serial.println(mOutput);

        lastMessage = now;
    }

    delay(10);
    
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

