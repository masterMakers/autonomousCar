
#include <PID_v1.h>
#include "DualVNH5019MotorShield.h"
#include <Encoder.h>

// Tuning parameters
float Kp = 0;
float Ki = 10;
float Kd = 0;
double Input, Output;
double Setpoint = 0;
PID mPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const double K = 1000.0 / (240 / (2 * 0.1524 * 3.14159)); // (ms/s) / (ticksPerMeter)
                                        // ticksPerMeter = ticksPerRev / MetersPerRev

const unsigned long serialPing = 500; // ping interval in ms
unsigned long now = 0;
unsigned long lastMessage = 0;
unsigned long lastCompute = 0;
#define BUFFER_SIZE 8 // total message size = BUFFER_SIZE + 2

DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
Encoder motorEnc(2, 3);
long motorTicksPrev;

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println('A');
    delay(300);
  }
  int a = Serial.read();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    motorDriver.init();
    mPID.SetMode(AUTOMATIC);

    motorTicksPrev = motorEnc.read();
    lastCompute = millis();
    lastMessage = lastCompute;
}

void loop()
{
    // velocity calculation
    long motorTicks = motorEnc.read();
    now = millis();
    double dt = double(now - lastCompute);
    lastCompute = now;

    Input = K * ((double)temp) / dt; // m/s
    motorTicksPrev = motorTicks;

    mPID.Compute();
    motorDriver.setM2Speed(Output); //speed is between -400 and 400
    stopIfFault();

    if((now - lastMessage) > serialPing) {
        char buffer[BUFFER_SIZE];
        snprintf(buffer, BUFFER_SIZE, "%f", Input);
        Serial.print(buffer);
        Serial.print(' ');
        snprintf(buffer, BUFFER_SIZE, "%f", Output);
        Serial.println(buffer);

        // Serial.write('s');
        // Serial.write(Input);

        lastMessage = now;
    }

    if (Serial.available() > 0) { // check for new commands
        char inByte = (char)Serial.read();
        float val = Serial.parseFloat();

        switch(inByte) {
        case 's' :
            Setpoint = (double)val;
            break;
        case 'p' :
            Kp = val;
            mPID.SetTunings(Kp, Ki, Kd);
            break;
        case 'd' :
            Kd = val;
            mPID.SetTunings(Kp, Ki, Kd);
            break;
        case 'i' :
            Ki = val;
            mPID.SetTunings(Kp, Ki, Kd);
            break;
        default :
            // error
        }
    }
}
