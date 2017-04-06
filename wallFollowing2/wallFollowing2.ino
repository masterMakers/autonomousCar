#include <Encoder.h>
#include <Wire.h>
#include <PID_v1.h>
#include <NAxisMotion.h>
#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
Encoder motorEncL(18, 19);
Encoder motorEncR(2, 3);
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
double distances[numDistSensors];
const double M = 1 / 29.0 / 2.0; // distance calibration
int currDistSensor = 0;

//motor control
long motorTicksPrevL;
long motorTicksPrevR;
unsigned long lastEncoderTime = 0;
double motorVelL = 0.0, motorCmdL = 0.0, motorVelR = 0.0, motorCmdR = 0.0;
double desiredVelL = 1.0;
double desiredVelR = 1.0;
PID pidL(&motorVelL, &motorCmdL, &desiredVelL, 40.0, 0.0, 10.0, DIRECT);
PID pidR(&motorVelR, &motorCmdR, &desiredVelR, 40.0, 0.0, 10.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode
const double K = 1000.0 / (240 / (0.1524 * 3.14159)); // (ms/s) / (ticksPerMeter)
                                        // ticksPerMeter = ticksPerRev / MetersPerRev

//wall following
double nominalForwardSpeed = 0.5;
double rightWallDistance = 0.0, wallSteeringAmt = 0.0;
double desWallDistance = 30.0;
PID pidW(&rightWallDistance, &wallSteeringAmt, &desWallDistance, 
        0.00005, 0.0, 0.00001, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//high level control
int gait = 1; //1 is go straight fast, 2 means turn 90 degrees left
double wheelBase = 0.2; // m
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
    I2C.begin();
    motorDriver.init();
    
    IMU.initSensor(); // I2C Address can be changed here
    IMU.setOperationMode(OPERATION_MODE_NDOF);
    IMU.setUpdateMode(MANUAL);  //The default is AUTO. MANUAL requires calling
                                //update functions prior to read
    
    //initialize distance sensor pins
    for(int i = 0; i < numDistSensors; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
    initialYaw = IMU.readEulerHeading();

    mPID.SetOutputLimits(-400, 400);
    mPID.SetSampleTime(10);
    pidL.SetMode(AUTOMATIC);
    mPID.SetOutputLimits(-400, 400);
    mPID.SetSampleTime(10);
    pidR.SetMode(AUTOMATIC);
    mPID.SetOutputLimits(-5, 5);
    mPID.SetSampleTime(50);  // outer PID loop set to run ~5x slower than inner loop
    pidW.SetMode(AUTOMATIC);

    motorTicksPrevL = motorEncL.read();
    motorTicksPrevR = motorEncR.read();
    lastEncoderTime = millis();
    delay(10);
}

void loop() 
{
    wheelSpeedFeedback();
    
    updateSensorReadings();

    wallFollowingFeedback();

    gaitControl();

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

void wheelSpeedFeedback()
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
}

void updateSensorReadings()
{
    //send ultrasonic pulses
    digitalWrite(trigPins[currDistSensor], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPins[currDistSensor], LOW);
    //read time till pulse returns, convert time to distance (cm)
    unsigned long duration = pulseIn(echoPins[currDistSensor], HIGH, 5000);
    distances[currDistSensor] = double(duration) * M;
    if (distances[currDistSensor] == 0.0) { // ping never returned
        distances[currDistSensor] = 150.0;
    }
    currDistSensor = (currDistSensor + 1) % numDistSensors;

    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus(); // Update the Calibration Status
}

void wallFollowingFeedback()
{
    //right wall following
    rightWallDistance = distances[RIGHT_SIDE];
    pidW.Compute();
    if(rightWallDistance < 120.0) {
        wallDetected = true;
    }
}

void gaitControl()
{
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
            // TODO: include deadband region
            if (wallSteeringAmt > 0) {
                desiredVelL = nominalForwardSpeed - abs(wallSteeringAmt);
            } else if (wallSteeringAmt < 0) {
                desiredVelR = nominalForwardSpeed - abs(wallSteeringAmt);
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
            delay(1000); // bad
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

        if (wallDetected) {
            gait = 1;
        }
        break;
    }
}

// turning speed is always less than nominal speed
// robot turns in the direction of the slower wheel
double turningSpeed(double turningRadius)
{
    double Vl = nominalForwardSpeed;
    double a = (2 * turningRadius / wheelBase);
    double Vr = (a + 1) * Vl / (a - 1);
    return constrain(Vr, 0, nominalForwardSpeed);
}

// time to achieve a certain change in heading
double turningTime(double turningSpeed, double dHeading)
{
    double dV = nominalForwardSpeed - turningSpeed;
    double dt = dHeading * wheelBase / dV;
    return dt;
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

