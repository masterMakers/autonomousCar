#include <Encoder.h>
#include <Wire.h>
#include <PID_v1.h>
#include <NAxisMotion.h>
#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield motorDriver(11, 5, 13, A0, 7, 8, 12, A1);
Encoder motorEncL(2, 3);
Encoder motorEncR(18, 19);
NAxisMotion IMU;

//distance sensors
const int LEFT_SIDE = 0, RIGHT_SIDE = 1, FRONT = 2, BACK = 3, numDistSensors = 4;
const int trigPins[numDistSensors] = {24,26,28,32};  // TODO: fix back pins
const int echoPins[numDistSensors] = {25,27,29,33};
double distances[numDistSensors];
const double M = 1 / 29.0 / 2.0; // distance calibration
int currDistSensor = 0;

//wall following
const double wheelBase = 0.25; // m
double nominalForwardSpeed = 0.5;
double pathHeading = 0.0;
bool wallDetected = false;

double yaw = 0.0, wallSteeringAmt = 0.0, desiredYaw = 0.0;
PID pidW(&yaw, &wallSteeringAmt, &desiredYaw, 
        0.05, 0.0, 0.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//motor control
double motorVelL = 0.0, motorCmdL = 0.0, motorVelR = 0.0, motorCmdR = 0.0;
double desiredVelL = nominalForwardSpeed;
double desiredVelR = nominalForwardSpeed;

// kp = 150. ki = 15, kd = 2: tuned by Bereket on 6th April 2017
// kp = 150. ki = 80, kd = 2: tuned by Daniel and Puneet on 8th April 2017. 
// Gives close tracking and works for SetOutputLimits(-400, 400). 
// Tried to jump from -4.0 to +4.0 velocities and velocity changes in same direction
PID pidL(&motorVelL, &motorCmdL, &desiredVelL, 150.0, 80.0, 2.0, DIRECT);
PID pidR(&motorVelR, &motorCmdR, &desiredVelR, 150.0, 80.0, 2.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//high level control
enum gait {
  STRAIGHT,
  TURN_LEFT
};
gait currGait = STRAIGHT;

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
    // initialize heading
    yaw = IMU.readEulerHeading();
    desiredYaw = yaw;
    pathHeading = yaw;

    double K = 1000.0 / (240 / (0.1524 * 3.14159));
                // (ms/s) / (ticksPerMeter)
                // ticksPerMeter = ticksPerRev / MetersPerRev
    pidL.SetOutputLimits(0, 400);  // -400 for backwards motion
    pidL.SetSampleTime(50);
    pidL.SetWheelParam(K);
    pidL.SetMode(AUTOMATIC);
    pidR.SetOutputLimits(0, 400);  // -400 for backwards motion
    pidR.SetSampleTime(50);
    pidR.SetWheelParam(K);
    pidR.SetMode(AUTOMATIC);
    pidW.SetOutputLimits(-3, 3);
    pidW.SetSampleTime(200);  // outer PID loop set to run ~4x slower than inner loop
    pidW.SetMode(AUTOMATIC);

    delay(10);
}

void loop() 
{
    wheelSpeedFeedback();
    
    updateSensorReadings();

    gaitControl();

    yawFeedback();

    if(debug) {
        Serial.print(" Yaw: ");
        Serial.print(yaw); //Heading, deg
        Serial.print(" Des_yaw: ");
        Serial.print(desiredYaw); //deg
        Serial.print(" L_Des_Vel: ");
        Serial.print(desiredVelL);
        Serial.print(" R_Des_Vel: ");
        Serial.print(desiredVelR);
        Serial.print(" Wall_Detected: ");
        Serial.print(wallDetected);
        Serial.print(" Gait:");
        Serial.print(currGait);    
        Serial.print(" Left_sensor:");
        Serial.print(distances[LEFT_SIDE]);
        Serial.print(" Right_sensor:");
        Serial.println(distances[RIGHT_SIDE]);
    }
}

void wheelSpeedFeedback()
{
    pidL.ComputeVelocity(motorEncL.read());
    motorDriver.setM2Speed(motorCmdL);
    pidR.ComputeVelocity(motorEncR.read());
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
    unsigned long duration = pulseIn(echoPins[currDistSensor], HIGH, 10000);
    distances[currDistSensor] = double(duration) * M;
    if (distances[currDistSensor] == 0.0) { // ping never returned
        distances[currDistSensor] = 150.0;
    }
    currDistSensor = (currDistSensor + 1) % numDistSensors;
    
    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus(); // Update the Calibration Status
    yaw = IMU.readEulerHeading();
    
    if(distances[RIGHT_SIDE] < 120.0) {
        wallDetected = true;
    }
}

void yawFeedback()
{
    pidW.ComputeAngle();
    desiredVelL = nominalForwardSpeed;
    desiredVelR = nominalForwardSpeed;

    if (wallSteeringAmt > 0.0) {
        desiredVelL = nominalForwardSpeed - wallSteeringAmt;
    } else if (wallSteeringAmt < 0.0) {
        desiredVelR = nominalForwardSpeed + wallSteeringAmt;
    }
}

void wallFollowing()
{
    if(distances[RIGHT_SIDE] < 15.0) {
        desiredYaw = pathHeading + 10.0;
    } else if(distances[LEFT_SIDE] < 15.0) {
        desiredYaw = pathHeading - 10.0;
    } else {
        desiredYaw = pathHeading;
    }
}

void gaitControl()
{
    // finite state automaton
    switch(currGait) {
    case STRAIGHT:
        wallFollowing();

        if(distances[RIGHT_SIDE] > 120.0 && wallDetected) {
            currGait = TURN_LEFT;
            desiredYaw = pathHeading + 90.0;
        }
        break;
    case TURN_LEFT:
        if(abs(yaw - desiredYaw) < 5.0) {
            currGait = STRAIGHT;
            pathHeading = yaw;
        }
        break;
    }
}

// turning speed is always less than nominal speed
// robot turns in the direction of the slower wheel
double turningSpeed(double turningRadius)
{
    double V1 = nominalForwardSpeed;
    double a = (2 * turningRadius / wheelBase);
    double V2 = (a + 1) * V1 / (a - 1);
    return constrain(V2, 0, nominalForwardSpeed);
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


