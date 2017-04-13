// TODO
// try to increase initial covariance of the heading, improve wallfollowing when initial heading is unknown
// turning should be better on wooden surface
// better cut off on turns, buffer yaw or something
// how to quickly stop when turn is detected
// test brake on ground, test -400 as brake
// test pid around yaw for pre jump centering
// try ramp jump
// 

#include <MatrixMath.h>
#include <KalmanFilter.h>
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
float distances[numDistSensors];
const float M = 1 / 29.0 / 2.0; // distance calibration
int currDistSensor = 0;

//Hall Following
float nominalForwardSpeed = 0.5;
double yaw = 0.0;
double pathHeading;
double desiredY = 0.6; //center of hallway (m)
double currentY, turnAmount, desiredYaw = 0.0;
PID pidHall(&currentY, &turnAmount, &desiredY, 0.5, 0.0, 0.0, DIRECT); // 0.5, 0.0, 0.0
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//Yaw Correction
PID pidYaw(&yaw, &turnAmount, &desiredYaw, 65.0, 8.0, 3.0, DIRECT); // 65.0, 8.0, 3.0
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//Ramp Detection
int reflSensorPin = A10;
int reflCounter = 0;
const int reflSize = 3;
float reflValues[reflSize];
float rampThreshold;
float maxSpeed = 5.0; //m/s
float zAccel;

//kalman filter
float leftWheelRevsPrevious;
float rightWheelRevsPrevious;
bool active_sensors[5] = {false, false, false, false, false};

float pose[3] = {0 , 0.6 , 0};
float pose_cov[3][3] = {{sq(0.0), 0, 0}, 
                        {0, sq(0.02), 0}, 
                        {0, 0, sq(0.1)}};
float measure[5];  // initialized to zero
float control_input[2];
KalmanFilter ekf(pose, pose_cov, measure, control_input);

//motor control
double motorVelL = 0.0, motorCmdL = 0.0, motorVelR = 0.0, motorCmdR = 0.0;
double desiredVelL = nominalForwardSpeed;
double desiredVelR = nominalForwardSpeed;

PID pidL(&motorVelL, &motorCmdL, &desiredVelL, 150.0, 80.0, 2.0, DIRECT);
PID pidR(&motorVelR, &motorCmdR, &desiredVelR, 150.0, 80.0, 2.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

//high level control
enum gait {
  STRAIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  JUMP
};
gait currGait = STRAIGHT;

enum jump {
  PREPARE,
  GO,
  JUMPING,
  IN_FLIGHT,
  POST_JUMP,
};
jump jumpState = PREPARE;

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

    //initialize reflectance sensor
    pinMode(reflSensorPin, INPUT);
    for(int i = 0; i < reflSize; i++) {
      reflValues[i] = 0.0;
    }
    rampThreshold = analogRead(reflSensorPin) * 1.1;
    
    //initialize heading
    yaw = ekf.degToRad( IMU.readEulerHeading() );
    pathHeading = yaw;

    float K = 1000.0 / (240 / (0.1524 * 3.14159));
                // (ms/s) / (ticksPerMeter)
                // ticksPerMeter = ticksPerRev / MetersPerRev
    pidL.SetOutputLimits(0, 400);  // -400 for backwards motion
    pidL.SetSampleTime(25); // 40 hz
    pidL.SetWheelParam(K);
    pidL.SetMode(AUTOMATIC);
    pidR.SetOutputLimits(0, 400);  // -400 for backwards motion
    pidR.SetSampleTime(25);
    pidR.SetWheelParam(K);
    pidR.SetMode(AUTOMATIC);
    pidHall.SetOutputLimits(-1, 1);
    pidHall.SetSampleTime(50);  // outer PID loop set to run ~2x slower than inner loop (20 hz)
    pidHall.SetMode(AUTOMATIC);
    pidYaw.SetOutputLimits(-350, 350);
    pidYaw.SetSampleTime(100);  // outer PID loop set to run ~4x slower than inner loop (10 hz)
    pidYaw.SetMode(AUTOMATIC);

    // sigma values
    float sig_ultrasonic = 0.08;
    float sig_imu = 2.5 * M_PI / 180.0;
    float sig_enc = 0.005;
    
    // dimension of vehicle
    float wheel_base = 10.0 * 0.0254;
    float margins = 4.0 * 0.0254;
    float track_width = 10.5 * 0.0254; 
    float wheel_radius = 0.0254 * 6.0 / 2.0;
    ekf.init(sig_ultrasonic, sig_imu, sig_enc, wheel_base, track_width, wheel_radius, margins);

    delay(10);
}

void loop() 
{  
    updateSensorReadings();
    
    kalmanUpdate();
   
    gaitControl();

    if(debug) {
      Serial.print(" L_Des_Vel: ");
      Serial.print(desiredVelL);
      Serial.print(" R_Des_Vel: ");
      Serial.print(desiredVelR);
      Serial.print(" Turn Amt: ");
      Serial.print(turnAmount);
      Serial.print(" X: ");
      Serial.print(pose[0]);
      Serial.print(" Y: ");
      Serial.print(currentY);
      Serial.print(" IMU Yaw: ");
      Serial.print(yaw); //Heading, deg
      Serial.print(" Desired Yaw: ");
      Serial.print(desiredYaw);
      Serial.print(" Current Gait: ");
      Serial.print(currGait);
//      ekf.printPose();
      Serial.print(" Left_sensor:");
      Serial.print(distances[LEFT_SIDE]);
      Serial.print(" Right_sensor:");
      Serial.print(distances[RIGHT_SIDE]);  
      Serial.println();
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
    currDistSensor = (currDistSensor + 1) % numDistSensors;
    //send ultrasonic pulses
    digitalWrite(trigPins[currDistSensor], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPins[currDistSensor], LOW);
    //read time till pulse returns, convert time to distance (cm)
    unsigned long duration = pulseIn(echoPins[currDistSensor], HIGH, 10000);
    distances[currDistSensor] = float(duration) * M;
    if (distances[currDistSensor] == 0.0) { // ping never returned
        distances[currDistSensor] = 150.0;
    }
        
    //update IMU information
    IMU.updateEuler();     
    IMU.updateCalibStatus(); // Update the Calibration Status
    IMU.updateAccel();
    yaw = ekf.degToRad( IMU.readEulerHeading() );
      
    zAccel = IMU.readAccelZ();
    
    //update wheel positions
    float leftWheelRevs = motorEncL.read() / 240.0;
    float rightWheelRevs = motorEncR.read() / 240.0;
    control_input[0] = leftWheelRevs - leftWheelRevsPrevious; // leftWheelDelta
    control_input[1] = rightWheelRevs - rightWheelRevsPrevious; // rightWheelDelta
    leftWheelRevsPrevious = leftWheelRevs;
    rightWheelRevsPrevious = rightWheelRevs;
}

void hallFollowing()
{
    currentY = pose[1];
    pidHall.Compute();
    desiredVelL = nominalForwardSpeed;
    desiredVelR = nominalForwardSpeed;
    if (turnAmount > 0.0) {
        desiredVelR = nominalForwardSpeed + abs(turnAmount);
    } else if (turnAmount < 0.0) {
        desiredVelL = nominalForwardSpeed + abs(turnAmount);
    }
}

void kalmanUpdate()
{
    for (int i = 0; i < 2; i++) {
        active_sensors[i] = (distances[i] < 140 && distances[i] > 0.5);// && i == currDistSensor);
    }
    active_sensors[2] = false;
    active_sensors[3] = false;
    active_sensors[4] = true;

    measure[0] = distances[LEFT_SIDE] / 100.0;
    measure[1] = distances[RIGHT_SIDE] / 100.0;
    measure[2] = distances[BACK] / 100.0;
    measure[3] = distances[FRONT] / 100.0;
    measure[4] = yaw;
    
    ekf.update(active_sensors);
}

void gaitControl()
{
    // finite state automaton
    switch(currGait) {
    case STRAIGHT:
        hallFollowing();
        wheelSpeedFeedback();
        
//        if(rampDetection()) {
//            currGait = JUMP;
//        }
        if((distances[RIGHT_SIDE] > 120.0) && (pose[0] > 1.5)) {
            currGait = TURN_RIGHT;
            desiredYaw = yaw - (M_PI / 2.0);
            motorDriver.setBrakes(400,400);
            delay(1000);
        } else if((distances[LEFT_SIDE] > 120.0) && (pose[0] > 1.5)) {
            currGait = TURN_LEFT;
            desiredYaw = yaw + (M_PI / 2.0);
            motorDriver.setBrakes(400,400);
            delay(1000);
        }
        break;
    case TURN_LEFT:
        if (turnLeft()) {
          reset(); 
        }
        break;
    case TURN_RIGHT:
        if (turnRight()) {
          reset(); 
        }
        break;
    case JUMP:
        wheelSpeedFeedback();
        if (doJump()) {
           currGait = STRAIGHT; 
        }
        break;
    }
}

bool rampDetection()
{
    int j = reflCounter % reflSize;
    reflValues[j] = analogRead(reflSensorPin);
    
    //filter out obviouse noise
    if (reflValues[j] > 1000) {
      reflValues[j] = reflValues[(reflCounter - 1) % reflSize];
    }
    //rolling average filter for smoothing
    float reflSmooth = 0.0;
    for(int i = 0; i < reflSize; i++) {
      reflSmooth += reflValues[i];
    }
    reflSmooth = reflSmooth / reflSize;
    
    //check for ramp
    reflCounter++;
    
    if(reflSmooth > rampThreshold) {
      return true; 
    }
    return false;
}

bool doJump()
{     
    switch(jumpState) {
    case PREPARE:
        if (correctYaw()) {
            jumpState = GO;
        }
    case GO:
        desiredVelL = maxSpeed;
        desiredVelR = maxSpeed;
        if (zAccel > 13) {
            jumpState = JUMPING;
        }
        break;
    case JUMPING:
        if (zAccel < 2) {
            jumpState = IN_FLIGHT;
        }
        break;
    case IN_FLIGHT:
        desiredVelL = 0.0;
        desiredVelR = 0.0;

        if (zAccel > 15) {
            jumpState = POST_JUMP;
        }
        break;
    case POST_JUMP:
        if (zAccel < 13) {
            delay(3000); //wait for bouncing and vibrations to stop
            return true;
        }
        break;
    }
    return false;
}

bool correctYaw()
{
    pidYaw.Compute();
    desiredVelL = turnAmount;
    desiredVelR = -turnAmount;
    //TODO: figure out when turn is over
    return false;
}

bool turnRight()
{
    pidYaw.Compute();
    motorDriver.setM1Speed(0); //right, motor command [-400 400]
    motorDriver.setM2Speed(-turnAmount); //left
    if (fabs(yaw - desiredYaw) < 5.0*(M_PI/180)) {
        return true;
    }
    return false;
}

bool turnLeft()
{
    pidYaw.Compute();
    motorDriver.setM1Speed(turnAmount); //right, motor command [-400 400]
    motorDriver.setM2Speed(0); //left
    if (fabs(yaw - desiredYaw) < 5.0*(M_PI/180)) {
        return true;
    }
    return false;
}

void reset()
{
    // reset pose and pose_cov
    ekf.reset();

    //reset relfectance array
    for(int i = 0; i < reflSize; i++) {
      reflValues[i] = 0.0;
    }
    
    //reset motor commands    
    motorDriver.setM1Speed(0); //right
    motorDriver.setM2Speed(0); //left
    
    //reset gait
    currGait = STRAIGHT;

    //reset IMU
    IMU.resetSensor(0x28); // default address
    //IMU.resetInterrupt();
    IMU.initSensor(); // I2C Address can be changed here
    IMU.setOperationMode(OPERATION_MODE_NDOF);
    IMU.setUpdateMode(MANUAL);  //The default is AUTO. MANUAL requires calling
                                //update functions prior to read
    delay(500);
    yaw = ekf.degToRad( IMU.readEulerHeading() );
    pathHeading = yaw;
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
