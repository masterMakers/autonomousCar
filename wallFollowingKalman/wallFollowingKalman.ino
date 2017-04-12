#include <MatrixMath.h>
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
double yaw = 0.0;
double zAccel;
double pathHeading;
double desiredY = 0.5; //center of hallway (m)
double turnAmount;
double currentY;
double desiredYaw;

//kalman filter
float leftWheelDelta;
float rightWheelDelta;
float leftWheelRevsPrevious;
float rightWheelRevsPrevious;
//float control_cov[2][2];
//float measure_cov[5][5];
//float last_pose[3];
//float last_pose_cov[3][3];
// float measurement[5];
// float control_input[2];

/*************************************************************************************
==== Define sigma^2 ===
*************************************************************************************/

float sig_ultrasonic = pow(0.08, 2);
float sig_imu = pow(2.5*3.14/180, 2);
float sig_enc = pow(0.005, 2);
  
/*************************************************************************************
==== dimension of vehicle ===
*************************************************************************************/
float wheel_base = 10*0.0254;
float track_width = 10.25*0.0254;
  
/*************************************************************************************
==== Initialization ====
*************************************************************************************/
float control_cov[2][2] = {{sig_enc, 0}, {0, sig_enc}};
float measure_cov[5][5] = {{sig_ultrasonic, 0, 0, 0, 0}, {0, sig_ultrasonic, 0, 0, 0}, {0, 0, sig_ultrasonic, 0, 0}, {0, 0, 0, sig_ultrasonic, 0}, {0, 0, 0, 0, sig_imu}};

float last_pose[3] = {0 , 0.4 , 0};
float last_pose_cov[3][3] = {{pow(0.0, 2), 0, 0}, {0, pow(0.02,2), 0}, {0, 0, pow(0.1,2)}};


PID pidHall(&currentY, &turnAmount, &desiredY, 
        0.05, 0.0, 0.0, DIRECT); 
        // args: input, output, setpoint, Kp, Ki, Kd, mode

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
    pathHeading = yaw;

    double K = 1000.0 / (240 / (0.1524 * 3.14159));
                // (ms/s) / (ticksPerMeter)
                // ticksPerMeter = ticksPerRev / MetersPerRev
    pidL.SetOutputLimits(-400, 400);  // -400 for backwards motion
    pidL.SetSampleTime(50);
    pidL.SetWheelParam(K);
    pidL.SetMode(AUTOMATIC);
    pidR.SetOutputLimits(-400, 400);  // -400 for backwards motion
    pidR.SetSampleTime(50);
    pidR.SetWheelParam(K);
    pidR.SetMode(AUTOMATIC);
    pidHall.SetOutputLimits(-3, 3);
    pidHall.SetSampleTime(200);  // outer PID loop set to run ~4x slower than inner loop
    pidHall.SetMode(AUTOMATIC);

    delay(10);
}

void loop() 
{
    wheelSpeedFeedback();
    
    updateSensorReadings();
    
//    long tic = millis();
    KalmanFilterUpdate();
//    long toc = millis() - tic;
//    Serial.println(toc);
    gaitControl();

    hallFeedback();

    if(debug) {
//        Serial.print(" Yaw: ");
//        Serial.print(yaw); //Heading, deg
//        Serial.print(" Des_yaw: ");
//        Serial.print(desiredYaw); //deg
//        Serial.print(" L_Des_Vel: ");
//        Serial.print(desiredVelL);
//        Serial.print(" R_Des_Vel: ");
//        Serial.print(desiredVelR);
//        Serial.print(" Gait:");
//        Serial.print(currGait);    
//        Serial.print(" Left_sensor:");
//        Serial.print(distances[LEFT_SIDE]);
//        Serial.print(" Right_sensor:");
//        Serial.print(distances[RIGHT_SIDE]);
          Matrix.Print(last_pose, 1, 3, "last_pose");
          Serial.print(" IMU_Yaw: ");
          Serial.print(yaw);
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
    IMU.updateAccel();
    yaw = IMU.readEulerHeading();
    yaw = fmod(yaw, 360.0);
    if(yaw >180.0)
      yaw = -(yaw - 360.0)*3.14/180.0;
    else
      yaw = -(yaw)*3.14/180.0; 
    
    zAccel = IMU.readAccelZ();
    
    //update wheel positions
    float leftWheelRevs = motorEncL.read() / 240.0;
    float rightWheelRevs = motorEncR.read() / 240.0;
    
    leftWheelDelta = leftWheelRevs - leftWheelRevsPrevious;
    rightWheelDelta = rightWheelRevs - rightWheelRevsPrevious;
    
    leftWheelRevsPrevious = leftWheelRevs;
    rightWheelRevsPrevious = rightWheelRevs;
}

void hallFeedback()
{
    pidHall.Compute();
    desiredVelL = nominalForwardSpeed;
    desiredVelR = nominalForwardSpeed;

    if (turnAmount > 0.0) {
        desiredVelR = nominalForwardSpeed + turnAmount;
    } else if (turnAmount < 0.0) {
        desiredVelL = nominalForwardSpeed + turnAmount;
    }
}

void KalmanFilterUpdate()
{
	/*************************************************************************************
	==== Read measurement data ====
	*************************************************************************************/
	/*************************************************************************************
	==== Read Control Input ====
	*************************************************************************************/
	/*************************************************************************************
	Prediction Step
	*************************************************************************************/
  
	float motion_left = leftWheelDelta*2.0*3.14*6.0/2.0*0.0254;
	float motion_right = rightWheelDelta*2.0*3.14*6.0/2.0*0.0254;

	float relative_displacement = (motion_right - motion_left)/track_width;
	float squareRootTerm = pow(track_width,2) - pow((motion_right - motion_left),2);
	float B[3][2] = {{cos(last_pose[2])/2, cos(last_pose[2])/2}, {sin(last_pose[2])/2, sin(last_pose[2])/2}, {-1/sqrt(squareRootTerm), 1/sqrt(squareRootTerm)}};
  
	float pose_pre[3] = {last_pose[0] + (motion_left + motion_right)*cos(last_pose[2])/2, last_pose[1] + (motion_left + motion_right)*sin(last_pose[2])/2, last_pose[2] + asin(relative_displacement)};

	float temp[3][2];

	float B_transpose[2][3];
	Matrix.Transpose((float*)B, 2, 3, (float*)B_transpose);

	float pose_cov_from_control[3][3];

	Matrix.Multiply((float*)B, (float*)control_cov, 3, 2, 2, (float*)temp);
	Matrix.Multiply((float*)temp, (float*)B_transpose, 3, 2, 3, (float*)pose_cov_from_control);


	float pose_cov_pre[3][3];
	Matrix.Add((float*)last_pose_cov, (float*)pose_cov_from_control, 3, 3, (float*)pose_cov_pre);

	/*************************************************************************************
	Correction Step
	*************************************************************************************/
	float distance_left = distances[LEFT_SIDE]/100.0;
	float distance_right = distances[RIGHT_SIDE]/100.0;
	float yaw_angle = yaw;
  
	float Z[3] = {pose_pre[1] + distance_left*cos(pose_pre[2]) + track_width/2*cos(pose_pre[2]), pose_pre[1] - distance_right*cos(pose_pre[2]) - track_width/2*cos(pose_pre[2]), pose_pre[2] - yaw_angle};
	float Z_obs[3] = {0.8, 0, 0};
  
	float C[3][3] = {{0, 1, -(distance_left + track_width/2)*sin(pose_pre[2])}, {0, 1, (distance_right + track_width/2)*sin(pose_pre[2])}, {0, 0, 1}};
	float D[3][5] = {{cos(pose_pre[2]), 0, 0, 0, 0,}, {0, -cos(pose_pre[2]), 0, 0, 0}, {0, 0, 0, 0, -1}};
//  Matrix.Print((float*)C, 3, 3, "C");
	Matrix.Multiply((float*)temp, (float*)B_transpose, 3, 2, 3, (float*)pose_cov_from_control);

	/*************************************************************************************
	Kalman Update
	K_matrix = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
	*************************************************************************************/

	float K_matrix[3][3];
	KalmanFilterConstant(3, 3, 5, (float*)C, (float*)D, (float*)pose_cov_pre, (float*)measure_cov, (float*)K_matrix);
//  Matrix.Print((float*)K_matrix, 3, 3, "K_matrix");
	/*************************************************************************************
	Update Pose
	x = x_pre + K*(Z_obs - Z);
	*************************************************************************************/
	float pose_update[3];
	float new_pose[3];
	float error[3];

	Matrix.Subtract(Z_obs, Z, 3, 1, error);
//  Matrix.Print(error, 1, 3, "error");
  
	Matrix.Multiply((float*)K_matrix, error, 3, 3, 1, pose_update);
//  Matrix.Print(pose_update, 1, 3, "pose_update");
 
	Matrix.Add(pose_pre, pose_update, 3, 1, new_pose);
	Matrix.Copy(new_pose, 3, 1, last_pose);
//  Matrix.Print(new_pose, 1, 3, "new_pose");
	/*************************************************************************************
	Update Pose_Covariance
	P = (eye(3) - K*C)*P_pre;
	*************************************************************************************/
	float identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	float K_multiply_C[3][3];
	Matrix.Multiply((float*)K_matrix, (float*)C, 3, 3, 3, (float*)K_multiply_C);

	float multiplier[3][3];
	Matrix.Subtract((float*)identity, (float*)K_multiply_C, 3, 3, (float*)multiplier);

	float new_pose_cov[3][3];
	Matrix.Multiply((float*)multiplier, (float*)pose_cov_pre, 3, 3, 3, (float*)new_pose_cov);
	Matrix.Copy((float*)new_pose_cov, 3, 3, (float*)last_pose_cov);
}

void KalmanFilterConstant(int a, int b, int c, float* C, float* D, float* pose_cov_pre, float* measure_cov, float* K_matrix)
{
	// C is a X b
	// D is a X c
	// pose_cov_pre is b X b
	// measure_cov is c X c

	// Using K_matrix = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
	// or, K_matrix = P_pre*C' * inverseTermEKF
	// where inverseTermEKF = inverse(firstTerm + secondTerm);
	// firstTerm = C * P_pre * C';
	// secondTerm = D*measure_cov*D';

	float C_transpose[b][a];
	Matrix.Transpose((float*)C, a, b, (float*)C_transpose);
//  Matrix.Print((float*)C_transpose, 3, 3, "C_transpose");
  
	float D_transpose[c][a];
	Matrix.Transpose((float*)D, a, c, (float*)D_transpose);
//  Matrix.Print((float*)D_transpose, 3, 3, "D_transpose");
  
	float C_multiply_P_cov_pre[a][b];
	float firstTerm[a][a];
  
	Matrix.Multiply((float*)C, (float*)pose_cov_pre, a, b, b, (float*)C_multiply_P_cov_pre); // a X b
	Matrix.Multiply((float*)C_multiply_P_cov_pre, (float*)C_transpose, a, b, a, (float*)firstTerm); // a X a
//  Matrix.Print((float*)firstTerm, 3, 3, "firstTerm");

  
  
	float D_multiply_measure_cov[a][c];
	float secondTerm[a][a];

	Matrix.Multiply((float*)D, (float*)measure_cov, a, c, c, (float*)D_multiply_measure_cov); // a X c
	Matrix.Multiply((float*)D_multiply_measure_cov, (float*)D_transpose, a, c, a, (float*)secondTerm); // a X a

	float inverseTermEKF[a][a];
	Matrix.Add((float*)firstTerm, (float*)secondTerm, a, a, (float*)inverseTermEKF);
	Matrix.Invert((float*)inverseTermEKF, a);

	float P_cov_pre_multiply_C_transpose[b][a];
	Matrix.Multiply((float*)pose_cov_pre, (float*)C_transpose, b, b, a, (float*)P_cov_pre_multiply_C_transpose); // b X a
	Matrix.Multiply((float*)P_cov_pre_multiply_C_transpose, (float*)inverseTermEKF, b, a, a, (float*)K_matrix); // b X a

}

void wallFollowing()
{
   
}

void gaitControl()
{
    // finite state automaton
    switch(currGait) {
    case STRAIGHT:
        wallFollowing();

        if(distances[RIGHT_SIDE] > 120.0) {
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


