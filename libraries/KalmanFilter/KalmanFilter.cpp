/**********************************************************************************************
 * Arduino Kalman Filter Library - Version 0.0.1
 * by Puneet Singhal and Bereket Abraham
 *
 * Only supports a differential drive robot with 4 ultrasonic sensors and yaw
 * Depends on the MatrixMath library
 **********************************************************************************************/

#include <KalmanFilter.h>

KalmanFilter::KalmanFilter(float (&pose_)[3], float (&pose_cov_)[3][3], float (&measure_)[5], float (&control_input_)[2])
                          : pose(pose_), pose_cov(pose_cov_), measure(measure_), control_input(control_input_) 
{ 
    memcpy(pose_initial, pose, sizeof(float) * 3);
    memcpy(pose_cov_initial, pose_cov, sizeof(float) * 3 * 3);
}

void KalmanFilter::init(float sig_ultrasonic, float sig_imu, float sig_enc,
        float wheel_base_, float track_width_, float wheel_radius_, float margins_)
{
    sig_ultrasonic = sq(sig_ultrasonic);
    sig_imu = sq(sig_imu);
    sig_enc = sq(sig_enc);

    control_cov[0][0] = sig_enc;
    control_cov[1][1] = sig_enc;

    measure_cov_full[0][0] = sig_ultrasonic;
    measure_cov_full[1][1] = sig_ultrasonic;
    measure_cov_full[2][2] = sig_ultrasonic;
    measure_cov_full[3][3] = sig_ultrasonic;
    measure_cov_full[4][4] = sig_imu;

    wheel_base = wheel_base_;
    track_width = track_width_;
    wheel_radius = wheel_radius_;
    margins = margins_;
}

float KalmanFilter::degToRad(float angle)
{
    angle = fmod(angle, 360.0);
    if(angle > 180.0) {
        angle = -(angle - 360.0) * M_PI / 180.0;
    } else {
        angle = -angle * M_PI / 180.0;   
    }
    return angle;
}

void KalmanFilter::printPose()
{
    Matrix.Print(pose, 1, 3, "pose");
}

void KalmanFilter::reset()
{
    memcpy(pose, pose_initial, sizeof(float) * 3);
    memcpy(pose_cov, pose_cov_initial, sizeof(float) * 3 * 3);
}

void KalmanFilter::update(bool (&sensor_flags)[5])
{
    /*************************************************************************************
    Prediction Step
    *************************************************************************************/
    float motion_left = control_input[0] * 2.0 * M_PI * 6.0 / 2.0 * 0.0254;
    float motion_right = control_input[1] * 2.0 * M_PI * 6.0 / 2.0 * 0.0254;

    float relative_displacement = (motion_right - motion_left) / track_width;
    float squareRootTerm = sq(track_width) - sq((motion_right - motion_left));
    float B[3][2] = {{cos(pose[2]) / 2.0, cos(pose[2]) / 2.0}, 
                     {sin(pose[2]) / 2.0, sin(pose[2]) / 2.0}, 
                     {-1.0 / sqrt(squareRootTerm), 1.0 / sqrt(squareRootTerm)}};

    float pose_pre[3] = {pose[0] + ((motion_left + motion_right) * cos(pose[2]) / 2.0), 
                         pose[1] + ((motion_left + motion_right) * sin(pose[2]) / 2.0), 
                         pose[2] + asin(relative_displacement)};

    float B_transpose[2][3];
    Matrix.Transpose((float*)B, 2, 3, (float*)B_transpose);
    float temp[3][2];
    Matrix.Multiply((float*)B, (float*)control_cov, 3, 2, 2, (float*)temp);
    float pose_cov_from_control[3][3];
    Matrix.Multiply((float*)temp, (float*)B_transpose, 3, 2, 3, (float*)pose_cov_from_control);
    float pose_cov_pre[3][3];
    Matrix.Add((float*)pose_cov, (float*)pose_cov_from_control, 3, 3, (float*)pose_cov_pre);

  /*************************************************************************************
  Correction Step
  *************************************************************************************/
  
  float Z_full[5] = {pose_pre[1] + (measure[0] * cos(pose_pre[2])) + (cos(pose_pre[2]) * track_width / 2.0), 
                     pose_pre[1] - (measure[1] * cos(pose_pre[2])) - (cos(pose_pre[2]) * track_width / 2.0), 
                     pose_pre[0] - (measure[2] + margins) * cos(pose_pre[2]),
                     pose_pre[0] + (measure[3] + wheel_base) * cos(pose_pre[2]), 
                     pose_pre[2] - measure[4]};
  
    float Z_obs_full[5] = {1.0, 0, 0, 8.0, 0};
    float D_full[5][5] = {{cos(pose_pre[2]), 0, 0, 0, 0}, 
                          {0, -cos(pose_pre[2]), 0, 0, 0}, 
                          {0, 0, 0, -cos(pose_pre[2]), 0}, 
                          {0, 0, cos(pose_pre[2]), 0, 0},
                          {0, 0, 0, 0, -1.0}};
    float C_full[5][3] = {{0, 1.0, -(measure[0] + (track_width / 2.0)) * sin(pose_pre[2])}, 
                          {0, 1.0, (measure[1] + (track_width / 2.0)) * sin(pose_pre[2])}, 
                          {1.0, 0, (measure[2] + margins) * sin(pose_pre[2])}, 
                          {1.0, 0, -(measure[3] + wheel_base) * sin(pose_pre[2])}, 
                          {0, 0, 1.0}};

    int num_active = 0;
    for(int k = 0; k < 5; k++) {
        if(sensor_flags[k]) {
            num_active++;
        }
    }

    float measure_cov[num_active][num_active];
    float Z[num_active];
    float Z_obs[num_active];
    float C[num_active][3];
    float D[num_active][num_active];
    // Z, Z_obs, C -> cut rows
    // D, measure_cov -> cut rows and cols
    int row = 0;
    for (int i = 0; i < 5; i++) { // row
        if (!sensor_flags[i]) {
            continue;
        }

        int col = 0;
        for(int j = 0; j < 5; j++) { // col
            if (sensor_flags[j]) {
                D[row][col] = D_full[i][j];
                measure_cov[row][col] = measure_cov_full[i][j];
                col++;
            }
        }

        C[row][0] = C_full[i][0];
        C[row][1] = C_full[i][1];
        C[row][2] = C_full[i][2];
        Z[row] = Z_full[i];
        Z_obs[row] = Z_obs_full[i];
        row++;
    }

    /*************************************************************************************
    Kalman Update
    K_gain = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
    *************************************************************************************/
    /*
      C is num_active X 3
      D is num_active X num_active
      pose_cov_pre is 3 X 3
      measure_cov is num_active X num_active

      Using K_gain = P_pre * C' / (C * P_pre * C' + D*measure_cov*D');
      or, K_gain = P_pre*C' * inverseTermEKF
      where inverseTermEKF = inverse(firstTerm + secondTerm);
      firstTerm = C * P_pre * C';
      secondTerm = D*measure_cov*D';
    */

    float K_gain[3][num_active];
    float C_transpose[3][num_active];
    Matrix.Transpose((float*)C, num_active, 3, (float*)C_transpose);
    //  Matrix.Print((float*)C_transpose, 3, num_active, "C_transpose");

    float D_transpose[num_active][num_active];
    Matrix.Transpose((float*)D, num_active, num_active, (float*)D_transpose);
    //  Matrix.Print((float*)D_transpose, num_active, num_active, "D_transpose");

    float C_multiply_P_cov_pre[num_active][3];
    float firstTerm[num_active][num_active];

    Matrix.Multiply((float*)C, (float*)pose_cov_pre, num_active, 3, 3, (float*)C_multiply_P_cov_pre); // num_active X 3
    Matrix.Multiply((float*)C_multiply_P_cov_pre, (float*)C_transpose, num_active, 3, num_active, (float*)firstTerm); // num_active X num_active
    //  Matrix.Print((float*)firstTerm, num_active, num_active, "firstTerm");

    float D_multiply_measure_cov[num_active][num_active];
    float secondTerm[num_active][num_active];

    Matrix.Multiply((float*)D, (float*)measure_cov, num_active, num_active, num_active, (float*)D_multiply_measure_cov); // num_active X num_active
    Matrix.Multiply((float*)D_multiply_measure_cov, (float*)D_transpose, num_active, num_active, num_active, (float*)secondTerm); // num_active X num_active

    float inverseTermEKF[num_active][num_active];
    Matrix.Add((float*)firstTerm, (float*)secondTerm, num_active, num_active, (float*)inverseTermEKF);
    Matrix.Invert((float*)inverseTermEKF, num_active);

    float P_cov_pre_multiply_C_transpose[3][num_active];
    Matrix.Multiply((float*)pose_cov_pre, (float*)C_transpose, 3, 3, num_active, (float*)P_cov_pre_multiply_C_transpose); // 3 X num_active
    Matrix.Multiply((float*)P_cov_pre_multiply_C_transpose, (float*)inverseTermEKF, 3, num_active, num_active, (float*)K_gain); // 3 X num_active
    //  Matrix.Print((float*)K_gain, 3, num_active, "K_gain");

    /*************************************************************************************
    Update Pose
    x = x_pre + K*(Z_obs - Z);
    *************************************************************************************/
    float pose_update[3];
    float error[num_active];

    Matrix.Subtract((float*)Z_obs, (float*)Z, num_active, 1, (float*)error);
    //  Matrix.Print(error, 1, num_active, "error");

    Matrix.Multiply((float*)K_gain, (float*)error, 3, num_active, 1, (float*)pose_update);
    //  Matrix.Print(pose_update, 1, 3, "pose_update");

    Matrix.Add((float*)pose_pre, (float*)pose_update, 3, 1, (float*)pose);

    /*************************************************************************************
    Update Pose_Covariance
    P = (eye(3) - K*C)*P_pre;
    *************************************************************************************/
    float K_multiply_C[3][3];
    Matrix.Multiply((float*)K_gain, (float*)C, 3, num_active, 3, (float*)K_multiply_C);

    float multiplier[3][3];
    Matrix.Subtract((float*)identity, (float*)K_multiply_C, 3, 3, (float*)multiplier);

    Matrix.Multiply((float*)multiplier, (float*)pose_cov_pre, 3, 3, 3, (float*)pose_cov);
}

