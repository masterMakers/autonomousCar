#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#define LIBRARY_VERSION 0.0.1

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <MatrixMath.h>

class KalmanFilter
{
    public:

    KalmanFilter(float (&pose_)[3], float (&pose_cov_)[3][3], float (&measure_)[5], float (&control_input_)[2]);
    void init(float, float, float, float, float, float, float);

    void update(bool (&sensor_flags)[5]);      
                        
    float degToRad(float);

    void printPose();

    void reset();


    private:
    const float identity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float control_cov[2][2]; // initialized to zero
    float measure_cov_full[5][5];
    float pose_initial[3];
    float pose_cov_initial[3][3];

    float (&control_input)[2];
    float (&measure)[5];
    float (&pose)[3];
    float (&pose_cov)[3][3];

    float wheel_base;
    float track_width;
    float wheel_radius;
    float margins;
};
#endif // KALMAN_FILTER_H
