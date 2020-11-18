/*
 * KalmanFilter.h
 *
 *  Created on: 2014/7/23
 *      Author: MinYi
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include <string.h>

class KalmanFilter
{
public:
    KalmanFilter();
    ~KalmanFilter();

    double get_angle(double, double, double, int);
    double get_force(double, int);
    double get_q_bias();

private:
    double p_[3][2][2];
    double k_[2];

    double q_angle_;
    double q_bias_;
    double r_measure_;

    double angle_[3];
    double bias_[3];

    double force_r_[8];
    double force_q_;
    double force_w_;  

    double force_x_[8];
    double force_p_[8];
    double force_k_[8];
};

// extern double q_bias;
// double get_angle(float acc_angle_tmp, float gyro_angle_tmp, double dt, int i);

// extern double Force_r[8];

float get_Force(float Force_data, int i);

//*****single input - single output kalman system*****//
enum PID_Index{Pelvi,RX,RY,RZ,LX,LY,LZ};
int SISO_kalman_ini(int index, float A, float B, float H, float Q, float R, float X, float P);
float SISO_kalman(int index, float input);
//*****single input - single output kalman system*****//

#endif /* KALMANFILTER_H_ */
