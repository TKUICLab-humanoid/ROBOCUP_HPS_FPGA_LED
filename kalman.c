/*
 * kalman.c
 *
 *  Created on: 2014/7/23
 *      Author: MinYi
 */
#include "include/kalman.h"

KalmanFilter::KalmanFilter()
{
    int i=0;
    q_angle_ = 0.001;
    q_bias_ = 0.003;
    r_measure_ = 0.0003;    //0.0005

    memset(angle_, 0.0, sizeof(angle_));
    memset(bias_, 0.0, sizeof(bias_));

    force_q_ = 0.001;
    force_w_ = 0.0003;    //0.0005

    memset(force_r_, 0.0, sizeof(force_r_));
    memset(force_x_, 0.0, sizeof(force_x_));
    memset(force_p_, 0.0, sizeof(force_p_));
    memset(force_k_, 0.0, sizeof(force_k_));
}

KalmanFilter::~KalmanFilter()
{
}

double KalmanFilter::get_angle(double acc_angle_tmp, double gyro_angle_tmp, double dt, int i)
{
    double s, y;
    angle_[i] += (gyro_angle_tmp - q_bias_) * dt;

    p_[i][0][0] += dt * (dt * p_[i][1][1] - p_[i][0][1] - p_[i][1][0] + q_angle_);
    p_[i][0][1] -= dt * p_[i][1][1];
    p_[i][1][0] -= dt * p_[i][1][1];
    p_[i][1][1] += q_angle_ * dt;

    y = acc_angle_tmp - angle_[i];

    s = p_[i][0][0] + r_measure_;
    k_[0] = p_[i][0][0] / s;  //k = K Gain;
    k_[1] = p_[i][1][0] / s;

    angle_[i] += k_[0] * y;
    bias_[i] += k_[1] * y;

    p_[i][0][0] -= k_[0] * p_[i][0][0];
    p_[i][0][1] -= k_[0] * p_[i][0][1];
    p_[i][1][0] -= k_[1] * p_[i][0][0];
    p_[i][1][1] -= k_[1] * p_[i][0][1];

    return angle_[i];
}

double KalmanFilter::get_force(double Force_data, int i){
    force_x_[i] = force_x_[i] + force_w_;
    force_p_[i] = force_p_[i] + force_q_;
    
    force_k_[i] = force_p_[i] / (force_p_[i] + force_r_[i]);
    
    force_x_[i] = force_x_[i] + force_k_[i]*(Force_data - force_x_[i]);
    force_p_[i] = (1 - force_k_[i])*force_p_[i];
    
    return force_x_[i];
}

double KalmanFilter::get_q_bias()
{
    return q_bias_;
}

//*****single input - single output kalman system*****//
struct SISO_kal_sys{
    float A, B, H, Q, R, X, P;
    float K;
}Para_SISO_kal[7];
int SISO_kalman_ini(int index, float A, float B, float H, float Q, float R, float X, float P){
    Para_SISO_kal[index].A = A;
    Para_SISO_kal[index].B = B;
    Para_SISO_kal[index].H = H;
    Para_SISO_kal[index].Q = Q;
    Para_SISO_kal[index].R = R;
    Para_SISO_kal[index].X = X;
    Para_SISO_kal[index].P = P;
    return 0;
}
float SISO_kalman(int index, float input){
    Para_SISO_kal[index].K = Para_SISO_kal[index].P / (Para_SISO_kal[index].P + Para_SISO_kal[index].R);
    Para_SISO_kal[index].X = Para_SISO_kal[index].X + Para_SISO_kal[index].K * (input - Para_SISO_kal[index].X);
    Para_SISO_kal[index].P = (1 - Para_SISO_kal[index].K) * Para_SISO_kal[index].P + Para_SISO_kal[index].Q;
    
    return Para_SISO_kal[index].X;
}
//*****single input - single output kalman system*****//
