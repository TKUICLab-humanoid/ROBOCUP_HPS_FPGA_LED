#include "PID_Controller.h"
#include "stdio.h"

#define max(x,y)  ( x>y?x:y )
#define min(x,y)  ( x<y?x:y )

struct Points_Struct{
    double KP;
    double KI;
    double KD;
    
    double error_prior;
    double integral;
    double PID_bias;
    
    double PID_error;
    double derivative;
    double output;
}PID[3],PID_motor[21];
//-----------------Fuzzy---------------//
double PID_control(int index, double desired_value, double actual_value, double iteration_time){
    
    PID[index].PID_error = desired_value - actual_value;
    PID[index].integral = PID[index].integral + (PID[index].PID_error*iteration_time);
    PID[index].derivative = (PID[index].PID_error - PID[index].error_prior)/iteration_time;
    
    PID[index].output = PID[index].KP*PID[index].PID_error + PID[index].KI*PID[index].integral + PID[index].KD*PID[index].derivative + PID[index].PID_bias;
    PID[index].error_prior = PID[index].PID_error;
    #ifdef PID_control_Debug
        //printf("desired_value: %.1f\tactual_value: %.1f\titeration_time: %.1f\n",desired_value,actual_value,iteration_time);
        printf("PID_error: %.1f\tintegral: %.1f\tderivative: %.1f\toutput: %.1f\n",PID[index].PID_error,PID[index].integral,PID[index].derivative,PID[index].output);
    #endif
    return PID[index].output;
}
void PID_Initail(int index, double KP, double KI, double KD){
    PID_clean(index);
    PID_Parameters(index,KP,KI,KD);
}

void PID_Parameters(int index, double KP, double KI, double KD){
    PID[index].KP = KP;
    PID[index].KI = KI;
    PID[index].KD = KD;
}

void PID_clean(int index){
    PID[index].integral = 0;
    PID[index].derivative = 0;
    PID[index].error_prior = 0;
}

//-----------------Fuzzy for motor---------------//
double PID_control_motor(int index, double desired_value, double actual_value, double iteration_time){
    
    PID_motor[index].PID_error = desired_value - actual_value;
    PID_motor[index].integral = PID_motor[index].integral + (PID_motor[index].PID_error*iteration_time);
    PID_motor[index].derivative = (PID_motor[index].PID_error - PID_motor[index].error_prior)/iteration_time;
    
    PID_motor[index].output = PID_motor[index].KP*PID_motor[index].PID_error + PID_motor[index].KI*PID_motor[index].integral + PID_motor[index].KD*PID_motor[index].derivative + PID_motor[index].PID_bias;
    PID_motor[index].error_prior = PID_motor[index].PID_error;
    #ifdef PID_control_Debug
        //printf("desired_value: %.1f\tactual_value: %.1f\titeration_time: %.1f\n",desired_value,actual_value,iteration_time);
        printf("PID_error: %.1f\tintegral: %.1f\tderivative: %.1f\toutput: %.1f\n",PID_motor[index].PID_error,PID_motor[index].integral,PID_motor[index].derivative,PID_motor[index].output);
    #endif
    return PID_motor[index].output;
}
void PID_Initail_motor(int index, double KP, double KI, double KD){
    PID_clean_motor(index);
    PID_Parameters_motor(index,KP,KI,KD);
}

void PID_Parameters_motor(int index, double KP, double KI, double KD){
    PID_motor[index].KP = KP;
    PID_motor[index].KI = KI;
    PID_motor[index].KD = KD;
}

void PID_clean_motor(int index){
    PID_motor[index].integral = 0;
    PID_motor[index].derivative = 0;
    PID_motor[index].error_prior = 0;
}
