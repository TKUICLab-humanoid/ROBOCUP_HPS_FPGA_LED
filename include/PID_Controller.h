#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

/*******************Debug Define******************************/
/* Turn debugging on */
//#define PID_control_Debug
/********************************************************/

#endif /*PID_CONTROLLER_H_*/
//enum PID_Index{Pelvis};

double PID_control(int index, double desired_value, double actual_value, double iteration_time);
void PID_Initail();
void PID_Parameters(int index, double KP, double KI, double KD);
void PID_clean(int index);

double PID_control_motor(int index, double desired_value, double actual_value, double iteration_time);
void PID_Initail_motor();
void PID_Parameters_motor(int index, double KP, double KI, double KD);
void PID_clean_motor(int index);

