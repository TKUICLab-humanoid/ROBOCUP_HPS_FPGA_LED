#ifndef RMOTOR_H_
#define RMOTOR_H_
/******************* Include libarary*********************/
//#include <stdio.h>
//#include <unistd.h>
//#include <io.h>
//#include "system.h"
//#include "altera_avalon_pio_regs.h"
//#include "altera_avalon_uart_regs.h"
//#include "sys/alt_flash.h"
//#include "sys/alt_irq.h"
//#include "sys/alt_alarm.h"
//#include "alt_types.h"
//#include "stdbool.h"
/********************************************************/
/******************* Include module**********************/
//#include "Fuzzy_shift.h"
//#include "Fuzzy_ZMP.h"
//#include "FlashModule.h"
//#include "Inverse_kinematic.h"
//#include "Feedback_Control.h"
//#include "PID_Controller.h"

/********************************************************/
extern int Motor_Data[21];
//extern int Uart_Flag;
/******************** Function **************************/
void Get_Motor_Data();
#endif /*RMOTOR_H_*/
