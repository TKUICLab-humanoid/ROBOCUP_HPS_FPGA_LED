#include "Rmotor.h"

int Motor_Data[21] = {0};
void Get_Motor_Data()
{
    Motor_Data[0] = (int)IORD(GPIO_RD_MOTOR_1_BASE,0);
    Motor_Data[1] = (int)IORD(GPIO_RD_MOTOR_2_BASE,0);
    Motor_Data[2] = (int)IORD(GPIO_RD_MOTOR_3_BASE,0);
    Motor_Data[3] = (int)IORD(GPIO_RD_MOTOR_4_BASE,0);
    Motor_Data[4] = (int)IORD(GPIO_RD_MOTOR_5_BASE,0);
    Motor_Data[5] = (int)IORD(GPIO_RD_MOTOR_6_BASE,0);
    Motor_Data[6] = (int)IORD(GPIO_RD_MOTOR_7_BASE,0);
    Motor_Data[7] = (int)IORD(GPIO_RD_MOTOR_8_BASE,0);
    Motor_Data[8] = (int)IORD(GPIO_RD_MOTOR_9_BASE,0);
    Motor_Data[9] = (int)IORD(GPIO_RD_MOTOR_10_BASE,0);
    Motor_Data[10] = (int)IORD(GPIO_RD_MOTOR_11_BASE,0);
    Motor_Data[11] = (int)IORD(GPIO_RD_MOTOR_12_BASE,0);
    Motor_Data[12] = (int)IORD(GPIO_RD_MOTOR_13_BASE,0);
    Motor_Data[13] = (int)IORD(GPIO_RD_MOTOR_14_BASE,0);
    Motor_Data[14] = (int)IORD(GPIO_RD_MOTOR_15_BASE,0);
    Motor_Data[15] = (int)IORD(GPIO_RD_MOTOR_16_BASE,0);
    Motor_Data[16] = (int)IORD(GPIO_RD_MOTOR_17_BASE,0);
    Motor_Data[17] = (int)IORD(GPIO_RD_MOTOR_18_BASE,0);
    Motor_Data[18] = (int)IORD(GPIO_RD_MOTOR_19_BASE,0);
    Motor_Data[19] = (int)IORD(GPIO_RD_MOTOR_20_BASE,0);
    Motor_Data[20] = (int)IORD(GPIO_RD_MOTOR_21_BASE,0);
}
