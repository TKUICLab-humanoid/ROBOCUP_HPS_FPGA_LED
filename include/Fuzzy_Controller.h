/*
 * Fuzzy_Controller.h
 *
 *  Created on: 2019/8/13
 *      Author: TKU_ICLab
 */

#ifndef FUZZY_CONTROLLER_H_
#define FUZZY_CONTROLLER_H_

/******************* Define******************************/
//#define fuzzy_debug    Turn debugging on
/********************************************************/

/******************* Parameter **************************/
//============ Fuzzy_X==================================
#define InputX_error_PB 4.4
#define InputX_error_PS 3
#define InputX_derror_PB 35.0
#define InputX_derror_PS 18.0

#define PB_X 0.25//0.07//0.015
#define PM_X 0.2//0.05//0.007
#define PS_X 0.15//0.03
#define ZE_X 0.0
#define NS_X -PS_X
#define NM_X -PM_X
#define NB_X -PB_X
//============ Fuzzy_Y==================================
#define InputY_error_PB 2.3//14
#define InputY_error_PS 1.2//7
#define InputY_derror_PB 60.0//700
#define InputY_derror_PS 30.0//200

#define PB_Y 1.0//2.5
#define PM_Y 0.8
#define PS_Y 0.5
#define ZE_Y 0.0
#define NS_Y -PS_Y
#define NM_Y -PM_Y
#define NB_Y -PB_Y
//============ Fuzzy_Pitch==================================
#define Pitch_error_PB 14
#define Pitch_error_PS 10
#define Pitch_derror_PB 35.0
#define Pitch_derror_PS 20.0

#define PB_P 0.01
#define PM_P 0.0065
#define PS_P 0.003
#define ZE_P 0.0
#define NS_P -PS_P
#define NM_P -PM_P
#define NB_P -PB_P
////============ Fuzzy_Roll==================================
#define Roll_error_PB 23
#define Roll_error_PS 15
#define Roll_derror_PB 60.0
#define Roll_derror_PS 50.0

#define PB_R 0.01
#define PM_R 0.0065
#define PS_R 0.003
#define ZE_R 0.0
#define NS_R -PS_R
#define NM_R -PM_R
#define NB_R -PB_R

#define MIN(i,j) (i<j)?i:j

/********************************************************/

/******************* Include libarary********************/
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
//#include <io.h>
//#include "system.h"
/*******************************************************/

/******************* Include module*********************/
/*******************************************************/

/******************* Function***************************/
class FuzzyController
{
public:
    FuzzyController();
    ~FuzzyController();

    void fuzzy_g1(double Input_1, double *Value_1);
    void fuzzy_g2(double Input_2, double *Value_2);
    void weight();
    double fuzzy_x_control(double Error, double Error_dot);
    double fuzzy_y_control(double Error, double Error_dot);
    double fuzzy_pitch_control(double Error, double Error_dot);
    double fuzzy_roll_control(double Error, double Error_dot);

private:
    double value_x_1_[5];  //error
    double value_x_2_[5];  //error_dot
    double value_y_1_[5];  //error
    double value_y_2_[5];  //error_dot
    double value_p_1_[5];  //error
    double value_p_2_[5];  //error_dot
    double value_r_1_[5];  //error
    double value_r_2_[5];  //error_dot

    double rule_x_[5][5];
    double rule_y_[5][5];
    double rule_p_[5][5];
    double rule_r_[5][5];

    double muv_e_[5];
    double muv_ce_[5];
    double weight_value_[5][5];
};
/********************************************************/

#endif
