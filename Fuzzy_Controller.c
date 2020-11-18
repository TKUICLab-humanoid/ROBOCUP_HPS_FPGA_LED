/*
 * Fuzzy_Controller.c
 *
 *  Created on: 2019/8/13
 *      Author: TKU_ICLab
 */
#include "include/Fuzzy_Controller.h"

FuzzyController::FuzzyController()
{
    double value_x_1_[5] = {-InputX_error_PB, -InputX_error_PS, 0, InputX_error_PS, InputX_error_PB};  //error
    double value_x_2_[5] = {-InputX_derror_PB, -InputX_derror_PS, 0, InputX_derror_PS, InputX_derror_PB};  //error_dot
    double value_y_1_[5] = {-InputY_error_PB, -InputY_error_PS, 0, InputY_error_PS, InputY_error_PB};  //error
    double value_y_2_[5] = {-InputY_derror_PB, -InputY_derror_PS, 0, InputY_derror_PS, InputY_derror_PB};  //error_dot
    double value_p_1_[5] = {-Pitch_error_PB, -Pitch_error_PS, 0, Pitch_error_PS, Pitch_error_PB};  //error
    double value_p_2_[5] = {-Pitch_derror_PB, -Pitch_derror_PS, 0, Pitch_derror_PS, Pitch_derror_PB};  //error_dot
    double value_r_1_[5] = {-Roll_error_PB, -Roll_error_PS, 0, Roll_error_PS, Roll_error_PB};  //error
    double value_r_2_[5] = {-Roll_derror_PB, -Roll_error_PS, 0, Roll_derror_PS, Roll_derror_PB};  //error_dot

    double rule_x_[5][5] = {{NB_X, NB_X, NS_X, ZE_X, PS_X},
                      {NB_X, NM_X, NS_X, PS_X, PM_X},
                      {NM_X, NS_X, ZE_X, PS_X, PM_X},
                      {NM_X, NS_X, PS_X, PM_X, PB_X},
                      {NS_X, ZE_X, PS_X, PB_X, PB_X}};
    double rule_y_[5][5] = {{NB_Y, NB_Y, NM_Y, NS_Y, ZE_Y},
                        {NM_Y, NM_Y, NS_Y, ZE_Y, PS_Y},
                        {NM_Y, NS_Y, ZE_Y, PS_Y, PM_Y},
                        {NS_Y, ZE_Y, PS_Y, PM_Y, PM_Y},
                        {ZE_Y, PS_Y, PM_Y, PB_Y, PB_Y}};
    double rule_p_[5][5] = {{NB_P, NB_P, NM_P, NS_P, ZE_P},
                        {NM_P, NM_P, NS_P, ZE_P, PS_P},
                        {NM_P, NS_P, ZE_P, PS_P, PM_P},
                        {NS_P, ZE_P, PS_P, PM_P, PM_P},
                        {ZE_P, PS_P, PM_P, PB_P, PB_P}};
    double rule_r_[5][5] = {{NB_R, NB_R, NM_R, NS_R, ZE_R},
                        {NM_R, NM_R, NS_R, ZE_R, PS_R},
                        {NM_R, NS_R, ZE_R, PS_R, PM_R},
                        {NS_R, ZE_R, PS_R, PM_R, PM_R},
                        {ZE_R, PS_R, PM_R, PB_R, PB_R}};

    memset(muv_e_, 0.0, sizeof(muv_e_));
    memset(muv_ce_, 0.0, sizeof(muv_ce_));
    memset(weight_value_, 0.0, sizeof(weight_value_));
}

FuzzyController::~FuzzyController()
{   }

void FuzzyController::fuzzy_g1(double input_1, double *value_1)
{
   if (input_1 <= value_1[0])
   {
       muv_e_[0] = 1;
       muv_e_[1] = 0;
       muv_e_[2] = 0;
       muv_e_[3] = 0;
       muv_e_[4] = 0;
   }
   else if (input_1 > value_1[0] && input_1 <= value_1[1])
   {
       muv_e_[0] = ( double )(input_1-value_1[1])/(value_1[0]-value_1[1]);
       muv_e_[1] = ( double )(input_1-value_1[0])/(value_1[1]-value_1[0]);
       muv_e_[2] = 0;
       muv_e_[3] = 0;
       muv_e_[4] = 0;
   }
   else if (input_1 > value_1[1] && input_1 <= value_1[2])
   {
       muv_e_[0] = 0;
       muv_e_[1] = ( double )(input_1-value_1[2])/(value_1[1]-value_1[2]);
       muv_e_[2] = ( double )(input_1-value_1[1])/(value_1[2]-value_1[1]);
       muv_e_[3] = 0;
       muv_e_[4] = 0;
   }
   else if (input_1 > value_1[2] && input_1 <= value_1[3])
   {
       muv_e_[0] = 0;
       muv_e_[1] = 0;
       muv_e_[2] = ( double )(input_1-value_1[3])/(value_1[2]-value_1[3]);
       muv_e_[3] = ( double )(input_1-value_1[2])/(value_1[3]-value_1[2]);
       muv_e_[4] = 0;
   }
   else if (input_1 > value_1[3] && input_1 <= value_1[4])
   {
       muv_e_[0] = 0;
       muv_e_[1] = 0;
       muv_e_[2] = 0;
       muv_e_[3] = ( double )(input_1-value_1[4])/(value_1[3]-value_1[4]);
       muv_e_[4] = ( double )(input_1-value_1[3])/(value_1[4]-value_1[3]);
   }
    else if (input_1 > value_1[4])
    {
       muv_e_[0] = 0;
       muv_e_[1] = 0;
       muv_e_[2] = 0;
       muv_e_[3] = 0;
       muv_e_[4] = 1;
   }
}
void FuzzyController::fuzzy_g2(double input_2, double* value_2)
{
   if (input_2 <= value_2[0])
   {
       muv_ce_[0] = 1;
       muv_ce_[1] = 0;
       muv_ce_[2] = 0;
       muv_ce_[3] = 0;
       muv_ce_[4] = 0;
   }
   else if (input_2 > value_2[0] && input_2 <= value_2[1])
   {
       muv_ce_[0] = ( double )(input_2-value_2[1])/(value_2[0]-value_2[1]);
       muv_ce_[1] = ( double )(input_2-value_2[0])/(value_2[1]-value_2[0]);
       muv_ce_[2] = 0;
       muv_ce_[3] = 0;
       muv_ce_[4] = 0;
   }
    else if (input_2 > value_2[1] && input_2 <= value_2[2])
    {
       muv_ce_[0] = 0;
       muv_ce_[1] = ( double )(input_2-value_2[2])/(value_2[1]-value_2[2]);
       muv_ce_[2] = ( double )(input_2-value_2[1])/(value_2[2]-value_2[1]);
       muv_ce_[3] = 0;
       muv_ce_[4] = 0;
   }
   else if (input_2 > value_2[2] && input_2 <= value_2[3])
   {
       muv_ce_[0] = 0;
       muv_ce_[1] = 0;
       muv_ce_[2] = ( double )(input_2-value_2[3])/(value_2[2]-value_2[3]);
       muv_ce_[3] = ( double )(input_2-value_2[2])/(value_2[3]-value_2[2]);
       muv_ce_[4] = 0;
   }
   else if(input_2 > value_2[3] && input_2 <= value_2[4])
   {
       muv_ce_[0] = 0;
       muv_ce_[1] = 0;
       muv_ce_[2] = 0;
       muv_ce_[3] = ( double )(input_2-value_2[4])/(value_2[3]-value_2[4]);
       muv_ce_[4] = ( double )(input_2-value_2[3])/(value_2[4]-value_2[3]);
   }
   else if (input_2 > value_2[4])
   {
       muv_ce_[0] = 0;
       muv_ce_[1] = 0;
       muv_ce_[2] = 0;
       muv_ce_[3] = 0;
       muv_ce_[4] = 1;
   }
}
void FuzzyController::weight()
{
    int E_count;
    int CE_count;
    for (E_count = 0; E_count < 5; E_count++)
    {
        for(CE_count = 0; CE_count< 5; CE_count++)
        {
            weight_value_[E_count][CE_count] = MIN(muv_e_[E_count], muv_ce_[CE_count]);
        }
    }
}

double FuzzyController::fuzzy_x_control(double error, double error_dot)
{
	double Fuzzy_Output = 0.0;
	double U = 0.0, D = 0.0;
    int i, j;
	fuzzy_g1(error, value_x_1_);
	fuzzy_g2(error_dot, value_x_2_);
	weight();
	for (i = 0; i < 5; i++)
	{
	   for (j = 0; j < 5; j++)
	   {
		   U = (U + rule_x_[i][j] * weight_value_[i][j]) ;
		   D = (D + weight_value_[i][j]);
	   }
	}

	Fuzzy_Output = (double)(U/D);
	return Fuzzy_Output;
}
double FuzzyController::fuzzy_y_control(double error, double error_dot)
{

	double Fuzzy_Output = 0.0;
	double U = 0.0, D = 0.0;
    int i, j;
	fuzzy_g1(error, value_y_1_);
	fuzzy_g2(error_dot, value_y_2_);
	weight();
	for (i = 0; i < 5; i++)
	{
	   for (j = 0; j < 5; j++)
	   {
		   U = (U + rule_y_[i][j] * weight_value_[i][j]) ;
		   D = (D + weight_value_[i][j]);
	   }
	}

	Fuzzy_Output = (double)(U/D);
	return Fuzzy_Output;
}
double FuzzyController::fuzzy_pitch_control(double error, double error_dot)
{
	double Fuzzy_Output = 0.0;
    double U = 0.0, D = 0.0;
    int i, j;
    fuzzy_g1(error, value_p_1_);
    fuzzy_g2(error_dot, value_p_2_);
    weight();
    for (i = 0; i < 5; i++)
    {
        for (j = 0; j < 5; j++)
        {
            U = (U + rule_p_[i][j] * weight_value_[i][j]) ;
            D = (D + weight_value_[i][j]);
        }
    }

    Fuzzy_Output = (double)(U/D);
    return Fuzzy_Output;
}
double FuzzyController::fuzzy_roll_control(double error, double error_dot)
{
	double Fuzzy_Output = 0.0;
	double U = 0.0, D = 0.0;
    int i, j;
	fuzzy_g1(error, value_r_1_);
	fuzzy_g2(error_dot, value_r_2_);
	weight();
	for (i = 0; i < 5; i++)
	{
	   for (j = 0; j < 5; j++)
	   {
		   U = (U + rule_r_[i][j] * weight_value_[i][j]) ;
		   D = (D + weight_value_[i][j]);
	   }
	}

	Fuzzy_Output = (double)(U/D);
	return Fuzzy_Output;
}


