#include "include/WalkingTrajectory.h"

WalkingTrajectory::WalkingTrajectory()
{
    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Thta.clear();
    left_Thta.clear();
    test.clear();
    wosc_foot_x_r = 0;
    wosc_foot_x_l = 0;
    wosc_foot_x_last = 0;
    wosc_foot_x_now = 0;
}

WalkingTrajectory::~WalkingTrajectory()
{

}

void WalkingTrajectory::walkingprocess(int walking_mode)
{
    switch(walking_mode)
    {
    case 6:
    case 5:
    case 0:
        walkfunction();//Single Step
        break;
    case 8:
    case 7:
    case 1:
        continuouswalk();//Continuous
        break;
    case 2:
    case 3:
        LC_walkfunction();
        break;
    case 4:
        longjump_function();
        break;
    default:
        break;
    }
    inversekinmaticsinfo();
}

void WalkingTrajectory::walkfunction()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    /************************************************************************/
    /* Set flags of moving cases                                            */
    /************************************************************************/
    int moving_mode;
    if (parameterinfo->YUpdate > 0)
    {
        if (parameterinfo->THTAUpdate == 0)
        {
            moving_mode = Left_Shift;
        }
        else
        {
            moving_mode = Left_Protect;
        }
    }
    else if (parameterinfo->YUpdate < 0)
    {
        if (parameterinfo->THTAUpdate == 0)
        {
            moving_mode = Right_Shift;
        }
        else
        {
            moving_mode = Right_Protect;
        }
    }
    else
    {
        if (parameterinfo->THTAUpdate > 0)
        {
            moving_mode = Left_Turn;
        }
        else if (parameterinfo->THTAUpdate < 0)
        {
            moving_mode = Right_turn;
        }
        else
        {
            moving_mode = Straight;
        }
    }
    ///************************************************************************/
    ///* Switch moving cases from flags                                       */
    ///************************************************************************/
    //    parameterinfo->complan.isfirststep = msg.isfirststep;
    //    parameterinfo->complan.islaststep = msg.islaststep;
    switch (moving_mode)
    {
    case Right_turn:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_ )//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix



        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);

        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep || parameterinfo->complan.islaststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
        }
        break;
    case Right_Protect:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower

        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate*0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else if (parameterinfo->THTAUpdate > 0) // Turn Left
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate * -1, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate, 0, time_shift);	// Swing leg turn left;
        }
        else if (parameterinfo->THTAUpdate < 0) // Turn Right
        {
            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        break;

    case Left_Turn:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;	// Swing leg turn left
        }
        else
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left;
        }
        break;
    case Left_Protect:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate , 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        }

        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2/4);
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate*0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else if (parameterinfo->THTAUpdate > 0) // Turn Left
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate, 0, time_shift);
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left;
        }
        else if (parameterinfo->THTAUpdate < 0) // Turn Right
        {

            parameterinfo->points.Right_Thta =  OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.3 * parameterinfo->THTAUpdate, 0, time_shift); //Swing leg turn right
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.7 * parameterinfo->THTAUpdate * -1, 0, time_shift);
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        break;

    case Right_Shift:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range-parameterinfo->YUpdate*0.25, 0, PI_2, time_point_ + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, -0/180*PI, 0, time_point_); //Swing leg turn right //Parameters->Shift_Lfoot_RTurn
            parameterinfo->points.Left_Thta = 0;
        }

        break;
    case Left_Shift:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_Right_foot = 0;
            parameterinfo->points.Y_Left_foot = 0;
        }
        else
        {
            parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
            parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        }
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->parameters.X_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T2/4);

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range+parameterinfo->YUpdate*0.25, 0, PI_2, time_shift + parameterinfo->parameters.Period_T/4);//OSC_COM_LockRange
        }

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (parameterinfo->complan.isfirststep)
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = 0;
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;
            parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0/180*PI, PI, time_shift);//Parameters->Shift_Rfoot_LTurn
        }

        break;
    case Straight:

        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0 , PI, time_point_);//Parameters->Shift_LX_fix

        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        if (parameterinfo->complan.islaststep)
        {
            parameterinfo->points.X_COM = 0;
        }
        else
        {
            parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
        }
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange

        parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange,
        if (parameterinfo->complan.islaststep)
        {
            parameterinfo->points.Right_Thta = 0;//821
            parameterinfo->points.Left_Thta = 0;//821
        }
        else
        {
            parameterinfo->points.Right_Thta = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
            parameterinfo->points.Left_Thta  = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_);//-Parameters->Str_Lfoot_Rturn
        }
        break;
    default:
        parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);//Parameters->BASE_delrho_x
        parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);//Parameters->BASE_delrho_x
        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate , 0, time_point_);
        parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_point_+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        parameterinfo->points.Z_COM = 0;//parameterinfo->parameters.COM_Height + OSC_move_x_advance(parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        //        //ROS_INFO("Save");
        // SaveData();
    }

}


void WalkingTrajectory::continuouswalk()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_shift);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0, PI_2, time_point_);
    parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
    parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);

    parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    parameterinfo->points.Y_COM = OSC_COM_Y(parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_COM_Z(parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);
    // Turn
    if (parameterinfo->THTAUpdate > 0) // Turn Left
    {

        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_shift);
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left

    }
    else if (parameterinfo->THTAUpdate < 0) // Turn Right
    {
        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_point_);//Swing leg turn right
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_point_);
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }
    // Shift
    if(parameterinfo->complan.isfirststep)
    {
        parameterinfo->points.Y_Right_foot = 0;
        parameterinfo->points.Y_Left_foot = 0;
    }
    else if (parameterinfo->YUpdate < 0 && !parameterinfo->Isfrist_right_shift)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_point_);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_point_);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_shift);	// Turn fix for shift  //Parameters->Shift_Rfoot_LTurn
        parameterinfo->points.Left_Thta = 0;
    }
    else if (parameterinfo->YUpdate > 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_point_); // Turn fix for shift  //Parameters->Shift_Lfoot_RTurn
    }
    else
    {
        //parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);// Open_value_R
        //parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
    }
    // Add turn offset
    //parameterinfo->points.Y_Right_foot += 0;//Y_Right_foot_tmp_;
    //parameterinfo->points.Y_Left_foot += 0;//Y_Left_foot_tmp_;
    // Forward modify
    // if (parameterinfo->XUpdate != 0)
    // {
    //     parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_); //817(Lturn) //Parameters->Str_Rfoot_Lturn
    //     parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift); //-Parameters->Str_Lfoot_Rturn
    // }

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        //ROS_INFO("save");
        SaveData();
    }
}

void WalkingTrajectory::osccontinuouswalk()//JJ
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;
    double wv_x = WOSC_Waist_V(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, 0, time_point_, parameterinfo->parameters.Sample_Time);//- (1.5*parameterinfo->parameters.Ts+(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time))
    //double rx = WOSC_Foot_X(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, 0, time_point_, parameterinfo->parameters.Sample_Time);//- 1.5*parameterinfo->parameters.Ts
    double absx = WOSC_Foot_X(parameterinfo->parameters.Ts, parameterinfo->parameters.Vmin, parameterinfo->parameters.Vmax, parameterinfo->parameters.Period_T, PI, time_point_, parameterinfo->parameters.Sample_Time);//- 1.5*parameterinfo->parameters.Ts
    
    if(time_point_ <= parameterinfo->parameters.Ts*4*3)
    {
        parameterinfo->points.X_Right_foot = 0;
        parameterinfo->points.X_Left_foot = 0;
    }
    else
    {
        parameterinfo->points.X_Right_foot = wosc_foot_x_r-wv_x-(200*(parameterinfo->parameters.Vmax-parameterinfo->parameters.Vmin));
        parameterinfo->points.X_Left_foot = wosc_foot_x_l-wv_x-(200*(parameterinfo->parameters.Vmax-parameterinfo->parameters.Vmin));
    }
    parameterinfo->points.Z_Right_foot = WOSC_Foot_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, 0, time_point_,1);
    parameterinfo->points.Z_Left_foot = WOSC_Foot_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, 0, time_point_,0);
    //    parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
    //    parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);

    parameterinfo->points.X_COM = 0;//wv_x;//OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    parameterinfo->points.Y_COM = WOSC_Waist_Y(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);//OSC_COM_Y(parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, time_point_);
    parameterinfo->points.Z_COM =  parameterinfo->parameters.COM_Height + WOSC_Waist_Z(parameterinfo->parameters.Ts, parameterinfo->parameters.Period_T, parameterinfo->parameters.Z_Swing_Range, 0, time_point_);

    parameterinfo->points.wv_x_COM = wv_x;
    parameterinfo->points.abs_x_feet = absx;
    parameterinfo->points.foot_x_r = wosc_foot_x_r;
    parameterinfo->points.foot_x_l = wosc_foot_x_l;

    //OSC_COM_Z(parameterinfo->parameters.Period_T, parameterinfo->parameters.Z_Swing_Range, PI_2, time_point_);
    // Turn

    if (parameterinfo->THTAUpdate > 0) // Turn Left
    {

        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_shift);
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_shift);	// Swing leg turn left

    }
    else if (parameterinfo->THTAUpdate < 0) // Turn Right
    {
        parameterinfo->points.Right_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate * -1, 0, time_point_);//Swing leg turn right
        parameterinfo->points.Left_Thta = OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0.5 * parameterinfo->THTAUpdate, 0, time_point_);
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta = 0;
    }
    // Shift
    if (parameterinfo->YUpdate > 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_shift);	// Turn fix for shift  //Parameters->Shift_Rfoot_LTurn
        parameterinfo->points.Left_Thta = 0;
    }
    else if (parameterinfo->YUpdate < 0)
    {
        parameterinfo->points.Y_Right_foot = OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (0.5 - 0) * parameterinfo->YUpdate, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Y_Left_foot =  OSC_move_shift_y(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, (2 + 0) * parameterinfo->YUpdate * -1, 0, time_shift);//Parameters->Shift_delPower
        parameterinfo->points.Right_Thta = 0;
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0*PI/180, 0, time_point_); // Turn fix for shift  //Parameters->Shift_Lfoot_RTurn
    }
    else
    {
        parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T,0, PI, time_shift);// Open_value_R
        parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_point_);//Open_value_L
    }
    // Add turn offset
    parameterinfo->points.Y_Right_foot += 0;//Y_Right_foot_tmp_;
    parameterinfo->points.Y_Left_foot += 0;//Y_Left_foot_tmp_;
    // Forward modify
    if (parameterinfo->XUpdate != 0)
    {
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_); //817(Lturn) //Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift); //-Parameters->Str_Lfoot_Rturn
    }

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        // SaveData();
    }
}

void WalkingTrajectory::LC_walkfunction()
{
    int time_shift = 0;
  //  int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

//    case Straight:

    parameterinfo->points.Y_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_);//Open_value_R
    parameterinfo->points.Y_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift);//Open_value_L
    
    if (parameterinfo->complan.lift_lock_x != 0)
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));//2.4462*7;
        //ROS_INFO("0%f", parameterinfo->points.X_Right_foot);
        if (parameterinfo->points.X_Right_foot > parameterinfo->complan.lift_lock_x)
        {
            parameterinfo->points.X_Right_foot = parameterinfo->complan.lift_lock_x;
            //ROS_INFO("1%f", parameterinfo->points.X_Right_foot);
        }else{
            parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));//2.4462*7;
            parameterinfo->complan.lift_lock_x = 0;
            //ROS_INFO("L%f", parameterinfo->points.X_Right_foot);
        }
        //ROS_INFO("2%f", parameterinfo->points.X_Right_foot);

    }
    else
    {
        parameterinfo->points.X_Right_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, time_shift)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, time_shift));//2.4462*7;
        parameterinfo->points.X_Left_foot = (OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate ,0 , PI_2, parameterinfo->complan.time_point_)+OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, PI, parameterinfo->complan.time_point_));//2.4462*7;
        parameterinfo->complan.lift_lock_x = parameterinfo->points.X_Right_foot;
        //ROS_INFO("%f", parameterinfo->points.X_Right_foot);
    }


    if (parameterinfo->parameters.BASE_LIFT_Z > 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_lifemove_UPz(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_-parameterinfo->parameters.Period_T/16, parameterinfo->parameters.Sample_Time);
        parameterinfo->points.Z_Left_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->complan.time_point_+parameterinfo->parameters.Period_T/16);
    }
    else if (parameterinfo->parameters.BASE_LIFT_Z < 0)
    {
        parameterinfo->points.Z_Right_foot = OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_shift);
        parameterinfo->points.Z_Left_foot = OSC_lifemove_DOWNz(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, parameterinfo->parameters.BASE_LIFT_Z, parameterinfo->complan.time_point_+parameterinfo->parameters.Period_T/16, parameterinfo->parameters.Sample_Time);
    }

    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.X_COM = 0;
    }else{
        parameterinfo->points.X_COM = OSC_COM_Lift_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, parameterinfo->parameters.X_Swing_COM, parameterinfo->complan.time_point_);
    }
    parameterinfo->points.Y_COM = OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range, 0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, parameterinfo->complan.time_point_);

    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Thta = 0;//821
        parameterinfo->points.Left_Thta = 0;//821
    }else{
        parameterinfo->points.Right_Thta += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta  += OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, parameterinfo->complan.time_point_);//-Parameters->Str_Lfoot_Rturn
    }
    

    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        //        //ROS_INFO("Save");
        // SaveData();
    }
}

void WalkingTrajectory::longjump_function()
{
    int time_shift = 0;
    int time_point_ = parameterinfo->complan.time_point_;

    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    /************************************************************************/
    /* Set flags of moving cases                                            */
    /************************************************************************/

    parameterinfo->points.X_Right_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange , parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_);
    parameterinfo->points.X_Left_foot = OSC_move_x_advance(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->XUpdate, 0 , PI_2, time_point_);
    parameterinfo->points.Y_Right_foot = 0.0;
    parameterinfo->points.Y_Left_foot = 0.0;
    parameterinfo->points.Z_Right_foot = -OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
    parameterinfo->points.Z_Left_foot = -OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, parameterinfo->ZUpdate, PI, time_point_);
    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.X_COM = 0;
    }
    else
    {
        parameterinfo->points.X_COM = OSC_COM_X(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T2, parameterinfo->XUpdate, 0, time_point_);
    }
    parameterinfo->points.Y_COM = 0.0;//OSC_move_x_advance(0, parameterinfo->parameters.Period_T, parameterinfo->parameters.Y_Swing_Range,0, PI_2, time_shift+parameterinfo->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
    parameterinfo->points.Z_COM = parameterinfo->parameters.COM_Height + OSC_move_x_advance(0, parameterinfo->parameters.Period_T2, parameterinfo->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + parameterinfo->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange,
    if (parameterinfo->complan.islaststep)
    {
        parameterinfo->points.Right_Thta = 0;//821
        parameterinfo->points.Left_Thta = 0;//821
    }
    else
    {
        parameterinfo->points.Right_Thta = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
        parameterinfo->points.Left_Thta  = 0;//+= OSC_Rotate(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T, 0, 0, time_point_);//-Parameters->Str_Lfoot_Rturn
    }


    osc_move_x_r.push_back(parameterinfo->points.X_Right_foot);
    osc_move_x_l.push_back(parameterinfo->points.X_Left_foot);
    osc_move_y_r.push_back(parameterinfo->points.Y_Right_foot);
    osc_move_y_l.push_back(parameterinfo->points.Y_Left_foot);
    osc_move_z_r.push_back(parameterinfo->points.Z_Right_foot);
    osc_move_z_l.push_back(parameterinfo->points.Z_Left_foot);
    osc_move_com_x.push_back(parameterinfo->points.X_COM);
    osc_move_com_y.push_back(parameterinfo->points.Y_COM);
    osc_move_com_z.push_back(parameterinfo->points.Z_COM);
    right_Thta.push_back(parameterinfo->points.Right_Thta);
    left_Thta.push_back(parameterinfo->points.Left_Thta);
    test.push_back(parameterinfo->counter);

    if(parameterinfo->complan.walking_stop)
    {
        // //ROS_INFO("Save");
        // SaveData();
    }
}

void WalkingTrajectory::inversekinmaticsinfo()
{
	int time_shift = 0;
	int time_point_ = parameterinfo->complan.time_point_;
	time_shift =  parameterinfo->complan.time_point_ + 0.5 * parameterinfo->parameters.Period_T;

    parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
    parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot;
    parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
    parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
    parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
    parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot;
    parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
    parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;

	if (parameterinfo->walking_mode == 0)//Single
	{
		if(/*parameterinfo->X > 3 ||*/ parameterinfo->Y > 0.2 || parameterinfo->Y < -0.2)
		{
			if(parameterinfo->complan.walking_state == StopStep)
			{
				parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
				parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, -0.3, -PI/2, time_point_, 2);
				parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
				parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
				parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
				parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, 0.3, -PI/2, time_shift, 2);
				parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
				parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
			}
			else if(parameterinfo->complan.walking_state == StartStep)
			{
				parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
				parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, -0.3, PI/2, time_point_, 2);
				parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
				parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
				parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
				parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, 0.3, PI/2, time_shift, 2);
				parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
				parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
			}
			else
			{
				parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
				parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot - 0.3;
				parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
				parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
				parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
				parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + 0.3;
				parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
				parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
			}
		}
		else
		{
			parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
			parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot;
			parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
			parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
			parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
			parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot;
			parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
			parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
		}
	}
	else if(parameterinfo->walking_mode == 1)//Continuous
	{
		if(parameterinfo->complan.walking_state == StopStep)
		{
			parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
			parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, -0.5, -PI/2, time_point_, 2);
			parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
			parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
			parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
			parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, 0.5, -PI/2, time_shift, 2);
			parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
			parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
		}
		else if(parameterinfo->complan.walking_state == StartStep)
		{
			parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
			parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, -0.5, PI/2, time_point_, 2);
			parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
			parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
			parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
			parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + OSC_move_z(parameterinfo->parameters.OSC_LockRange, parameterinfo->parameters.Period_T*2, 0.5, PI/2, time_shift, 2);
			parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
			parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
		}
		else
		{
			parameterinfo->points.IK_Point_RX = parameterinfo->points.X_COM + parameterinfo->points.X_Right_foot;
			parameterinfo->points.IK_Point_RY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Right_foot - 0.5;
			parameterinfo->points.IK_Point_RZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Right_foot;
			parameterinfo->points.IK_Point_RThta = parameterinfo->points.Right_Thta;
			parameterinfo->points.IK_Point_LX = parameterinfo->points.X_COM + parameterinfo->points.X_Left_foot;
			parameterinfo->points.IK_Point_LY = -1 * parameterinfo->points.Y_COM + parameterinfo->points.Y_Left_foot + 0.5;
			parameterinfo->points.IK_Point_LZ = parameterinfo->points.Z_COM - parameterinfo->points.Z_Left_foot;
			parameterinfo->points.IK_Point_LThta = parameterinfo->points.Left_Thta;
		}
	}

    // parameterinfo->ik_parameters.R_Goal[0] = parameterinfo->points.IK_Point_RX;
    // parameterinfo->ik_parameters.R_Goal[1] = parameterinfo->points.IK_Point_RY - 4;
    // parameterinfo->ik_parameters.R_Goal[2] = 21.7 - parameterinfo->points.IK_Point_RZ;
    // parameterinfo->ik_parameters.R_Goal[3] = parameterinfo->points.IK_Point_RThta;
    // parameterinfo->ik_parameters.L_Goal[0] = parameterinfo->points.IK_Point_LX;
    // parameterinfo->ik_parameters.L_Goal[1] = parameterinfo->points.IK_Point_LY + 4;
    // parameterinfo->ik_parameters.L_Goal[2] = 21.7 - parameterinfo->points.IK_Point_LZ;
    // parameterinfo->ik_parameters.L_Goal[3] = parameterinfo->points.IK_Point_LThta;
    // printf("R_X: %f, R_Y: %f, R_Z: %f, R_T: %f\n", parameterinfo->points.IK_Point_RX, parameterinfo->points.IK_Point_RY, parameterinfo->points.IK_Point_RZ, parameterinfo->points.IK_Point_RThta);
    // printf("L_X: %f, L_Y: %f, L_Z: %f, L_T: %f\n\n", parameterinfo->points.IK_Point_LX, parameterinfo->points.IK_Point_LY, parameterinfo->points.IK_Point_LZ, parameterinfo->points.IK_Point_LThta);
	
}

string WalkingTrajectory::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void WalkingTrajectory::SaveData()
{
    string savedText = "R_move_X\tL_move_X\t"
                       "R_move_Y\tL_move_Y\t"
                       "R_move_Z\tL_move_Z\t"
                       "move_COM_X\tmove_COM_Y\tmove_COM_Z\t"
                       "R_Thta\tL_Thta\tPoint\n";
    char path[200] = "/data";
    // strcpy(path, PATH.c_str());
    strcat(path, "/Trajectory_Record.xls");
    //char filename[]="/home/iclab/Desktop/OBS0722SLOW/newsystem/src/strategy/src/spartanrace/Parameter/Trajectory_Record.ods";

    fstream fp;
    fp.open(path, ios::out);

    fp<<savedText;

    for(int i = 0; i < test.size(); i++)
    {
        savedText = DtoS(osc_move_x_r[i]) + "\t"
                + DtoS(osc_move_x_l[i]) + "\t"
                + DtoS(osc_move_y_r[i]) + "\t"
                + DtoS(osc_move_y_l[i]) + "\t"
                + DtoS(osc_move_z_r[i]) + "\t"
                + DtoS(osc_move_z_l[i]) + "\t"
                + DtoS(osc_move_com_x[i]) + "\t"
                + DtoS(osc_move_com_y[i]) + "\t"
                + DtoS(osc_move_com_z[i]) + "\t"
                + DtoS(right_Thta[i]) + "\t"
                + DtoS(left_Thta[i]) + "\t"
                + DtoS(test[i]) + "\n";
        fp<<savedText;
    }
    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Thta.clear();
    left_Thta.clear();
    test.clear();
    fp.close();
}

double WalkingTrajectory::OSC_COM_Lift_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double omega_com_x;
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range / 2);
    double T_divbyunlock = period_T * (1-range) / 2;
    omega_com_x = 2 * PI / period_T;
    rho_x = rho_com_x * (range) * delta_com_x;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t =(double)time_t_temp / 1000;

    if (time_t >= 0 && time_t < T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI);
    }
    else if (time_t < period_T && time_t > (period_T - T_divbylock))
    {
        return rho_x * sin((1.5*PI) + (2 * PI / (T_divbyunlock * 4)) * (time_t - (period_T - T_divbylock))+PI);
    }
    else
    {
        return rho_x * sin((PI / 2) + (2 * PI / (T_divbyunlock * 4)) * (time_t - T_divbylock) + PI);
    }
}
double WalkingTrajectory::OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms)
{
    double omega_x;
    double period_T = period_T_ms * 0.001;//sec
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_x = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return rho_x *(1 + BASE_delrho_x);
    }
    else if(time_t >= range * T_divby4 && time_t < (T_divby2 - (range * T_divby4)) )
    {
        if (time_t<=T_divby4)
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
        else
        {
            return rho_x* (1 - BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
    }
    else if(time_t >= T_divby2 - (range * T_divby4) && time_t < T_divby2 + (range * T_divby4))
    {
        return -rho_x * (1 - BASE_delrho_x);
    }
    else if(time_t >= T_divby2 + (range * T_divby4) && time_t < period_T - (range * T_divby4))
    {
        if (time_t <= period_T - T_divby4)
        {
            return rho_x * (1 - BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
        else
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
    }
    else
    {
        return rho_x *(1 + BASE_delrho_x) ;

    }
}

double WalkingTrajectory::OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms, int div_omega)
{
    double omega_z;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % ( int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_z = 2 * PI / period_T / (1-range);
	omega_z = omega_z / (double)div_omega;

    if(time_t >= 0 && time_t < T_divby2 + range * T_divby4)
    {
        return 0;
    }
    else if(time_t >= T_divby2 + range * T_divby4 && time_t < period_T-(range * T_divby4))
    {
        return rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
    }
    else
    {
        return 0;
    }
}

double WalkingTrajectory::OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double omega_com_x;
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range/2);
    double T_divbyunlock = period_T * (1-range)/2;
    omega_com_x = 2 * PI / period_T;
    rho_x = rho_com_x * (range) * (parameterinfo->parameters.X_Swing_Range);
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    if(time_t >= 0 && time_t <  T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI );
    }
    else if(time_t < period_T && time_t >  (period_T - T_divbylock))
    {
        return rho_x * sin( (1.5 * PI) + (2 * PI / (T_divbylock * 4)) * (time_t - (period_T - T_divbylock))+ PI );
    }
    else
    {
        return rho_x * sin( (PI / 2) + (2 * PI / (T_divbyunlock * 4)) * (time_t - T_divbylock)+ PI );
    }
}

double WalkingTrajectory::OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    double omega_y;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    // new
    double r1 = T_divby4;
    double r2 = T_divby2 - (range * T_divby4);
    double r3 = T_divby2 + (range * T_divby4);
    double r4 = 3*T_divby4;
    omega_y = 2 * PI / period_T / (1-range);
    if (time_t >= 0 && time_t < range * T_divby4  )
    {
        return 0;
    }
    else if(time_t >=  range * T_divby4  && time_t < r1)
    {
        return rho_y * sin(omega_y * (time_t - range * T_divby4) + delta_y);
    }
    else if (time_t >= r1 && time_t < r3)
    {
        return rho_y;
    }
    else if (time_t >= r3 && time_t <= r4)
    {
        //return rho_y * sin(omega_y * (time_t - range * 3 * T_divby4 - period_T) + delta_y);
        return rho_y * sin(omega_y * (time_t - 3*T_divby4) + delta_y + PI);
    }
    else
    {
        return 0;
    }
}

double WalkingTrajectory::OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms)
{
    double omega_com_y;
    double period_T = period_T_ms * 0.001;
    omega_com_y = 2 * PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_y * sin(omega_com_y * time_t + delta_com_y);
}

double WalkingTrajectory::OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms)
{
    double omega_com_z;
    double period_T = period_T_ms * 0.001;
    omega_com_z =  2*PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_z * sin(omega_com_z * time_t + delta_com_z);
}

double WalkingTrajectory::OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    return OSC_move_shift_y( range,  period_T_ms,  rho_y,  delta_y,  time_t_ms);
}

double WalkingTrajectory::WOSC_Waist_V(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time)
{

    int Td = period_T_ms-Ts;
    int Tssd = 2*Ts;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;//parameterinfo->parameters.abswaistx +
    double time_t = (double)time_t_temp / 1000;

    if(time_t_temp >= 0 && time_t_temp < Ts)
        parameterinfo->parameters.abswaistx += (0.5*(Vmax-Vmin)*(1-cos(PI*time_t_temp/(double)Ts))+Vmin)*(period_T_ms/(double)(Sample_Time));
    else if(time_t_temp >= Ts && time_t_temp < Td)
        parameterinfo->parameters.abswaistx += (Vmax)*(period_T_ms/(double)(Sample_Time));
    else if(time_t_temp >= Td && time_t_temp < period_T_ms)
        parameterinfo->parameters.abswaistx += (0.5*(Vmax-Vmin)*(1-cos(PI*(time_t_temp-Td-Ts)/(double)Ts))+Vmin)*(period_T_ms/(double)(Sample_Time));

    return parameterinfo->parameters.abswaistx;
}

double WalkingTrajectory::WOSC_Foot_X(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time)
{
    wosc_foot_x_last = wosc_foot_x_now;

    int Td = period_T_ms-Ts;
    double a = (Vmax*Td+Vmin*Ts)/PI/(period_T_ms/Ts)*4;
    int Tssd = 2*Ts;

    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)(period_T_ms*4);
    double time_t = (double)time_t_temp / 1000;

    double q = 2*PI*time_t_ms/(Tssd*2.0);
    wosc_foot_x_now = a*(q-sin(q+delta));

    double x = wosc_foot_x_now - wosc_foot_x_last;

    if((time_t_temp >= 0 && time_t_temp < period_T_ms) || (time_t_temp >= period_T_ms*3 && time_t_temp < period_T_ms*4))
    {
        wosc_foot_x_r += x;
        wosc_foot_x_l = wosc_foot_x_l;
    }
    else if(time_t_temp >= period_T_ms && time_t_temp < period_T_ms*3)
    {
        wosc_foot_x_l += x;
        wosc_foot_x_r = wosc_foot_x_r;
    }

    return wosc_foot_x_now;

}

double WalkingTrajectory::WOSC_Foot_Z(int Ts, double period_T_ms, double Hfoot, double delta, int time_t_ms, int state)
{
    int Tssd = 2*Ts;
    double q = PI*time_t_ms/Tssd;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    int aa = (time_t_ms-period_T_ms)/(period_T_ms/(period_T_ms/2.0/period_T_ms));

    if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return 0;
    else
        if( (aa%2) == state)
            return Hfoot/2*(1-cos(q+PI));
        else if( (aa%2) == state)
            return Hfoot/2*(1-cos(q+PI));
        else
            return 0;
}

double WalkingTrajectory::WOSC_Waist_Y(int Ts, double period_T_ms, double Ay, double delta_z, int time_t_ms)
{
    int Tssd = 2*Ts;
    double q = 2*PI*time_t_ms/Tssd/2.0/2.0;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    int aa = (time_t_ms/Ts)/(period_T_ms/(period_T_ms/2/Ts));

    if(time_t_ms < Ts*2)
        return 0;
    else if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return (Ay+0.2)*(cos(q));//+PI/2+PI/2
    else
        return Ay*(cos(q));//+PI/2+PI/2
}

double WalkingTrajectory::WOSC_Waist_Z(int Ts, double period_T_ms, double Az, double delta_z, int time_t_ms)
{
    int Tssd = 2*Ts;
    double q = 2*PI*time_t_ms/Tssd/2.0;
    double period_T = period_T_ms * 0.001;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if(time_t_ms < Ts*2+Ts*4+Ts*4)
        return 0;
    else
        return Az*(cos(q+PI/4+PI/4));

}

double WalkingTrajectory::OSC_lifemove_DOWNz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time)
{
    double omega_z;
    double outputZ;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if (life_high < 0)
    {
        time_t = fabs(time_t -(period_T_ms / 1000.0));
        parameterinfo->complan.lift_lock_x = 0;
        omega_z = 2 * PI / period_T / (1-range);
    }
    omega_z = 2 * PI / period_T / (1-range);
    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return 0;
    } 
    else if(time_t >= range * T_divby4 && time_t < T_divby2 - range * T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * T_divby4));
        if (time_t >= T_divby4 && outputZ <= fabs(life_high))
        {
            return fabs(life_high);
        }
        parameterinfo->complan.lift_lock_x = 0;
        return outputZ;
    }
    else if(time_t >= T_divby2 - range * T_divby4 && time_t < T_divby2 + range * T_divby4 - period_T/Sample_Time*2)
    {
        return fabs(life_high);
    }
    else if(time_t >= T_divby2 + range * T_divby4 - period_T/Sample_Time*2 && time_t < T_divby2 + T_divby4 - period_T/Sample_Time*2)
    {
        time_t = time_t + period_T/Sample_Time*2;
        outputZ = rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
        outputZ = -(outputZ - fabs(life_high));
        if (outputZ < fabs(life_high) && outputZ >= 0)
        {
            return outputZ;
        }else if (outputZ < 0)
        {
            return 0;
        }else{
            return fabs(life_high);
        }
    }else{
        return 0;
    }

}

double WalkingTrajectory::OSC_lifemove_UPz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time)
{
    double omega_z;
    double outputZ;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    if (life_high < 0)
    {
        time_t = fabs(time_t -(period_T_ms / 1000.0));
        parameterinfo->complan.lift_lock_x = 0;
        omega_z = 2 * PI / period_T / (1-range);
    }
    omega_z = 2 * PI / period_T / (1-range);
    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return 0;
    } 
    else if(time_t >= range * T_divby4 && time_t < T_divby2 - range * T_divby4)
    {
        outputZ = rho_z * sin(omega_z * (time_t - range * T_divby4));
        if (time_t >= T_divby4 && outputZ <= fabs(life_high))
        {
            return fabs(life_high);
        }
        parameterinfo->complan.lift_lock_x = 0;
        return outputZ;
    }
    else if(time_t >= T_divby2 - range * T_divby4 && time_t < T_divby2 + range * T_divby4 - period_T/Sample_Time*3)
    {
        return fabs(life_high);
    }
    else if(time_t >= T_divby2 + range * T_divby4 - period_T/Sample_Time*3 && time_t < T_divby2 + T_divby4 - period_T/Sample_Time*3)
    {
        time_t = time_t + period_T/Sample_Time*3;
        outputZ = rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
        outputZ = -(outputZ - fabs(life_high));
        if (outputZ < fabs(life_high) && outputZ >= 0)
        {
            return outputZ;
        }else if (outputZ < 0)
        {
            return 0;
        }else{
            return fabs(life_high);
        }
    }else{
        return 0;
    }

}

