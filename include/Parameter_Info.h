#ifndef PARAMETER_INFO_H_
#define PARAMETER_INFO_H_

#include <iostream>
#include <stdlib.h>

/*****************************************************************************
** Includes
*****************************************************************************/
enum WalkingState
{
    StartStep,
    FirstStep,
    Repeat,
    StopStep,
    MarkTimeStep,
    ForwardStep,
    BackwardStep
};
typedef enum
{
    etReady,
    etBusy
}etFPGAState;
/*****************************************************************************
** Class
*****************************************************************************/
class Parameters
{
public:
    Parameters();
    ~Parameters();

    double X_Swing_Range;
    double Y_Swing_Range;
    double Z_Swing_Range;
    double Y_Swing_Shift;
    int Period_T;
    int Period_T2;
    int Period_COM_Y;
    int Sample_Time;
    double OSC_LockRange;
    double BASE_Default_Z;
    int Ts;
    double Vmax;
    double Vmin;
    double COM_Height;
    double abswaistx;

    double X_Swing_COM;
    double BASE_LIFT_Z;

//for kick
    double Kick_Point_X;
    double Kick_Point_Y;
    double Kick_Point_Z;
    double Back_Point_X;
    double Back_Point_Z;

    double Support_Foot_Hip_Upper_Pitch;
    double Support_Foot_Ankle_Upper_Pitch;
    double Kick_Foot_Ankle_Upper_Pitch;
    
// clock_t system_start;
// clock_t system_end;
};

class COMPlan
{
public:
    COMPlan();
    ~COMPlan();

    int sample_point_;
    int time_point_;
    bool isfirststep;
    bool islaststep;
    int walking_state;
    bool walking_stop;
    double lift_lock_x;
    bool isLfootfirst;
};
class Points
{

public:
    Points();
    ~Points();

    double X_Right_foot;
    double X_Left_foot;
    double Y_Right_foot;
    double Y_Left_foot;
    double Z_Right_foot;
    double Z_Left_foot;
    double X_COM;
    double Y_COM;
    double Z_COM;
    double Right_Thta;
    double Left_Thta;

    double wv_x_COM;
    double abs_x_feet;
    double foot_x_r;
    double foot_x_l;

    double IK_Point_RX;
    double IK_Point_RY;
    double IK_Point_RZ;
    double IK_Point_RThta;
    double IK_Point_LX;
    double IK_Point_LY;
    double IK_Point_LZ;
    double IK_Point_LThta;
};

class ParameterInfo
{

public:
    ParameterInfo();
    ~ParameterInfo();

    Parameters parameters;
    Points points;
    COMPlan complan;

    bool rosflag;
    bool doflag;
    bool walkingpointflag;
    double X;
    double Y;
    double Z;
    double THTA;

    bool hand;

    int walking_mode;
    int time_point_;
    //int walking_state;

    double XUpdate;
    double YUpdate;
    double ZUpdate;
    double THTAUpdate;

    bool WalkFlag;
    bool FpgaFlag;
    bool IsParametersLoad;
    int counter;

    int serialack;
    bool cpgack;
    bool Repeat;

    int *temp_motion_test;

    int control;
    int FPGAState;
    bool CPGalready;

    float w1[5],w2[5],w[25],rule[25];

    // std_msgs::Int16 NowStep;

    bool Isfrist_right_shift;
};


extern ParameterInfo* parameterinfo;
#endif /* PARAMETER_INFO_H_ */
