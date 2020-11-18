#ifndef INVERSE_KINEMATIC_H_
#define INVERSE_KINEMATIC_H_

/******************* Include libarary*********************/
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <fstream>
#include <vector>
#include <map>
#include <iostream>

#include "alt_types.h"
#include "hps_0.h"
/********************************************************/

/******************* Include module**********************/
#include "Initial.h"
#include "Parameter_Info.h"
#include "Feedback_Control.h"
#include "DataModule.h"
#include "Rmotor.h"
#include "PID_Controller.h"
/********************************************************/

/******************* Define******************************/
#define PI      3.1415926535897932384626433832795   //pi
#define PI_2    1.5707963267948966192313216916398   //pi/2
#define PI_3_2  4.7123889803846898576939650749193   //3*pi/2
#define PI_3    1.0471975511965977461542144610932   //pi/3
#define PI_TO_OUTPUT 651.8986469044032953093479120548 //360/(2pi)*4096/360
#define PI_6	PI / 6.0

#define Position_Zero   1024                       //Zero of Position    180��
#define Position_PI_2   2047                        //pi/2 of Position
#define Position_PI     3072                       //pi of Position
#define Max_value       4095
#define Min_value       0
#define Max_speed       1023//20000
#define Gain_Min_speed  3000.0//4000.0
#define Min_speed       1//3000
#define ALL_Speed_Gain  1//2
#define ALL_Angle_Gain  1

#define Max_CPG_X		 30
#define Min_CPG_X		-30
#define Max_CPG_Y		 10
#define Min_CPG_Y		-10
#define Max_CPG_Z		 50
#define Min_CPG_Z		 30

#define SPEED_TRANS 32.0303030303030303030303030303 // (32767 / 1023)
//#define Debug_IK
/********************************************************/
#define COM_HEIGHT 24.3
//#define Length_Pelvis 18.5
//#define Length_Leg 30
#define STAND_OFFSET_RX 0//-2.8
#define STAND_OFFSET_RY 0//-0.3
#define STAND_OFFSET_RZ 0//0.2
#define STAND_OFFSET_LX 0//-0.6
#define STAND_OFFSET_LY 0//0.2
#define STAND_OFFSET_LZ 0//-0.1

#define SHOULDER_TO_COM 19.5

#define MID 0
#define STAND 0
#define RIGHT 1
#define LEFT 2

/******************* Debug**** **************************/
#define Inverse_kinematic_Debug
//#define One_Motion
#define One_Step
//#define Use_Speed_min_gain
#define Use_PID_Speed
/********************************************************/
/******************* Parameter **************************/
#define Robot1
 
/********************************************************/

struct Points_Struct{
	int P_Table[21];
	double UThta[21];
	double Thta[21];
	double Old_Thta[21];
    double Right_Thta;          //-pi/2~pi/2
    double Left_Thta;           //-pi/2~pi/2
    double X_Right_foot;        //cm
    double X_Left_foot;         //cm
    double X_COM;               //cm
    double Y_Right_foot;        //cm
    double Y_Left_foot;         //cm
    double Y_COM;               //cm
    double Z_Right_foot;        //cm
    double Z_Left_foot;         //cm
    double Z_COM;               //cm
    double Inverse_PointR_X;    //cm
    double Last_Inverse_PointR_X;//cm
    double Inverse_PointR_Y;    //cm
    double Last_Inverse_PointR_Y;//cm
    double Inverse_PointR_Z;    //cm
    double Inverse_PiontR_Thta; //-pi/2~pi/2
    double Inverse_PointL_X;    //cm
    double Inverse_PointL_Y;    //cm
    double Inverse_PointL_Z;    //cm
    double Inverse_PiontL_Thta; //-pi/2~pi/2
    double Inverse_Pointbody_X; //cm
    double Inverse_Pointbody_Y; //cm
    double Inverse_Pointbody_Z; //cm
    double Inverse_Piontbody_Thta;//-pi/2~pi/2
    
    
    //===================UNCONTROL===============================
    double Inverse_Uncontrol_PointR_X;    //cm
    double Inverse_Uncontrol_PointR_Y;    //cm
    double Inverse_Uncontrol_PointR_Z;    //cm
    double Inverse_Uncontrol_PointL_X;    //cm
    double Inverse_Uncontrol_PointL_Y;    //cm
    double Inverse_Uncontrol_PointL_Z;    //cm
};

struct Parameters_Struct{
    double Phase_Shift;     //-pi~pi
    double X_Swing_Range;   //cm
    double Y_Swing_Range;   //cm
    double COM_Height;      //cm
    double l1;              //cm
    double l2;              //cm
    double R_X_Offset;      //cm
    double R_Y_Offset;      //cm
    double R_Z_Offset;      //cm
    double L_X_Offset;      //cm
    double L_Y_Offset;      //cm
    double L_Z_Offset;      //cm
    double COM_X_Offset;    //cm
    double COM_Y_Offset;    //cm
    double COM_Z_Offset;    //cm
    double R_Open;
    double L_Open;
    double Body_Pitch;      //-pi~pi
    double Body_Pitch_tmp;      //-pi~pi
    double Body_Roll;       //-pi~pi
    double Body_Yaw;        //-pi~pi
   //---------------Period_parameters---------------//
    int Period_T;           //ms
    int Period_T2;          //ms
    int Sample_Time;
    double Phase;
    double Phase_2; 
    double Open_Phase;
//--------------Walk_Parameters------------------//
    double Push_Rate;       //?%
//--------------Foot_Motor_Compensate------------------//
    double Thta[12];

};

struct Status_Struct{
	int LoR;
	int Turn_LoR;
	double Pre_Y;
	int Move_ok_Y;
	int Up_FooT;
};

class Locus
{
public:
    Locus();
    ~Locus();

    void do_motion();
    void set_point_by_body();
    void get_cpg_with_offset();
    void calculate_robot_status();
    void control_by_robot_status();

    int locus_data_[8];
    int locus_time_;
    unsigned char walking_state_;
    unsigned char sensor_mode_;
};

class InverseKinematic
{
public:
    InverseKinematic();
    ~InverseKinematic();

    void initial_angle_gain();
    void initial_speed_gain();
    void initial_inverse_kinematic();
    void initial_points();
    void initial_parameters();
    void initial_points_process();
    void calculate_inverse_kinematic(int Motion_Delay);
    std::string DtoS(double value);
	std::map<std::string, std::vector<double>> map_motor;       
    void saveData();
    unsigned short update_crc(unsigned short , unsigned char *, unsigned short);

    unsigned char packet_char_[203];
    
private:
    double speed_gain_[21];
    double angle_gain_[21];
    int angle_[21];
    int output_base_[21];
    int thta_base_[21];
    double past_thta_[21];
    double delay_time_[21];
    double past_delay_time_[21];
    unsigned int output_speed_[21];
    unsigned int output_angle_[21];
    double rotate_body_l_;
    bool flag_;
    int name_cont_;
    bool old_walking_stop;
 
};

#endif /*INVERSE_KINEMATIC_H_*/
