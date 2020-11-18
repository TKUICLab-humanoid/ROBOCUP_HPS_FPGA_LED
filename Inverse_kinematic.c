#include "include/Inverse_Kinematic.h"



struct Points_Struct Points;
struct Parameters_Struct Parameters;
struct Status_Struct RobotStatus;

extern Initial init;
extern Datamodule datamodule;
extern BalanceControl balance;
extern kickgait_space::KickingGait kickinggait;

Locus::Locus()
{

}

Locus::~Locus()
{
	
}

void Locus::do_motion()
{
	bool avalon_locus_idle = false;
	
	for(;;)
	{
		if(avalon_locus_idle)
		{
			*(uint32_t *)(init.h2p_avalon_locus_addr_1) = 0x01;
			usleep(100);
			*(uint32_t *)(init.h2p_avalon_locus_addr_1) = 0;
			break;
		}
		else
		{
			avalon_locus_idle = *(uint32_t *)(init.h2p_avalon_locus_addr_1);
		}
	}
}

void Locus::set_point_by_body()
{
	Points.Inverse_PointR_X = 0 - Points.Inverse_Pointbody_X;
	Points.Inverse_PointR_Y = 0 - Points.Inverse_Pointbody_Y;
	Points.Inverse_PointR_Z = COM_HEIGHT - Points.Inverse_Pointbody_Z;
	Points.Inverse_PiontR_Thta = Points.Inverse_Piontbody_Thta;
	Points.Inverse_PointL_X = 0 - Points.Inverse_Pointbody_X;
	Points.Inverse_PointL_Y = 0 - Points.Inverse_Pointbody_Y;
	Points.Inverse_PointL_Z = COM_HEIGHT - Points.Inverse_Pointbody_Z;
	Points.Inverse_PiontL_Thta = Points.Inverse_Piontbody_Thta;
}

void Locus::get_cpg_with_offset(){
	Points.Inverse_Uncontrol_PointR_X	= parameterinfo->points.IK_Point_RX;
    Points.Inverse_Uncontrol_PointR_Y	= parameterinfo->points.IK_Point_RY;
    Points.Inverse_PointR_Z				= parameterinfo->points.IK_Point_RZ;
    Points.Inverse_PiontR_Thta			= parameterinfo->points.IK_Point_RThta;
    Points.Inverse_Uncontrol_PointL_X	= parameterinfo->points.IK_Point_LX;
    Points.Inverse_Uncontrol_PointL_Y	= parameterinfo->points.IK_Point_LY;
    Points.Inverse_PointL_Z				= parameterinfo->points.IK_Point_LZ;
    Points.Inverse_PiontL_Thta			= parameterinfo->points.IK_Point_LThta;

	Points.Inverse_PointR_X = Points.Inverse_Uncontrol_PointR_X	+STAND_OFFSET_RX;
	Points.Inverse_PointR_Y = Points.Inverse_Uncontrol_PointR_Y	+STAND_OFFSET_RY;
	Points.Inverse_PointR_Z = Points.Inverse_PointR_Z			+STAND_OFFSET_RZ;
	Points.Inverse_PointL_X = Points.Inverse_Uncontrol_PointL_X	+STAND_OFFSET_LX;
	Points.Inverse_PointL_Y = Points.Inverse_Uncontrol_PointL_Y	+STAND_OFFSET_LY;
	Points.Inverse_PointL_Z = Points.Inverse_PointL_Z			+STAND_OFFSET_LZ;

	// printf("L_X: %f, L_Y: %f, L_Z: %f, L_T: %f\n", Points.Inverse_PointL_X, Points.Inverse_PointL_Y, Points.Inverse_PointL_Z, Points.Inverse_PiontL_Thta);
	// printf("R_X: %f, R_Y: %f, R_Z: %f, R_T: %f\n\n",Points.Inverse_PointR_X, Points.Inverse_PointR_Y, Points.Inverse_PointR_Z, Points.Inverse_PiontR_Thta);
	// printf("W_STATE = %d\tS_MODE = %d\tL_TIME = %d\n", walking_state_, sensor_mode_, (locus_time_&0xff));
}
void Locus::calculate_robot_status(){

	//利用末端點的y(身體會往反方向晃)得知此刻機器人重心在哪邊
	if(Points.Inverse_Uncontrol_PointL_Y < 0)
	{
		if(RobotStatus.LoR == RIGHT)
		{
			RobotStatus.Move_ok_Y = 0;
		}
		RobotStatus.LoR = LEFT;
	}
	else if(Points.Inverse_Uncontrol_PointL_Y > 0)
	{
		if(RobotStatus.LoR == LEFT)
		{
			RobotStatus.Move_ok_Y = 0;
		}
		RobotStatus.LoR = RIGHT;
	}else{
		RobotStatus.LoR = MID;
	}

	//此刻的末端點y與上一刻的末端點y比較，判斷重心正在向哪邊移動
	if(Points.Inverse_Uncontrol_PointL_Y < RobotStatus.Pre_Y)
	{
		RobotStatus.Turn_LoR = LEFT;
	}
	else if(Points.Inverse_Uncontrol_PointL_Y > RobotStatus.Pre_Y)
	{
		RobotStatus.Turn_LoR = RIGHT;
	}
	RobotStatus.Pre_Y = Points.Inverse_Uncontrol_PointL_Y;

	//藉由末端點的z座標，判斷哪一隻腳抬起來
	if(Points.Inverse_Uncontrol_PointR_Z > Points.Inverse_Uncontrol_PointL_Z){
		RobotStatus.Up_FooT = LEFT;
	}else if(Points.Inverse_Uncontrol_PointR_Z < Points.Inverse_Uncontrol_PointL_Z){
		RobotStatus.Up_FooT = RIGHT;
	}else{
		RobotStatus.Up_FooT = MID;
	}
}

void Locus::control_by_robot_status(){
	double tmp = 0;
	double base_L = PI_2;
	double base_R = PI_2;

	tmp = atan2(Points.Inverse_PointL_X, (Points.Inverse_PointL_Z+SHOULDER_TO_COM)) * PI_2_Angle;	//左手擺的幅度跟隨右腳往前的幅度
	tmp *= 0.1;

	if(tmp > PI_6)
		tmp = PI_6;
	else if(tmp < -PI_6)
		tmp = -PI_6;

	if(Points.Inverse_PointL_X > 0)
	{
		Points.Thta[0] = base_L - tmp;//base - (base - tmp);
		Points.Thta[4] = base_R - tmp;//base - (base - tmp);
	}
	else if(Points.Inverse_PointL_X < 0)
	{
		Points.Thta[0] = base_L - tmp;//base + (base - tmp);
		Points.Thta[4] = base_R - tmp;//base + (base - tmp);
	}
	else
	{
		Points.Thta[0] = base_L;
		Points.Thta[4] = base_R;
	}
}

InverseKinematic::InverseKinematic()
{
	rotate_body_l_ = 0.0;
	flag_ = false;

	name_cont_ = 0;
	old_walking_stop = true;
	std::vector<double> temp;
	// if(map_motor.empty())
	// {
		map_motor["motor_11"] = temp;
        map_motor["motor_15"] = temp;
        map_motor["motor_17"] = temp;
        map_motor["motor_21"] = temp;
	// }
}

InverseKinematic::~InverseKinematic()
{

}

void InverseKinematic::initial_angle_gain()
{

	angle_gain_[0] = ALL_Angle_Gain * 1;
	angle_gain_[1] = ALL_Angle_Gain * 1;	//1
	angle_gain_[2] = ALL_Angle_Gain * 1;
	angle_gain_[3] = ALL_Angle_Gain * 1;

	angle_gain_[4] = ALL_Angle_Gain * 1;
	angle_gain_[5] = ALL_Angle_Gain * 1;
	angle_gain_[6] = ALL_Angle_Gain * 1;
	angle_gain_[7] = ALL_Angle_Gain * 1;		//1

	angle_gain_[8] = ALL_Angle_Gain * 1;

	angle_gain_[9] = ALL_Angle_Gain * 1;
	angle_gain_[10] = ALL_Angle_Gain * 1;
	angle_gain_[11] = ALL_Angle_Gain * 1;
	angle_gain_[12] = ALL_Angle_Gain * 1;
	angle_gain_[13] = ALL_Angle_Gain * 1;
	angle_gain_[14] = ALL_Angle_Gain * 1;

	angle_gain_[15] = ALL_Angle_Gain * 1;
	angle_gain_[16] = ALL_Angle_Gain * 1;
	angle_gain_[17] = ALL_Angle_Gain * 1;
	angle_gain_[18] = ALL_Angle_Gain * 1;
	angle_gain_[19] = ALL_Angle_Gain * 1;
	angle_gain_[20] = ALL_Angle_Gain * 1;
}

void InverseKinematic::initial_speed_gain()
{
	speed_gain_[0] = ALL_Speed_Gain * 1;
	speed_gain_[1] = ALL_Speed_Gain * 1;//0.8;//2.3;//2;
	speed_gain_[2] = ALL_Speed_Gain * 1;//0.8;
	speed_gain_[3] = ALL_Speed_Gain * 1;//2;

	speed_gain_[4] = ALL_Speed_Gain * 1;
	speed_gain_[5] = ALL_Speed_Gain * 1;//2;//3;
	speed_gain_[6] = ALL_Speed_Gain * 1;
	speed_gain_[7] = ALL_Speed_Gain * 1;//0.8;//2.3;//2;

	speed_gain_[8] = ALL_Speed_Gain * 1;//0.8;

	speed_gain_[9] = ALL_Speed_Gain * 1;//2;
	speed_gain_[10] = ALL_Speed_Gain * 2;
	speed_gain_[11] = ALL_Speed_Gain * 1;//2;//3;
	speed_gain_[12] = ALL_Speed_Gain * 1;
	speed_gain_[13] = ALL_Speed_Gain * 1;
	speed_gain_[14] = ALL_Speed_Gain * 2;

	speed_gain_[15] = ALL_Speed_Gain * 1;
	speed_gain_[16] = ALL_Speed_Gain * 2;
	speed_gain_[17] = ALL_Speed_Gain * 1;
	speed_gain_[18] = ALL_Speed_Gain * 1;
	speed_gain_[19] = ALL_Speed_Gain * 1;
	speed_gain_[20] = ALL_Speed_Gain * 2;
}

void InverseKinematic::initial_inverse_kinematic()
{
	//Locus_time = 30;

	initial_parameters();
	
	double Motion_Delay = Parameters.Period_T/Parameters.Sample_Time;           //600 / 24 = 25
	initial_points();
	initial_points_process();

	Points.Inverse_PointR_X = Points.X_COM + Points.X_Right_foot;               //Parameters.COM_X_Offset + Parameters.R_X_Offset;
	Points.Inverse_PointR_Y = -Points.Y_COM + Points.Y_Right_foot;
	Points.Inverse_PointR_Z = Points.Z_COM - Points.Z_Right_foot;
	Points.Inverse_PiontR_Thta = Points.Right_Thta;                             //-pi/2~pi/2
	
	Points.Inverse_PointL_X = Points.X_COM + Points.X_Left_foot;
	Points.Inverse_PointL_Y = -Points.Y_COM + Points.Y_Left_foot;
	Points.Inverse_PointL_Z = Points.Z_COM - Points.Z_Left_foot; 
	Points.Inverse_PiontL_Thta = Points.Left_Thta;    
	
	// printf("\nR_X: %f, R_Y: %f, R_Z: %f, R_T: %f\nL_X: %f, L_Y: %f, L_Z: %f, L_T: %f\n\n",Points.Inverse_PointR_X, Points.Inverse_PointR_Y, Points.Inverse_PointR_Z, Points.Inverse_PiontR_Thta, Points.Inverse_PointL_X, Points.Inverse_PointL_Y, Points.Inverse_PointL_Z, Points.Inverse_PiontL_Thta);
	calculate_inverse_kinematic(Motion_Delay);
	int i;
	for(i = 0; i < 21; i++)//===============================================i=0>i=9
	{   
		if(Points.P_Table[i])
		{
			angle_[i] = Max_value - (Points.Thta[i] * PI_TO_OUTPUT + Position_Zero);
		}
		else
		{
			angle_[i] = Points.Thta[i] * PI_TO_OUTPUT + Position_Zero;
		}
		output_base_[i] = thta_base_[i] - angle_[i];
		output_angle_[i] = thta_base_[i];
		output_speed_[i] = 100;
		past_thta_[i] = Points.Thta[i];
		Points.UThta[i] = Points.Thta[i];
		delay_time_[i] = 256;
		past_delay_time_[i] = 256;
	}
	output_base_[14] += 0;
	output_base_[20] -= 0;
	output_base_[10] += 0;
	output_base_[16] -= 0;
	Parameters.Body_Pitch_tmp = Parameters.Body_Pitch;


}

void InverseKinematic::initial_parameters(){
	Parameters.Phase_Shift = PI;      //-pi~pi
	Parameters.X_Swing_Range = 1;     //cm
	Parameters.Y_Swing_Range = 3;     //cm
	Parameters.COM_Height = COM_HEIGHT;     //cm//////	21.7
	Parameters.l1 = 14;             //cm Upper	10th: 12.5 11st: 10.2
	Parameters.l2 = 14;             //cm Down	10th: 12.5 11st: 10.4
	//Parameters.R_X_Offset = 2.5;      //cm -5
	Parameters.R_X_Offset = 0;      //cm -5/////////////////////////////////////
	Parameters.R_Y_Offset = 0;        //cm -1.5
	Parameters.R_Z_Offset = 0;        //cm
	//Parameters.L_X_Offset = 2.5;      //cm -5
	Parameters.L_X_Offset = 0;      //cm -5/////////////////////////////////////
	Parameters.L_Y_Offset = 0;        //cm 1.5
	Parameters.L_Z_Offset = 0;        //cm
	Parameters.COM_X_Offset = 0;      //cm 0
	Parameters.COM_Y_Offset = 0;      //cm -0.5
	Parameters.COM_Z_Offset = 0;      //cm
	Parameters.R_Open = -3;
	Parameters.L_Open = 3.5;
	//Parameters.Body_Pitch = -11*PI/180;
	Parameters.Body_Pitch = 0;         /////////////////////////////////////
	Parameters.Body_Pitch_tmp = 0;
	//---------------Period_Parameters---------------//
	Parameters.Phase = 0;
	Parameters.Phase_2 = 0;
	Parameters.Open_Phase = 0;
	Parameters.Period_T = 600;        //ms
	Parameters.Period_T2 = 300;       //ms
	Parameters.Sample_Time = 24;//24
	//--------------Walk_Parameters------------------//
	Parameters.Push_Rate = 0.40;      //?%
}
void InverseKinematic::initial_points()
{
	int i;
	for(i = 0; i < 9; i++)  
		Points.Thta[i] = PI_2;

	Points.P_Table[0] = 0;                     //Positive
	Points.P_Table[1] = 0;                     //Positive
	Points.P_Table[2] = 0;                     //Positive
	Points.P_Table[3] = 0;                     //Positive
	Points.P_Table[4] = 0;                     //Positive
	Points.P_Table[5] = 0;                     //Positive
	Points.P_Table[6] = 0;                     //Positive
	Points.P_Table[7] = 0;                     //Positive
	Points.P_Table[8] = 0;                     //Positive
	Points.P_Table[9] = 0;                     //Positive
	Points.P_Table[10] = 0;                    //Positive
	Points.P_Table[11] = 0;                    //Positive
	Points.P_Table[12] = 0;                    //Positive
	Points.P_Table[13] = 1;                    //Negitive
	Points.P_Table[14] = 1;                    //Negitive
	Points.P_Table[15] = 0;                    //Positive
	Points.P_Table[16] = 0;                    //Pogitive
	Points.P_Table[17] = 1;                    //Negitive
	Points.P_Table[18] = 1;                    //Negitive
	Points.P_Table[19] = 0;                    //Positive
	Points.P_Table[20] = 1;                    //Negitive

#ifdef Robot1    
	for(i = 0; i < 21; i++)
	{
		thta_base_[i] = datamodule.totalangle_[i];
	}
#else
	thta_base_[0] = 3044;
	thta_base_[1] = 466;
	thta_base_[2] = 511;
	thta_base_[3] = 464;
	thta_base_[4] = 1044;
	thta_base_[5] = 551;
	thta_base_[6] = 511;
	thta_base_[7] = 564;
	thta_base_[8] = 2048;
	thta_base_[9] = 2048;
	thta_base_[10] = 2048;
	thta_base_[11] = 1753;
	thta_base_[12] = 2637;
	thta_base_[13] = 2343;
	thta_base_[14] = 2048;
	thta_base_[15] = 2070;
	thta_base_[16] = 2048;
	thta_base_[17] = 2343;
	thta_base_[18] = 1460;
	thta_base_[19] = 1753;
	thta_base_[20] = 2048;
#endif
}

void InverseKinematic::initial_points_process()
{
	Points.Right_Thta = 0;                                                 //-pi/2~pi/2
	Points.Left_Thta = 0;                                                  //-pi/2~pi/2
	
	Points.X_Right_foot = Parameters.R_X_Offset;                          //cm
	Points.X_Left_foot = Parameters.L_X_Offset;                           //cm
	Points.X_COM =  Parameters.COM_X_Offset;                              //cm
	
	Points.Y_Right_foot = Parameters.R_Y_Offset ;                         //cm
	Points.Y_Left_foot = Parameters.L_Y_Offset;                           //cm
	Points.Y_COM = Parameters.COM_Y_Offset;                               //cm
	
	Points.Z_Right_foot =  Parameters.R_Z_Offset;                         //cm
	Points.Z_Left_foot = Parameters.L_Z_Offset;                           //cm
	Points.Z_COM = Parameters.COM_Height + Parameters.COM_Z_Offset;       //cm
}

void InverseKinematic::calculate_inverse_kinematic(int Motion_Delay)
{
    double R_Lyz, L_Lyz, R_Lxyz, L_Lxyz;
    double RX_2, RY_2, RZ_2, LX_2, LY_2, LZ_2;
    double l1_2, l2_2, l1_l2, RL_2, LL_2;
    int i;

	// printf("//////////////\nR_X: %f, R_Y: %f, R_Z: %f, R_T: %f\nL_X: %f, L_Y: %f, L_Z: %f, L_T: %f\n\n",Points.Inverse_PointR_X, Points.Inverse_PointR_Y, Points.Inverse_PointR_Z, Points.Inverse_PiontR_Thta, Points.Inverse_PointL_X, Points.Inverse_PointL_Y, Points.Inverse_PointL_Z, Points.Inverse_PiontL_Thta);

    RX_2 = Points.Inverse_PointR_X * Points.Inverse_PointR_X;
    RY_2 = Points.Inverse_PointR_Y * Points.Inverse_PointR_Y;
    RZ_2 = Points.Inverse_PointR_Z * Points.Inverse_PointR_Z;
    LX_2 = Points.Inverse_PointL_X * Points.Inverse_PointL_X;
    LY_2 = Points.Inverse_PointL_Y * Points.Inverse_PointL_Y;
    LZ_2 = Points.Inverse_PointL_Z * Points.Inverse_PointL_Z;
    l1_2 = Parameters.l1 * Parameters.l1; /////大腿長度平方
    l2_2 = Parameters.l2 * Parameters.l2; /////小腿長度平方
    l1_l2 = Parameters.l1 + Parameters.l2; /////大腿+小腿長度
    R_Lyz = sqrt(RY_2 + RZ_2);  // Lr for roll
    L_Lyz = sqrt(LY_2 + LZ_2); //Ll   for roll
    R_Lxyz = sqrt(RX_2 + RZ_2 + RY_2); //LR for pit
    L_Lxyz = sqrt(LX_2 + LZ_2 + LY_2); //LL for pit
    if(R_Lxyz > (l1_l2))
        R_Lxyz = l1_l2;
    if(L_Lxyz > (l1_l2))
        L_Lxyz = l1_l2;
    RL_2 = R_Lxyz * R_Lxyz;
    LL_2 = L_Lxyz * L_Lxyz;

    Points.Thta[9] = Points.Inverse_PiontL_Thta + PI_2;
    if(Points.Inverse_PointL_Y == 0)
    {
        Points.Thta[10] = PI_2;    //pi/2
    }
    else
    {
        Points.Thta[10] = atan2(Points.Inverse_PointL_Z, Points.Inverse_PointL_Y);//0.17896442525332481013377688544073-rotate_body_l_

    }

    if(Points.Inverse_PointL_X == 0)
    {
        Points.Thta[11] = PI_2 - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz));   //左腳髖關
    }
//    else if(Points.Inverse_PointL_X > 0)//////////////
//    {
//        Points.Thta[11] = PI - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(L_Lyz,-Points.Inverse_PointL_X);
//    }
//    else
//    {
//        Points.Thta[11] = PI - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(L_Lyz,-Points.Inverse_PointL_X);
//    }/////////////////////////////////////
    else if(Points.Inverse_PointL_X > 0)//////////////
    {
        Points.Thta[11] = PI_2 - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(Points.Inverse_PointL_X,L_Lyz);
    }
    else
    {
        Points.Thta[11] = PI - acos((l1_2 + LL_2 - l2_2)/(2*Parameters.l1*L_Lxyz)) - atan2(L_Lyz,-Points.Inverse_PointL_X);
    }/////////////////////////////////////

    Points.Thta[12] = PI - acos((l1_2 + l2_2 -LL_2)/(2*Parameters.l1*Parameters.l2)); //左膝
    Points.Thta[13] = PI - Points.Thta[11] -Points.Thta[12]; //左踝

    if(flag_ ==  0)
        Points.Thta[14] = PI - Points.Thta[10];
    else
        Points.Thta[14] = PI - Points.Thta[10]-rotate_body_l_;

    Points.Thta[15] = Points.Inverse_PiontR_Thta + PI_2;
  //  printf("Points.Inverse_PiontR_Thta = %f\n",Points.Inverse_PiontR_Thta);

    if(Points.Inverse_PointR_Y == 0)
    {
        Points.Thta[16] = PI_2;
    }
    else
    {
        Points.Thta[16] = atan2(Points.Inverse_PointR_Z, Points.Inverse_PointR_Y);
    }

    if(Points.Inverse_PointR_X == 0)
    {
        Points.Thta[17] = PI_2 - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz));
    }
//    else if(Points.Inverse_PointR_X > 0)////////////////////////
//    {
//        Points.Thta[17] = PI - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(R_Lyz,-Points.Inverse_PointR_X);
//    }
//    else
//    {
//        Points.Thta[17] = PI - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(R_Lyz,-Points.Inverse_PointR_X);
//    }///////////////////////////////
    else if(Points.Inverse_PointR_X > 0)////////////////////////
    {
        Points.Thta[17] = PI_2 - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(Points.Inverse_PointR_X,R_Lyz);
    }
    else
    {
        Points.Thta[17] = PI - acos((l1_2 + RL_2 - l2_2)/(2*Parameters.l1*R_Lxyz)) - atan2(R_Lyz,-Points.Inverse_PointR_X);
    }///////////////////////////////

    Points.Thta[18] = PI - acos((l1_2 + l2_2 - RL_2)/(2*Parameters.l1*Parameters.l2));
    Points.Thta[19] = PI - Points.Thta[17] - Points.Thta[18];

    if(flag_ ==  0)
        Points.Thta[20] = PI - Points.Thta[16];
    else
        Points.Thta[20] = PI - Points.Thta[16]-rotate_body_l_;

// be
	map_motor.find("motor_11")->second.push_back(Points.Thta[10] * PI_TO_OUTPUT);
	map_motor.find("motor_15")->second.push_back(Points.Thta[14] * PI_TO_OUTPUT);
	map_motor.find("motor_17")->second.push_back(Points.Thta[16] * PI_TO_OUTPUT);
	map_motor.find("motor_21")->second.push_back(Points.Thta[20] * PI_TO_OUTPUT);

	if(old_walking_stop == false && parameterinfo->complan.walking_stop == true)
	{
		balance.saveData();
		saveData();
	}
	old_walking_stop = parameterinfo->complan.walking_stop;

// end
	balance.control_after_ik_calculation();

	if(kickinggait.kicking_process_flag_)
	{
		kickinggait.hipPostureControl();
		kickinggait.ankleBalanceControl();
		kickinggait.hipPitchControl();
	}

    for( i = 0; i < 21; i++)
    {
		// if(i==12)
		// 	printf("thta 13 = %f, ag 13 = %f\t", Points.Thta[i], angle_gain_[i]);
		
		// Points.Thta[i] = Points.Thta[i] * angle_gain_[i];
		
		// if(i==12)
		// 	printf("thta 13 = %f, ag 13 = %f\n", Points.Thta[i], angle_gain_[i]);
        if(Points.P_Table[i])
        {
            output_angle_[i] = (unsigned int)(Max_value - (Points.Thta[i] * PI_TO_OUTPUT + Position_Zero));
        }
        else
        {
            output_angle_[i] = (unsigned int)(Points.Thta[i] * PI_TO_OUTPUT + Position_Zero);
        }
        output_angle_[i] += output_base_[i];

        double different_thta;
        different_thta = fabs( past_thta_[i] - Points.Thta[i]);
        if(different_thta > 0.0)
        {
        	delay_time_[i] = (unsigned int)(different_thta/(2*PI) * (1000/Motion_Delay) * 60 / 0.229);	// ((percent of circle(rad)) / ((delta t/1000)*60(min))) / 0.229(rpm/unit)
        }
        past_thta_[i] = Points.Thta[i];
        output_speed_[i] = delay_time_[i]  * SPEED_TRANS;
        output_speed_[i] = output_speed_[i] * speed_gain_[i];
		//----------------------printf-----------------------------
        #ifdef Auto_Stand
            if(i == 10 || i == 11 || i == 12 || i == 13 || i == 14   || i == 16 || i == 17 || i == 18 || i == 19)
                printf("%d:%d ", i+1, output_angle_[i]);
            else if(i == 20)
                printf("%d:%d\n", i+1, output_angle_[i]);
        #endif
        //---------------------------------------------------------
        if(output_angle_[i] >Max_value)
        {
            output_angle_[i] = Max_value;
        }
        else if(output_angle_[i] <= 0)
        {
            output_angle_[i] = 0;
        }
        if (output_speed_[i] <= 0)
        {
            output_speed_[i] = 0;
        }
        else if(output_speed_[i] > 32767)
        {
            output_speed_[i] = 32767;
		}
		// if(i>9 && i<15)
			// printf("IK %d:\t%d\tSP %d:\t%d\n", i+1, output_angle_[i], i+1, output_speed_[i]);
		*((uint32_t *)init.robot_motion_addr+(2*i+1)) = output_speed_[i];
		*((uint32_t *)init.robot_motion_addr+(2*i)) = output_angle_[i];
		// printf("32\n");
    }
	// printf("\n");
	*((uint32_t *)init.robot_motion_addr+(42)) = Motion_Delay;
	*((uint32_t *)init.robot_motion_addr+(43)) = 0x00000070;

    unsigned short blk_size = 0;
	// Header
	packet_char_[0] = 0xFF;
	packet_char_[1] = 0xFF;
	packet_char_[2] = 0xFD;
	// Reserved
	packet_char_[3] = 0x00;
	// ID
	packet_char_[4] = 0xFE;
	// Length      The length after the Packet Length field (Instruction, Parameter, CRC fields). Packet Length = number of Parameters + 3
	packet_char_[5] = 0x2b;       //0x2b = 43(decimal)
	packet_char_[6] = 0;
	// Instruction
	packet_char_[7] = 0x83;       //write         // 0x83 = sync write
	// Parameter
	packet_char_[8] = 0x70;       // addressL             // Velocity: 0x70 = 112 Position: 0x74(hex) = 116(dec)
	packet_char_[9] = 0;

	packet_char_[10] = 0x08;      // data length(byte)
	packet_char_[11] = 0x00;

	//left hand
	for(i=0; i<4; i++)
	{
		packet_char_[12+i*9] = i+1;      //MotorID
		packet_char_[13+i*9] = output_speed_[i] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[14+i*9] = (output_speed_[i] >> 8) & 0xFF;
		packet_char_[15+i*9] = (output_speed_[i] >> 16) & 0xFF;
		packet_char_[16+i*9] = (output_speed_[i] >> 24) & 0xFF;
		packet_char_[17+i*9] = output_angle_[i] & 0xFF;    // positionL
		packet_char_[18+i*9] = (output_angle_[i] >> 8) & 0xFF;
		packet_char_[19+i*9] = (output_angle_[i] >> 16) & 0xFF;
		packet_char_[20+i*9] = (output_angle_[i] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short lh_crc_value = update_crc(0, packet_char_, blk_size);

	//right hand
	for(i=0; i<4; i++)
	{
		packet_char_[12+i*9] = i+5;      //MotorID
		packet_char_[13+i*9] = output_speed_[i+4] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[14+i*9] = (output_speed_[i+4] >> 8) & 0xFF;
		packet_char_[15+i*9] = (output_speed_[i+4] >> 16) & 0xFF;
		packet_char_[16+i*9] = (output_speed_[i+4] >> 24) & 0xFF;
		packet_char_[17+i*9] = output_angle_[i+4] & 0xFF;    // positionL
		packet_char_[18+i*9] = (output_angle_[i+4] >> 8) & 0xFF;
		packet_char_[19+i*9] = (output_angle_[i+4] >> 16) & 0xFF;
		packet_char_[20+i*9] = (output_angle_[i+4] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short rh_crc_value = update_crc(0, packet_char_, blk_size);

	//foot
	packet_char_[5] = 0x46; // 0x46 = 70  77-7

	packet_char_[12] = 0x09;
	packet_char_[13] = output_speed_[8] & 0xFF;
	packet_char_[14] = (output_speed_[8] >> 8) & 0xFF;
	packet_char_[15] = (output_speed_[8] >> 16) & 0xFF;
	packet_char_[16] = (output_speed_[8] >> 24) & 0xFF;
	packet_char_[17] = output_angle_[8] & 0xFF;
	packet_char_[18] = (output_angle_[8] >> 8) & 0xFF;
	packet_char_[19] = (output_angle_[8] >> 16) & 0xFF;
	packet_char_[20] = (output_angle_[8] >> 24) & 0xFF;

	//left foot
	for(i=0; i<6; i++)
	{
		packet_char_[21+i*9] = i+10;      //MotorID
		packet_char_[22+i*9] = output_speed_[i+9] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[23+i*9] = (output_speed_[i+9] >> 8) & 0xFF;
		packet_char_[24+i*9] = (output_speed_[i+9] >> 16) & 0xFF;
		packet_char_[25+i*9] = (output_speed_[i+9] >> 24) & 0xFF;
		packet_char_[26+i*9] = output_angle_[i+9] & 0xFF;    // positionL
		packet_char_[27+i*9] = (output_angle_[i+9] >> 8) & 0xFF;
		packet_char_[28+i*9] = (output_angle_[i+9] >> 16) & 0xFF;
		packet_char_[29+i*9] = (output_angle_[i+9] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short lf_crc_value = update_crc(0, packet_char_, blk_size);

	//right foot
	for(i=0; i<6; i++)
	{
		packet_char_[21+i*9] = i+16;      //MotorID
		packet_char_[22+i*9] = output_speed_[i+15] & 0xFF;    //Profile Velocity      //initial value = 0
		packet_char_[23+i*9] = (output_speed_[i+15] >> 8) & 0xFF;
		packet_char_[24+i*9] = (output_speed_[i+15] >> 16) & 0xFF;
		packet_char_[25+i*9] = (output_speed_[i+15] >> 24) & 0xFF;
		packet_char_[26+i*9] = output_angle_[i+15] & 0xFF;    // positionL
		packet_char_[27+i*9] = (output_angle_[i+15] >> 8) & 0xFF;
		packet_char_[28+i*9] = (output_angle_[i+15] >> 16) & 0xFF;
		packet_char_[29+i*9] = (output_angle_[i+15] >> 24) & 0xFF;
	}

	blk_size = 5 + packet_char_[5];
	unsigned short rf_crc_value = update_crc(0, packet_char_, blk_size);

	*((uint32_t *)init.robot_motion_addr+(44)) = (lh_crc_value << 16) + rh_crc_value;
	*((uint32_t *)init.robot_motion_addr+(45)) = (lf_crc_value << 16) + rf_crc_value;

	// printf("%d",output_angle_[10]);

	
}

string InverseKinematic::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();
    return str;
}

void InverseKinematic::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/IK_motor"+tmp+".csv";
    strcat(path, tmp.c_str());

	
    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<double>>::iterator it_motor;

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		savedText += it_motor->first;
		if(it_motor == --map_motor.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_motor = map_motor.begin();
	int max_size = it_motor->second.size();

	for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
	{
		if(max_size < it_motor->second.size())
            max_size = it_motor->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        {
            if(i < it_motor->second.size())
            {
                if(it_motor == --map_motor.end())
                {
                    savedText += std::to_string(it_motor->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_motor->second[i]) + ",";
                }
            }
            else
            {
                if(it_motor == --map_motor.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_motor = map_motor.begin(); it_motor != map_motor.end(); it_motor++)
        it_motor->second.clear();

    name_cont_++;

}

unsigned short InverseKinematic::update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}
