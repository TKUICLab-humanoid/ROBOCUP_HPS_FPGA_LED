#include "include/Feedback_Control.h"
// #include "include/data_txt.h"
/////////////////////////posture///////////////////////
FuzzyController fuzzy;

extern struct Points_Struct Points;
extern Locus locus;
extern SensorDataProcess sensor;
extern Walkinggait walkinggait;

BalanceControl::BalanceControl()
{
	
	original_ik_point_rz_ = 0.0;
	original_ik_point_lz_ = 0.0;
	last_step_ = StopStep;
	now_step_ = StopStep;
	sup_foot_ = doublefeet;

	name_cont_ = 0;

	ZMP_process = new ZMPProcess;
	// initialize(30);
}

BalanceControl::~BalanceControl()
{
	delete ZMP_process;
}

void BalanceControl::initialize(const int control_cycle_msec)
{
	for(int i = 0; i < sizeof(init_imu_value)/sizeof(init_imu_value[0]); i++)
        init_imu_value[i].initialize();
    for(int i = 0; i < sizeof(pres_imu_value)/sizeof(pres_imu_value[0]); i++)
        pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(prev_imu_value)/sizeof(prev_imu_value[0]); i++)
        prev_imu_value[i].initialize();
    for(int i = 0; i < sizeof(ideal_imu_value)/sizeof(ideal_imu_value[0]); i++)
        ideal_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_pres_imu_value)/sizeof(passfilter_pres_imu_value[0]); i++)
        passfilter_pres_imu_value[i].initialize();
    for(int i = 0; i < sizeof(passfilter_prev_imu_value)/sizeof(passfilter_prev_imu_value[0]); i++)
        passfilter_prev_imu_value[i].initialize();

    leftfoot_hip_roll_value.initialize();
    leftfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();
    rightfoot_hip_pitch_value.initialize();
	CoM_EPx_value.initialize();

    PIDleftfoot_hip_roll.initParam();
    PIDleftfoot_hip_pitch.initParam();
	PIDrightfoot_hip_roll.initParam();
    PIDrightfoot_hip_pitch.initParam();
	PIDleftfoot_zmp_x.initParam();
	PIDleftfoot_zmp_y.initParam();
	PIDrightfoot_zmp_x.initParam();
	PIDrightfoot_zmp_y.initParam();
	PIDCoM_x.initParam();

    for(int i = 0; i < sizeof(butterfilter_imu)/sizeof(butterfilter_imu[0]); i++)
        butterfilter_imu[i].initialize();

    PIDleftfoot_hip_roll.setValueLimit(80, -80);
    PIDleftfoot_hip_pitch.setValueLimit(150, -150);
    PIDleftfoot_hip_roll.setKpid(0.012, 0, 0);
    PIDleftfoot_hip_pitch.setKpid(0.03, 0, 0);
    PIDleftfoot_hip_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDleftfoot_hip_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDrightfoot_hip_roll.setValueLimit(80, -80);
    PIDrightfoot_hip_pitch.setValueLimit(150, -150);
    PIDrightfoot_hip_roll.setKpid(0.012, 0, 0);
    PIDrightfoot_hip_pitch.setKpid(0.03, 0, 0);
    PIDrightfoot_hip_roll.setControlGoal(init_imu_value[(int)imu::roll].pos);
    PIDrightfoot_hip_pitch.setControlGoal(init_imu_value[(int)imu::pitch].pos);

	PIDleftfoot_zmp_x.setValueLimit(7, -7);
	PIDleftfoot_zmp_y.setValueLimit(7, -7);
	PIDleftfoot_zmp_x.setKpid(0.0125, 0, 0);
	PIDleftfoot_zmp_y.setKpid(0.0125, 0, 0);
	PIDleftfoot_zmp_x.setControlGoal(0);
	PIDleftfoot_zmp_y.setControlGoal(4.5);

	PIDrightfoot_zmp_x.setValueLimit(7, -7);
	PIDrightfoot_zmp_y.setValueLimit(7, -7);
	PIDrightfoot_zmp_x.setKpid(0.0125, 0, 0);
	PIDrightfoot_zmp_y.setKpid(0.0125, 0, 0);
	PIDrightfoot_zmp_x.setControlGoal(0);
	PIDrightfoot_zmp_y.setControlGoal(-4.5);
	
	PIDCoM_x.setValueLimit(7, -7);
	PIDCoM_x.setKpid(0.02, 0, 0);
	PIDCoM_x.setControlGoal(0);

	leftfoot_hip_roll = 0;
    leftfoot_hip_pitch = 0;
    leftfoot_ankle_roll = 0;
    leftfoot_ankle_pitch = 0;
	rightfoot_hip_roll = 0;
    rightfoot_hip_pitch = 0;
    rightfoot_ankle_roll = 0;
    rightfoot_ankle_pitch = 0;

	for(int i = 0; i < 3; i++)init_imu_value[i].pos = sensor.rpy_[i];


	std::vector<float> temp;
	if(map_roll.empty())
	{
		map_roll["init_roll_pos"] = temp;
        map_roll["smaple_times_count"] = temp;
        map_roll["pres_roll_pos"] = temp;
        map_roll["passfilter_pres_roll_pos"] = temp;
        map_roll["ideal_roll_vel"] = temp;
        map_roll["pres_roll_vel"] = temp;
        map_roll["passfilter_pres_roll_vel"] = temp;
        map_roll["left_control_once_roll"] = temp;
        map_roll["left_control_total_roll"] = temp;
		map_roll["right_control_once_roll"] = temp;
        map_roll["right_control_total_roll"] = temp;
	}

	if(map_pitch.empty())
	{
		map_pitch["init_pitch_pos"] = temp;
        map_pitch["smaple_times_count"] = temp;
        map_pitch["pres_pitch_pos"] = temp;
        map_pitch["passfilter_pres_pitch_pos"] = temp;
        map_pitch["ideal_pitch_vel"] = temp;
        map_pitch["pres_pitch_vel"] = temp;
        map_pitch["passfilter_pres_pitch_vel"] = temp;
        map_pitch["left_control_once_pitch"] = temp;
        map_pitch["left_control_total_pitch"] = temp;
		map_pitch["right_control_once_pitch"] = temp;
        map_pitch["right_control_total_pitch"] = temp;
	}

	if(map_ZMP.empty())
	{
		map_ZMP["pres_ZMP_left_pos_x"] = temp;
        map_ZMP["pres_ZMP_left_pos_y"] = temp;
        map_ZMP["pres_ZMP_right_pos_x"] = temp;
        map_ZMP["pres_ZMP_right_pos_y"] = temp;
        map_ZMP["pres_ZMP_feet_pos_x"] = temp;
        map_ZMP["pres_ZMP_feet_pos_y"] = temp;

        map_ZMP["delta_v"] = temp;

        map_ZMP["raw_sensor_data_0"] = temp;
        map_ZMP["raw_sensor_data_1"] = temp;
        map_ZMP["raw_sensor_data_2"] = temp;
        map_ZMP["raw_sensor_data_3"] = temp;
        map_ZMP["raw_sensor_data_4"] = temp;
        map_ZMP["raw_sensor_data_5"] = temp;
        map_ZMP["raw_sensor_data_6"] = temp;
        map_ZMP["raw_sensor_data_7"] = temp;

        map_ZMP["sensor_force_0"] = temp;
        map_ZMP["sensor_force_1"] = temp;
        map_ZMP["sensor_force_2"] = temp;
        map_ZMP["sensor_force_3"] = temp;
        map_ZMP["sensor_force_4"] = temp;
        map_ZMP["sensor_force_5"] = temp;
        map_ZMP["sensor_force_6"] = temp;
        map_ZMP["sensor_force_7"] = temp;

		map_ZMP["leftfoot_control_once_EPx"] = temp;
		map_ZMP["leftfoot_control_once_EPy"] = temp;
		map_ZMP["leftfoot_control_total_EPx"] = temp;
		map_ZMP["leftfoot_control_total_EPy"] = temp;

		map_ZMP["rightfoot_control_once_EPx"] = temp;
		map_ZMP["rightfoot_control_once_EPy"] = temp;
		map_ZMP["rightfoot_control_total_EPx"] = temp;
		map_ZMP["rightfoot_control_total_EPy"] = temp;

		map_ZMP["new_EP_lx"] = temp;
		map_ZMP["new_EP_rx"] = temp;
		map_ZMP["new_EP_ly"] = temp;
		map_ZMP["new_EP_ry"] = temp;
	}

	if(map_CoM.empty())
	{
		map_CoM["CoM_x_control"] = temp;

		map_CoM["new_EP_lx"] = temp;
		map_CoM["new_EP_rx"] = temp;
	}

	map_roll.find("init_roll_pos")->second.push_back(init_imu_value[(int)imu::roll].pos);
	map_pitch.find("init_pitch_pos")->second.push_back(init_imu_value[(int)imu::pitch].pos);
}

void BalanceControl::get_sensor_value()
{
	int i;
	double rpy_radian[3] = {0};
	for(int i=0; i<3; i++)
		rpy_radian[i] = sensor.rpy_[i] * DEGREE2RADIAN;
	
	roll_imu_filtered_ = roll_imu_lpf_.get_filtered_output(rpy_radian[0]);
	pitch_imu_filtered_ = pitch_imu_lpf_.get_filtered_output(rpy_radian[1]);
	roll_over_limit_ = (fabs(roll_imu_filtered_) > 0.209 ? true : false);
	pitch_over_limit_ = (fabs(pitch_imu_filtered_) > 0.297 ? true : false);
	two_feet_grounded_ = (original_ik_point_rz_ == original_ik_point_lz_ ? true : false);

	cog_roll_offset_ = sensor.imu_desire_[0] * DEGREE2RADIAN;
	cog_pitch_offset_ = sensor.imu_desire_[1] * DEGREE2RADIAN;
	double cog_y_filtered = roll_imu_filtered_ - cog_roll_offset_;
	double cog_x_filtered = pitch_imu_filtered_ - cog_pitch_offset_;
	if(Points.Inverse_PointR_Z != Points.Inverse_PointL_Z)
	{
		foot_cog_x_ = 26.2 * sin(cog_x_filtered);
		foot_cog_y_ = 26.2 * sin(cog_y_filtered);
	}
	else
	{
		foot_cog_x_ = 0;
		foot_cog_y_ = 0;
	}

    if( (fabs( rpy_radian[0] ) > RPY_ROLL_LIMIT || ( rpy_radian[1] ) > RPY_PITCH_LIMIT) && sensor.fall_Down_Flag_ == false) //over plus limit >> forward fall down
    {
		if(sensor.fall_Down_Status_ == 'F')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'F';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'F';			
		}

    }
	else if(  ( rpy_radian[1] ) < -(RPY_PITCH_LIMIT)  && sensor.fall_Down_Flag_ == false) //over minus limit >> backward fall down
    {
		if(sensor.fall_Down_Status_ == 'B')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'B';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'B';			
		}
    }
	else if( (fabs( rpy_radian[0] ) < RPY_STAND_RANGE && fabs( rpy_radian[1] ) < RPY_STAND_RANGE) && sensor.fall_Down_Flag_ == true) 
    {
		if(sensor.fall_Down_Status_ == 'S')
		{
			gettimeofday(&timer_end_, NULL);
			sensor.fall_Down_Status_ = 'S';
		}
		else
		{
			gettimeofday(&timer_start_, NULL);
			sensor.fall_Down_Status_ = 'S';			
		}	
    }
	else
	{
		gettimeofday(&timer_start_, NULL);
		sensor.fall_Down_Status_ = sensor.fall_Down_Status_ ;
	}
	
	timer_dt_ = (double)(1000000.0 * (timer_end_.tv_sec - timer_start_.tv_sec) + (timer_end_.tv_usec - timer_start_.tv_usec));	

	if(timer_dt_ >= 2000000.0 && (sensor.fall_Down_Status_ == 'F' || sensor.fall_Down_Status_ == 'B') )//timer_dt_ unit is us 1sec = 1000000
	{
		sensor.fall_Down_Flag_ = true;
	}
	else if(timer_dt_ >= 4000000.0 && sensor.fall_Down_Status_ == 'S')
	{
		sensor.fall_Down_Flag_ = false;
	}
	else
	{
		sensor.fall_Down_Flag_ = sensor.fall_Down_Flag_;		
	}

}

void BalanceControl::setSupportFoot()
{
	original_ik_point_rz_ = parameterinfo->points.IK_Point_RZ;
	original_ik_point_lz_ = parameterinfo->points.IK_Point_LZ;

	pre_sup_foot_ = sup_foot_;
	if(parameterinfo->points.IK_Point_RZ != parameterinfo->points.IK_Point_LZ)
	{
		if(parameterinfo->points.IK_Point_RY > 0)
			sup_foot_ = rightfoot;
		else if(parameterinfo->points.IK_Point_RY < 0)
			sup_foot_ = leftfoot;
	}
	else
	{
		sup_foot_ = doublefeet;
	}
}

void BalanceControl::resetControlValue()
{
	leftfoot_hip_pitch_value.initialize();
	leftfoot_hip_roll_value.initialize();
	rightfoot_hip_pitch_value.initialize();
	rightfoot_hip_roll_value.initialize();

	leftfoot_EPx_value.initialize();
	leftfoot_EPy_value.initialize();
	rightfoot_EPx_value.initialize();
	rightfoot_EPy_value.initialize();
}




void BalanceControl::balance_control()
{
	int i;
	LinearAlgebra LA;

	// original_ik_point_rz_ = parameterinfo->points.IK_Point_RZ;
	// original_ik_point_lz_ = parameterinfo->points.IK_Point_LZ;

	for(i=0; i<3; i++)prev_imu_value[i].pos = pres_imu_value[i].pos;
    for(i=0; i<3; i++)pres_imu_value[i].pos = sensor.rpy_[i];

	//check if switch foot or not
	// if((pre_sup_foot_ != sup_foot_) && (sup_foot_ != doublefeet))
	// {
	//		
	// }

	//----------- pitch ---------------------
	pres_imu_value[(int)imu::pitch].vel = (pres_imu_value[(int)imu::pitch].pos-prev_imu_value[(int)imu::pitch].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::pitch].pos = butterfilter_imu[(int)imu::pitch].pos.getValue(pres_imu_value[(int)imu::pitch].pos);
	passfilter_pres_imu_value[(int)imu::pitch].vel = butterfilter_imu[(int)imu::pitch].vel.getValue(pres_imu_value[(int)imu::pitch].vel);
	passfilter_prev_imu_value[(int)imu::pitch] = passfilter_pres_imu_value[(int)imu::pitch];
	
	//----------- roll ----------------------
	pres_imu_value[(int)imu::roll].vel = (pres_imu_value[(int)imu::roll].pos-prev_imu_value[(int)imu::roll].pos)/(0.03);
	passfilter_pres_imu_value[(int)imu::roll].pos = butterfilter_imu[(int)imu::roll].pos.getValue(pres_imu_value[(int)imu::roll].pos);
	passfilter_pres_imu_value[(int)imu::roll].vel = butterfilter_imu[(int)imu::roll].vel.getValue(pres_imu_value[(int)imu::roll].vel);
	passfilter_prev_imu_value[(int)imu::roll] = passfilter_pres_imu_value[(int)imu::roll];

	PIDleftfoot_hip_pitch.setControlGoal(ideal_imu_value[(int)imu::pitch].vel);
	PIDleftfoot_hip_roll.setControlGoal(ideal_imu_value[(int)imu::roll].vel);

	CoM_EPx_value.control_value_once = 0.5 * PIDCoM_x.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].pos);

	parameterinfo->points.IK_Point_LX -= CoM_EPx_value.control_value_once;
	parameterinfo->points.IK_Point_RX -= CoM_EPx_value.control_value_once;

	if(sup_foot_ == leftfoot)
	{
		//sup
		//----------- pitch ---------------------
		leftfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		leftfoot_hip_pitch_value.control_value_total += leftfoot_hip_pitch_value.control_value_once;

		leftfoot_hip_pitch -= leftfoot_hip_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		leftfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		leftfoot_hip_roll_value.control_value_total += leftfoot_hip_roll_value.control_value_once;
		
		leftfoot_hip_roll += leftfoot_hip_roll_value.control_value_total/180.0*PI;

		//swing
		rightfoot_hip_pitch_value.initialize();
		rightfoot_hip_roll_value.initialize();
		rightfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		rightfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
	}
	else if(sup_foot_ == rightfoot)
	{
		//sup
		//----------- pitch ---------------------
		rightfoot_hip_pitch_value.control_value_once = PIDleftfoot_hip_pitch.calculateExpValue(passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
		rightfoot_hip_pitch_value.control_value_total += rightfoot_hip_pitch_value.control_value_once;

		rightfoot_hip_pitch -= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		//----------- roll ----------------------
		rightfoot_hip_roll_value.control_value_once = PIDleftfoot_hip_roll.calculateExpValue_roll(passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
		rightfoot_hip_roll_value.control_value_total += rightfoot_hip_roll_value.control_value_once;
		
		rightfoot_hip_roll += rightfoot_hip_roll_value.control_value_total/180.0*PI;

		//swing
		leftfoot_hip_pitch_value.initialize();
		leftfoot_hip_roll_value.initialize();
		leftfoot_hip_pitch = 0;//-= rightfoot_hip_pitch_value.control_value_total/180.0*PI;
		leftfoot_hip_roll = 0;//+= rightfoot_hip_roll_value.control_value_total/180.0*PI;
	}
	// else if(sup_foot_ == doublefeet)
	// {
	// 	leftfoot_hip_pitch_value.initialize();
	// 	leftfoot_hip_roll_value.initialize();
	// 	rightfoot_hip_pitch_value.initialize();
	// 	rightfoot_hip_roll_value.initialize();
	// }

	map_roll.find("left_control_once_roll")->second.push_back(leftfoot_hip_roll_value.control_value_once);
	map_roll.find("left_control_total_roll")->second.push_back(leftfoot_hip_roll_value.control_value_total);
	map_roll.find("right_control_once_roll")->second.push_back(rightfoot_hip_roll_value.control_value_once);
	map_roll.find("right_control_total_roll")->second.push_back(rightfoot_hip_roll_value.control_value_total);
	map_roll.find("smaple_times_count")->second.push_back(30);
    map_roll.find("pres_roll_pos")->second.push_back(pres_imu_value[(int)imu::roll].pos);
    map_roll.find("passfilter_pres_roll_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].pos);
    map_roll.find("ideal_roll_vel")->second.push_back(ideal_imu_value[(int)imu::roll].vel);
    map_roll.find("pres_roll_vel")->second.push_back(pres_imu_value[(int)imu::roll].vel);
    map_roll.find("passfilter_pres_roll_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::roll].vel);

	map_pitch.find("left_control_once_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_once);
	map_pitch.find("left_control_total_pitch")->second.push_back(leftfoot_hip_pitch_value.control_value_total);
	map_pitch.find("right_control_once_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_once);
	map_pitch.find("right_control_total_pitch")->second.push_back(rightfoot_hip_pitch_value.control_value_total);
	map_pitch.find("smaple_times_count")->second.push_back(30);
    map_pitch.find("pres_pitch_pos")->second.push_back(pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("passfilter_pres_pitch_pos")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("ideal_pitch_vel")->second.push_back(ideal_imu_value[(int)imu::pitch].vel);
    map_pitch.find("pres_pitch_vel")->second.push_back(pres_imu_value[(int)imu::pitch].vel);
    map_pitch.find("passfilter_pres_pitch_vel")->second.push_back(passfilter_pres_imu_value[(int)imu::pitch].vel);

	map_CoM.find("CoM_x_control")->second.push_back(CoM_EPx_value.control_value_once);
	map_CoM.find("new_EP_lx")->second.push_back(parameterinfo->points.IK_Point_LX);
	map_CoM.find("new_EP_rx")->second.push_back(parameterinfo->points.IK_Point_RX);
}

void BalanceControl::endPointControl()
{
	int raw_sensor_data_tmp[8];
	for(int i=0; i<4; i++)raw_sensor_data_tmp[i] = sensor.press_left_[i];
	for(int i=4; i<8; i++)raw_sensor_data_tmp[i] = sensor.press_right_[i-4];
	prev_ZMP = pres_ZMP;
	ZMP_process->setpOrigenSensorData(raw_sensor_data_tmp);
	pres_ZMP = ZMP_process->getZMPValue();

	// map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(pres_ZMP.left_pos.x);
	// map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(pres_ZMP.left_pos.y);
	// map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(pres_ZMP.right_pos.x);
	// map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(pres_ZMP.right_pos.y);
	// map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(pres_ZMP.feet_pos.x);
	// map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(pres_ZMP.feet_pos.y);
	
	pres_ZMP.feet_pos.x = foot_cog_x_;
	pres_ZMP.feet_pos.y = foot_cog_y_;

	double *sensor_force = ZMP_process->getpSensorForce();
	int *raw_sensor_data = ZMP_process->getpOrigenSensorData();

	map_ZMP.find("sensor_force_0")->second.push_back(sensor_force[0]);
	map_ZMP.find("sensor_force_1")->second.push_back(sensor_force[1]);
	map_ZMP.find("sensor_force_2")->second.push_back(sensor_force[2]);
	map_ZMP.find("sensor_force_3")->second.push_back(sensor_force[3]);
	map_ZMP.find("sensor_force_4")->second.push_back(sensor_force[4]);
	map_ZMP.find("sensor_force_5")->second.push_back(sensor_force[5]);
	map_ZMP.find("sensor_force_6")->second.push_back(sensor_force[6]);
	map_ZMP.find("sensor_force_7")->second.push_back(sensor_force[7]);

	map_ZMP.find("raw_sensor_data_0")->second.push_back(raw_sensor_data[0]);
	map_ZMP.find("raw_sensor_data_1")->second.push_back(raw_sensor_data[1]);
	map_ZMP.find("raw_sensor_data_2")->second.push_back(raw_sensor_data[2]);
	map_ZMP.find("raw_sensor_data_3")->second.push_back(raw_sensor_data[3]);
	map_ZMP.find("raw_sensor_data_4")->second.push_back(raw_sensor_data[4]);
	map_ZMP.find("raw_sensor_data_5")->second.push_back(raw_sensor_data[5]);
	map_ZMP.find("raw_sensor_data_6")->second.push_back(raw_sensor_data[6]);
	map_ZMP.find("raw_sensor_data_7")->second.push_back(raw_sensor_data[7]);

	if(sup_foot_ == leftfoot)
	{
		leftfoot_EPx_value.control_value_once = PIDleftfoot_zmp_x.calculateExpValue(pres_ZMP.feet_pos.x);
		leftfoot_EPx_value.control_value_total += leftfoot_EPx_value.control_value_once;

		// parameterinfo->points.IK_Point_LX += leftfoot_EPx_value.control_value_total;
		// parameterinfo->points.IK_Point_RX += leftfoot_EPx_value.control_value_total;

		rightfoot_EPx_value.initialize();
		rightfoot_EPy_value.initialize();
	}
	else if(sup_foot_ == rightfoot)
	{
		rightfoot_EPx_value.control_value_once = PIDleftfoot_zmp_x.calculateExpValue(pres_ZMP.feet_pos.x);
		rightfoot_EPx_value.control_value_total += rightfoot_EPx_value.control_value_once;

		// parameterinfo->points.IK_Point_LX += rightfoot_EPx_value.control_value_total;
		// parameterinfo->points.IK_Point_RX += rightfoot_EPx_value.control_value_total;

		leftfoot_EPx_value.initialize();
		leftfoot_EPy_value.initialize();
	}

	map_ZMP.find("leftfoot_control_once_EPx")->second.push_back(leftfoot_EPx_value.control_value_once);
	map_ZMP.find("leftfoot_control_total_EPx")->second.push_back(leftfoot_EPx_value.control_value_total);
	map_ZMP.find("rightfoot_control_once_EPx")->second.push_back(rightfoot_EPx_value.control_value_once);
	map_ZMP.find("rightfoot_control_total_EPx")->second.push_back(rightfoot_EPx_value.control_value_total);

	// float LIPM_vel = (float)(-(supfoot_EPx_value.control_value_total/walkinggait.Tc_));

	map_ZMP.find("new_EP_lx")->second.push_back(parameterinfo->points.IK_Point_LX);
	map_ZMP.find("new_EP_rx")->second.push_back(parameterinfo->points.IK_Point_RX);
	map_ZMP.find("new_EP_ly")->second.push_back(parameterinfo->points.IK_Point_LY);
	map_ZMP.find("new_EP_ry")->second.push_back(parameterinfo->points.IK_Point_RY);
}

float BalanceControl::calculateCOMPosbyLIPM(float pos_adj, float vel)
{
	// return pos_adj*walkinggait.cosh(walkinggait.t_/walkinggait.Tc_)+walkinggait.Tc_*vel*walkinggait.sinh(walkinggait.t_/walkinggait.Tc_);
	return pos_adj*(walkinggait.cosh(walkinggait.t_/walkinggait.Tc_)-walkinggait.sinh(walkinggait.t_/walkinggait.Tc_));
}

void BalanceControl::control_after_ik_calculation()
{
	if(sup_foot_ == leftfoot)
	{
		Points.Thta[10] += leftfoot_hip_roll;
		Points.Thta[11] += leftfoot_hip_pitch;
		Points.Thta[13] += leftfoot_ankle_pitch;
		Points.Thta[14] += leftfoot_ankle_roll;

		Points.Thta[16] += rightfoot_hip_roll;
		Points.Thta[17] += rightfoot_hip_pitch;
		Points.Thta[19] += rightfoot_ankle_pitch;
		Points.Thta[20] += rightfoot_ankle_roll;

		// if(leftfoot_hip_roll > 0)
		// 	Points.Thta[5] += leftfoot_hip_roll;
		// else
		// 	Points.Thta[1] += leftfoot_hip_roll;
	}
	else if(sup_foot_ == rightfoot)
	{
		Points.Thta[10] += leftfoot_hip_roll;
		Points.Thta[11] += leftfoot_hip_pitch;
		Points.Thta[13] += leftfoot_ankle_pitch;
		Points.Thta[14] += leftfoot_ankle_roll;

		Points.Thta[16] += rightfoot_hip_roll;
		Points.Thta[17] += rightfoot_hip_pitch;
		Points.Thta[19] += rightfoot_ankle_pitch;
		Points.Thta[20] += rightfoot_ankle_roll;

		// if(rightfoot_hip_roll > 0)
		// 	Points.Thta[5] += rightfoot_hip_roll;
		// else
		// 	Points.Thta[1] += rightfoot_hip_roll;
	}

	// compensate
	// double gain = 1.05;	
	if(Points.Inverse_PointR_Y < 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{
		// Points.Thta[10] = PI_2 - (Points.Thta[10] - PI_2) *1;
		// Points.Thta[16] = PI_2 + (Points.Thta[16] - PI_2) *1;			
		
		double tmp = (Points.Thta[10] * 1.075) - Points.Thta[10];
		Points.Thta[10] -= tmp;
		Points.Thta[16] *= 1.075;//1.025;


		// double tmp = (Points.Thta[10] * 1.05) - Points.Thta[10];
		// Points.Thta[10] -= tmp;
		// Points.Thta[16] *= 1.05;//1.025;

		// double tmp = (PI_2 - Points.Thta[10]) * gain;
		// Points.Thta[10] = PI_2 - fabs(tmp);
		// tmp = (PI_2 - Points.Thta[16]) * gain;
		// Points.Thta[16] = PI_2 + fabs(tmp);
	} 
	else if(Points.Inverse_PointR_Y > 0 && (original_ik_point_rz_ != original_ik_point_lz_))
	{
		// Points.Thta[10] = PI_2 + (Points.Thta[10] - PI_2) *1;
		// Points.Thta[16] = PI_2 - (Points.Thta[16] - PI_2) *1;	

		double tmp = (Points.Thta[10] * 1.075) - Points.Thta[10];
		Points.Thta[10] -= tmp;
		Points.Thta[16] *= 1.075;//1.025;

		// double tmp = (Points.Thta[10] * 1.05) - Points.Thta[10];
		// Points.Thta[10] -= tmp;
		// Points.Thta[16] *= 1.05;//1.025;

		// double tmp = (PI_2 - Points.Thta[10]) * gain;
		// Points.Thta[10] = PI_2 - fabs(tmp);
		// tmp = (PI_2 - Points.Thta[16]) * gain;
		// Points.Thta[16] = PI_2 + fabs(tmp);
	}
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error = 0;
    this->pre_error = 0;
    this->errors = 0;
    this->errord = 0;
    this->x1c = 0;
    this->x2c = 0;
    this->x3c = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
    this->upper_limit = 0;
    this->lower_limit = 0;
}

PID_Controller::PID_Controller()
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    this->error = 0;
    this->pre_error = 0;
    this->errors = 0;
    this->errord = 0;
    this->x1c = 0;
    this->x2c = 0;
    this->x3c = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

PID_Controller::~PID_Controller()
{
    
}

void PID_Controller::initParam()
{
    this->pre_error = 0;
    this->error = 0;
    this->errors = 0;
    this->errord = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

void PID_Controller::setKpid(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID_Controller::setControlGoal(float x1c, float x2c, float x3c)
{
    this->x1c = x1c;
    this->x2c = x2c;
    this->x3c = x3c;
}

float PID_Controller::calculateExpValue(float value)//Expected value
{
    this->pre_value = this->value;
    this->value = value;
    this->pre_error = this->error;
    this->error = this->x1c - this->value;
    this->errors += this->error*0.03;
    if(this->pre_error == 0)
    {
        this->errord = 0;
    } 
    else
    {
        this->errord = (this->error - this->pre_error)/0.03;
    }
    this->exp_value = this->Kp*this->error + this->Ki*this->errors + this->Kd*this->errord;
    if(this->exp_value > this->upper_limit)
    {
        return this->upper_limit;
    }
    else if(this->exp_value < this->lower_limit)
    {
        return this->lower_limit;
    }
    else
    {
        return this->exp_value;
    }
}

float PID_Controller::calculateExpValue_roll(float value)//Expected value
{
	int non_control_area = 5;
	if(fabs(value) < 3)
	{	
		value = 0;
	}
	else
	{
		if(value <= non_control_area && value >= -non_control_area)
		{
    		this->error = this->x1c - value;
		}
		else
		{
			if(value<-non_control_area)
			{
    			this->error = (this->x1c - non_control_area) - value;
			}
			else if(value > non_control_area)
			{
			    this->error = (this->x1c + non_control_area) - value;	
			}
			else
			{
				this->error = this->x1c - value;
			}
		}
		
	}
	

    this->pre_value = this->value;
    this->value = value;
    this->pre_error = this->error;
    // this->error = this->x1c - this->value;
    this->errors += this->error*0.03;
    if(this->pre_error == 0)
    {
        this->errord = 0;
    } 
    else
    {
        this->errord = (this->error - this->pre_error)/0.03;
    }
    this->exp_value = this->Kp*this->error + this->Ki*this->errors + this->Kd*this->errord;
    if(this->exp_value > this->upper_limit)
    {
        return this->upper_limit;
    }
    else if(this->exp_value < this->lower_limit)
    {
        return this->lower_limit;
    }
    else
    {
        return this->exp_value;
    }
}


void PID_Controller::setValueLimit(float upper_limit, float lower_limit)
{
    this->upper_limit = upper_limit;
    this->lower_limit = lower_limit;
}

float PID_Controller::getError()
{
    return this->error;
}

float PID_Controller::getErrors()
{
    return this->errors;
}

float PID_Controller::getErrord()
{
    return this->errord;
}

ButterWorthParam ButterWorthParam::set(float a1, float a2, float b1, float b2)
{
    ButterWorthParam temp;
    temp.a_[0] = a1;
    temp.a_[1] = a2;
    temp.b_[0] = b1;
    temp.b_[1] = b2;
    return temp;
}

ButterWorthFilter::ButterWorthFilter()
{

}

ButterWorthFilter::~ButterWorthFilter()
{

}

void ButterWorthFilter::initialize(ButterWorthParam param)
{
    param_ = param;
    prev_output_ = 0;
    prev_value_ = 0;
}

float ButterWorthFilter::getValue(float present_value)
{
    if(prev_output_ == 0 && prev_value_ == 0)
    {
        prev_output_ = param_.b_[0]*present_value/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
    else
    {
        prev_output_ = (param_.b_[0]*present_value + param_.b_[1]*prev_value_ - param_.a_[1]*prev_output_)/param_.a_[0];
        prev_value_ = present_value;
        return prev_output_;
    }
}

BalanceLowPassFilter::BalanceLowPassFilter()
{
	cut_off_freq_ = 1.0;
	control_cycle_sec_ = 0.008;
	prev_output_ = 0;

	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

BalanceLowPassFilter::~BalanceLowPassFilter()
{	}

void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;
	control_cycle_sec_ = control_cycle_sec;
	prev_output_ = 0;

	if(cut_off_frequency > 0)
		alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
		alpha_ = 1;
}

void BalanceLowPassFilter::set_cut_off_frequency(double cut_off_frequency)
{
	cut_off_freq_ = cut_off_frequency;

	if(cut_off_frequency > 0)
	alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
	else
	alpha_ = 1;
}

double BalanceLowPassFilter::get_cut_off_frequency(void)
{
	return cut_off_freq_;
}

double BalanceLowPassFilter::get_filtered_output(double present_raw_value)
{
	prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
	return prev_output_;
}

BalancePDController::BalancePDController()
{
	desired_ = 0;
	curr_err_ = 0;
	prev_err_ = 0;

	//test

	//2020 0701 E305 WL  0.014
	//big 0.016  for other strategy 0.020  SP 0.019
	p_gain_ = 1;
	d_gain_ = 0.008;
}

BalancePDController::~BalancePDController()
{	}

void BalancePDController::set_desired(double desired)
{
	desired_ = desired;
}

void BalancePDController::set_control_cycle_time(double control_cycle_sec)
{
	control_cycle_sec_ = control_cycle_sec;
}

double BalancePDController::get_feedback(double present_sensor_output)
{
	prev_err_ = curr_err_;
	curr_err_ = desired_ - present_sensor_output;

	return (p_gain_*curr_err_ + d_gain_*(curr_err_ - prev_err_)/control_cycle_sec_);
}

BalanceControlUsingPDController::BalanceControlUsingPDController()
{
	control_cycle_sec_ = 0.008;

	// balance enable
	roll_control_enable_ = 1.0;
	pitch_control_enable_ = 1.0;

	// desired pose
	desired_robot_to_cob_ = Eigen::MatrixXd::Identity(4, 4);
	desired_robot_to_right_foot_ = Eigen::MatrixXd::Identity(4, 4);
	desired_robot_to_left_foot_ = Eigen::MatrixXd::Identity(4, 4);

	// sensed values
	current_imu_roll_rad_per_sec_ = 0;
	current_imu_pitch_rad_per_sec_ = 0;

	// manual cob adjustment
	cob_x_manual_adjustment_m_ = 0;
	cob_y_manual_adjustment_m_ = 0;
	cob_z_manual_adjustment_m_ = 0;

	// balance algorithm result
	foot_roll_adjustment_by_imu_roll_ = 0;
	foot_pitch_adjustment_by_imu_pitch_ = 0;

	// maximum adjustment
	cob_x_adjustment_abs_max_m_ = 0.05;
	cob_y_adjustment_abs_max_m_ = 0.05;
	cob_z_adjustment_abs_max_m_ = 0.05;
	cob_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
	cob_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
	cob_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;
	foot_x_adjustment_abs_max_m_ = 0.1;
	foot_y_adjustment_abs_max_m_ = 0.1;
	foot_z_adjustment_abs_max_m_ = 0.1;
	foot_roll_adjustment_abs_max_rad_  = 30.0*DEGREE2RADIAN;
	foot_pitch_adjustment_abs_max_rad_ = 30.0*DEGREE2RADIAN;
	foot_yaw_adjustment_abs_max_rad_   = 30.0*DEGREE2RADIAN;

	mat_robot_to_cob_modified_        = Eigen::MatrixXd::Identity(4,4);
	mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4,4);
	mat_robot_to_left_foot_modified_  = Eigen::MatrixXd::Identity(4,4);
	pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
	pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);;
	pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);;
}

BalanceControlUsingPDController::~BalanceControlUsingPDController()
{	}

void BalanceControlUsingPDController::initialize(const int control_cycle_msec)
{
	control_cycle_sec_ = control_cycle_msec * 0.001;

	pose_cob_adjustment_.fill(0);
	pose_right_foot_adjustment_.fill(0);
 	pose_left_foot_adjustment_.fill(0);

	roll_imu_lpf_.initialize(control_cycle_sec_, 1.0);
	pitch_imu_lpf_.initialize(control_cycle_sec_, 1.0);
}

void BalanceControlUsingPDController::set_roll_control_enable(bool enable)
{
	if(enable)
		roll_control_enable_ = 1.0;
	else
		roll_control_enable_ = 0.0;
}

void BalanceControlUsingPDController::set_pitch_control_enable(bool enable)
{
	if(enable)
		pitch_control_enable_ = 1.0;
	else
		pitch_control_enable_ = 0.0;
}

void BalanceControlUsingPDController::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
	LinearAlgebra LA;

	pose_cob_adjustment_.fill(0);
	pose_right_foot_adjustment_.fill(0);
	pose_left_foot_adjustment_.fill(0);

	double roll_imu_filtered = roll_imu_lpf_.get_filtered_output(current_imu_roll_rad_per_sec_);
	double pitch_imu_filtered = pitch_imu_lpf_.get_filtered_output(current_imu_pitch_rad_per_sec_);

	foot_roll_adjustment_by_imu_roll_ = -0.1*roll_control_enable_*foot_roll_imu_ctrl_.get_feedback(roll_imu_filtered);
	foot_pitch_adjustment_by_imu_pitch_ = -0.1*pitch_control_enable_*foot_pitch_imu_ctrl_.get_feedback(pitch_imu_filtered);

	Eigen::MatrixXd mat_orientation_adjustment_by_imu = LA.getRotation4d(foot_roll_adjustment_by_imu_roll_, foot_pitch_adjustment_by_imu_pitch_, 0.0);
	Eigen::MatrixXd mat_r_xy, mat_l_xy;
	mat_r_xy.resize(4, 1);
	mat_r_xy.coeffRef(0, 0) = desired_robot_to_right_foot_.coeff(0, 3) - 0.5*(desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
	mat_r_xy.coeffRef(1, 0) = desired_robot_to_right_foot_.coeff(1, 3) - 0.5*(desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
	mat_r_xy.coeffRef(2, 0) = 0.0;
	mat_r_xy.coeffRef(3, 0) = 1;

	mat_l_xy.resize(4, 1);
	mat_l_xy.coeffRef(0, 0) = desired_robot_to_left_foot_.coeff(0, 3) - 0.5*(desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
	mat_l_xy.coeffRef(1, 0) = desired_robot_to_left_foot_.coeff(1, 3) - 0.5*(desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
	mat_l_xy.coeffRef(2, 0) = 0.0;
	mat_l_xy.coeffRef(3, 0) = 1;

	mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
	mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

	// sum of sensory balance result
	pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
	pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
	pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

	pose_right_foot_adjustment_.coeffRef(0) = 0;
	pose_right_foot_adjustment_.coeffRef(1) = 0;
	pose_right_foot_adjustment_.coeffRef(2) = mat_r_xy.coeff(2, 0);
	pose_right_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_imu_roll_;
	pose_right_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_imu_pitch_;

	pose_left_foot_adjustment_.coeffRef(0) = 0;
	pose_left_foot_adjustment_.coeffRef(1) = 0;
	pose_left_foot_adjustment_.coeffRef(2) = mat_l_xy.coeff(2, 0);
	pose_left_foot_adjustment_.coeffRef(3) = foot_roll_adjustment_by_imu_roll_;
	pose_left_foot_adjustment_.coeffRef(4) = foot_pitch_adjustment_by_imu_pitch_;

	// check limitation
	pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
	pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
	pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
	pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
	pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
	pose_cob_adjustment_.coeffRef(5) = 0;

	pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
	pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
	pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
	pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
	pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
	pose_right_foot_adjustment_.coeffRef(5) = 0;

	pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
	pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
	pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
	pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
	pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
	pose_left_foot_adjustment_.coeffRef(5) = 0;

	Eigen::MatrixXd cob_rotation_adj = LA.getRotationZ(pose_cob_adjustment_.coeff(5)) * LA.getRotationY(pose_cob_adjustment_.coeff(4)) * LA.getRotationX(pose_cob_adjustment_.coeff(3));
	Eigen::MatrixXd rf_rotation_adj = LA.getRotationZ(pose_right_foot_adjustment_.coeff(5)) * LA.getRotationY(pose_right_foot_adjustment_.coeff(4)) * LA.getRotationX(pose_right_foot_adjustment_.coeff(3));
	Eigen::MatrixXd lf_rotation_adj = LA.getRotationZ(pose_left_foot_adjustment_.coeff(5)) * LA.getRotationY(pose_left_foot_adjustment_.coeff(4)) * LA.getRotationX(pose_left_foot_adjustment_.coeff(3));
	mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
	mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
	mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

	mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
	mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
	mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

	mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
	mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
	mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

	mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
	mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
	mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

	*robot_to_cob_modified        = mat_robot_to_cob_modified_;
	*robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
	*robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}

void BalanceControlUsingPDController::set_desired_pose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
	desired_robot_to_cob_        = robot_to_cob;
	desired_robot_to_right_foot_ = robot_to_right_foot;
	desired_robot_to_left_foot_  = robot_to_left_foot;
}

void BalanceControlUsingPDController::set_desired_cob_imu(double imu_roll, double imu_pitch)
{
	foot_roll_imu_ctrl_.desired_  = imu_roll;
	foot_pitch_imu_ctrl_.desired_ = imu_pitch;
}

void BalanceControlUsingPDController::set_current_imu_sensor_output(double imu_roll, double imu_pitch)
{
	current_imu_roll_rad_per_sec_  = imu_roll;
	current_imu_pitch_rad_per_sec_ = imu_pitch;
}

void BalanceControlUsingPDController::set_maximum_adjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                          double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                          double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                          double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
	cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
	cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
	cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
	cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
	cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
	cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;
	foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
	foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
	foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
	foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
	foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
	foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

void BalanceControlUsingPDController::set_cob_manual_adjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_x()
{
  return cob_x_manual_adjustment_m_;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_y()
{
  return cob_y_manual_adjustment_m_;
}

double BalanceControlUsingPDController::get_cob_manual_adjustment_z()
{
  return cob_z_manual_adjustment_m_;
}

LinearAlgebra::LinearAlgebra()
{	}

LinearAlgebra::~LinearAlgebra()
{	}

Eigen::Matrix3d LinearAlgebra::getRotationX(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		1.0, 0.0, 0.0,
		0.0, cos(angle), -sin(angle),
		0.0, sin(angle), cos(angle);

	return rotation;
}

Eigen::Matrix3d LinearAlgebra::getRotationY(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		cos(angle), 0.0, sin(angle),
		0.0, 1.0, 0.0,
		-sin(angle), 0.0, cos(angle);

	return rotation;
}

Eigen::Matrix3d LinearAlgebra::getRotationZ(double angle)
{
	Eigen::Matrix3d rotation(3,3);

	rotation <<
		cos(angle), -sin(angle), 0.0,
		sin(angle), cos(angle), 0.0,
		0.0, 0.0, 1.0;

	return rotation;
}

Eigen::Matrix4d LinearAlgebra::getRotation4d(double roll, double pitch, double yaw )
{
	double sr = sin(roll), cr = cos(roll);
	double sp = sin(pitch), cp = cos(pitch);
	double sy = sin(yaw), cy = cos(yaw);

	Eigen::Matrix4d mat_roll;
	Eigen::Matrix4d mat_pitch;
	Eigen::Matrix4d mat_yaw;

	mat_roll <<
		1, 0, 0, 0,
		0, cr, -sr, 0,
		0, sr, cr, 0,
		0, 0, 0, 1;

	mat_pitch <<
		cp, 0, sp, 0,
		0, 1, 0, 0,
		-sp, 0, cp, 0,
		0, 0, 0, 1;

	mat_yaw <<
		cy, -sy, 0, 0,
		sy, cy, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

	return mat_rpy;
}

string BalanceControl::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void BalanceControl::saveData()
{
	//------roll------
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Roll"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_roll;

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		savedText += it_roll->first;
		if(it_roll == --map_roll.end())
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
	it_roll = map_roll.begin();
	int max_size = it_roll->second.size();

	for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
	{
		if(max_size < it_roll->second.size())
            max_size = it_roll->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        {
            if(i < it_roll->second.size())
            {
                if(it_roll == --map_roll.end())
                {
                    savedText += std::to_string(it_roll->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_roll->second[i]) + ",";
                }
            }
            else
            {
                if(it_roll == --map_roll.end())
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
    for(it_roll = map_roll.begin(); it_roll != map_roll.end(); it_roll++)
        it_roll->second.clear();

	//------pitch------
	char path2[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_Pitch_"+tmp+".csv";
    strcat(path2, tmp.c_str());
    fp.open(path2, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_pitch;

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		savedText += it_pitch->first;
		if(it_pitch == --map_pitch.end())
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
	it_pitch = map_pitch.begin();
	max_size = it_pitch->second.size();

	for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
	{
		if(max_size < it_pitch->second.size())
            max_size = it_pitch->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        {
            if(i < it_pitch->second.size())
            {
                if(it_pitch == --map_pitch.end())
                {
                    savedText += std::to_string(it_pitch->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_pitch->second[i]) + ",";
                }
            }
            else
            {
                if(it_pitch == --map_pitch.end())
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
    for(it_pitch = map_pitch.begin(); it_pitch != map_pitch.end(); it_pitch++)
        it_pitch->second.clear();

	//------ZMP------
	char path3[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_ZMP_"+tmp+".csv";
    strcat(path3, tmp.c_str());
    fp.open(path3, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_ZMP;

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		savedText += it_ZMP->first;
		if(it_ZMP == --map_ZMP.end())
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
	it_ZMP = map_ZMP.begin();
	max_size = it_ZMP->second.size();

	for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
	{
		if(max_size < it_ZMP->second.size())
            max_size = it_ZMP->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        {
            if(i < it_ZMP->second.size())
            {
                if(it_ZMP == --map_ZMP.end())
                {
                    savedText += std::to_string(it_ZMP->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_ZMP->second[i]) + ",";
                }
            }
            else
            {
                if(it_ZMP == --map_ZMP.end())
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
    for(it_ZMP = map_ZMP.begin(); it_ZMP != map_ZMP.end(); it_ZMP++)
        it_ZMP->second.clear();

	//------CoM------
	char path4[200] = "/data";
	tmp = std::to_string(name_cont_);
	tmp = "/Feedback_Control_CoM_"+tmp+".csv";
    strcat(path4, tmp.c_str());
    fp.open(path4, std::ios::out);
	savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_CoM;

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		savedText += it_CoM->first;
		if(it_CoM == --map_CoM.end())
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
	it_CoM = map_CoM.begin();
	max_size = it_CoM->second.size();

	for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
	{
		if(max_size < it_CoM->second.size())
            max_size = it_CoM->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        {
            if(i < it_CoM->second.size())
            {
                if(it_CoM == --map_CoM.end())
                {
                    savedText += std::to_string(it_CoM->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_CoM->second[i]) + ",";
                }
            }
            else
            {
                if(it_CoM == --map_CoM.end())
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
    for(it_CoM = map_CoM.begin(); it_CoM != map_CoM.end(); it_CoM++)
        it_CoM->second.clear();

	name_cont_++;
}
