#ifndef FEEDBACK_CONTROL_H_
#define FEEDBACK_CONTROL_H_

//#include "system.h"
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include "Inverse_kinematic.h"
#include "Fuzzy_Controller.h"
#include "Walkinggait.h"
#include "Sensor.h"
#include "kalman.h"
#include "math.h"
#include "DefineDataStruct.h"
#include "ZMPProcess.h"
class ZMPProcess;

using namespace std;

#define PI                  3.1415926535897932384626433832795   //pi
#define PI_2                1.5707963267948966192313216916398   //pi/2
#define Angle_2_PI          PI/180
#define PI_2_Angle          180/PI
#define DEGREE2RADIAN		(M_PI / 180.0)
#define RADIAN2DEGREE		(180.0/ M_PI)

/*******************Function Define******************************/
#define Auto_Gyro_offset
/********************************************************/

#define GRAVITATIONAL_ACCELERATION 9.8	//9.8(m/s^2)
#define ANKLE_HEIGHT 2.6 //2.6(cm)
#define COM_HEIGHT_TO_GROUND (COM_HEIGHT + ANKLE_HEIGHT)
// #define IMU_HEIGHT 7.462 //7.462(cm)

/////////////////////////Posture///////////////////////
#define Acc_offset_X            	0
#define Acc_offset_Y            	0
#define Acc_offset_Z            	0
#define Gyro_manual_offset_X    	0
#define Gyro_manual_offset_Y    	0
#define Gyro_manual_offset_Z    	0
#define kalman_manual_offset_Pitch	0
#define kalman_manual_offset_Roll	0
#define kalman_manual_offset_Yaw	0
#define Gyro_LSB_2_Angle        	16.4
#define Acc_LSB_2_G					2048.0
#define index_Pitch             	0
#define index_Roll              	1
#define index_Yaw					2
#define Posture_Gain				0.3
////////////////////////////////////////////////////////

/////////////////////////Zmp Control///////////////////////
#define DOUBLE_FEET_WEIGHT_FAR_Y	10.5	//7.6 cm
#define DOUBLE_FEET_WEIGHT_NEAR_Y	3.0	//3.0 cm
#define DOUBLE_FEET_WEIGHT_X		7.15	//4.4 cm
#define DOUBLE_FEET_BALANCE_POINT_Y	4.5	//4.5 cm
#define SINGLE_FOOT_WEIGHT_FAR_Y	3.1	//3.1 cm
#define SINGLE_FOOT_WEIGHT_NEAR_Y	1.5	//1.5 cm
#define SINGLE_FOOT_WEIGHT_X		7.15	//4.4 cm

#define RIGHT_PRESS_SHIFT	4
#define SINGLE_FOOT_WEIGHT_EQUAL_Y	3.9
///////////////////////////////////////////////////////////

//////////////////////for Fall down & Get up/////////////////////////////
#define RPY_ROLL_LIMIT 		1.133	//65 degree
#define RPY_PITCH_LIMIT 	1.133	//65 degree
#define RPY_STAND_RANGE 	0.52	//30 degree
/////////////////////////////////////////////////////////

enum class imu {roll = 0,pitch,yaw};
typedef enum {leftfoot = 0,rightfoot,doublefeet} etSupFoot;


class ButterWorthParam
{
public:
    static ButterWorthParam set(float a1, float a2, float b1, float b2);
    float a_[2];
    float b_[2];
};

class ButterWorthFilter
{
public:
    ButterWorthFilter();
    ~ButterWorthFilter();
    void initialize(ButterWorthParam param);
    float getValue(float present_value);
private:
    ButterWorthParam param_;
    float prev_output_;
    float prev_value_;
};


class PID_Controller
{
public:
    PID_Controller(float Kp, float Ki, float Kd);
    PID_Controller();
    ~PID_Controller();
    void initParam();
    void setKpid(double Kp, double Ki, double Kd);
    void setControlGoal(float x1c = 0, float x2c = 0, float x3c = 0);
    void setValueLimit(float upper_limit, float lower_limit);
    // void setDataValue(float value);
    float calculateExpValue(float value);
    float calculateExpValue_roll(float value);	
    float getError();
    float getErrors();
    float getErrord();
    // void setErrorValue(float error);
    // void setErrorsValue(float errors);
    // void setErrordValue(float errord);
    // float getFixValue();
private:
    double Kp;
    double Ki;
    double Kd;
    float error;
    float pre_error;
    float errors;
    float errord;
    float x1c;
    float x2c;
    float x3c;
    float exp_value;
    float value;
    float pre_value;
    float upper_limit;
    float lower_limit;
};

typedef struct IMUParameter IMUParam;
struct IMUParameter
{
    float pos;
    float vel;
    void initialize()
    {
        pos = 0;
        vel = 0;
    }
    // float acc;
};

typedef struct BalanceParameter BalanceParam;
struct BalanceParameter
{
    float control_value_total;
    float control_value_once;
    void initialize()
    {
        control_value_total = 0;
        control_value_once = 0;
    }
};

typedef struct ButterWorthIMUParameter ButterWorthIMUParam;
struct ButterWorthIMUParameter
{
    ButterWorthFilter pos;
    ButterWorthFilter vel;
    void initialize()
    {
        pos.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
        vel.initialize(ButterWorthParam::set(1, -0.676819, 0.161590, 0.161590)); //fs = 33 , fc = 2, n = 1;
    }
};

class BalancePDController
{
public:
	BalancePDController();
	~BalancePDController();

	void set_desired(double desired);
	void set_control_cycle_time(double control_cycle_sec);
	double get_feedback(double present_sensor_output);

	double desired_;
	double p_gain_;
	double d_gain_;

private:
	double curr_err_;
	double prev_err_;
	double control_cycle_sec_;
};

class BalanceLowPassFilter
{
public:
	BalanceLowPassFilter();
	~BalanceLowPassFilter();

	void initialize(double control_cycle_sec, double cut_off_frequency);
	void set_cut_off_frequency(double cut_off_frequency);
	double get_cut_off_frequency(void);
	double get_filtered_output(double present_raw_value);

private:
	double cut_off_freq_;
	double control_cycle_sec_;
	double alpha_;
	double prev_output_;
};

class BalanceControlUsingPDController
{
public:
	BalanceControlUsingPDController();
	~BalanceControlUsingPDController();

	void initialize(const int control_cycle_msec);
	void set_roll_control_enable(bool enable);
	void set_pitch_control_enable(bool enable);
	void set_force_torque_balance_enable(bool enable);

	void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

	void set_desired_pose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

	void set_desired_cob_imu(double imu_roll, double imu_pitch);

	void set_current_imu_sensor_output(double imu_roll, double imu_pitch);

	void set_maximum_adjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
								double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
								double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
								double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

	void set_cob_manual_adjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
	double get_cob_manual_adjustment_x();
	double get_cob_manual_adjustment_y();
	double get_cob_manual_adjustment_z();

	BalanceLowPassFilter roll_imu_lpf_;
	BalanceLowPassFilter pitch_imu_lpf_;

	BalancePDController foot_roll_imu_ctrl_;
	BalancePDController foot_pitch_imu_ctrl_;

private:
	int balance_control_error_;
	double control_cycle_sec_;

	// balance enable
	double roll_control_enable_;
	double pitch_control_enable_;

	// desired pose
	Eigen::MatrixXd desired_robot_to_cob_;	// center of body
	Eigen::MatrixXd desired_robot_to_right_foot_;
	Eigen::MatrixXd desired_robot_to_left_foot_;

	// sensed values
	double current_imu_roll_rad_per_sec_, current_imu_pitch_rad_per_sec_;

	// manual cob adjustment
	double cob_x_manual_adjustment_m_;
	double cob_y_manual_adjustment_m_;
	double cob_z_manual_adjustment_m_;

	// result of balance control
	double foot_roll_adjustment_by_imu_roll_;
	double foot_pitch_adjustment_by_imu_pitch_;

	// sum of results of balance control
	Eigen::VectorXd pose_cob_adjustment_;
	Eigen::VectorXd pose_right_foot_adjustment_;
	Eigen::VectorXd pose_left_foot_adjustment_;

	Eigen::MatrixXd mat_robot_to_cob_modified_;
	Eigen::MatrixXd mat_robot_to_right_foot_modified_;
	Eigen::MatrixXd mat_robot_to_left_foot_modified_;

	// maximum adjustment
	double cob_x_adjustment_abs_max_m_;
	double cob_y_adjustment_abs_max_m_;
	double cob_z_adjustment_abs_max_m_;
	double cob_roll_adjustment_abs_max_rad_;
	double cob_pitch_adjustment_abs_max_rad_;
	double cob_yaw_adjustment_abs_max_rad_;

	double foot_x_adjustment_abs_max_m_;
	double foot_y_adjustment_abs_max_m_;
	double foot_z_adjustment_abs_max_m_;
	double foot_roll_adjustment_abs_max_rad_;
	double foot_pitch_adjustment_abs_max_rad_;
	double foot_yaw_adjustment_abs_max_rad_;
};

class BalanceControl
{
public:
	BalanceControl();
	~BalanceControl();
	
	//		Fall down & Get up	timer  for	count time
	struct timeval timer_start_, timer_end_;
    double timer_dt_;

	//	test
	void initialize(const int control_cycle_msec);
	void get_sensor_value();
	void balance_control();
	void control_after_ik_calculation();


	// LIPM
	void setSupportFoot();
	void resetControlValue();
	void endPointControl();
	float calculateCOMPosbyLIPM(float pos_adj, float vel);

	double control_cycle_sec_;
	double foot_roll_adj_by_imu_roll_;
	double foot_pitch_adj_by_imu_pitch_;

	BalanceLowPassFilter roll_imu_lpf_;
	BalanceLowPassFilter pitch_imu_lpf_;

	BalancePDController foot_roll_imu_ctrl_;
	BalancePDController foot_pitch_imu_ctrl_;

	// sum of results of balance control
	Eigen::VectorXd desired_robot_to_rf_;
	Eigen::VectorXd desired_robot_to_lf_;
	Eigen::VectorXd pose_rf_adj_;
	Eigen::VectorXd pose_lf_adj_;
	Eigen::VectorXd robot_to_rf_modified_;
	Eigen::VectorXd robot_to_lf_modified_;

	// maximum adjustment
	double cob_x_adjustment_abs_max_m_;
	double cob_y_adjustment_abs_max_m_;
	double cob_z_adjustment_abs_max_m_;
	double cob_roll_adjustment_abs_max_rad_;
	double cob_pitch_adjustment_abs_max_rad_;
	double cob_yaw_adjustment_abs_max_rad_;

	double foot_x_adjustment_abs_max_m_;
	double foot_y_adjustment_abs_max_m_;
	double foot_z_adjustment_abs_max_m_;
	double foot_roll_adjustment_abs_max_rad_;
	double foot_pitch_adjustment_abs_max_rad_;
	double foot_yaw_adjustment_abs_max_rad_;

	BalancePDController x_adj_by_cog_ctrl_;
	BalancePDController y_adj_by_cog_ctrl_;

	BalanceLowPassFilter foot_roll_adj_imu_lpf_;
	BalanceLowPassFilter foot_pitch_adj_imu_lpf_;
	BalanceLowPassFilter x_adj_cog_lpf_;
	BalanceLowPassFilter y_adj_cog_lpf_;

	double roll_imu_filtered_;
	double pitch_imu_filtered_;
	bool two_feet_grounded_;
	bool roll_over_limit_;
	bool pitch_over_limit_;
	int landing_foot_;
	double cog_roll_offset_;
	double cog_pitch_offset_;
	double foot_cog_x_;
	double foot_cog_y_;
	double x_adj_by_cog_;
	double y_adj_by_cog_;
	double original_ik_point_rz_, original_ik_point_lz_;
	double ankle_pitch_;

	//LIPM
	etSupFoot sup_foot_, pre_sup_foot_;

	IMUParam init_imu_value[3];
    IMUParam pres_imu_value[3];
    IMUParam prev_imu_value[3];
    IMUParam ideal_imu_value[3];
    IMUParam passfilter_pres_imu_value[3];
    IMUParam passfilter_prev_imu_value[3];

	ButterWorthIMUParam butterfilter_imu[3];

	BalanceParam leftfoot_hip_roll_value;
    BalanceParam leftfoot_hip_pitch_value;
	BalanceParam rightfoot_hip_roll_value;
	BalanceParam rightfoot_hip_pitch_value;
	PID_Controller PIDleftfoot_hip_roll;
    PID_Controller PIDleftfoot_hip_pitch;
	PID_Controller PIDrightfoot_hip_roll;
    PID_Controller PIDrightfoot_hip_pitch;
    
	BalanceParam leftfoot_EPx_value;	//EP = End point
	BalanceParam leftfoot_EPy_value;
	BalanceParam rightfoot_EPx_value;
	BalanceParam rightfoot_EPy_value;
	PID_Controller PIDleftfoot_zmp_x;
	PID_Controller PIDleftfoot_zmp_y;
	PID_Controller PIDrightfoot_zmp_x;
	PID_Controller PIDrightfoot_zmp_y;

	BalanceParam CoM_EPx_value;
	PID_Controller PIDCoM_x;

	ZMPParam pres_ZMP;
    ZMPParam prev_ZMP;
    ZMPParam ideal_ZMP;

	ZMPProcess *ZMP_process;

	// float supfoot_hip_roll;
    // float supfoot_hip_pitch;
    // float supfoot_ankle_roll;
    // float supfoot_ankle_pitch;
	// float swingfoot_hip_roll;
    // float swingfoot_hip_pitch;
    // float swingfoot_ankle_roll;
    // float swingfoot_ankle_pitch;
	float leftfoot_hip_roll;
    float leftfoot_hip_pitch;
    float leftfoot_ankle_roll;
    float leftfoot_ankle_pitch;
	float rightfoot_hip_roll;
    float rightfoot_hip_pitch;
    float rightfoot_ankle_roll;
    float rightfoot_ankle_pitch;

	// for debug
	int now_step_, last_step_;
	void saveData();
    string DtoS(double value);
	vector<double> vec_roll, vec_pitch;
	vector<float> left_control_value_once, left_control_value_total;
	vector<float> right_control_value_once, right_control_value_total;
	vector<float> lf_hr;	//leftfoot hip roll
	vector<float> lf_hp;
	vector<float> lf_ar;	//leftfoot ankle roll
	vector<float> lf_ap;
	vector<float> rf_hr;	//rightfoot hip roll
	vector<float> rf_hp;
	vector<float> rf_ar;	//rightfoot ankle roll
	vector<float> rf_ap;
	vector<double> fb_cog_x;
	vector<double> fb_cog_y;
	vector<double> fb_roll;
	vector<double> fb_pitch;
	std::map<std::string, float> map_param;
    std::map<std::string, std::vector<float>> map_roll;
    std::map<std::string, std::vector<float>> map_pitch;
	std::map<std::string, std::vector<float>> map_ZMP;
	std::map<std::string, std::vector<float>> map_CoM;

	int name_cont_;
	//LIPM end
};



class LinearAlgebra
{
public:
	LinearAlgebra();
	~LinearAlgebra();

	Eigen::Matrix3d getRotationX(double angle);
	Eigen::Matrix3d getRotationY(double angle);
	Eigen::Matrix3d getRotationZ(double angle);
	Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw);
};

/////////////////////////posture///////////////////////
enum IPC_Contorl_SensorMode
{
	etNone, etRoll, etPitch, etRollPitch, etFS, etFSRoll, etFSPitch, etFSRollPitch
};

typedef enum
{
	etLeftfoot, etRightfoot, etBothfoot
}LandingFoot;

#endif /*FEEDBACK_CONTROL_H_*/
