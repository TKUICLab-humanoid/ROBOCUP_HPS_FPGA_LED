#ifndef KICKINGGAIT_H_
#define KICKINGGAIT_H_

#include "B_Spline.h"
#include "Parameter_Info.h"
#include "DefineDataStruct.h"
#include "ZMPProcess.h"
#include "MBK_control.h"
#include "Sensor.h"
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <sys/time.h>
#include <cmath>
#include <vector>
#include <map>
class ZMPProcess;
namespace kickgait_space
{
#define PI 3.1415926535897932384626433832795
enum class imu {roll = 0, pitch, yaw};

//fs = 33, fc = 2, use matlab [b, a] = butter(1, fc/(fs/2)); you will get a1 a2 b1 b2
typedef struct ButterWirthParamter ButterWirthParam;
struct ButterWirthParamter
{
    float a_[2];
    float b_[2];
    void initailize();
    void set(float a1, float a2, float b1, float b2);
};

class ButterWirthFilter
{
public:
    ButterWirthFilter();
    ~ButterWirthFilter();
    void initialize();
    void setButterWirthParam(float a1, float a2, float b1, float b2);
    float getValue(float present_value);
private:
    ButterWirthParam param_;
    float prev_output_;
    float prev_value_;
};

class Kick_PID_Controller
{
public:
    Kick_PID_Controller(float Kp, float Ki, float Kd);
    Kick_PID_Controller();
    ~Kick_PID_Controller();
    void initParam();
    void setKpid(double Kp, double Ki, double Kd);
    void setControlGoal(float x1c);
    void setValueLimit(float upper_limit, float lower_limit);
    float calculateExpValue(float value);
    float getError();
    float getErrors();
    float getErrord();
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

typedef struct Kick_IMUParameter Kick_IMUParam;
struct Kick_IMUParameter
{
    float pos;
    float vel;
    void initialize();
    // float acc;
};

typedef struct Kick_BalanceParameter Kick_BalanceParam;
struct Kick_BalanceParameter
{
    float control_value_total;
    float control_value_once;
    void initialize();
    // float acc;
};

typedef struct ButterWirthIMUParameter ButterWirthIMUParam;
struct ButterWirthIMUParameter
{
    ButterWirthFilter pos;
    ButterWirthFilter vel;
    void initialize();
};

typedef struct ButterWirthZMPParameter ButterWirthZMPParam;
struct ButterWirthZMPParameter
{
    ButterWirthFilter pos_x;
    ButterWirthFilter pos_y;
    ButterWirthFilter vel_x;
    ButterWirthFilter vel_y;
    ButterWirthFilter acc_x;
    ButterWirthFilter acc_y;
    void initialize();
};

typedef struct ButterWirthForceParameter ButterWirthForceParam;
struct ButterWirthForceParameter
{
    ButterWirthFilter zero;
    ButterWirthFilter one;
    ButterWirthFilter two;
    ButterWirthFilter three;
    void initialize();
};

class BalanceControl
{
public:
    BalanceControl();
    ~BalanceControl();
    void initialize();

    Kick_IMUParam init_imu_value[3];
    Kick_IMUParam pres_imu_value[3];
    Kick_IMUParam prev_imu_value[3];
    Kick_IMUParam ideal_imu_value[3];
    Kick_IMUParam passfilter_pres_imu_value[3];
    Kick_IMUParam passfilter_prev_imu_value[3];
    ZMPParam pres_ZMP;
    ZMPParam prev_ZMP;
    ZMPParam ideal_ZMP;
    ZMPParam boundary_ZMP;
    ZMPParam passfilter_pres_ZMP;

    Kick_IMUParam pres_ankle_roll;
    Kick_IMUParam prev_ankle_roll;
    Kick_IMUParam pres_ankle_pitch;
    Kick_IMUParam prev_ankle_pitch;
    Kick_IMUParam ideal_ankle_pitch;
    Kick_IMUParam ideal_ankle_roll;
    Kick_BalanceParam supfoot_ankle_roll_value;
    Kick_BalanceParam supfoot_ankle_pitch_value;

    Kick_BalanceParam supfoot_hip_roll_value;
    Kick_BalanceParam supfoot_hip_pitch_value;
    Kick_BalanceParam supfoot_EPx_value; //EPx = End Point X
    Kick_BalanceParam supfoot_EPy_value; //EPx = End Point X
    
    Kick_PID_Controller PIDsupfoot_hip_roll;
    Kick_PID_Controller PIDsupfoot_hip_pitch;
    Kick_PID_Controller PIDsupfoot_EPx;
    Kick_PID_Controller PIDsupfoot_EPy;

    ButterWirthIMUParam butterfilter_imu[3];
    ButterWirthZMPParam butterfilter_ZMPsupfoot;

    ZMPProcess *ZMP_process;
};

class KickingGait
{
public:
    KickingGait();
    ~KickingGait();
    void initialize();
    void updateControlPoint();
    void rightKickBall();
    void leftKickBall();
    void kickingProcess(int walking_mode);
    void kickingCycle(int walking_mode);
    void SaveData();
    void hipPitchControl();
    void ankleBalanceControl();
    Point3DParam endPointBalanceControl();
    void hipPostureControl();
    std::map<std::string, float> map_param;
    std::map<std::string, std::vector<float>> map_roll;
    std::map<std::string, std::vector<float>> map_pitch;
    std::map<std::string, std::vector<float>> map_kickgait;
    std::map<std::string, std::vector<float>> map_ZMP;

public:
    B_Spline B_spline;
    BalanceControl balance;
    std::vector<B_Spline_Param> vB_spline_param;

    float supfoot_hip_roll;
    float supfoot_hip_pitch;
    float supfoot_ankle_roll;
    float supfoot_ankle_pitch;
    float kick_foot_ankle_pitch;

    bool init_param_flag_;
    bool kicking_process_flag_;
    bool force_delay_flag_;
    bool force_stop_sample_point_flag_;
    int name_cont = 0;

private:
    double *T_cnt;
    double *T_cnt_sum;
    unsigned char ankle_balance_count;
    bool pitch_flag;
    bool roll_flag;
};

}

#endif /* KICKINGGAIT_H_ */