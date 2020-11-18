#ifndef WALKINGTRAJECTORY_H_
#define WALKINGTRAJECTORY_H_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
// #include <string>
#include <math.h>
// #include "../../ParameterInfo/parameterinfo.hpp"
#include <vector>
#include <string.h>
#include "Parameter_Info.h"

//#include "../../../strategy/src/StrategyNameAndPath.h"

using namespace std;

#define PI 3.1415926535897932384626433832795	//pi
#define PI_2 1.5707963267948966192313216916398	//pi/2
enum MoveState
{
    Straight,
    Right_Shift,
    Left_Shift,
    Right_Protect,
    Left_Protect,
    Right_turn,
    Left_Turn
};

class WalkingTrajectory
{
public:
    double wosc_foot_x_r;
    double wosc_foot_x_l;
    double wosc_foot_x_last;
    double wosc_foot_x_now;
    double lift_lock_x;
    vector<double> osc_move_x_r;
    vector<double> osc_move_x_l;
    vector<double> osc_move_y_r;
    vector<double> osc_move_y_l;
    vector<double> osc_move_z_r;
    vector<double> osc_move_z_l;
    vector<double> osc_move_com_x;
    vector<double> osc_move_com_y;
    vector<double> osc_move_com_z;
    vector<double> right_Thta;
    vector<double> left_Thta;
    vector<double> test;

    WalkingTrajectory();
    ~WalkingTrajectory();
    // void initRosNodeHandle(ros::NodeHandle &n);
    void walkingprocess(int walking_mode);
    void walkfunction();
    void LC_walkfunction();
    void longjump_function();
    void continuouswalk();
    void osccontinuouswalk();
    void inversekinmaticsinfo();
    void SaveData();
    string DtoS(double value);

    double OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms);
    double OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms, int div_omega = 1);
    double OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms);
    double OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
    double OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
    double OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms);
    double OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms);

    double WOSC_Waist_V(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time);
    double WOSC_Foot_X(int Ts, double Vmin, double Vmax, double period_T_ms, double delta, int time_t_ms, int Sample_Time);
    double WOSC_Foot_Z(int Ts, double period_T_ms, double Hfoot, double delta_z, int time_t_ms, int state);
    double WOSC_Waist_Y(int Ts, double period_T_ms, double Ay, double delta_z, int time_t_ms);
    double WOSC_Waist_Z(int Ts, double period_T_ms, double Az, double delta_z, int time_t_ms);
    double OSC_COM_Lift_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms);
    double OSC_lifemove_DOWNz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time);
    double OSC_lifemove_UPz(double range, double period_T_ms, double rho_z, double delta_z, double life_high, int time_t_ms, int Sample_Time);
};
#endif /* WALKINGTRAJECTORY_H_ */
