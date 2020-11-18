/*
 * walkinggait.h
 *
 *  Created on: 2019/09/21
 *      Author: Yu-Chih, Wang
 */

#ifndef WALKINGGAIT_H_
#define WALKINGGAIT_H_

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "Initial.h"
#include "WalkingCycle.h"
#include "WalkingTrajectory.h"
#include "KickingGait.h"
// #include <std_msgs/String.h>

#define WALKING_INTERVAL 30000.0   //30 ms

typedef enum
{
    etStart = 0x01,
    etStop = 0x02,
    etChangeValue = 0x04
}etWalkingCmd;

class WalkingGaitByLIPM
{
public:
    WalkingGaitByLIPM();
    ~WalkingGaitByLIPM();

    void initialize();
    void resetParameter();
    void process();

    double wComVelocityInit(double x0, double xt, double px, double t, double T);
    double wComPosition(double x0, double vx0, double px, double t, double T);
    double wFootPosition(const double start, const double length, const double t, const double T, const double T_DSP);
    double wFootPositionRepeat(const double start, const double length, const double t, const double T, const double T_DSP);
    double wFootPositionZ(const double height, const double t, const double T, const double T_DSP);
    double wFootTheta(const double theta, bool reverse, const double t, const double T, const double T_DSP);

    double unit_step(double x);
    double sinh(double x);
    double cosh(double x);

    //for test
    string DtoS(double value);
    void saveData();

// private:
    bool is_parameter_load_;
    bool ready_to_stop_;
    int moving_state_;
    int period_t_;
    int time_point_, sample_point_, sample_time_;
    int now_step_, pre_step_;
    int g_;
    int step_, left_step_, right_step_;
    double TT_, t_;
    double Tc_;
    double step_length_, width_size_, lift_height_, now_length_, now_width_;
    double theta_, abs_theta_, last_theta_;
    double last_step_length_, now_left_length_, now_right_length_, last_length_;
    double shift_length_, last_shift_length_, now_shift_, now_left_shift_, now_right_shift_, last_shift_;
    double step_point_lx_, step_point_rx_, step_point_ly_, step_point_ry_;
    double step_point_lz_, step_point_rz_, step_point_lthta_, step_point_rthta_;
    double vx0_, vy0_, px_, py_, pz_;
    double lpx_, rpx_, lpy_, rpy_, lpz_, rpz_, lpt_, rpt_;

    bool plot_once_, if_finish_;
    
    int name_cont_;
	std::map<std::string, std::vector<float>> map_walk;
};

class Walkinggait : public WalkingGaitByLIPM
{
public:
    Walkinggait();
    ~Walkinggait();

    void load_parameter();
    void update_parameter();
    void load_walkdata();
    void update_walkdata(); 
    void calculate_point_trajectory();
    void walking_timer();

    double x_, y_, z_, thta_;
    char walking_cmd_, sensor_mode_;
    
    struct timeval timer_start_, timer_end_;
    double timer_dt_;
    bool get_parameter_flag_;
    bool get_walkdata_flag_;
    bool locus_flag_;
    int motion_delay_;

private:
    bool update_parameter_flag_;
    bool update_walkdata_flag_;
    bool continuous_stop_flag_;
    int parameter_[6];
    int walkdata_[3];
};

#endif /* WALKINGGAIT_H_ */