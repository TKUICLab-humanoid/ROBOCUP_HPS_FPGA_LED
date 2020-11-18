#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdio.h>
#include <string.h>
#include "Initial.h"

class SensorDataProcess
{
public:
	SensorDataProcess();
	~SensorDataProcess();

	void sensor_data_send();
	void sensor_package_generate();
	void send_sensor_data_to_ipc();
	void load_sensor_setting();
	void update_sensor_setting();

    void load_imu();
    void update_imu();

    void set_desire_rpy();

    //test
    double rpy_[3];
    double imu_desire_[3];

    //fall down
    bool fall_Down_Flag_;
    bool stop_Walk_Flag_;
    char fall_Down_Status_; // S = stand ; F = forward fall down ;B = backward fall down
    unsigned int stand_status_package_;
        
    //press sensor 
    void load_press_left();
    void load_press_right();
    void update_press_right();
    void update_press_left();

    int press_right_[4];
    int press_left_[4];


private:
    int rpy_from_fpga_[3];
    double rpy_raw_[3];
    double rpy_offset_[3];
    unsigned char sensor_data_to_ipc_[22];
    // double imu_desire_[3];
    bool update_sensor_setting_flag_;
    bool get_sensor_setting_flag_;
    bool update_imu_flag_;
    bool get_imu_flag_;
    unsigned int sensor_setting_[2];
    unsigned int imu_[2];

    bool sensor_request_;
    bool imu_offset_reset_;   // imu offset reset
    bool force_state_;

    //press sensor
    bool update_press_right_flag_;
    bool update_press_left_flag_;
    //right press
    unsigned int press_receive_data_right[2];
    int press_right_raw_[4];
    int press_right_offset_[4];    
    //left  press
    unsigned int press_receive_data_left[2];
    int press_left_raw_[4];
    int press_left_offset_[4];         
    //for press test end
};

#endif