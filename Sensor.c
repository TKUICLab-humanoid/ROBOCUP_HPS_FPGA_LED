#include "include/Sensor.h"

extern Initial init;

SensorDataProcess::SensorDataProcess()
{
    update_sensor_setting_flag_ = false;
    get_sensor_setting_flag_ = false;
    sensor_request_ = false;
    imu_offset_reset_ = false;
    force_state_ = false;
    memset(rpy_, 0.0, sizeof(rpy_));
    memset(rpy_raw_, 0.0, sizeof(rpy_raw_));
    memset(rpy_offset_, 0.0, sizeof(rpy_offset_));
    memset(sensor_data_to_ipc_, 0, sizeof(sensor_data_to_ipc_));
}

SensorDataProcess::~SensorDataProcess()
{
    
}

void SensorDataProcess::sensor_package_generate()
{
    if(sensor_request_ && get_sensor_setting_flag_)
    {
        int i=0;
        int cnt = 0;
        int press_data_counter = 0;        
        short sensor_data[11] = {0};

        for(i=0; i<3; i++)
        {
            sensor_data[i] = (short)(rpy_[i] * 100.0);
            if(sensor_data[i] < 0)
            {
                short tmp = ~(sensor_data[i]) + 1;
                sensor_data[i] = (0x8000 | (tmp & 0x7FFF));
            }
        }

        //press sensor data
        press_data_counter = 0;
        for(i = 3;i<7;i++)//left foot press
        {
            sensor_data[i] = (short)(press_left_[press_data_counter] );
            if(sensor_data[i] < 0)
            {
                short tmp = ~(sensor_data[i]) + 1;
                sensor_data[i] = (0x8000 | (tmp & 0x7FFF));
            }    
            press_data_counter++;        
        }
        press_data_counter = 0;
        for(i = 7;i<11;i++)//right foot press
        {
            sensor_data[i] = (short)(press_right_[press_data_counter] );
            if(sensor_data[i] < 0)
            {
                short tmp = ~(sensor_data[i]) + 1;
                sensor_data[i] = (0x8000 | (tmp & 0x7FFF));
            }     
            press_data_counter++;       
        }

        for(i=0; i<22; i+=2)
        {
            sensor_data_to_ipc_[i] = (unsigned char)((sensor_data[cnt] >> 8) & 0xFF);
            sensor_data_to_ipc_[i+1] = (unsigned char)(sensor_data[cnt++] & 0xFF);
        }
        //for fall down
        if(stop_Walk_Flag_ == true)
        {
            if(fall_Down_Status_ == 'F')
            {
                stand_status_package_ = 'F';//  0x46
            }
            if(fall_Down_Status_ == 'B')
            {
                stand_status_package_ = 'B';//   0x42        
            }
        }
        else //stop_Walk_Flag_ = false & fall_Down_Status_ = 'S'
        {
            stand_status_package_ = 'S';//    0x53        
        }

        *((uint32_t *)init.sensor_data_addr) = (0x5354F7 << 8) + sensor_data_to_ipc_[0];
        *((uint32_t *)init.sensor_data_addr+(1)) = (sensor_data_to_ipc_[1] << 24) + (sensor_data_to_ipc_[2] << 16) + (sensor_data_to_ipc_[3] << 8) + sensor_data_to_ipc_[4];
        *((uint32_t *)init.sensor_data_addr+(2)) = (sensor_data_to_ipc_[5] << 24) + (stand_status_package_ << 16) + (0xFF << 8) + sensor_data_to_ipc_[6];
        *((uint32_t *)init.sensor_data_addr+(3)) = (sensor_data_to_ipc_[7] << 24) + (sensor_data_to_ipc_[8] << 16) + (sensor_data_to_ipc_[9] << 8) + sensor_data_to_ipc_[10];
        *((uint32_t *)init.sensor_data_addr+(4)) = (sensor_data_to_ipc_[11] << 24) + (sensor_data_to_ipc_[12] << 16) + (sensor_data_to_ipc_[13] << 8) + sensor_data_to_ipc_[14];
        *((uint32_t *)init.sensor_data_addr+(5)) = (sensor_data_to_ipc_[15] << 24) + (sensor_data_to_ipc_[16] << 16) + (sensor_data_to_ipc_[17] << 8) + sensor_data_to_ipc_[18];
        *((uint32_t *)init.sensor_data_addr+(6)) = (sensor_data_to_ipc_[19] << 24) + (sensor_data_to_ipc_[20] << 16) + (sensor_data_to_ipc_[21] << 8) + 0x45;

        send_sensor_data_to_ipc();
        get_sensor_setting_flag_ = false;
    }
}

void SensorDataProcess::send_sensor_data_to_ipc()
{
    bool sensor_read_idle = false;

    for(;;)
    {
        if(sensor_read_idle)
        {
            *(uint32_t *)(init.h2p_avalon_sensor_data_addr_1) = 0x01;
            usleep(100);
            *(uint32_t *)(init.h2p_avalon_sensor_data_addr_1) = 0;
            break;
        }
        else
        {
            sensor_read_idle = *(uint32_t *)(init.h2p_avalon_sensor_data_addr_1);
        }
    }
}

void SensorDataProcess::load_sensor_setting()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_sensor_setting_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_sensor_setting_addr)
            {
                state = 1;
                continue;
            }
            else
            {
                break;
            }
        }
        else if(state == 1)
        {
            if(count <= 1)
            {
                sensor_setting_[count] = *(uint32_t *)init.p2h_sensor_setting_addr;
                count++;
                *(uint32_t *)init.h2p_read_sensor_setting_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_sensor_setting_pulse_addr = 0;
                continue;
            }
            else
            {
                update_sensor_setting_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_sensor_setting();
}

void SensorDataProcess::update_sensor_setting()
{
    if(update_sensor_setting_flag_)
    {
        int count = 0;
        short int tmp_angle = 0;

        tmp_angle = (sensor_setting_[count] >> 16) & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            imu_desire_[0] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            imu_desire_[0] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }

        tmp_angle = sensor_setting_[count++] & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            imu_desire_[1] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            imu_desire_[1] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }

        tmp_angle = (sensor_setting_[count] >> 16) & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            imu_desire_[2] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            imu_desire_[2] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }
        
        sensor_request_ = (sensor_setting_[count] >> 8) & 0x01;
        imu_offset_reset_ = (sensor_setting_[count] >> 8) & 0x02;
        force_state_ = (sensor_setting_[count] >> 8) & 0x04;
      
        
        get_sensor_setting_flag_ = true;
        if(imu_offset_reset_)
        {
            for(count=0; count<3; count++)
                rpy_offset_[count] = rpy_raw_[count];
            imu_offset_reset_ = false;
        }
    }
}

void SensorDataProcess::load_imu()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_imu_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_imu_addr)
            {
                state = 1;
                continue;
            }
            else
            {
                break;
            }
        }
        else if(state == 1)
        {
            if(count <= 1)
            {
                imu_[count] = *(uint32_t *)init.p2h_imu_addr;
                count++;
                *(uint32_t *)init.h2p_read_imu_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_imu_pulse_addr = 0;
                continue;
            }
            else
            {
                update_imu_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_imu();
}

void SensorDataProcess::update_imu()
{
    if(update_imu_flag_)
    {
        int count = 0;
        short int tmp_angle = 0;

        if(imu_[0] == imu_[1])
            return;

        tmp_angle = (imu_[count] >> 16) & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            rpy_raw_[0] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            rpy_raw_[0] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }

        tmp_angle = imu_[count++] & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            rpy_raw_[1] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            rpy_raw_[1] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }

        tmp_angle = (imu_[count] >> 16) & 0xFFFF;
        if(tmp_angle & 0x8000)
        {
            rpy_raw_[2] = (double)(( ~(tmp_angle & 0x7FFF) + 1 ) / 100.0);
        }
        else
        {
            rpy_raw_[2] = (double)((tmp_angle & 0x7FFF) / 100.0);
        }

        for(count=0; count<3; count++)
        {
            rpy_[count] = rpy_raw_[count] - rpy_offset_[count];
            if(rpy_[count] < -180)
                rpy_[count] += 360;
            else if(rpy_[count] > 180)
                rpy_[count] -= 360;
        }
    }
}

void SensorDataProcess::load_press_left()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_press_left_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_press_sensor_left_addr)
            {
                state = 1;
                continue;
            }
            else
            {
                break;
            }
        }
        else if(state == 1)
        {
            if(count <= 1)
            {
                press_receive_data_left[count] = *(uint32_t *)init.p2h_press_sensor_left_addr;
                count++;
                *(uint32_t *)init.h2p_read_press_sensor_left_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_press_sensor_left_pulse_addr = 0;
                continue;
            }
            else
            {
                update_press_left_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_press_left();
}

void SensorDataProcess::load_press_right()
{
    int state = 0;
    int count = 0;

    for(;;)
    {
        if(state == 0)
        {
            update_press_right_flag_ = false;
            if(*(uint32_t *)init.p2h_set_hps_read_press_sensor_right_addr)
            {
                state = 1;
                continue;
            }
            else
            {
                break;
            }
        }
        else if(state == 1)
        {
            if(count <= 1)
            {
                press_receive_data_right[count] = *(uint32_t *)init.p2h_press_sensor_right_addr;
                count++;
                *(uint32_t *)init.h2p_read_press_sensor_right_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_press_sensor_right_pulse_addr = 0;
                continue;
            }
            else
            {
                update_press_right_flag_ = true;
                state = 0;
                break;
            }
        }
    }
    update_press_right();
}

void SensorDataProcess::update_press_left()
{
    if(update_press_left_flag_)
    {
        int count = 0;
        int tmp_press = 0;

        tmp_press = (press_receive_data_left[count] >> 16) & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            // press_left_raw_[0] = (int)(((tmp_press & 0x7FFF) * (-1)) );
            press_left_raw_[0] = (int)(0);
        }
        else
        {
            press_left_raw_[0] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = press_receive_data_left[count++] & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_left_raw_[1] = (int)(0);
        }
        else
        {
            press_left_raw_[1] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = (press_receive_data_left[count] >> 16) & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_left_raw_[2] = (int)(0);
        }
        else
        {
            press_left_raw_[2] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = press_receive_data_left[count] & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_left_raw_[3] = (int)(0);
        }
        else
        {
            press_left_raw_[3] = (int)((tmp_press & 0x7FFF) );
        }

        for(count=0; count<4; count++)
        {
            //press_left_[count] = press_left_raw_[count] - press_left_offset_[count];
            press_left_[count] = press_left_raw_[count];  
        }

    }
}

void SensorDataProcess::update_press_right()
{
    if(update_press_right_flag_)
    {
        int count = 0;
        int tmp_press = 0;

        tmp_press = (press_receive_data_right[count] >> 16) & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_right_raw_[0] = (int)(0);
        }
        else
        {
            press_right_raw_[0] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = press_receive_data_right[count++] & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_right_raw_[1] = (int)(0);
        }
        else
        {
            press_right_raw_[1] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = (press_receive_data_right[count] >> 16) & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_right_raw_[2] = (int)(0);
        }
        else
        {
            press_right_raw_[2] = (int)((tmp_press & 0x7FFF) );
        }

        tmp_press = press_receive_data_right[count] & 0xFFFF;
        if(tmp_press & 0x8000)
        {
            press_right_raw_[3] = (int)(0);
        }
        else
        {
            press_right_raw_[3] = (int)((tmp_press & 0x7FFF) );
        }

        for(count=0; count<4; count++)
        {
            //press_right_[count] = press_right_raw_[count] - press_right_offset_[count];
            press_right_[count] = press_right_raw_[count];  
        }

    }
}