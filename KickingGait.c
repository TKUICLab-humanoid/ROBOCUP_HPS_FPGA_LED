#include "include/KickingGait.h"
#include "include/data_txt.h"
using namespace kickgait_space;
extern struct Points_Struct Points;
extern SensorDataProcess sensor;

KickingGait::KickingGait()
{ 
    T_cnt = nullptr;
    T_cnt_sum = nullptr;
    init_param_flag_ = false;
    kicking_process_flag_ = false;
    roll_flag = false;
    pitch_flag = false;
    this->initialize();
} 
 
KickingGait::~KickingGait() 
{
    delete[] T_cnt;
    delete[] T_cnt_sum;
}
 
void KickingGait::initialize()
{
    balance.initialize();

    balance.PIDsupfoot_hip_roll.setValueLimit(300, -300);
    balance.PIDsupfoot_hip_roll.setKpid(0, 0, 0);
    balance.PIDsupfoot_hip_roll.setControlGoal(balance.init_imu_value[(int)imu::roll].vel);

    balance.PIDsupfoot_hip_pitch.setKpid(0, 0, 0); // 0.4 0.0 0.5
    balance.PIDsupfoot_hip_pitch.setValueLimit(300, -300);
    balance.PIDsupfoot_hip_pitch.setControlGoal(balance.init_imu_value[(int)imu::pitch].vel);

    balance.PIDsupfoot_EPx.setValueLimit(50, -50);
    balance.PIDsupfoot_EPx.setKpid(0, 0, 0);
    balance.PIDsupfoot_EPx.setControlGoal(0);

    balance.PIDsupfoot_EPy.setValueLimit(50, -50);
    balance.PIDsupfoot_EPy.setKpid(0, 0, 0);
    balance.PIDsupfoot_EPy.setControlGoal(0);

    balance.ideal_ZMP.left_pos.set(0, 0);
    balance.ideal_ZMP.right_pos.set(0, 0);
    balance.ideal_ZMP.feet_pos.set(0, 0);

    balance.boundary_ZMP.left_pos.set(3, 2);
    balance.boundary_ZMP.right_pos.set(3, 2);
    balance.boundary_ZMP.feet_pos.set(3, 4);

    vB_spline_param.clear();
    force_delay_flag_ = false;
    force_stop_sample_point_flag_ = false;

    supfoot_hip_roll = 0;
    supfoot_hip_pitch = 0;
    supfoot_ankle_roll = 0;
    supfoot_ankle_pitch = 0;
    kick_foot_ankle_pitch = 0;
    ankle_balance_count = 0;

    for(int i = 0; i < 3; i++)balance.init_imu_value[i].pos = sensor.rpy_[i];
    if(balance.init_imu_value[(int)imu::roll].pos == 0 && balance.init_imu_value[(int)imu::pitch].pos == 0 && balance.init_imu_value[(int)imu::yaw].pos == 0)
        return; //initialize failed, try again;
    for(int i = 0; i < sizeof(ideal_p_arry)/sizeof(ideal_p_arry[0]); i++)
    { 
        ideal_p_arry_roll[i] = ideal_p_arry[i];// + balance.init_imu_value[(int)imu::roll].pos;
        ideal_p_arry_pitch[i] = ideal_p_arry[i];// + balance.init_imu_value[(int)imu::pitch].pos;
    }
    
    // T = 3600, 停止T = 1200, 收腳T = 1200
    // if(T_cnt == nullptr)T_cnt = new double[6]{1/12.0, 1/12.0, 1/12.0, 4/12.0, 4/12.0, 1/12.0};
    // if(T_cnt_sum == nullptr)T_cnt_sum = new double[6]{1/12.0, 2/12.0, 3/12.0, 7/12.0, 11/12.0, 1};

    // if(T_cnt == nullptr)T_cnt = new double[6]{1/6.0, 1/6.0, 1/6.0, 0.0, 2/6.0, 1/6.0};
    // if(T_cnt_sum == nullptr)T_cnt_sum = new double[6]{1/6.0, 2/6.0, 3/6.0, 3/6.0, 5/6.0, 1};

    // if(T_cnt == nullptr)
    // {
    //     T_cnt = new double[8]{2, 2, 2, 2, 2, 2, 1, 2};
    //     for(int i = 0; i < 8; i++)
    //         T_cnt[i] /= 15.0;
    // }
    // if(T_cnt_sum == nullptr)
    // {
    //     T_cnt_sum = new double[8]{2, 4, 6, 8, 10, 12, 13, 15};
    //     for(int i = 0; i < 8; i++)
    //         T_cnt_sum[i] /= 15.0;
    // }
    // //側移、抬腳、踢球、停、收腳、停、放下、回正

    if(T_cnt == nullptr)
    {
        T_cnt = new double[9]{2, 2, 0, 2, 2, 2, 0, 0, 2};
        for(int i = 0; i < 9; i++)
            T_cnt[i] /= 12.0;
    }
    if(T_cnt_sum == nullptr)
    {
        T_cnt_sum = new double[9]{2, 4, 4, 6, 8, 10, 10, 10, 12};
        // for(int i = 0; i < 9; i++)
        // {
        //     for(int j = 0; j <= i; j++)
        //     {
        //         T_cnt_sum[i] += T_cnt[j];
        //     }
        // }
        for(int i = 0; i < 9; i++)
            T_cnt_sum[i] /= 12.0;
    }
    //側移、抬腳、停、踢球、停、收腳、停、放下、回正

    // map_param.clear();
    // map_roll.clear();
    // map_pitch.clear();
    // map_kickgait.clear();
    std::vector<float> temp;
    if(map_kickgait.empty())
    {
        map_kickgait["smaple_times_count"] = temp;
        map_kickgait["smaple_times"] = temp;
        map_kickgait["R_Px"] = temp;
        map_kickgait["R_Py"] = temp;
        map_kickgait["R_Pz"] = temp;
        map_kickgait["L_Px"] = temp;
        map_kickgait["L_Py"] = temp;
        map_kickgait["L_Pz"] = temp;
        map_kickgait["supfoot_hip_pitch"] = temp;
        map_kickgait["supfoot_hip_roll"] = temp;
        map_kickgait["kick_foot_ankle_pitch"] = temp;

        map_kickgait["zcontrol_total_supfoot_EP_x"] = temp;
        map_kickgait["zcontrol_total_supfoot_EP_y"] = temp;
    }
    
    if(map_roll.empty())
    {
        map_roll["init_roll_pos"] = temp;
        map_roll["smaple_times_count"] = temp;
        map_roll["pres_roll_pos"] = temp;
        map_roll["passfilter_pres_roll_pos"] = temp;
        map_roll["ideal_roll_vel"] = temp;
        map_roll["pres_roll_vel"] = temp;
        map_roll["passfilter_pres_roll_vel"] = temp;
        map_roll["control_once_roll"] = temp;
        map_roll["control_total_roll"] = temp;
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
        map_pitch["control_once_pitch"] = temp;
        map_pitch["control_total_pitch"] = temp;
    }

    if(map_ZMP.empty())
    {
        map_ZMP["pres_ZMP_left_pos_x"] = temp;
        map_ZMP["pres_ZMP_left_pos_y"] = temp;
        map_ZMP["pres_ZMP_right_pos_x"] = temp;
        map_ZMP["pres_ZMP_right_pos_y"] = temp;
        map_ZMP["pres_ZMP_feet_pos_x"] = temp;
        map_ZMP["pres_ZMP_feet_pos_y"] = temp;

        map_ZMP["pres_ZMPsupfoot_pos_x"] = temp;
        map_ZMP["pres_ZMPsupfoot_pos_y"] = temp;
        map_ZMP["pres_ZMPsupfoot_vel_x"] = temp;
        map_ZMP["pres_ZMPsupfoot_vel_y"] = temp;
        
        map_ZMP["passfilter_pres_ZMPsupfoot_pos_x"] = temp;
        map_ZMP["passfilter_pres_ZMPsupfoot_pos_y"] = temp;
        map_ZMP["passfilter_pres_ZMPsupfoot_vel_x"] = temp;
        map_ZMP["passfilter_pres_ZMPsupfoot_vel_y"] = temp;
        map_ZMP["passfilter_pres_ZMPsupfoot_acc_x"] = temp;
        map_ZMP["passfilter_pres_ZMPsupfoot_acc_y"] = temp;

        map_ZMP["ideal_ZMPsupfoot_pos_x"] = temp;
        map_ZMP["ideal_ZMPsupfoot_pos_y"] = temp;
        

        map_ZMP["boundary_ZMPsupfoot_pos_x"] = temp;
        map_ZMP["boundary_ZMPsupfoot_pos_y"] = temp;

        map_ZMP["control_once_supfoot_EP_x"] = temp;
        map_ZMP["control_once_supfoot_EP_y"] = temp;
        map_ZMP["control_total_supfoot_EP_x"] = temp;
        map_ZMP["control_total_supfoot_EP_y"] = temp;
        
        map_ZMP["pres_ZMPsupfoot_ideal_vel_x"] = temp;
        map_ZMP["pres_ZMPsupfoot_ideal_vel_y"] = temp;
        map_ZMP["pres_ZMPsupfoot_ideal_acc_x"] = temp;
        map_ZMP["pres_ZMPsupfoot_ideal_acc_y"] = temp;

        map_ZMP["delta_v"] = temp;

        map_ZMP["origen_sensor_data_0"] = temp;
        map_ZMP["origen_sensor_data_1"] = temp;
        map_ZMP["origen_sensor_data_2"] = temp;
        map_ZMP["origen_sensor_data_3"] = temp;
        map_ZMP["origen_sensor_data_4"] = temp;
        map_ZMP["origen_sensor_data_5"] = temp;
        map_ZMP["origen_sensor_data_6"] = temp;
        map_ZMP["origen_sensor_data_7"] = temp;

        map_ZMP["sensor_force_0"] = temp;
        map_ZMP["sensor_force_1"] = temp;
        map_ZMP["sensor_force_2"] = temp;
        map_ZMP["sensor_force_3"] = temp;
        map_ZMP["sensor_force_4"] = temp;
        map_ZMP["sensor_force_5"] = temp;
        map_ZMP["sensor_force_6"] = temp;
        map_ZMP["sensor_force_7"] = temp;

        // map_ZMP["theta"] = temp;
        // map_ZMP["vel"] = temp;
        // map_ZMP["once"] = temp;
        // map_ZMP["total"] = temp;
    }

    map_roll.find("init_roll_pos")->second.push_back(balance.init_imu_value[(int)imu::roll].pos);
    map_pitch.find("init_pitch_pos")->second.push_back(balance.init_imu_value[(int)imu::pitch].pos);
    updateControlPoint();
    init_param_flag_ = true;
}
 
void KickingGait::updateControlPoint()
{
    vB_spline_param.clear();
    double &Y_Swing = parameterinfo->parameters.Y_Swing_Range;
    // if(parameterinfo->walking_mode == 9)
    // {
    //     Y_Swing > 0 ? Y_Swing = -Y_Swing : Y_Swing = Y_Swing;
    //     for(int i = 0; i < 201; i++)
    //     {
    //         ideal_ZMP_py_arry[i] = ideal_origen_ZMP_py_arry[i] - parameterinfo->parameters.Back_Point_Z;
    //         ideal_ZMP_vy_arry[i] = ideal_origen_ZMP_vy_arry[i];
    //     }
    // }
    
    // else if(parameterinfo->walking_mode == 10)
    // {
    //     Y_Swing > 0 ? Y_Swing = Y_Swing : Y_Swing = -Y_Swing;
    //     for(int i = 0; i < 201; i++)
    //     {
    //         ideal_ZMP_py_arry[i] = ideal_origen_ZMP_py_arry[i] + 1;
    //         ideal_ZMP_vy_arry[i] = ideal_origen_ZMP_vy_arry[i];
    //     }
    // }
    
    
    float K_X = parameterinfo->parameters.Kick_Point_X;//20;
    float K_Y = parameterinfo->parameters.Kick_Point_Y;//0;
    float K_Z = parameterinfo->parameters.Kick_Point_Z;//6.5;
    // float B_X = parameterinfo->parameters.Back_Point_X;//-12;
    // float B_Z = parameterinfo->parameters.Back_Point_Z;//13.5;
    // float K_X = 20;
    // float K_Y = 0;
    // float K_Z = 6.5;
    float B_X = -12;
    float B_Z = 13.5;
    // Kp = 0.4, Kd = 0.5
    // balance.PIDsupfoot_EPx.setKpid(parameterinfo->parameters.Kick_Point_X/100.0, 0, 0);
    balance.PIDsupfoot_EPy.setKpid(0, 0, 0);
    // balance.PIDsupfoot_EPy.setKpid(parameterinfo->parameters.Kick_Point_Y/100.0, 0, 0);
    // balance.PIDsupfoot_EPx.setKpid(0, 0, 0);
    // if(parameterinfo->parameters.Kick_Point_Z == 1)
    {
        // balance.PIDsupfoot_hip_pitch.setKpid(parameterinfo->parameters.Kick_Point_Y/100.0, 0, parameterinfo->parameters.Back_Point_Z/100.0);
        balance.PIDsupfoot_hip_roll.setKpid(parameterinfo->parameters.Back_Point_X/100.0, 0, parameterinfo->parameters.Back_Point_Z/100.0);
    }
    // else
    // {
        balance.PIDsupfoot_hip_pitch.setKpid(0, 0, 0);
        // balance.PIDsupfoot_hip_roll.setKpid(0, 0, 0);
    // }
    // balance.PIDsupfoot_hip_pitch.setKpid(parameterinfo->parameters.Kick_Point_X/100.0, 0, parameterinfo->parameters.Kick_Point_Y/100.0);
    // balance.PIDsupfoot_hip_roll.setKpid(parameterinfo->parameters.Kick_Point_X/100.0, 0, parameterinfo->parameters.Kick_Point_Y/100.0);
    B_Spline_Param B_Spline_param;
    std::vector<Point3DParam> vP;
    Point3DParam P;
    vP.push_back(P.set(0, Y_Swing, 0));
    vP.push_back(P.set(0, Y_Swing, B_Z/2.0));
    vP.push_back(P.set(B_X*2/3.0, Y_Swing+K_Y/3.0, B_Z/2.0));
    vP.push_back(P.set(B_X, Y_Swing+K_Y*2/3.0, B_Z*2/3.0));
    vP.push_back(P.set(B_X, Y_Swing+K_Y, B_Z));
    vB_spline_param.push_back(B_Spline_param.generate(vP, B_spline.generateClampedKnotVector(vP.size()-1, B_Spline_Param::g_k)));
    vP.clear();

    vP.push_back(P.set(B_X, Y_Swing+K_Y, B_Z));
    vP.push_back(P.set(abs(K_X-B_X)/3.0+B_X, Y_Swing+K_Y, B_Z-(B_Z-K_Z)*1.15));
    vP.push_back(P.set(K_X, Y_Swing+K_Y, K_Z));
    vB_spline_param.push_back(B_Spline_param.generate(vP, B_spline.generateClampedKnotVector(vP.size()-1, B_Spline_Param::g_k)));
    vP.clear();

    vP.push_back(P.set(K_X, Y_Swing+K_Y, K_Z));
    vP.push_back(P.set((K_X)*2/3.0, Y_Swing+K_Y/2.0, K_Z*11/16.0));
    vP.push_back(P.set((K_X)/4.0, Y_Swing, K_Z*3/4.0));
    vP.push_back(P.set(0, Y_Swing, K_Z/2.0));
    vP.push_back(P.set(0, Y_Swing, 0));
    vB_spline_param.push_back(B_Spline_param.generate(vP, B_spline.generateClampedKnotVector(vP.size()-1, B_Spline_Param::g_k)));
    vP.clear();
 
 
    map_param["Support_Foot_Hip_Upper_Pitch"] = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    map_param["Support_Foot_Ankle_Upper_Pitch"] = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    map_param["Kick_Foot_Ankle_Upper_Pitch"] = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    map_param["Period_T"] = parameterinfo->parameters.Period_T;
    map_param["Sample_Time"] = parameterinfo->parameters.Sample_Time;
    map_param["Y_Swing"] = Y_Swing;
    map_param["K_X"] = K_X;
    map_param["K_Y"] = K_Y;
    map_param["K_Z"] = K_Z;
    map_param["B_X"] = B_X; 
    map_param["B_Z"] = B_Z;
    map_param["Kp"] = parameterinfo->parameters.Kick_Point_X/100.0;
    map_param["Kd"] = parameterinfo->parameters.Kick_Point_Y/100.0;
}

void KickingGait::kickingCycle(int walking_mode)
{
    if(parameterinfo->complan.walking_state == StartStep)
    {
        if(init_param_flag_)
        {
            if(!force_stop_sample_point_flag_)parameterinfo->complan.sample_point_++;
            if(parameterinfo->complan.sample_point_ > parameterinfo->parameters.Sample_Time)
            {
                parameterinfo->complan.walking_state = StopStep;
                SaveData();
                init_param_flag_ = false;
                kicking_process_flag_ = false;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->complan.sample_point_ = 0;
            } 
            else if(parameterinfo->complan.sample_point_ > 0)
            {
                kicking_process_flag_ = true;
                kickingProcess(walking_mode);
            }
        }
        else 
        {
            initialize();
        }
    }
    else if(parameterinfo->complan.walking_state == StopStep && kicking_process_flag_)
    {
        SaveData();
        init_param_flag_ = false;
        kicking_process_flag_ = false;
        parameterinfo->complan.walking_stop = true;
        parameterinfo->complan.sample_point_ = 0;
    }
}

void KickingGait::kickingProcess(int walking_mode)
{
    if(walking_mode == 9)
    {
        rightKickBall();
    }
    else if(walking_mode == 10)
    {
        leftKickBall();
    }
}

void KickingGait::rightKickBall()
{
    // Point3DParam R_P; 
    // Point3DParam L_P;

    // float T_ms = parameterinfo->parameters.Period_T - 600;
    // float T = T_ms/1000.0;
    // float support_foot_hip_upper_pitch = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    // float kick_foot_ankle_upper_pitch = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    // float support_foot_ankle_upper_pitch = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    // float Y_Swing = parameterinfo->parameters.Y_Swing_Range;
    // int smaple_times_count = parameterinfo->complan.sample_point_;
    // int smaple_point = T_ms/30;
    // if(smaple_times_count < (int)(smaple_point*T_cnt_sum[0]))
    // {
    //     float omega = 2*PI/(T*T_cnt[0]);
    //     float t = smaple_times_count*0.03;
    //     R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[0]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[1]))
    // {
    //     float omega = 2*PI/(T*T_cnt[1]);
    //     float t = ((smaple_times_count+1)-smaple_point*T_cnt_sum[0])*0.03;
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[0])*(1.0/(smaple_point*T_cnt[1]));
    //     R_P = B_spline.C(u, vB_spline_param[0]);
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[1]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[2]))
    // {
    //     float omega = 2*PI/(T*T_cnt[2]);
    //     float t1 = (smaple_point*T_cnt_sum[2]-(smaple_times_count+1))*0.03;
    //     float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[1])*0.03;
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[1])*(1.0/(smaple_point*T_cnt[2]));
    //     R_P = B_spline.C(u, vB_spline_param[1]);
    //     L_P.set(0, Y_Swing, 0);
    //     // this->supfoot_hip_pitch = (support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI) + (-support_foot_hip_upper_pitch/2.0)*(omega*t2-sin(omega*t2))/(2*PI))/180.0*PI;
    //     this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[2]));
    // }
    // else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[3]))
    // {
    //     R_P = B_spline.C(1, vB_spline_param[1]);
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[3]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    // {
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[3])*(1.0/(smaple_point*T_cnt[4]));
    //     R_P = B_spline.C(u, vB_spline_param[2]);
    //     L_P.set(0, Y_Swing, 0);
    //     float omega2 = 2*PI/(T*T_cnt[4]);
    //     float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[3])*0.03;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega2*t2-sin(omega2*t2))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     if(smaple_times_count < (int)(smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)))
    //     {
    //         float omega1 = 2*PI/(T*T_cnt[4]/2.0);
    //         float t1 = (smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)-(smaple_times_count+1))*0.03;
    //         this->supfoot_hip_pitch = 0;
    //     }
    //     else
    //     {
    //         this->supfoot_hip_pitch = 0;
    //     }
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[4]));
    // }
    // else if (smaple_times_count <= smaple_point)
    // {
    //     float omega = 2*PI/(T*T_cnt[5]);
    //     float t = (smaple_point-smaple_times_count)*0.03;
    //     R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[5]));
    // }
    // else
    // { 
    //     R_P.set(0, 0, 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back(-1);
    // } 

    // Point3DParam tmep_EP_total;
    // if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    // {
        
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    // {
    //     // tmep_EP_total = endPointBalanceControl();
    //     // L_P.x -= tmep_EP_total.x;
    //     // L_P.y -= tmep_EP_total.y;
    //     // R_P.y -= tmep_EP_total.y;
    // }
    // else if (smaple_times_count <= smaple_point)//DSP
    // {
        
    // }

    // parameterinfo->points.IK_Point_RX       = R_P.x; 
    // parameterinfo->points.IK_Point_RY       = R_P.y;
    // parameterinfo->points.IK_Point_RZ       = parameterinfo->parameters.COM_Height - R_P.z;
    // parameterinfo->points.IK_Point_RThta    = 0;
    // parameterinfo->points.IK_Point_LX       = L_P.x;
    // parameterinfo->points.IK_Point_LY       = L_P.y;
    // parameterinfo->points.IK_Point_LZ       = parameterinfo->parameters.COM_Height - L_P.z;
    // parameterinfo->points.IK_Point_LThta    = 0;

    // map_kickgait.find("smaple_times_count")->second.push_back(smaple_times_count);
    // map_kickgait.find("R_Px")->second.push_back(R_P.x);
    // map_kickgait.find("R_Py")->second.push_back(R_P.y);
    // map_kickgait.find("R_Pz")->second.push_back(R_P.z);
    // map_kickgait.find("L_Px")->second.push_back(L_P.x);
    // map_kickgait.find("L_Py")->second.push_back(L_P.y);
    // map_kickgait.find("L_Pz")->second.push_back(L_P.z);

    // map_kickgait.find("zcontrol_total_supfoot_EP_x")->second.push_back(tmep_EP_total.x);
    // map_kickgait.find("zcontrol_total_supfoot_EP_y")->second.push_back(tmep_EP_total.y);

    // Point3DParam R_P; 
    // Point3DParam L_P;

    // float T_ms = parameterinfo->parameters.Period_T - 600;
    // float T = T_ms/1000.0;
    // float support_foot_hip_upper_pitch = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    // float kick_foot_ankle_upper_pitch = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    // float support_foot_ankle_upper_pitch = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    // float Y_Swing = parameterinfo->parameters.Y_Swing_Range;
    // int smaple_times_count = parameterinfo->complan.sample_point_;
    // int smaple_point = T_ms/30;
    // if(smaple_times_count < (int)(smaple_point*T_cnt_sum[0]))
    // {
    //     float omega = 2*PI/(T*T_cnt[0]);
    //     float t = smaple_times_count*0.03;
    //     R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[0]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[1]))
    // {
    //     float omega = 2*PI/(T*T_cnt[1]);
    //     float t = ((smaple_times_count+1)-smaple_point*T_cnt_sum[0])*0.03;
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[0])*(1.0/(smaple_point*T_cnt[1]));
    //     R_P = B_spline.C(u, vB_spline_param[0]);
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[1]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[2]))
    // {
    //     float omega = 2*PI/(T*T_cnt[2]);
    //     float t1 = (smaple_point*T_cnt_sum[2]-(smaple_times_count+1))*0.03;
    //     float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[1])*0.03;
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[1])*(1.0/(smaple_point*T_cnt[2]));
    //     R_P = B_spline.C(u, vB_spline_param[1]);
    //     L_P.set(0, Y_Swing, 0);
    //     // this->supfoot_hip_pitch = (support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI) + (-support_foot_hip_upper_pitch/2.0)*(omega*t2-sin(omega*t2))/(2*PI))/180.0*PI;
    //     this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[2]));
    // }
    // else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[3]))
    // {
    //     R_P = B_spline.C(1, vB_spline_param[1]);
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[3]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    // {
    //     float u = (smaple_times_count - smaple_point*T_cnt_sum[3])*(1.0/(smaple_point*T_cnt[4]));
    //     R_P = B_spline.C(u, vB_spline_param[2]);
    //     L_P.set(0, Y_Swing, 0);
    //     float omega2 = 2*PI/(T*T_cnt[4]);
    //     float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[3])*0.03;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega2*t2-sin(omega2*t2))/(2*PI)/180.0*PI;
    //     this->supfoot_hip_roll = 0;
    //     if(smaple_times_count < (int)(smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)))
    //     {
    //         float omega1 = 2*PI/(T*T_cnt[4]/2.0);
    //         float t1 = (smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)-(smaple_times_count+1))*0.03;
    //         this->supfoot_hip_pitch = 0;
    //     }
    //     else
    //     {
    //         this->supfoot_hip_pitch = 0;
    //     }
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[4]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[5]))
    // {
    //     R_P = B_spline.C(1, vB_spline_param[2]);
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[5]));
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[6]))
    // {
    //     float omega = 2*PI/(T*T_cnt[6]);
    //     float t = (smaple_point*T_cnt_sum[6]-(smaple_times_count+1))*0.03;
    //     R_P.set(0, Y_Swing, (parameterinfo->parameters.Kick_Point_Z/2.0)*(omega*t-sin(omega*t))/(2*PI));
    //     L_P.set(0, Y_Swing, 0);
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[6]));
    // }
    // else if (smaple_times_count <= smaple_point)
    // {
    //     float omega = 2*PI/(T*T_cnt[7]);
    //     float t = (smaple_point-smaple_times_count)*0.03;
    //     R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
    //     map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[7]));
    // }
    // else
    // { 
    //     R_P.set(0, 0, 0);
    //     L_P = R_P;
    //     this->supfoot_hip_pitch = 0;
    //     this->supfoot_hip_roll = 0;
    //     this->kick_foot_ankle_pitch = 0;
    //     map_kickgait.find("smaple_times")->second.push_back(-1);
    // } 

    // Point3DParam tmep_EP_total;
    // tmep_EP_total.initialize();
    // if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    // {
        
    // }
    // else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    // {
    //     // tmep_EP_total = endPointBalanceControl();
    //     // L_P.x -= tmep_EP_total.x;
    //     // L_P.y -= tmep_EP_total.y;
    //     // R_P.y -= tmep_EP_total.y;
    // }
    // else if (smaple_times_count <= smaple_point)//DSP
    // {
        
    // }

    // parameterinfo->points.IK_Point_RX       = R_P.x; 
    // parameterinfo->points.IK_Point_RY       = R_P.y;
    // parameterinfo->points.IK_Point_RZ       = parameterinfo->parameters.COM_Height - R_P.z;
    // parameterinfo->points.IK_Point_RThta    = 0;
    // parameterinfo->points.IK_Point_LX       = L_P.x;
    // parameterinfo->points.IK_Point_LY       = L_P.y;
    // parameterinfo->points.IK_Point_LZ       = parameterinfo->parameters.COM_Height - L_P.z;
    // parameterinfo->points.IK_Point_LThta    = 0;

    // map_kickgait.find("smaple_times_count")->second.push_back(smaple_times_count);
    // map_kickgait.find("R_Px")->second.push_back(R_P.x);
    // map_kickgait.find("R_Py")->second.push_back(R_P.y);
    // map_kickgait.find("R_Pz")->second.push_back(R_P.z);
    // map_kickgait.find("L_Px")->second.push_back(L_P.x);
    // map_kickgait.find("L_Py")->second.push_back(L_P.y);
    // map_kickgait.find("L_Pz")->second.push_back(L_P.z);

    // map_kickgait.find("zcontrol_total_supfoot_EP_x")->second.push_back(tmep_EP_total.x);
    // map_kickgait.find("zcontrol_total_supfoot_EP_y")->second.push_back(tmep_EP_total.y);


    Point3DParam R_P; 
    Point3DParam L_P;

    float T_ms = parameterinfo->parameters.Period_T - 600;
    float T = T_ms/1000.0;
    float support_foot_hip_upper_pitch = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    float kick_foot_ankle_upper_pitch = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    float support_foot_ankle_upper_pitch = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    float Y_Swing = parameterinfo->parameters.Y_Swing_Range;
    int smaple_times_count = parameterinfo->complan.sample_point_;
    int smaple_point = T_ms/30;
    if(smaple_times_count < (int)(smaple_point*T_cnt_sum[0]))
    {
        float omega = 2*PI/(T*T_cnt[0]);
        float t = smaple_times_count*0.03;
        R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
        L_P = R_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[0]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[1]))
    {
        float omega = 2*PI/(T*T_cnt[1]);
        float t = ((smaple_times_count+1)-smaple_point*T_cnt_sum[0])*0.03;
        float u = (smaple_times_count - smaple_point*T_cnt_sum[0])*(1.0/(smaple_point*T_cnt[1]));
        R_P = B_spline.C(u, vB_spline_param[0]);
        L_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[1]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[2]))
    {
        R_P = B_spline.C(1, vB_spline_param[0]);
        L_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = support_foot_hip_upper_pitch/180.0*PI;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[2]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[3]))
    {
        float omega = 2*PI/(T*T_cnt[3]);
        float t1 = (smaple_point*T_cnt_sum[3]-(smaple_times_count+1))*0.03;
        float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[2])*0.03;
        float u = (smaple_times_count - smaple_point*T_cnt_sum[2])*(1.0/(smaple_point*T_cnt[3]));
        if(smaple_times_count == 30)u = 0;
        R_P = B_spline.C(u, vB_spline_param[1]);
        L_P.set(0, Y_Swing, 0);
        // this->supfoot_hip_pitch = (support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI) + (-support_foot_hip_upper_pitch/2.0)*(omega*t2-sin(omega*t2))/(2*PI))/180.0*PI;
        this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[3]));
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    {
        R_P = B_spline.C(1, vB_spline_param[1]);
        L_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[4]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[5]))
    {
        float u = (smaple_times_count - smaple_point*T_cnt_sum[4])*(1.0/(smaple_point*T_cnt[5]));
        R_P = B_spline.C(u, vB_spline_param[2]);
        L_P.set(0, Y_Swing, 0);
        float omega2 = 2*PI/(T*T_cnt[5]);
        float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[4])*0.03;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega2*t2-sin(omega2*t2))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        if(smaple_times_count < (int)(smaple_point*(T_cnt_sum[4]+T_cnt[5]/2.0)))
        {
            float omega1 = 2*PI/(T*T_cnt[5]/2.0);
            float t1 = (smaple_point*(T_cnt_sum[4]+T_cnt[5]/2.0)-(smaple_times_count+1))*0.03;
            this->supfoot_hip_pitch = 0;
        }
        else
        {
            this->supfoot_hip_pitch = 0;
        }
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[5]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[6]))
    {
        R_P = B_spline.C(1, vB_spline_param[2]);
        L_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[6]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[7]))
    {
        float omega = 2*PI/(T*T_cnt[7]);
        float t = ((smaple_times_count+1)-smaple_point*T_cnt_sum[6])*0.03;
        R_P.set(0, Y_Swing, parameterinfo->parameters.Kick_Point_Z/2.0 - (parameterinfo->parameters.Kick_Point_Z/2.0-1)*(omega*t-sin(omega*t))/(2*PI));
        L_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[7]));
    }
    else if (smaple_times_count <= smaple_point)
    {
        float omega = 2*PI/(T*T_cnt[8]);
        float t = (smaple_point-smaple_times_count)*0.03;
        // R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 1.0*(omega*t-sin(omega*t))/(2*PI));
        // L_P.set(0, R_P.y, 0);
        R_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
        L_P = R_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[8]));
    }
    else
    { 
        R_P.set(0, 0, 0);
        L_P = R_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back(-1);
    } 

    Point3DParam tmep_EP_total;
    tmep_EP_total.initialize();
    if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    {
        
    }
    else if(smaple_times_count < (int)smaple_point*T_cnt_sum[1])
    {
        
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[7]))
    {
        // tmep_EP_total = endPointBalanceControl();
        // L_P.x -= tmep_EP_total.x;
        // L_P.y -= tmep_EP_total.y;
        // R_P.y -= tmep_EP_total.y;
    }
    else if (smaple_times_count <= smaple_point)//DSP
    {
        
    }

    parameterinfo->points.IK_Point_RX       = R_P.x; 
    parameterinfo->points.IK_Point_RY       = R_P.y;
    parameterinfo->points.IK_Point_RZ       = parameterinfo->parameters.COM_Height - R_P.z;
    parameterinfo->points.IK_Point_RThta    = 0;
    parameterinfo->points.IK_Point_LX       = L_P.x;
    parameterinfo->points.IK_Point_LY       = L_P.y;
    parameterinfo->points.IK_Point_LZ       = parameterinfo->parameters.COM_Height - L_P.z;
    parameterinfo->points.IK_Point_LThta    = 0;

    map_kickgait.find("smaple_times_count")->second.push_back(smaple_times_count);
    map_kickgait.find("R_Px")->second.push_back(R_P.x);
    map_kickgait.find("R_Py")->second.push_back(R_P.y);
    map_kickgait.find("R_Pz")->second.push_back(R_P.z);
    map_kickgait.find("L_Px")->second.push_back(L_P.x);
    map_kickgait.find("L_Py")->second.push_back(L_P.y);
    map_kickgait.find("L_Pz")->second.push_back(L_P.z);

    map_kickgait.find("zcontrol_total_supfoot_EP_x")->second.push_back(tmep_EP_total.x);
    map_kickgait.find("zcontrol_total_supfoot_EP_y")->second.push_back(tmep_EP_total.y);
}

void KickingGait::leftKickBall()
{
    Point3DParam R_P;
    Point3DParam L_P;

    float T_ms = parameterinfo->parameters.Period_T - 600;
    float T = T_ms/1000.0;
    float support_foot_hip_upper_pitch = parameterinfo->parameters.Support_Foot_Hip_Upper_Pitch;
    float kick_foot_ankle_upper_pitch = parameterinfo->parameters.Kick_Foot_Ankle_Upper_Pitch;
    float support_foot_ankle_upper_pitch = parameterinfo->parameters.Support_Foot_Ankle_Upper_Pitch;
    float Y_Swing = parameterinfo->parameters.Y_Swing_Range;
    int smaple_times_count = parameterinfo->complan.sample_point_;
    int smaple_point = T_ms/30;

    if(smaple_times_count < (int)(smaple_point*T_cnt_sum[0]))
    {
        float omega = 2*PI/(T*T_cnt[0]);
        float t = smaple_times_count*0.03;
        L_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
        R_P = L_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[0]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[1]))
    {
        float omega = 2*PI/(T*T_cnt[1]);
        float t = ((smaple_times_count+1)-smaple_point*T_cnt_sum[0])*0.03;
        float u = (smaple_times_count - smaple_point*T_cnt_sum[0])*(1.0/(smaple_point*T_cnt[1]));
        L_P = B_spline.C(u, vB_spline_param[0]);
        R_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[1]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[2]))
    {
        float omega = 2*PI/(T*T_cnt[2]);
        float t1 = (smaple_point*T_cnt_sum[2]-(smaple_times_count+1))*0.03;
        float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[1])*0.03;
        float u = (smaple_times_count - smaple_point*T_cnt_sum[1])*(1.0/(smaple_point*T_cnt[2]));
        L_P = B_spline.C(u, vB_spline_param[1]);
        R_P.set(0, Y_Swing, 0);
        // this->supfoot_hip_pitch = (support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI) + (-support_foot_hip_upper_pitch/2.0)*(omega*t2-sin(omega*t2))/(2*PI))/180.0*PI;
        this->supfoot_hip_pitch = support_foot_hip_upper_pitch*(omega*t1-sin(omega*t1))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[2]));
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[3]))
    {
        L_P = B_spline.C(1, vB_spline_param[1]);
        R_P.set(0, Y_Swing, 0);
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[3]));
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    {
        float u = (smaple_times_count - smaple_point*T_cnt_sum[3])*(1.0/(smaple_point*T_cnt[4]));
        L_P = B_spline.C(u, vB_spline_param[2]);
        R_P.set(0, Y_Swing, 0);
        float omega2 = 2*PI/(T*T_cnt[4]);
        float t2 = ((smaple_times_count+1)-smaple_point*T_cnt_sum[3])*0.03;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega2*t2-sin(omega2*t2))/(2*PI)/180.0*PI;
        this->supfoot_hip_roll = 0;
        if(smaple_times_count < (int)(smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)))
        {
            float omega1 = 2*PI/(T*T_cnt[4]/2.0);
            float t1 = (smaple_point*(T_cnt_sum[3]+T_cnt[4]/2.0)-(smaple_times_count+1))*0.03;
            this->supfoot_hip_pitch = 0;
        }
        else
        {
            this->supfoot_hip_pitch = 0;
        }
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[4]));
    }
    else if (smaple_times_count <= smaple_point)
    {
        float omega = 2*PI/(T*T_cnt[5]);
        float t = (smaple_point-smaple_times_count)*0.03;
        L_P.set(0, Y_Swing*(omega*t-sin(omega*t))/(2*PI), 0);
        R_P = L_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = kick_foot_ankle_upper_pitch*(omega*t-sin(omega*t))/(2*PI)/180.0*PI;
        map_kickgait.find("smaple_times")->second.push_back((int)(smaple_point*T_cnt_sum[5]));
    }
    else
    { 
        L_P.set(0, 0, 0);
        R_P = L_P;
        this->supfoot_hip_pitch = 0;
        this->supfoot_hip_roll = 0;
        this->kick_foot_ankle_pitch = 0;
        map_kickgait.find("smaple_times")->second.push_back(-1);
    } 

    Point3DParam tmep_EP_total;
    tmep_EP_total.initialize();
    if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    {
        
    }
    else if (smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))
    {
        tmep_EP_total = endPointBalanceControl();
        R_P.x -= tmep_EP_total.x;
        R_P.y -= tmep_EP_total.y;
        L_P.y -= tmep_EP_total.y;
    }
    else if (smaple_times_count <= smaple_point)//DSP
    {
        
    }

    parameterinfo->points.IK_Point_RX       = R_P.x; 
    parameterinfo->points.IK_Point_RY       = R_P.y;
    parameterinfo->points.IK_Point_RZ       = parameterinfo->parameters.COM_Height - R_P.z;
    parameterinfo->points.IK_Point_RThta    = 0;
    parameterinfo->points.IK_Point_LX       = L_P.x;
    parameterinfo->points.IK_Point_LY       = L_P.y;
    parameterinfo->points.IK_Point_LZ       = parameterinfo->parameters.COM_Height - L_P.z;
    parameterinfo->points.IK_Point_LThta    = 0;

    map_kickgait.find("smaple_times_count")->second.push_back(smaple_times_count);
    map_kickgait.find("R_Px")->second.push_back(R_P.x);
    map_kickgait.find("R_Py")->second.push_back(R_P.y);
    map_kickgait.find("R_Pz")->second.push_back(R_P.z);
    map_kickgait.find("L_Px")->second.push_back(L_P.x);
    map_kickgait.find("L_Py")->second.push_back(L_P.y);
    map_kickgait.find("L_Pz")->second.push_back(L_P.z);

    map_kickgait.find("zcontrol_total_supfoot_EP_x")->second.push_back(tmep_EP_total.x);
    map_kickgait.find("zcontrol_total_supfoot_EP_y")->second.push_back(tmep_EP_total.y);
}

void KickingGait::hipPitchControl()
{
    if(parameterinfo->walking_mode == 9)
    {
        Points.Thta[10] += this->supfoot_hip_roll;
        Points.Thta[11] += this->supfoot_hip_pitch;
        Points.Thta[13] += this->supfoot_ankle_pitch;
        Points.Thta[14] += this->supfoot_ankle_roll;

        Points.Thta[19] += this->kick_foot_ankle_pitch;
    }
    else if(parameterinfo->walking_mode == 10)
    {
        //pitch maybe inverse
        Points.Thta[16] += this->supfoot_hip_roll;
        Points.Thta[17] += this->supfoot_hip_pitch;

        Points.Thta[13] += this->kick_foot_ankle_pitch;
    }
}

void KickingGait::ankleBalanceControl()
{ 
    int origen_sensor_data_temp[8];
    for(int i = 0; i < 4; i++)origen_sensor_data_temp[i] = sensor.press_left_[i];
    for(int i = 4; i < 8; i++)origen_sensor_data_temp[i] = sensor.press_right_[i-4];
    balance.prev_ZMP = balance.pres_ZMP;
    balance.ZMP_process->setpOrigenSensorData(origen_sensor_data_temp);
    balance.pres_ZMP = balance.ZMP_process->getZMPValue();


    // balance.passfilter_pres_ZMP.left_pos.x = balance.butterfilter_ZMPsupfoot.pos_x.getValue(balance.pres_ZMP.left_pos.x);
    // balance.passfilter_pres_ZMP.left_pos.y = balance.butterfilter_ZMPsupfoot.pos_y.getValue(balance.pres_ZMP.left_pos.y);


    float T_ms = parameterinfo->parameters.Period_T - 600;
    float T = T_ms/1000.0;
    int smaple_point = T_ms/30;
    int smaple_times_count = parameterinfo->complan.sample_point_;
    // balance.pres_ZMP.left.set(left_ZMP_X, left_ZMP_Y);
    // balance.pres_ZMP.right.set(right_ZMP_X, right_ZMP_Y);
    // balance.pres_ZMP.feet.set((left_ZMP_X + right_ZMP_X)/2.0, (left_ZMP_Y + right_ZMP_Y)/2.0);
    // double m = 4.5;
    // double g = 9.8;
    // double ZMP_x = 0;
    // double ZMP_y = 0;
    // double dx = 0;
    // double dy = 0.3;
    // double L = 0;
    // double F = 0;
    // double m_theta = 0;
    // double zmp_theta = 0;
    // double touqe = 0;
    // double touqe_x = 0;
    // double touqe_y = 0;

    double ZMP_x = 0;
    double ZMP_y = 0;
    double dx = 0;
    double dy = 0.38;
    double L = 0;
    double m_theta = 0;
    double control_theta = 0;
    double ZMP_v_x = 0;
    double ZMP_v_y = 0;
    double m_theta_y = 0;
    double control_theta_y = 0;
    // if(parameterinfo->walking_mode == 9)
    
    if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    {
        // map_ZMP.find("control_once_ankle_pitch")->second.push_back(0);
        // map_ZMP.find("control_total_ankle_pitch")->second.push_back(0);
        // map_ZMP.find("control_ankle_pitch_theta")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[7]))//SSP
    { 
        // ZMP_x = balance.passfilter_pres_ZMP.left.x/100.0;
        // ZMP_y = balance.passfilter_pres_ZMP.left.y/100.0;
        // dx = std::sqrt(ZMP_x*ZMP_x + ZMP_y*ZMP_y);
        // m_theta = std::atan2(dy,dx);
        // L = std::sqrt(dx*dx + dy*dy);
        // F = m*g*std::cos(m_theta);
        // touqe = F*L;
        // zmp_theta = std::atan2(balance.passfilter_pres_ZMP.left.y, balance.passfilter_pres_ZMP.left.x);
        // touqe_x = touqe*std::cos(zmp_theta);
        // touqe_y = touqe*std::sin(zmp_theta);

 
        if(ankle_balance_count % 4 == 0)
        {
            ankle_balance_count = 0;
            ZMP_x = balance.pres_ZMP.left_pos.x/100.0;
            ZMP_y = balance.pres_ZMP.left_pos.y/100.0;
            m_theta = std::atan2(dy,ZMP_x);
            control_theta = PI/2.0-m_theta;
            control_theta = control_theta/PI*180.0;

            m_theta_y = std::atan2(dy,ZMP_y);
            control_theta_y = PI/2.0-m_theta_y;
            control_theta_y = control_theta_y/PI*180.0;

            balance.prev_ankle_roll = balance.pres_ankle_roll;
            balance.pres_ankle_roll.pos = control_theta_y;
            balance.pres_ankle_roll.vel = (balance.pres_ankle_roll.pos - balance.prev_ankle_roll.pos)/0.12;
            balance.ideal_ankle_roll.vel = getIdealV(balance.pres_ZMP.left_pos.y, ideal_ZMP_py_arry, ideal_ZMP_vy_arry);
            
            balance.PIDsupfoot_EPy.setControlGoal(balance.ideal_ankle_roll.vel);
            balance.supfoot_EPy_value.control_value_once = balance.PIDsupfoot_EPy.calculateExpValue(balance.pres_ankle_roll.vel)*0.12;

            balance.prev_ankle_pitch = balance.pres_ankle_pitch;
            balance.pres_ankle_pitch.pos = control_theta;
            balance.pres_ankle_pitch.vel = (balance.pres_ankle_pitch.pos - balance.prev_ankle_pitch.pos)/0.12;
            balance.ideal_ankle_pitch.vel = getIdealV(balance.pres_ZMP.left_pos.x, ideal_ZMP_px_arry, ideal_ZMP_vx_arry);
            
            balance.PIDsupfoot_EPx.setControlGoal(balance.ideal_ankle_pitch.vel);
            balance.supfoot_EPx_value.control_value_once = balance.PIDsupfoot_EPx.calculateExpValue(balance.pres_ankle_pitch.vel)*0.12;
            
            // map_ZMP.find("theta")->second.push_back(balance.pres_ankle_pitch.pos);
            // map_ZMP.find("vel")->second.push_back(balance.pres_ankle_pitch.vel);
            // map_ZMP.find("once")->second.push_back(balance.supfoot_EPx_value.control_value_once);
            // map_ZMP.find("total")->second.push_back(balance.supfoot_EPx_value.control_value_total);
            map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(balance.pres_ZMP.left_pos.x);
            map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(balance.pres_ZMP.left_pos.y);
            map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(balance.pres_ZMP.right_pos.x);
            map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(balance.pres_ZMP.right_pos.y);
            map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(balance.pres_ZMP.feet_pos.x);
            map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(balance.pres_ZMP.feet_pos.y);

            map_ZMP.find("pres_ZMPsupfoot_pos_x")->second.push_back(balance.pres_ZMP.left_pos.x);
            map_ZMP.find("pres_ZMPsupfoot_pos_y")->second.push_back(balance.pres_ZMP.left_pos.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_x")->second.push_back(balance.passfilter_pres_ZMP.left_pos.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_y")->second.push_back(balance.passfilter_pres_ZMP.left_pos.y);
            
            map_ZMP.find("pres_ZMPsupfoot_vel_x")->second.push_back(balance.pres_ZMP.left_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_vel_y")->second.push_back(balance.pres_ZMP.left_vel.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_x")->second.push_back(balance.passfilter_pres_ZMP.left_vel.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_y")->second.push_back(balance.passfilter_pres_ZMP.left_vel.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_acc_x")->second.push_back(balance.passfilter_pres_ZMP.left_acc.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_acc_y")->second.push_back(balance.passfilter_pres_ZMP.left_acc.y);
            
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_x")->second.push_back(balance.ideal_ZMP.left_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_y")->second.push_back(balance.ideal_ZMP.left_vel.y);
            map_ZMP.find("pres_ZMPsupfoot_ideal_acc_x")->second.push_back(balance.ideal_ZMP.left_acc.x);
            map_ZMP.find("pres_ZMPsupfoot_ideal_acc_y")->second.push_back(balance.ideal_ZMP.left_acc.y);

            // map_ZMP.find("delta_v")->second.push_back(test_PPP);

            map_ZMP.find("ideal_ZMPsupfoot_pos_x")->second.push_back(balance.ideal_ZMP.left_pos.x);
            map_ZMP.find("ideal_ZMPsupfoot_pos_y")->second.push_back(balance.ideal_ZMP.left_pos.y);
            
            map_ZMP.find("boundary_ZMPsupfoot_pos_x")->second.push_back(balance.boundary_ZMP.left_pos.x);
            map_ZMP.find("boundary_ZMPsupfoot_pos_y")->second.push_back(balance.boundary_ZMP.left_pos.y);
            
            map_ZMP.find("control_once_supfoot_EP_x")->second.push_back(balance.supfoot_EPx_value.control_value_once);
            map_ZMP.find("control_once_supfoot_EP_y")->second.push_back(balance.supfoot_EPy_value.control_value_once);
            map_ZMP.find("control_total_supfoot_EP_x")->second.push_back(balance.supfoot_EPx_value.control_value_total);
            map_ZMP.find("control_total_supfoot_EP_y")->second.push_back(balance.supfoot_EPy_value.control_value_total);

            double *sensor_force = balance.ZMP_process->getpSensorForce();
            int *origen_sensor_data = balance.ZMP_process->getpOrigenSensorData();

            map_ZMP.find("sensor_force_0")->second.push_back(sensor_force[0]);
            map_ZMP.find("sensor_force_1")->second.push_back(sensor_force[1]);
            map_ZMP.find("sensor_force_2")->second.push_back(sensor_force[2]);
            map_ZMP.find("sensor_force_3")->second.push_back(sensor_force[3]);
            map_ZMP.find("sensor_force_4")->second.push_back(sensor_force[4]);
            map_ZMP.find("sensor_force_5")->second.push_back(sensor_force[5]);
            map_ZMP.find("sensor_force_6")->second.push_back(sensor_force[6]);
            map_ZMP.find("sensor_force_7")->second.push_back(sensor_force[7]);
 
            map_ZMP.find("origen_sensor_data_0")->second.push_back(origen_sensor_data[0]);
            map_ZMP.find("origen_sensor_data_1")->second.push_back(origen_sensor_data[1]);
            map_ZMP.find("origen_sensor_data_2")->second.push_back(origen_sensor_data[2]);
            map_ZMP.find("origen_sensor_data_3")->second.push_back(origen_sensor_data[3]);
            map_ZMP.find("origen_sensor_data_4")->second.push_back(origen_sensor_data[4]);
            map_ZMP.find("origen_sensor_data_5")->second.push_back(origen_sensor_data[5]);
            map_ZMP.find("origen_sensor_data_6")->second.push_back(origen_sensor_data[6]);
            map_ZMP.find("origen_sensor_data_7")->second.push_back(origen_sensor_data[7]);
        }
        balance.supfoot_EPx_value.control_value_total += balance.supfoot_EPx_value.control_value_once/4.0;
        if(balance.supfoot_EPx_value.control_value_total > 0)balance.supfoot_EPx_value.control_value_total = 0;
        this->supfoot_ankle_pitch = -balance.supfoot_EPx_value.control_value_total/180*PI;
        balance.supfoot_EPy_value.control_value_total += balance.supfoot_EPy_value.control_value_once/4.0;
        this->supfoot_ankle_roll = -balance.supfoot_EPy_value.control_value_total/180*PI;
        ankle_balance_count++;



        // if(balance.pres_ZMP.left_pos.x > balance.boundary_ZMP.left_pos.x || balance.pres_ZMP.left_pos.x < balance.boundary_ZMP.left_pos.x)
        // {
        //     balance.PIDsupfoot_EPx.setKpid(parameterinfo->parameters.Kick_Point_X/100, 0, 0);
        // }
        // else if(balance.pres_ZMP.left_pos.x > balance.boundary_ZMP.left_pos.x*2.0/3.0 || balance.pres_ZMP.left_pos.x < balance.boundary_ZMP.left_pos.x*2.0/3.0)
        // {
        //     balance.PIDsupfoot_EPx.setKpid(parameterinfo->parameters.Kick_Point_Y/100, 0, 0);
        // }
        // else if(balance.pres_ZMP.left_pos.x > balance.boundary_ZMP.left_pos.x*1.0/3.0 || balance.pres_ZMP.left_pos.x < balance.boundary_ZMP.left_pos.x*1.0/3.0)
        // {
        //     balance.PIDsupfoot_EPx.setKpid(parameterinfo->parameters.Kick_Point_Z/100, 0, 0);
        // }
        // else
        // {
        //     balance.PIDsupfoot_EPx.setKpid(0, 0, 0);
        // }
        
        
        // balance.PIDsupfoot_EPx.setControlGoal(0);
        // if(ankle_balance_count % 4 == 0)
        // {
        //     ankle_balance_count = 0;
        //     balance.supfoot_EPx_value.control_value_once = balance.PIDsupfoot_EPx.calculateExpValue(control_theta);
        // }
        // balance.supfoot_EPx_value.control_value_total += balance.supfoot_EPx_value.control_value_once/4.0;
        // this->supfoot_ankle_pitch -= balance.supfoot_EPx_value.control_value_total/180.0*PI;
        // map_ZMP.find("control_once_ankle_pitch")->second.push_back(balance.supfoot_EPx_value.control_value_once);
        // map_ZMP.find("control_total_ankle_pitch")->second.push_back(balance.supfoot_EPx_value.control_value_total);
        // map_ZMP.find("control_ankle_pitch_theta")->second.push_back(control_theta);
        
        // if( (balance.pres_ZMP.left_pos.x > balance.ideal_ZMP.left_pos.x + balance.boundary_ZMP.left_pos.x)
        //     || (balance.pres_ZMP.left_pos.x < balance.ideal_ZMP.left_pos.x - balance.boundary_ZMP.left_pos.x)
        //     || (balance.pres_ZMP.left_pos.y > balance.ideal_ZMP.left_pos.y + balance.boundary_ZMP.left_pos.y)
        //     || (balance.pres_ZMP.left_pos.y < balance.ideal_ZMP.left_pos.y - balance.boundary_ZMP.left_pos.y))
       
       
        // if(1)
        // {
        //     // if (smaple_times_count == (int)(smaple_point*T_cnt_sum[1]))
        //     // {
        //     //     force_delay_flag_ = true;
        //     //     force_stop_sample_point_flag_ = true;
        //     // }
        //     if (smaple_times_count == (int)(smaple_point*T_cnt_sum[2]))
        //     {
        //         force_delay_flag_ = true;
        //         force_stop_sample_point_flag_ = true;
        //     }
        // }
        // else
        // {
        //     force_delay_flag_ = false;
        //     force_stop_sample_point_flag_ = false;
        // }
    }
    else if (smaple_times_count <= smaple_point)//DSP
    {
        // float omega = 2*PI/(T*T_cnt[8]);
        // float t = (smaple_point-smaple_times_count)*0.03;
        // float control_roll_temp = balance.supfoot_EPy_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        // float control_pitch_temp = balance.supfoot_EPx_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        // this->supfoot_ankle_pitch = -control_pitch_temp/180.0*PI;
        // this->supfoot_ankle_roll = -control_pitch_temp/180.0*PI;


        // map_ZMP.find("control_once_ankle_pitch")->second.push_back(0);
        // map_ZMP.find("control_total_ankle_pitch")->second.push_back(control_pitch_temp);
        // map_ZMP.find("control_ankle_pitch_theta")->second.push_back(0);
    }
    else
    {
        float omega = 2*PI/(0.6);
        float t = (smaple_point+20-smaple_times_count)*0.03;
        float control_roll_temp = balance.supfoot_EPy_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        float control_pitch_temp = balance.supfoot_EPx_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        this->supfoot_ankle_pitch = -control_pitch_temp/180.0*PI;
        this->supfoot_ankle_roll = -control_pitch_temp/180.0*PI;
        // this->supfoot_ankle_pitch -= 0;
        // map_ZMP.find("control_once_ankle_pitch")->second.push_back(0);
        // map_ZMP.find("control_total_ankle_pitch")->second.push_back(0);
        // map_ZMP.find("control_ankle_pitch_theta")->second.push_back(0);
    }
}

void KickingGait::hipPostureControl()
{
    for(int i = 0; i < 3; i++)balance.prev_imu_value[i].pos = balance.pres_imu_value[i].pos;
    for(int i = 0; i < 3; i++)balance.pres_imu_value[i].pos = sensor.rpy_[i] - balance.init_imu_value[i].pos;
    float T_ms = parameterinfo->parameters.Period_T - 600;
    float T = T_ms/1000.0;
    int smaple_point = T_ms/30;
    int smaple_times_count = parameterinfo->complan.sample_point_;

    //----------- pitch ---------------------
    balance.ideal_imu_value[(int)imu::pitch].vel = getIdealV(balance.pres_imu_value[(int)imu::pitch].pos, ideal_p_arry_pitch, ideal_v_arry);
    balance.pres_imu_value[(int)imu::pitch].vel = (balance.pres_imu_value[(int)imu::pitch].pos-balance.prev_imu_value[(int)imu::pitch].pos)/(0.03);
    balance.passfilter_pres_imu_value[(int)imu::pitch].pos = balance.butterfilter_imu[(int)imu::pitch].pos.getValue(balance.pres_imu_value[(int)imu::pitch].pos);
    balance.passfilter_pres_imu_value[(int)imu::pitch].vel = balance.butterfilter_imu[(int)imu::pitch].vel.getValue(balance.pres_imu_value[(int)imu::pitch].vel);
    balance.passfilter_prev_imu_value[(int)imu::pitch] = balance.passfilter_pres_imu_value[(int)imu::pitch];
    //----------- roll ----------------------
    balance.ideal_imu_value[(int)imu::roll].vel = getIdealV(balance.pres_imu_value[(int)imu::roll].pos, ideal_p_arry_roll, ideal_v_arry);
    balance.pres_imu_value[(int)imu::roll].vel = (balance.pres_imu_value[(int)imu::roll].pos-balance.prev_imu_value[(int)imu::roll].pos)/(0.03);
    balance.passfilter_pres_imu_value[(int)imu::roll].pos = balance.butterfilter_imu[(int)imu::roll].pos.getValue(balance.pres_imu_value[(int)imu::roll].pos);
    balance.passfilter_pres_imu_value[(int)imu::roll].vel = balance.butterfilter_imu[(int)imu::roll].vel.getValue(balance.pres_imu_value[(int)imu::roll].vel);
    balance.passfilter_prev_imu_value[(int)imu::roll] = balance.passfilter_pres_imu_value[(int)imu::roll];

    if(smaple_times_count == 1)
    {
        balance.pres_imu_value[(int)imu::pitch].vel = 0;
        balance.passfilter_pres_imu_value[(int)imu::pitch].vel = 0;
        balance.pres_imu_value[(int)imu::roll].vel = 0;
        balance.passfilter_pres_imu_value[(int)imu::roll].vel = 0;
    }

    if(smaple_times_count < (int)smaple_point*T_cnt_sum[0])//DSP
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[1]))//SSP
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[2]))//SSP
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[3]))//SSP
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[4]))//SSP
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }
    else if(smaple_times_count < (int)(smaple_point*T_cnt_sum[7]))//SSP //5
    { 
        //----------- pitch ----------------------
        balance.PIDsupfoot_hip_pitch.setControlGoal(balance.ideal_imu_value[(int)imu::pitch].vel);
        balance.supfoot_hip_pitch_value.control_value_once = balance.PIDsupfoot_hip_pitch.calculateExpValue(balance.passfilter_pres_imu_value[(int)imu::pitch].vel)*0.03;//dt = 0.03
        balance.supfoot_hip_pitch_value.control_value_total += balance.supfoot_hip_pitch_value.control_value_once;
        if(balance.supfoot_hip_pitch_value.control_value_total < 0)balance.supfoot_hip_pitch_value.control_value_total= 0 ; 
        this->supfoot_hip_pitch -= balance.supfoot_hip_pitch_value.control_value_total/180.0*PI;
        map_pitch.find("control_once_pitch")->second.push_back(balance.supfoot_hip_pitch_value.control_value_once);
        map_pitch.find("control_total_pitch")->second.push_back(balance.supfoot_hip_pitch_value.control_value_total);
        //----------- roll ----------------------
        balance.PIDsupfoot_hip_roll.setControlGoal(balance.ideal_imu_value[(int)imu::roll].vel);
        balance.supfoot_hip_roll_value.control_value_once = balance.PIDsupfoot_hip_roll.calculateExpValue(balance.passfilter_pres_imu_value[(int)imu::roll].vel)*0.03;//dt = 0.03;
        balance.supfoot_hip_roll_value.control_value_total += balance.supfoot_hip_roll_value.control_value_once;
        if(balance.supfoot_hip_roll_value.control_value_total > 0)balance.supfoot_hip_roll_value.control_value_total = 0;//right kick
        this->supfoot_hip_roll += balance.supfoot_hip_roll_value.control_value_total/180.0*PI;
        map_roll.find("control_once_roll")->second.push_back(balance.supfoot_hip_roll_value.control_value_once);
        map_roll.find("control_total_roll")->second.push_back(balance.supfoot_hip_roll_value.control_value_total);
    }
    else if (smaple_times_count <= smaple_point)//DSP
    {
        float omega = 2*PI/(T*T_cnt[8]);
        float t = (smaple_point-smaple_times_count)*0.03;
        float control_roll_temp = balance.supfoot_hip_roll_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        float control_pitch_temp = balance.supfoot_hip_pitch_value.control_value_total*(omega*t-sin(omega*t))/(2*PI);
        this->supfoot_hip_roll += control_roll_temp/180.0*PI;
        this->supfoot_hip_pitch -= control_pitch_temp/180.0*PI;
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(control_roll_temp);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(control_pitch_temp);
    }
    else
    {
        map_roll.find("control_once_roll")->second.push_back(0);
        map_roll.find("control_total_roll")->second.push_back(0);
        map_pitch.find("control_once_pitch")->second.push_back(0);
        map_pitch.find("control_total_pitch")->second.push_back(0);
    }

    map_roll.find("smaple_times_count")->second.push_back(smaple_times_count);
    map_roll.find("pres_roll_pos")->second.push_back(balance.pres_imu_value[(int)imu::roll].pos);
    map_roll.find("passfilter_pres_roll_pos")->second.push_back(balance.passfilter_pres_imu_value[(int)imu::roll].pos);
    map_roll.find("ideal_roll_vel")->second.push_back(balance.ideal_imu_value[(int)imu::roll].vel);
    map_roll.find("pres_roll_vel")->second.push_back(balance.pres_imu_value[(int)imu::roll].vel);
    map_roll.find("passfilter_pres_roll_vel")->second.push_back(balance.passfilter_pres_imu_value[(int)imu::roll].vel);

    map_pitch.find("smaple_times_count")->second.push_back(smaple_times_count);
    map_pitch.find("pres_pitch_pos")->second.push_back(balance.pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("passfilter_pres_pitch_pos")->second.push_back(balance.passfilter_pres_imu_value[(int)imu::pitch].pos);
    map_pitch.find("ideal_pitch_vel")->second.push_back(balance.ideal_imu_value[(int)imu::pitch].vel);
    map_pitch.find("pres_pitch_vel")->second.push_back(balance.pres_imu_value[(int)imu::pitch].vel);
    map_pitch.find("passfilter_pres_pitch_vel")->second.push_back(balance.passfilter_pres_imu_value[(int)imu::pitch].vel);

    map_kickgait.find("supfoot_hip_pitch")->second.push_back(supfoot_hip_pitch);
    map_kickgait.find("supfoot_hip_roll")->second.push_back(supfoot_hip_roll);
    map_kickgait.find("kick_foot_ankle_pitch")->second.push_back(kick_foot_ankle_pitch);
}

Point3DParam KickingGait::endPointBalanceControl()
{ 
    Point3DParam R_P; 
    Point3DParam L_P;
    if(ankle_balance_count % 4 == 0)
    {
        ankle_balance_count = 0;
        int origen_sensor_data_temp[8];
        for(int i = 0; i < 4; i++)origen_sensor_data_temp[i] = sensor.press_left_[i];
        for(int i = 4; i < 8; i++)origen_sensor_data_temp[i] = sensor.press_right_[i-4];
        balance.prev_ZMP = balance.pres_ZMP;
        balance.ZMP_process->setpOrigenSensorData(origen_sensor_data_temp);
        balance.pres_ZMP = balance.ZMP_process->getZMPValue();
        if(parameterinfo->walking_mode == 9)
        {
            balance.pres_ZMP.left_vel.x = (balance.pres_ZMP.left_pos.x - balance.prev_ZMP.left_pos.x)/0.12;
            balance.pres_ZMP.left_vel.y = (balance.pres_ZMP.left_pos.y - balance.prev_ZMP.left_pos.y)/0.12;
            balance.pres_ZMP.left_acc.x = (balance.pres_ZMP.left_vel.x - balance.prev_ZMP.left_vel.x)/0.12;
            balance.pres_ZMP.left_acc.y = (balance.pres_ZMP.left_vel.y - balance.prev_ZMP.left_vel.y)/0.12;
            balance.ideal_ZMP.left_vel.x = getIdealV(balance.pres_ZMP.left_pos.x, ideal_ZMP_px_arry, ideal_ZMP_vx_arry);
            balance.ideal_ZMP.left_vel.y = getIdealV(balance.pres_ZMP.left_pos.y, ideal_ZMP_py_arry, ideal_ZMP_vy_arry);
            
            // balance.ideal_ZMP.left_acc.x = getIdealV(balance.pres_ZMP.left_pos.x, ideal_origen_ZMP_acc_px_arry, ideal_origen_ZMP_acc_x_arry);
            

            balance.passfilter_pres_ZMP.left_pos.x = balance.butterfilter_ZMPsupfoot.pos_x.getValue(balance.pres_ZMP.left_pos.x);
            balance.passfilter_pres_ZMP.left_pos.y = balance.butterfilter_ZMPsupfoot.pos_y.getValue(balance.pres_ZMP.left_pos.y);
            balance.passfilter_pres_ZMP.left_vel.x = balance.butterfilter_ZMPsupfoot.vel_x.getValue(balance.pres_ZMP.left_vel.x);
            balance.passfilter_pres_ZMP.left_vel.y = balance.butterfilter_ZMPsupfoot.vel_y.getValue(balance.pres_ZMP.left_vel.y);
            balance.passfilter_pres_ZMP.left_acc.x = balance.butterfilter_ZMPsupfoot.acc_x.getValue(balance.pres_ZMP.left_acc.x);
            balance.passfilter_pres_ZMP.left_acc.y = balance.butterfilter_ZMPsupfoot.acc_y.getValue(balance.pres_ZMP.left_acc.y);

            
            balance.PIDsupfoot_EPx.setControlGoal(balance.ideal_ZMP.left_vel.x);
            balance.supfoot_EPx_value.control_value_once = balance.PIDsupfoot_EPx.calculateExpValue(balance.passfilter_pres_ZMP.left_vel.x)*0.12;
            balance.PIDsupfoot_EPy.setControlGoal(balance.ideal_ZMP.left_vel.y);
            balance.supfoot_EPy_value.control_value_once = balance.PIDsupfoot_EPy.calculateExpValue(balance.passfilter_pres_ZMP.left_vel.y)*0.12;
            
            // if(balance.pres_ZMP.left_pos.x < 1.6 && balance.pres_ZMP.left_pos.x > -1.6)
            // {
            //     if(balance.passfilter_pres_ZMP.left_vel.x > 2 && balance.supfoot_EPx_value.control_value_once < 0)
            //     {
            //         balance.supfoot_EPx_value.control_value_once *= parameterinfo->parameters.Kick_Point_Z/10.0; 
            //     }
            //     else if(balance.passfilter_pres_ZMP.left_vel.x < -2 && balance.supfoot_EPx_value.control_value_once > 0)
            //     {
            //         balance.supfoot_EPx_value.control_value_once *= parameterinfo->parameters.Kick_Point_Z/10.0;
            //     }
            // }
            // if(balance.pres_ZMP.left_pos.x > -1.6 && balance.passfilter_pres_ZMP.left_acc.x > 0)
            //     balance.supfoot_EPx_value.control_value_once *= parameterinfo->parameters.Kick_Point_Z/10.0;
            // if(balance.pres_ZMP.left_pos.x < 1.6 && balance.passfilter_pres_ZMP.left_acc.x < 0)
            //     balance.supfoot_EPx_value.control_value_once *= parameterinfo->parameters.Kick_Point_Z/10.0;

            map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(balance.pres_ZMP.left_pos.x);
            map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(balance.pres_ZMP.left_pos.y);
            map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(balance.pres_ZMP.right_pos.x);
            map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(balance.pres_ZMP.right_pos.y);
            map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(balance.pres_ZMP.feet_pos.x);
            map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(balance.pres_ZMP.feet_pos.y);

            map_ZMP.find("pres_ZMPsupfoot_pos_x")->second.push_back(balance.pres_ZMP.left_pos.x);
            map_ZMP.find("pres_ZMPsupfoot_pos_y")->second.push_back(balance.pres_ZMP.left_pos.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_x")->second.push_back(balance.passfilter_pres_ZMP.left_pos.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_y")->second.push_back(balance.passfilter_pres_ZMP.left_pos.y);
            
            map_ZMP.find("pres_ZMPsupfoot_vel_x")->second.push_back(balance.pres_ZMP.left_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_vel_y")->second.push_back(balance.pres_ZMP.left_vel.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_x")->second.push_back(balance.passfilter_pres_ZMP.left_vel.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_y")->second.push_back(balance.passfilter_pres_ZMP.left_vel.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_acc_x")->second.push_back(balance.passfilter_pres_ZMP.left_acc.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_acc_y")->second.push_back(balance.passfilter_pres_ZMP.left_acc.y);
            
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_x")->second.push_back(balance.ideal_ZMP.left_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_y")->second.push_back(balance.ideal_ZMP.left_vel.y);
            map_ZMP.find("pres_ZMPsupfoot_ideal_acc_x")->second.push_back(balance.ideal_ZMP.left_acc.x);
            map_ZMP.find("pres_ZMPsupfoot_ideal_acc_y")->second.push_back(balance.ideal_ZMP.left_acc.y);

            // map_ZMP.find("delta_v")->second.push_back(test_PPP);

            map_ZMP.find("ideal_ZMPsupfoot_pos_x")->second.push_back(balance.ideal_ZMP.left_pos.x);
            map_ZMP.find("ideal_ZMPsupfoot_pos_y")->second.push_back(balance.ideal_ZMP.left_pos.y);
            
            map_ZMP.find("boundary_ZMPsupfoot_pos_x")->second.push_back(balance.boundary_ZMP.left_pos.x);
            map_ZMP.find("boundary_ZMPsupfoot_pos_y")->second.push_back(balance.boundary_ZMP.left_pos.y);
            
            map_ZMP.find("control_once_supfoot_EP_x")->second.push_back(balance.supfoot_EPx_value.control_value_once);
            map_ZMP.find("control_once_supfoot_EP_y")->second.push_back(balance.supfoot_EPy_value.control_value_once);
        }
        else if(parameterinfo->walking_mode == 10)
        {
            balance.pres_ZMP.right_vel.x = (balance.pres_ZMP.right_pos.x - balance.prev_ZMP.right_pos.x)/0.12;
            balance.pres_ZMP.right_vel.y = (balance.pres_ZMP.right_pos.y - balance.prev_ZMP.right_pos.y)/0.12;
            balance.ideal_ZMP.right_vel.x = getIdealV(balance.pres_ZMP.right_pos.x, ideal_ZMP_px_arry, ideal_ZMP_vx_arry);
            balance.ideal_ZMP.right_vel.y = getIdealV(balance.pres_ZMP.right_pos.y, ideal_ZMP_py_arry, ideal_ZMP_vy_arry);

            balance.passfilter_pres_ZMP.right_pos.x = balance.butterfilter_ZMPsupfoot.pos_x.getValue(balance.pres_ZMP.right_pos.x);
            balance.passfilter_pres_ZMP.right_pos.y = balance.butterfilter_ZMPsupfoot.pos_y.getValue(balance.pres_ZMP.right_pos.y);
            balance.passfilter_pres_ZMP.right_vel.x = balance.butterfilter_ZMPsupfoot.vel_x.getValue(balance.pres_ZMP.right_vel.x);
            balance.passfilter_pres_ZMP.right_vel.y = balance.butterfilter_ZMPsupfoot.vel_y.getValue(balance.pres_ZMP.right_vel.y);
            
            balance.PIDsupfoot_EPx.setControlGoal(balance.ideal_ZMP.right_vel.x);
            balance.supfoot_EPx_value.control_value_once = balance.PIDsupfoot_EPx.calculateExpValue(balance.passfilter_pres_ZMP.right_vel.x)*0.12;
            balance.PIDsupfoot_EPy.setControlGoal(balance.ideal_ZMP.right_vel.y);
            balance.supfoot_EPy_value.control_value_once = balance.PIDsupfoot_EPy.calculateExpValue(balance.passfilter_pres_ZMP.right_vel.y)*0.12;
            map_ZMP.find("pres_ZMP_left_pos_x")->second.push_back(balance.pres_ZMP.left_pos.x);
            map_ZMP.find("pres_ZMP_left_pos_y")->second.push_back(balance.pres_ZMP.left_pos.y);
            map_ZMP.find("pres_ZMP_right_pos_x")->second.push_back(balance.pres_ZMP.right_pos.x);
            map_ZMP.find("pres_ZMP_right_pos_y")->second.push_back(balance.pres_ZMP.right_pos.y);
            map_ZMP.find("pres_ZMP_feet_pos_x")->second.push_back(balance.pres_ZMP.feet_pos.x);
            map_ZMP.find("pres_ZMP_feet_pos_y")->second.push_back(balance.pres_ZMP.feet_pos.y);

            map_ZMP.find("pres_ZMPsupfoot_pos_x")->second.push_back(balance.pres_ZMP.right_pos.x);
            map_ZMP.find("pres_ZMPsupfoot_pos_y")->second.push_back(balance.pres_ZMP.right_pos.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_x")->second.push_back(balance.passfilter_pres_ZMP.right_pos.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_pos_y")->second.push_back(balance.passfilter_pres_ZMP.right_pos.y);
            
            map_ZMP.find("pres_ZMPsupfoot_vel_x")->second.push_back(balance.pres_ZMP.right_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_vel_y")->second.push_back(balance.pres_ZMP.right_vel.y);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_x")->second.push_back(balance.passfilter_pres_ZMP.right_vel.x);
            map_ZMP.find("passfilter_pres_ZMPsupfoot_vel_y")->second.push_back(balance.passfilter_pres_ZMP.right_vel.y);
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_x")->second.push_back(balance.ideal_ZMP.right_vel.x);
            map_ZMP.find("pres_ZMPsupfoot_ideal_vel_y")->second.push_back(balance.ideal_ZMP.right_vel.y);

            map_ZMP.find("ideal_ZMPsupfoot_pos_x")->second.push_back(balance.ideal_ZMP.right_pos.x);
            map_ZMP.find("ideal_ZMPsupfoot_pos_y")->second.push_back(balance.ideal_ZMP.right_pos.y);
            
            map_ZMP.find("boundary_ZMPsupfoot_pos_x")->second.push_back(balance.boundary_ZMP.right_pos.x);
            map_ZMP.find("boundary_ZMPsupfoot_pos_y")->second.push_back(balance.boundary_ZMP.right_pos.y);
            
            map_ZMP.find("control_once_supfoot_EP_x")->second.push_back(balance.supfoot_EPx_value.control_value_once);
            map_ZMP.find("control_once_supfoot_EP_y")->second.push_back(balance.supfoot_EPy_value.control_value_once);
        }
        double *sensor_force = balance.ZMP_process->getpSensorForce();
        int *origen_sensor_data = balance.ZMP_process->getpOrigenSensorData();

        map_ZMP.find("sensor_force_0")->second.push_back(sensor_force[0]);
        map_ZMP.find("sensor_force_1")->second.push_back(sensor_force[1]);
        map_ZMP.find("sensor_force_2")->second.push_back(sensor_force[2]);
        map_ZMP.find("sensor_force_3")->second.push_back(sensor_force[3]);
        map_ZMP.find("sensor_force_4")->second.push_back(sensor_force[4]);
        map_ZMP.find("sensor_force_5")->second.push_back(sensor_force[5]);
        map_ZMP.find("sensor_force_6")->second.push_back(sensor_force[6]);
        map_ZMP.find("sensor_force_7")->second.push_back(sensor_force[7]);

        map_ZMP.find("origen_sensor_data_0")->second.push_back(origen_sensor_data[0]);
        map_ZMP.find("origen_sensor_data_1")->second.push_back(origen_sensor_data[1]);
        map_ZMP.find("origen_sensor_data_2")->second.push_back(origen_sensor_data[2]);
        map_ZMP.find("origen_sensor_data_3")->second.push_back(origen_sensor_data[3]);
        map_ZMP.find("origen_sensor_data_4")->second.push_back(origen_sensor_data[4]);
        map_ZMP.find("origen_sensor_data_5")->second.push_back(origen_sensor_data[5]);
        map_ZMP.find("origen_sensor_data_6")->second.push_back(origen_sensor_data[6]);
        map_ZMP.find("origen_sensor_data_7")->second.push_back(origen_sensor_data[7]);

    }
    // if(ankle_balance_count == 0)
        // balance.supfoot_EPx_value.control_value_total += balance.supfoot_EPx_value.control_value_once;
    
    // if(ankle_balance_count == 0)
        // balance.supfoot_EPy_value.control_value_total += balance.supfoot_EPy_value.control_value_once;
     
    balance.supfoot_EPx_value.control_value_total += balance.supfoot_EPx_value.control_value_once/4.0;
    balance.supfoot_EPy_value.control_value_total += balance.supfoot_EPy_value.control_value_once/4.0;

    L_P.set(balance.supfoot_EPx_value.control_value_total, balance.supfoot_EPy_value.control_value_total, 0);
    ankle_balance_count++;
    return L_P;
}
 
void KickingGait::SaveData()
{
    // --------------------roll----------------------
    char path[200] = "/data";
    std::string aa = std::to_string(name_cont);
    aa = "/KickingGait_Record_Roll"+aa+".csv";
    strcat(path, aa.c_str());

    std::fstream fp;
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
    // --------------------Pitch----------------------
    char path1[200] = "/data";
    aa = std::to_string(name_cont);
    aa = "/KickingGait_Record_Pitch"+aa+".csv";
    strcat(path1, aa.c_str());
    fp.open(path1, std::ios::out);
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
    // --------------------ZMP----------------------
    char path4[200] = "/data";
    aa = std::to_string(name_cont);
    aa = "/KickingGait_Record_ZMP"+aa+".csv";
    strcat(path4, aa.c_str());
    fp.open(path4, std::ios::out);
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
    // --------------------kickgait----------------------
    char path3[200] = "/data";
    aa = std::to_string(name_cont);
    aa = "/KickingGait_Record_Kickgait"+aa+".csv";
    strcat(path3, aa.c_str());
    fp.open(path3, std::ios::out);
    savedText = "";

    std::map<std::string, std::vector<float>>::iterator it_kickgait;
    for(it_kickgait = map_kickgait.begin(); it_kickgait != map_kickgait.end(); it_kickgait++)
    {
        savedText += it_kickgait->first;
        if(it_kickgait == --map_kickgait.end())
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
    it_kickgait = map_kickgait.begin();
    max_size = it_kickgait->second.size();

    for(it_kickgait = map_kickgait.begin(); it_kickgait != map_kickgait.end(); it_kickgait++)
    {
        if(max_size < it_kickgait->second.size())
            max_size = it_kickgait->second.size();
    } 
    for(int i = 0; i < max_size; i++)
    {
        for(it_kickgait = map_kickgait.begin(); it_kickgait != map_kickgait.end(); it_kickgait++)
        {
            if(i < it_kickgait->second.size())
            {
                if(it_kickgait == --map_kickgait.end())
                {
                    savedText += std::to_string(it_kickgait->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_kickgait->second[i]) + ",";
                }
            }
            else
            {
                if(it_kickgait == --map_kickgait.end())
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
    for(it_kickgait = map_kickgait.begin(); it_kickgait != map_kickgait.end(); it_kickgait++)
        it_kickgait->second.clear();
    // --------------------Param----------------------
    char path2[200] = "/data";
    aa = std::to_string(name_cont);
    aa = "/KickingGait_Record_Param"+aa+".csv";
    strcat(path2, aa.c_str());
    fp.open(path2, std::ios::out);
    savedText = "";

    std::map<std::string, float>::iterator it_param;
    for(it_param = map_param.begin(); it_param != map_param.end(); it_param++)
    {
        savedText += it_param->first;
        if(it_param == --map_param.end())
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
    for(it_param = map_param.begin(); it_param != map_param.end(); it_param++)
    {
        if(it_param != --map_param.end())
        {
            savedText += std::to_string(it_param->second)+",";
        }
        else
        {
            savedText += std::to_string(it_param->second)+"\n";
            fp<<savedText;
            savedText = "";
        }
    }
    fp.close();
    name_cont++;
}

Kick_PID_Controller::Kick_PID_Controller(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kp = Kp;
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

Kick_PID_Controller::Kick_PID_Controller()
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kp = 0;
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

Kick_PID_Controller::~Kick_PID_Controller()
{
    
}

void Kick_PID_Controller::initParam()
{
    this->pre_error = 0;
    this->error = 0;
    this->errors = 0;
    this->errord = 0;
    this->exp_value = 0;
    this->value = 0;
    this->pre_value = 0;
}

void Kick_PID_Controller::setKpid(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void Kick_PID_Controller::setControlGoal(float x1c)
{
    this->x1c = x1c;
    this->x2c = x2c;
    this->x3c = x3c;
}

float Kick_PID_Controller::calculateExpValue(float value)
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
        this->errord = (this->error - this->pre_error)/0.03;//因該直接=IMU Gyro value 才對
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

void Kick_PID_Controller::setValueLimit(float upper_limit, float lower_limit)
{
    this->upper_limit = upper_limit;
    this->lower_limit = lower_limit;
}

float Kick_PID_Controller::getError()
{
    return this->error;
}

float Kick_PID_Controller::getErrors()
{
    return this->errors;
}

float Kick_PID_Controller::getErrord()
{
    return this->errord;
}

ButterWirthFilter::ButterWirthFilter()
{

}

ButterWirthFilter::~ButterWirthFilter()
{
 
}

void ButterWirthFilter::initialize()
{
    param_.initailize();
    prev_output_ = 0;
    prev_value_ = 0;
}

void ButterWirthFilter::setButterWirthParam(float a1, float a2, float b1, float b2)
{
    param_.set(a1,a2,b1,b2);
}

float ButterWirthFilter::getValue(float present_value)
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

kickgait_space::BalanceControl::BalanceControl()
{
    ZMP_process = new ZMPProcess;
    initialize();
}
  
kickgait_space::BalanceControl::~BalanceControl()
{
    delete ZMP_process;
}

void kickgait_space::BalanceControl::initialize()
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

    pres_ZMP.initialize();
    prev_ZMP.initialize();
    ideal_ZMP.initialize();
    boundary_ZMP.initialize();
    passfilter_pres_ZMP.initialize();

    supfoot_hip_roll_value.initialize();
    supfoot_hip_pitch_value.initialize();
    supfoot_EPx_value.initialize();
    supfoot_EPy_value.initialize();

    PIDsupfoot_hip_roll.initParam();
    PIDsupfoot_hip_pitch.initParam();
    PIDsupfoot_EPx.initParam();
    PIDsupfoot_EPy.initParam();

    for(int i = 0; i < sizeof(butterfilter_imu)/sizeof(butterfilter_imu[0]); i++)
        butterfilter_imu[i].initialize();
    butterfilter_ZMPsupfoot.initialize();

    ZMP_process->initialize();

    pres_ankle_roll.initialize();
    prev_ankle_roll.initialize();
    pres_ankle_pitch.initialize();
    prev_ankle_pitch.initialize();
    ideal_ankle_pitch.initialize();
    ideal_ankle_roll.initialize();
    supfoot_ankle_roll_value.initialize();
    supfoot_ankle_pitch_value.initialize();
}

void ButterWirthParam::initailize()
{
    for(int i = 0; i < sizeof(a_)/sizeof(a_[0]); i++)a_[i] = 0;
    for(int i = 0; i < sizeof(b_)/sizeof(b_[0]); i++)b_[i] = 0;
}

void ButterWirthParam::set(float a1, float a2, float b1, float b2)
{
    a_[0] = a1;
    a_[1] = a2;
    b_[0] = b1;
    b_[1] = b2;
}

void Kick_BalanceParam::initialize()
{
    control_value_total = 0;
    control_value_once = 0;
}

void ButterWirthIMUParam::initialize()
{
    pos.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    vel.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
}

void ButterWirthZMPParam::initialize()
{
    pos_x.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    pos_y.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    vel_x.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    vel_y.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    acc_x.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    acc_y.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
}

void ButterWirthForceParam::initialize()
{
    zero.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    one.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    two.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
    three.setButterWirthParam(1, -0.676819, 0.161590, 0.161590); //fs = 33 , fc = 2, n = 1;
}

void Kick_IMUParam::initialize()
{
    pos = 0;
    vel = 0;
}