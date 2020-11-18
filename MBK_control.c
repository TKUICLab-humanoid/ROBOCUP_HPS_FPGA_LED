#include "include/MBK_control.h"

MBK_Parameter::MBK_Parameter()
{
    M = 0;
    B = 0;
    K = 0;
    Position = 0;
    Velocity = 0;
    Acceleration = 0;
    Pd = 0;
    Vd = 0;
    Limit_Enable = false;
    Pmax = 0;
    Pmin = 0;
}

MBK_Parameter::~MBK_Parameter()
{

}

MBK_Controller::MBK_Controller()
{

}

MBK_Controller::~MBK_Controller()
{

}

void MBK_Controller::Control(MBK_Parameter* Controller, float Force, float dt){
    
    Controller->Acceleration = ( Force + 
                                 Controller->B*(Controller->Vd - Controller->Velocity) + 
                                 Controller->K*(Controller->Pd - Controller->Position)
                               )/Controller->M;
    Controller->Velocity += Controller->Acceleration * dt;
    Controller->Position += Controller->Velocity * dt;
    
    if(Controller->Limit_Enable){
        if      (Controller->Position > Controller->Pmax)    Controller->Position = Controller->Pmax;
        else if (Controller->Position < Controller->Pmin)    Controller->Position = Controller->Pmin;
    }
}
void MBK_Controller::Ini(MBK_Parameter* Controller, float M, float B, float K, float Pd, float Vd){
    Controller->M = M;
    Controller->B = B;
    Controller->K = K;
    Controller->Position = 0;
    Controller->Velocity = 0;
    Controller->Acceleration = 0;
    Controller->Pd = Pd;
    Controller->Vd = Vd;
    Controller->Limit_Enable = 0;
}
void MBK_Controller::Limit(MBK_Parameter* Controller, float Pmax, float Pmin){
    Controller->Limit_Enable = 1;
    Controller->Pmax = Pmax;
    Controller->Pmin = Pmin;
}


