#ifndef MBK_CONTROL_H_
#define MBK_CONTROL_H_

class MBK_Parameter
{
public:
    MBK_Parameter();
    ~MBK_Parameter();
    float M,B,K;
    float Position,Velocity,Acceleration;
    float Pd,Vd;
    bool Limit_Enable;
    float Pmax,Pmin;
};

class MBK_Controller
{
public:
    MBK_Controller();
    ~MBK_Controller();
    void Control(MBK_Parameter* Controller, float Force, float dt);
    void Ini(MBK_Parameter* Controller, float M, float B, float K, float Pd = 0, float Vd = 0);
    void Limit(MBK_Parameter* Controller, float Pmax, float Pmin);
};


#endif /*MBK_CONTROL_H_*/
