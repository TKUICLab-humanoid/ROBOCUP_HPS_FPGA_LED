#include "include/B_Spline.h"
const int B_Spline_Param::g_k = 2;
void Point3DParam::initialize()
{
    x = 0;
    y = 0;
    z = 0;
}

Point3DParam Point3DParam::set(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    return *this;
}

B_Spline_Param B_Spline_Param::generate(std::vector<Point3DParam> &vP, std::vector<float> vknot_vector, int k)
{
    this->vP = vP;
    this->vknot_vector = vknot_vector;
    this->n = this->vP.size()-1;
    this->k = k;
    return *this;
}

void B_Spline_Param::updateControlPoint(std::vector<Point3DParam> &vP)
{
    this->vP = vP;
}

B_Spline::B_Spline()
{

}

B_Spline::~B_Spline()
{

}

std::vector<float> B_Spline::generateClampedKnotVector(int n, int k)
{
    std::vector<float> vknot_vector;
    for(int i = 0; i < (k+1); i++)vknot_vector.push_back(0);
    float j = 1.0/(n-k+1);
    for(int i = 1; i < (n-k+1); i++)vknot_vector.push_back(i*j);
    for(int i = 0; i < (k+1); i++)vknot_vector.push_back(1);
    return vknot_vector;
}

Point3DParam B_Spline::C(float u, B_Spline_Param &param)
{
    if(param.vknot_vector.size() != param.n+param.k+2)
    {
        std::printf("error in B_Spline::C(), vknot num=%d, n+k+2=%d\n", param.vknot_vector.size(), param.n+param.k+2);
    }
    Point3DParam sum;
    for(int i = 0; i < param.vP.size(); i++)
    {
        sum.x += param.vP[i].x*N(i, param.k, u, param.vknot_vector);
        sum.y += param.vP[i].y*N(i, param.k, u, param.vknot_vector);
        sum.z += param.vP[i].z*N(i, param.k, u, param.vknot_vector);
    }
    return sum;
}

float B_Spline::N(int i, int k, float u, std::vector<float> &vknot_vector)
{
    std::vector<float> &knot = vknot_vector;
    if(k == 0)
    {
        if(u == 1)
        {
            if(u >= knot[i] && u <= knot[i+1])
                return 1;
            else
                return 0;
        }
        else
        {
            if(u >= knot[i] && u < knot[i+1])
                return 1;
            else
                return 0;
        }
    }
    float A = 0, B = 0, C = 0, D = 0;
    if(knot[i+k]-knot[i] == 0)
    {
        A = 0;
    }
    else
    {
        A = (float)(u-knot[i])/(knot[i+k]-knot[i]);
    }
    if(knot[i+1+k]-knot[i+1] == 0)
    {
        B = 0;
    }
    else
    {
        B = (float)(knot[i+1+k]-u)/(knot[i+1+k]-knot[i+1]);
    }
    C = this->N(i, k-1, u, vknot_vector);
    D = this->N(i+1, k-1, u, vknot_vector);
    return A*C + B*D;
}