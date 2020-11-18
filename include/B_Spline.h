#ifndef B_Spline_H_
#define B_Spline_H_

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
// #include <sys/time.h>

// #include "Initial.h"

typedef struct Point3DParameter Point3DParam;
struct Point3DParameter
{
public:
    void initialize();
    Point3DParam set(float x, float y, float z);
    float x;
    float y;
    float z;
};

typedef struct B_Spline_Parameter B_Spline_Param;
struct B_Spline_Parameter
{
    void updateControlPoint(std::vector<Point3DParam> &vP);
    B_Spline_Param generate(std::vector<Point3DParam> &vP, std::vector<float> vknot_vector, int k = B_Spline_Param::g_k);
    static const int g_k;
    std::vector<float> vknot_vector;
    std::vector<Point3DParam> vP;
    int n;
    int k;
};

class B_Spline
{
public:
    B_Spline();
    ~B_Spline();
    Point3DParam C(float u, B_Spline_Param &param);
    std::vector<float> generateClampedKnotVector(int n, int k);
private:
    float N(int i, int k, float u, std::vector<float> &vknot_vector);
};

#endif /* B_Spline_H_ */