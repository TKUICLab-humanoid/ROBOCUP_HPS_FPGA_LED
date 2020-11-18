#ifndef DEFINE_DATA_STRUCT_H_
#define DEFINE_DATA_STRUCT_H_
typedef struct Point2DParameter Point2DParam;
struct Point2DParameter
{
    float x;
    float y;
    void initialize();
    Point2DParam set(float x, float y);
};

typedef struct ZMPParameter ZMPParam;
struct ZMPParameter
{
    Point2DParam left_pos;
    Point2DParam right_pos;
    Point2DParam feet_pos;
    Point2DParam left_vel;
    Point2DParam right_vel;
    Point2DParam feet_vel;
    Point2DParam left_acc;
    Point2DParam right_acc;
    Point2DParam feet_acc;
    void initialize();
};
#endif