#include"include/DefineDataStruct.h"
void Point2DParam::initialize()
{
    x = 0;
    y = 0;
}

Point2DParam Point2DParam::set(float x, float y)
{
    this->x = x;
    this->y = y;
    return *this;
}

void ZMPParam::initialize()
{
    left_pos.initialize();
    right_pos.initialize();
    feet_pos.initialize();
    left_vel.initialize();
    right_vel.initialize();
    feet_vel.initialize();
    left_acc.initialize();
    right_acc.initialize();
    feet_acc.initialize();

}