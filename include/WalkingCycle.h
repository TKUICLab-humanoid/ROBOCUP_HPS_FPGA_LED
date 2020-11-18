#ifndef WALKINGCYCLE_H
#define WALKINGCYCLE_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <math.h>
#include "Parameter_Info.h"


#define INCREASE_SLOPE 1
#define WALK_MAX_DISTANCE 2

class WalkingCycle
{ 
public:
    WalkingCycle();
    ~WalkingCycle();

    void walkingkindfunction(int walking_mode);
    void singlestepfunction(int walking_mode);
    void singlestepwalkingprocess();
    void continuoustepfunction();
    void continuouswalkingprocess();
    void LCwalkingprocess();
    void wosccontinuouswalkingprocess();

    double forwardValue_;
    double forwardCounter_;
    int slopeCounter_;
    int Sample_points_quater;

    double COM_Y_tmp;
    double Lockrange_tmp;

    int teststop;
    int ContMode;
    bool IsStop;

private:

};

    


#endif // WALKINGCYCLE_H
