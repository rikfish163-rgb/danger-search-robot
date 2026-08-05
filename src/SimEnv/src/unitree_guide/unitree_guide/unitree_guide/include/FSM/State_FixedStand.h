/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos[12] = {0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
                            0.0, 0.9, -1.8, 0.0, 0.9, -1.8};
    float _startPos[12];
    float _startPos_real[12];
    float real_stand_p[12] = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80};
    float real_stand_d[12]= {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    float _duration = 3.0;   // seconds
    float _settleDuration = 0.5;
    float _elapsed = 0;
    float _percent = 0;
    void setRampSimStanceGain(float percent);
};

#endif  // FIXEDSTAND_H
