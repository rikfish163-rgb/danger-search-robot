/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include "FSM/State_FixedStand.h"

namespace {
float readStandDuration()
{
    const float defaultDuration = 3.0f;
    const char *envValue = std::getenv("UNITREE_STAND_DURATION");
    if(envValue == nullptr || envValue[0] == '\0'){
        return defaultDuration;
    }

    char *end = nullptr;
    errno = 0;
    float duration = std::strtof(envValue, &end);
    if(errno != 0 || end == envValue || *end != '\0' || !std::isfinite(duration) || duration < 0.5f || duration > 8.0f){
        std::cout << "[WARNING] Invalid UNITREE_STAND_DURATION='" << envValue
                  << "', using default " << defaultDuration << "s." << std::endl;
        return defaultDuration;
    }
    return duration;
}

float readStandSettleDuration()
{
    const float defaultDuration = 0.5f;
    const char *envValue = std::getenv("UNITREE_STAND_SETTLE_DURATION");
    if(envValue == nullptr || envValue[0] == '\0'){
        return defaultDuration;
    }

    char *end = nullptr;
    errno = 0;
    float duration = std::strtof(envValue, &end);
    if(errno != 0 || end == envValue || *end != '\0' || !std::isfinite(duration) || duration < 0.0f || duration > 3.0f){
        std::cout << "[WARNING] Invalid UNITREE_STAND_SETTLE_DURATION='" << envValue
                  << "', using default " << defaultDuration << "s." << std::endl;
        return defaultDuration;
    }
    return duration;
}

float smoothStep(float value)
{
    if(value <= 0.0f){
        return 0.0f;
    }
    if(value >= 1.0f){
        return 1.0f;
    }
    return value * value * (3.0f - 2.0f * value);
}

float lerp(float start, float end, float percent)
{
    return start + (end - start) * percent;
}
}

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
    _duration = readStandDuration();
    _settleDuration = readStandSettleDuration();
    _elapsed = 0.0f;
    _percent = 0.0f;
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            setRampSimStanceGain(0.0f);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
        _startPos_real[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
    }
    _ctrlComp->setAllStance();
}

void State_FixedStand::run(){
    _elapsed += static_cast<float>(_ctrlComp->dt);
    if(_elapsed < _settleDuration){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            setRampSimStanceGain(0.0f);
        }
        for(int j=0; j<12; j++){
            _lowCmd->motorCmd[j].q = _startPos[j];
        }
        return;
    }

    _percent = (_elapsed - _settleDuration) / _duration;
    _percent = _percent > 1 ? 1 : _percent;
    const float smoothPercent = smoothStep(_percent);
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        setRampSimStanceGain(smoothPercent);
    }
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - smoothPercent)*_startPos[j] + smoothPercent*_targetPos[j];
    }

    if (real == true){
        for(int j=0; j<12; j++){
            std::vector<double> joint{(1 - smoothPercent)*_startPos_real[j] + \
                smoothPercent*_targetPos[j], 0, 0, real_stand_p[j], real_stand_d[j]};
            _ctrlComp->ioInterFreeDog->setCmd(j,joint);
        }
    }
}

void State_FixedStand::setRampSimStanceGain(float percent){
    const float gainPercent = smoothStep(percent);
    for(int legID=0; legID<4; legID++){
        const int hip = legID * 3;
        const int thigh = hip + 1;
        const int calf = hip + 2;

        _lowCmd->motorCmd[hip].mode = 10;
        _lowCmd->motorCmd[hip].Kp = lerp(15.0f, 95.0f, gainPercent);
        _lowCmd->motorCmd[hip].Kd = lerp(1.5f, 5.0f, gainPercent);

        _lowCmd->motorCmd[thigh].mode = 10;
        _lowCmd->motorCmd[thigh].Kp = lerp(15.0f, 95.0f, gainPercent);
        _lowCmd->motorCmd[thigh].Kd = lerp(1.5f, 5.0f, gainPercent);

        _lowCmd->motorCmd[calf].mode = 10;
        _lowCmd->motorCmd[calf].Kp = lerp(25.0f, 140.0f, gainPercent);
        _lowCmd->motorCmd[calf].Kd = lerp(2.0f, 7.0f, gainPercent);
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::RL ||
            _lowState->userCmd == UserCommand::RL_KEYBOARD){
        return FSMStateName::RL;
    }
    else{
        return FSMStateName::FIXEDSTAND;
    }
}
