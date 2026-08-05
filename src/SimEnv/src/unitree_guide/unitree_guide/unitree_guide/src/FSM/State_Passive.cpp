/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace {
bool simPassiveHoldEnabled()
{
    const char *envValue = std::getenv("UNITREE_SIM_PASSIVE_HOLD");
    if(envValue == nullptr || envValue[0] == '\0'){
        return true;
    }

    std::string value(envValue);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return !(value == "0" || value == "false" || value == "no" || value == "off");
}

const float downPos[12] = {-0.35f, 1.36f, -2.65f, 0.35f, 1.36f, -2.65f,
                           -0.50f, 1.36f, -2.65f, 0.50f, 1.36f, -2.65f};
}

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        _simHoldEnabled = simPassiveHoldEnabled();
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = downPos[i];
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = _simHoldEnabled ? 12.0f : 0.0f;
            _lowCmd->motorCmd[i].Kd = _simHoldEnabled ? 2.0f : 0.5f;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }
    else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 3;
            _lowCmd->motorCmd[i].tau = 0;
        }
    }

    if (real == true)
    {
        for(int j=0; j<12; j++){
            std::vector<double> joint{0, 0, 0, 0, 2};
            _ctrlComp->ioInterFreeDog->setCmd(j,joint);
        }
        _ctrlComp->ioInterFreeDog->sendCmd();
    }

    _ctrlComp->setAllSwing();
}

void State_Passive::run(){
    if(_ctrlComp->ctrlPlatform != CtrlPlatform::GAZEBO || !_simHoldEnabled){
        return;
    }
    setSimDownCommand();
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}

void State_Passive::setSimDownCommand(){
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = downPos[i];
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 12.0f;
        _lowCmd->motorCmd[i].Kd = 2.0f;
        _lowCmd->motorCmd[i].tau = 0;
    }
}
