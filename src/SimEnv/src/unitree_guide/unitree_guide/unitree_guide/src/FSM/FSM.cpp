/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unistd.h>

#ifdef COMPILE_WITH_ROS
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#endif

namespace {
const std::array<std::string, 12> kResetJointNames = {{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
}};

const std::array<double, 12> kResetJointPositions = {{
    -0.35, 1.36, -2.65,
     0.35, 1.36, -2.65,
    -0.50, 1.36, -2.65,
     0.50, 1.36, -2.65
}};

double getEnvDouble(const char *name, double fallback)
{
    const char *value = std::getenv(name);
    if(value == nullptr || value[0] == '\0'){
        return fallback;
    }

    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value, &end);
    if(errno != 0 || end == value || *end != '\0' || !std::isfinite(parsed)){
        return fallback;
    }
    return parsed;
}

#ifdef COMPILE_WITH_ROS
bool callEmptyService(const std::string &service_name, double timeout_seconds)
{
    if(!ros::service::waitForService(service_name, ros::Duration(timeout_seconds))){
        std::cout << "[WARNING] " << service_name << " is not available." << std::endl;
        return false;
    }

    std_srvs::Empty srv;
    if(!ros::service::call(service_name, srv)){
        std::cout << "[WARNING] Failed to call " << service_name << "." << std::endl;
        return false;
    }
    return true;
}
#endif
}

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    _stateList.rl = new State_RL(_ctrlComp);
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->ioInterFreeDog->sendRecv();
    if(handleResetCommand()){
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    if(!_ctrlComp->ioInter->hasFullStateFeedback()){
        if(!_waitingForStateFeedback){
            std::cout << "[INFO] Waiting for Gazebo joint state feedback before accepting stand command." << std::endl;
            _waitingForStateFeedback = true;
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    if(_waitingForStateFeedback){
        std::cout << "[INFO] Gazebo joint state feedback is ready." << std::endl;
        _waitingForStateFeedback = false;
    }
    if(!_ctrlComp->lowState->isFinite()){
        std::cout << "[WARNING] Gazebo state feedback is not finite; skipping control update." << std::endl;
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        if(!_fallSafetyLatched){
            std::cout << "[WARNING] Robot appears to have fallen. Switching to passive/down. Press 8 to reset pose." << std::endl;
            _fallSafetyLatched = true;
        }
        forcePassiveState();
        _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    _fallSafetyLatched = false;

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    case FSMStateName::RL:
        return _stateList.rl;
    break;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}

bool FSM::handleResetCommand(){
    if(_ctrlComp->lowState->userCmd != UserCommand::RESET){
        _resetCommandLatched = false;
        return false;
    }
    if(_resetCommandLatched){
        return true;
    }
    _resetCommandLatched = true;

    std::cout << "[INFO] Reset command received. Returning robot to the start pose." << std::endl;
    forcePassiveState();

    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        if(resetGazeboRobot()){
            std::cout << "[INFO] Gazebo robot reset complete. Press 2 to stand again." << std::endl;
        }else{
            std::cout << "[WARNING] Gazebo robot reset request failed. Check Gazebo service availability." << std::endl;
        }
    }else{
        std::cout << "[WARNING] Reset command is only supported in Gazebo simulation." << std::endl;
    }

    return true;
}

bool FSM::resetGazeboRobot(){
#ifndef COMPILE_WITH_ROS
    return false;
#else
    ros::NodeHandle nh;

    if(!callEmptyService("/gazebo/pause_physics", 1.0)){
        return false;
    }

    setResetDownCommand();
    _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
    usleep(30000);

    if(!ros::service::waitForService("/gazebo/set_model_configuration", ros::Duration(1.0))){
        std::cout << "[WARNING] /gazebo/set_model_configuration is not available." << std::endl;
        callEmptyService("/gazebo/unpause_physics", 1.0);
        return false;
    }
    if(!ros::service::waitForService("/gazebo/set_model_state", ros::Duration(1.0))){
        std::cout << "[WARNING] /gazebo/set_model_state is not available." << std::endl;
        callEmptyService("/gazebo/unpause_physics", 1.0);
        return false;
    }

    std::string robot_name = "a1";
    nh.getParam("/robot_name", robot_name);
    const std::string model_name = robot_name + "_gazebo";

    gazebo_msgs::SetModelState state_srv;
    state_srv.request.model_state.model_name = model_name;
    state_srv.request.model_state.reference_frame = "world";
    state_srv.request.model_state.pose.position.x = getEnvDouble("COMPETITION_ROBOT_X", 0.0);
    state_srv.request.model_state.pose.position.y = getEnvDouble("COMPETITION_ROBOT_Y", -3.2);
    state_srv.request.model_state.pose.position.z =
        getEnvDouble("COMPETITION_ROBOT_Z", 0.6) + getEnvDouble("COMPETITION_RESET_Z_OFFSET", 0.05);
    state_srv.request.model_state.pose.orientation =
        tf::createQuaternionMsgFromYaw(getEnvDouble("COMPETITION_ROBOT_YAW", 1.5708));
    state_srv.request.model_state.twist.linear.x = 0.0;
    state_srv.request.model_state.twist.linear.y = 0.0;
    state_srv.request.model_state.twist.linear.z = 0.0;
    state_srv.request.model_state.twist.angular.x = 0.0;
    state_srv.request.model_state.twist.angular.y = 0.0;
    state_srv.request.model_state.twist.angular.z = 0.0;

    ros::ServiceClient state_client =
        nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    if(!state_client.call(state_srv) || !state_srv.response.success){
        std::cout << "[WARNING] Failed to reset A1 model state: "
                  << state_srv.response.status_message << std::endl;
        callEmptyService("/gazebo/unpause_physics", 1.0);
        return false;
    }

    gazebo_msgs::SetModelConfiguration config_srv;
    config_srv.request.model_name = model_name;
    config_srv.request.urdf_param_name = "robot_description";
    config_srv.request.joint_names.assign(kResetJointNames.begin(), kResetJointNames.end());
    config_srv.request.joint_positions.assign(kResetJointPositions.begin(), kResetJointPositions.end());

    ros::ServiceClient config_client =
        nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    if(!config_client.call(config_srv) || !config_srv.response.success){
        std::cout << "[WARNING] Failed to reset A1 joint configuration: "
                  << config_srv.response.status_message << std::endl;
        callEmptyService("/gazebo/unpause_physics", 1.0);
        return false;
    }

    if(!state_client.call(state_srv) || !state_srv.response.success){
        std::cout << "[WARNING] Failed to apply A1 model state after joint reset: "
                  << state_srv.response.status_message << std::endl;
        callEmptyService("/gazebo/unpause_physics", 1.0);
        return false;
    }

    setResetDownCommand();
    _ctrlComp->ioInter->sendRecv(_ctrlComp->lowCmd, _ctrlComp->lowState);
    usleep(150000);

    const bool unpause_ok = callEmptyService("/gazebo/unpause_physics", 1.0);
    return unpause_ok;
#endif
}

void FSM::forcePassiveState(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->ioInter->setPassive();
    _ctrlComp->lowState->userCmd = UserCommand::L2_B;
    _ctrlComp->lowState->userValue.setZero();

    if(_currentState != _stateList.passive){
        if(_currentState != nullptr){
            _currentState->exit();
        }
        _currentState = _stateList.passive;
        _currentState->enter();
    }

    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
    setResetDownCommand();
}

void FSM::setResetDownCommand(){
    for(int i = 0; i < 12; ++i){
        _ctrlComp->lowCmd->motorCmd[i].mode = 10;
        _ctrlComp->lowCmd->motorCmd[i].q = static_cast<float>(kResetJointPositions[i]);
        _ctrlComp->lowCmd->motorCmd[i].dq = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].tau = 0.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kp = 12.0f;
        _ctrlComp->lowCmd->motorCmd[i].Kd = 2.0f;
    }
    _ctrlComp->setAllSwing();
}
