#include <iostream>
#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){
    _Kds = 0.0;
}

void State_Passive::enter(){
    for(int i=0; i<NUM_DOF; i++){
        _lowCmd->motorCmd[i].q = 0;
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 0;
        _lowCmd->motorCmd[i].Kd = _Kds;
        _lowCmd->motorCmd[i].tau = 0;
    }
    
}

void State_Passive::run(){
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::R2_A){
        return FSMStateName::WBC;
    }
    else if(_lowState->userCmd == UserCommand::SELECT){
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}