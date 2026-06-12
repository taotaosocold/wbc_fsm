#include "FSM/FSM.h"
#include <iostream>  

// 构造函数，给每个状态都初始化分配空间，不管是哪个类State_WBC还是State_AMP都是继承类FSMState
FSM::FSM(CtrlComponents *ctrlComp)
    // _ctrlComp(ctrlComp)就是把传入的参数ctrlComp传给_ctrlComp
    :_ctrlComp(ctrlComp){
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.loco = new State_Loco(_ctrlComp);
    _stateList.amp = new State_AMP(_ctrlComp);
    _stateList.mjamp = new State_MJAMP(_ctrlComp);
    _stateList.wbc = new State_WBC(_ctrlComp);
    initialize(); 
}

FSM::~FSM(){  
    // 释放状态机的空间
    _stateList.deletePtr();
}

void FSM::initialize(){
    // 初始化的时候，当前状态进入阻尼模式
    _currentState = _stateList.passive;
    _currentState -> enter();  
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;  

    std::cout<<"Press **start** to enter position control mode..."<<std::endl;
}

void FSM::run(){
    try{
        // 记录时间，确保频率保持在50Hz
        _startTime = getSystemTime();  
        // 通信接口，订阅和发布数据
        _ctrlComp->sendRecv(); 
        // 状态机处理，对于正常模式
        if(_mode == FSMMode::NORMAL){  
            // 这里就是根据状态机来执行不同的run函数，run函数内部会根据订阅的数据来构建观测和计算力矩
            // 而力矩的发布得等到下一次的循环sendRecv函数才会发布出去
            _currentState->run();  
            // 检查是否满足切换条件
            _nextStateName = _currentState->checkChange();    
            // 如果下一个状态和当前状态不一致
            if(_nextStateName != _currentState->_stateName){
                // 将mode改成切换模式  
                _mode = FSMMode::CHANGE;  
                _nextState = getNextState(_nextStateName); 
                std::cout << "Switched from " << _currentState->_stateNameString
                << " to " << _nextState->_stateNameString << std::endl; 
            }
        }
        // 如果是切换模式
        else if(_mode == FSMMode::CHANGE){  
            _currentState->exit();  
            _currentState = _nextState; 
            _currentState->enter();  
            _mode = FSMMode::NORMAL; 
            _currentState->run(); 
        }
        // 精确等待
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));  
    // 异常捕获
    }catch (const std::exception& e) {
        std::cerr << std::endl << "Caught exception: " << e.what() << std::endl;
        _ctrlComp->exitFlag = true;
    }
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
    case FSMStateName::LOCO: 
        return _stateList.loco;
    case FSMStateName::WBC:
        return _stateList.wbc;
    case FSMStateName::AMP:
        return _stateList.amp;
    case FSMStateName::MJAMP:
        return _stateList.mjamp;
    default:
        return _stateList.invalid;
        break;
    }
}