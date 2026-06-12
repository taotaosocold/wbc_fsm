#ifndef FSM_H
#define FSM_H
// 各个具体状态类，对应不同的机器人行为包括固定站立、被动模式、运动包括行走和奔跑、AMP以及全身控制
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_Loco.h"
#include "FSM/State_Amp.h"
#include "FSM/State_MJAmp.h"
#include "FSM/State_WBC.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
// 一个状态对象仓库，用指针存放所有可能的状态实例
struct FSMStateList{
    // 空状态/占位，永远不会进入的状态
    FSMState *invalid;
    // 阻尼模式，所有点击关闭，机器人瘫软
    State_Passive *passive;
    // 固定站立，用PD控制将所有关节锁死在预设状态
    State_FixedStand *fixedStand;
    // 遥控行走，locomotion策略
    State_Loco *loco;
    // beyondmimic
    State_WBC *wbc;
    // amp
    State_AMP *amp;
    // 模型会额外输入参考关节信息的amp
    State_MJAMP *mjamp;
    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete loco;
        delete wbc;
        delete amp; 
        delete mjamp;
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
// _ctrlComp：共享组件，状态可访问 IO 接口、指令、状态、参数等。
// _currentState：当前正在执行的状态对象。
// _nextState：下一周期要切换到的状态对象（预存，避免在 run 中重复查找）。
// _nextStateName：状态机模式机状态名称（如 LOCO, PASSIVE），由当前状态 run() 返回，用于决定切换。
// _stateList：上述状态仓库结构体实例，持有所有状态对象。
// _mode：有限状态机的工作模式（可能影响状态转移逻辑）。
// _startTime：记录程序开始时间（微秒级），可能用于计算运行时长。
// count：周期计数器，用于调试或周期性任务。
private:
    FSMState* getNextState(FSMStateName stateName);
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};


#endif  // FSM_H