#include "control/ControlFrame.h"
// 构造函数就是new一个有限状态机
ControlFrame::ControlFrame(CtrlComponents *ctrlComp):_ctrlComp(ctrlComp){
    _FSMController = new FSM(_ctrlComp);
} 

void ControlFrame::run(){

    _FSMController->run();
}