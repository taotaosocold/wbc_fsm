// 头文件保护宏，防止重复包含
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include <string>
#include <iostream>


struct CtrlComponents{
public:
    // 构造函数接收一个 IOInterface 指针（在 main 里就是 new IOSDK() 传进来的）
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        // 构造函数内部就是初始化三个变量空间，两个消息变量，一个内部控制退出标志（可能被某些错误逻辑置为 true）
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        exitFlag = false;
    }
    // 析构函数负责释放构造时分配的资源
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
    }
    // 构建相应的指针
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;

    double dt;
    bool *running;
    bool exitFlag;
    CtrlPlatform ctrlPlatform;
    // 函数定义在这里，这里调用ioInter的sendRecv函数
    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);  
    }



};

#endif  // CTRLCOMPONENTS_H