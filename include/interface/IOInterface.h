#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>
// 统一所有机器人通信方式的调用规范，让上层控制代码不必关心底层是用 Ethernet、CAN 还是其他协议
class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}
// 函数在.h可能会申明但是在对应的.cpp文件可能并不会存在这个函数的定义。但是纯虚函数就要求.cpp文件必须要求有这个函数，不然就报错，包括IOInterface.cpp
// 还有继承了类IOInterface的其他类对应的.cpp文件
// 每个控制周期完成一次完整的指令发送与状态接收
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
// 在本仓库里其实根本没有CmdPanel.cpp这个源文件，更别说对应的函数了
// 归零，让机器人所有输出归零
void zeroCmdPanel(){cmdPanel->setZero();}
// 将机器人切换到阻尼模式
void setPassive(){cmdPanel->setPassive();}
// 定义一个指针cmdPanel
protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H