#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include <vector>
#include <cstring>
#include <openssl/sha.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/IOSDK.h"

bool running = true;  

// 信号处理函数，当收到 SIGINT（通常由 Ctrl+C 产生）时执行
void ShutDown(int sig) 
{
    std::cout << "stop the controller" << std::endl;
    // 执行的就是把running赋值为false
    running = false;
}

void setProcessScheduler()  // 实时调度设置
{   // 设置进程为实时调度策略的函数
    pid_t pid = getpid();
    // 获取当前进程 ID
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    // 定义调度参数结构体，将优先级设置为 SCHED_FIFO 策略下的最大优先级值
    // SCHED_FIFO 是实时先进先出调度，一旦占用 CPU 会一直运行直到阻塞或主动让出，适合要求确定延时的控制任务
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv) {
    // 首先尝试将控制进程设为实时调度，确保控制周期的时间确定性
    setProcessScheduler();
    // 设置标准输出的浮点数格式：固定小数点表示，保留三位小数，使日志信息整齐可读
    std::cout << std::fixed << std::setprecision(3);
    // 声明 IO 接口基类指针和平台类型枚举变量。IOInterface 应该是 IOSDK 的基类
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    // 创建 IOSDK 实例并赋值给基类指针，这里负责与机器人硬件的实际通信
    ioInter = new IOSDK();
    // 设置控制平台为真实机器人（区别于仿真），这个枚举会影响 CtrlComponents 内部行为，例如选择不同的安全检查和通信逻辑
    ctrlPlat = CtrlPlatform::REALROBOT;
    // 自定义的类：创建控制组件集合对象，传入 IO 接口，这里是在堆上分配，必须用->去访问其成员
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    // 将平台类型存入控制组件，供后续模块使用
    ctrlComp->ctrlPlatform = ctrlPlat;
    // 设置控制周期为 0.02 秒（20 毫秒），即控制频率 50 Hz
    ctrlComp->dt = 0.02;
    // 将全局运行标志的地址传给 ctrlComp->running 指针
    ctrlComp->running = &running;
    // 自定义的类：创建控制主框架对象，传入配置好的组件集合，这里是在栈上分配可以用.来访问其成员
    ControlFrame ctrlFrame(ctrlComp);
    // signal是一个信号处理函数，而SIGINT是一个中断信号，也就是用户在终端按下ctrl+c就会产生出现中断去执行ShutDown函数。
    signal(SIGINT, ShutDown);
    // 主循环，只要running为true则一直跑ctrlFrame
    while (running) {
        if (ctrlComp->exitFlag) break;
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
