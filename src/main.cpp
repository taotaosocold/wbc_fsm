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
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/IOROS2.h"

bool running = true;

void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    setProcessScheduler();
    std::cout << std::fixed << std::setprecision(3);

    IOInterface *ioInter = new IOROS2();
    CtrlPlatform ctrlPlat = CtrlPlatform::MUJOCO;

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.001;
    ctrlComp->running = &running;

    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);

    while (running) {
        if (ctrlComp->exitFlag) break;
        ctrlFrame.run();
    }

    delete ctrlComp;
    rclcpp::shutdown();
    return 0;
}
