
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

#define NUM_DOF 25

class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

    double _Kds = 10;
    double _Kps = 10;
};

#endif  // PASSIVE_H