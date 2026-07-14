#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

#define NUM_DOF 25

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _targetPos[NUM_DOF] = {
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0
    };

    // float _targetPos[NUM_DOF] = {
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0,
    //     0.0, 0.0, 0.0, 0.0, 0.0
    // };


    float _startPos[NUM_DOF];
    float _duration = 2.0;
    float _phase = 0;
    bool _fixedstand_complete_flag;

    float Kps[NUM_DOF] = {
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,
        0.0, 0.0, 276.311,
        130.201, 130.201, 96.825, 130.201, 96.825,
        130.201, 130.201, 96.825, 130.201, 96.825
    };

    float Kds[NUM_DOF] = {
        17.591, 17.591, 9.951, 17.591, 9.951, 9.951,
        17.591, 17.591, 9.951, 17.591, 9.951, 9.951,
        0.0, 0.0, 17.591,
        8.289, 8.289, 6.164, 8.289, 6.164,
        8.289, 8.289, 6.164, 8.289, 6.164
    };
};

#endif  // FIXEDSTAND_H
