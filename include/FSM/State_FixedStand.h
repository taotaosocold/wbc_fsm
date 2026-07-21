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
        // left leg
        -0.1010930389, 0.1863300502, 0.1534859538, 0.1218007132, -0.0273471251, 0.0,
        // right leg
        -0.1159527600, -0.1466802061, -0.1086166725, 0.1400099695, 0.0167938694, 0.0,
        // waist_yaw, head_yaw, head_pitch
        -0.0127099706, -0.0008552494, 0.0,
        // left arm
        -0.0597212315, 0.7225397229, -0.0980680510, -0.2135050893, -0.0890462548,
        // right arm
        -0.0829682425, -0.8226424456, 0.0251893010, -0.1944101751, 0.1169776917,
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
