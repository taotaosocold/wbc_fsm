#ifndef STATE_LOCO_H
#define STATE_LOCO_H

#include "FSM/FSMState.h"
#include <onnxruntime_cxx_api.h>
#include "common/mathTools.h"
#include <cstring>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>

#define NUM_DOF 25

class State_Loco : public FSMState
{
public:
    State_Loco(CtrlComponents *ctrlComp);
    ~State_Loco() {}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    const std::vector<const char*> _input_names = {"obs", "h_in", "c_in"};
    const std::vector<const char*> _output_names = {"actions", "h_out", "c_out"};

    std::vector<int64_t> _input_shape;
    std::vector<int64_t> _output_shape;

    int64_t _obs_size_;
    int64_t _hidden_size_;
    int64_t _action_size_;

    std::vector<float> _h_state_;
    std::vector<float> _c_state_;

    bool _start_flag = false;
    bool _init_obs = false;
    float _targetPos_rl[NUM_DOF];
    float _last_targetPos_rl[NUM_DOF];

    int _low_level_num;

    void _loadPolicy();
    void _observations_compute();
    void _action_compute();

    const int _num_obs = 96;
    const int _num_obs_history = 1;
    const float clip_observations = 100.0;
    const float clip_actions = 100.0;
    const float action_scale = 0.25;

    const float scale_lin_vel = 1.0;
    const float scale_ang_vel = 1.0;
    float scale_dof_pos = 1.0;
    float scale_dof_vel = 1.0;

    float _joint_q[NUM_DOF];

    Vec3 _vCmdBody;
    Vec3 _vCmdBodyPast;
    double _dYawCmd;
    double _dYawCmdPast;
    Vec2 _vxLim, _vyLim, _wyawLim;
    double _cmdSmoothes;

    std::vector<float> _action;
    std::vector<float> _observation;
    const std::vector<float> _gravity_vec = std::vector<float>({0.0, 0.0, -1.0});
    float _anchor_terminate_thresh;
    bool _terminate_flag = false;
    std::string _model_path;
    void _getUserCmd();
    void _init_buffers();

    float dead_zone = 0.1;

    const float _default_dof_pos[NUM_DOF] = {
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0
    };

    const int dof_mapping[NUM_DOF] = {
        0, 3, 8, 13, 17, 21,
        1, 4, 9, 14, 18, 22,
        2, 5, 10, 6, 11, 15,
        19, 23, 7, 12, 16,
        20, 24
    };

    const double dof_Kps[NUM_DOF] = {
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,
        0.0, 0.0, 276.311,
        130.201, 130.201, 96.825, 130.201, 96.825,
        130.201, 130.201, 96.825, 130.201, 96.825
    };

    const double dof_Kds[NUM_DOF] = {
        17.591, 17.591, 9.951, 17.591, 9.951, 9.951,
        17.591, 17.591, 9.951, 17.591, 9.951, 9.951,
        0.0, 0.0, 17.591,
        8.289, 8.289, 6.164, 8.289, 6.164,
        8.289, 8.289, 6.164, 8.289, 6.164
    };
};

#endif // STATE_LOCO_H
