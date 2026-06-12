#ifndef WBC_H
#define WBC_H

#include "FSM/FSMState.h"
#include "common/read_traj.h"
#include "common/npz_reader.h"
#include "common/mathTools.h"
#include <onnxruntime_cxx_api.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <cmath>
#include <stdexcept>

#define NUM_DOF 25

class State_WBC : public FSMState
{
public:
    State_WBC(CtrlComponents *ctrlComp);
    ~State_WBC() = default;
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    // ONNX mode: read all 7 outputs
    // NPZ mode: only read actions
    const std::vector<const char*> _input_names = {"obs", "time_step"};
    const std::vector<const char*> _output_names_all = {
        "actions", "joint_pos", "joint_vel",
        "body_pos_w", "body_quat_w",
        "body_lin_vel_w", "body_ang_vel_w"
    };
    const std::vector<const char*> _output_names_actions = {"actions"};

    std::vector<int64_t> _input_shape;
    std::vector<int64_t> _output_shape;
    int64_t _obs_size_;
    int64_t _action_size_;

    bool _start_flag = false;
    float _targetPos_rl[NUM_DOF];
    float _last_targetPos_rl[NUM_DOF];

    void _loadPolicy();
    void _observations_compute();
    void _action_compute();

    const float clip_observations = 100.0;
    const float clip_actions = 100.0;
    const float action_scale = 0.25;

    const float scale_lin_vel = 1.0;
    const float scale_ang_vel = 1.0;
    float scale_dof_pos = 1.0;
    float scale_dof_vel = 1.0;
    float _joint_q[NUM_DOF];

    std::vector<float> _action;
    std::vector<float> _observation;

    // data source mode
    std::string _data_source;  // "onnx" or "npz"
    int _total_frames;

    // autoregressive reference (ONNX mode)
    std::vector<float> _ref_joint_pos;
    std::vector<float> _ref_joint_vel;

    // motion data from NPZ
    std::vector<float> _body_ang_vel_w;
    std::vector<uint32_t> _body_ang_vel_w_shape;

    std::vector<float> _body_lin_vel_w;
    std::vector<uint32_t> _body_lin_vel_w_shape;

    std::vector<float> _body_pos_w;
    std::vector<uint32_t> _body_pos_w_shape;

    std::vector<float> _body_quat_w;
    std::vector<uint32_t> _body_quat_w_shape;

    std::vector<int64_t> _fps;
    std::vector<uint32_t> _fps_shape;

    std::vector<float> _joint_pos;
    std::vector<uint32_t> _joint_pos_shape;

    std::vector<float> _joint_vel;
    std::vector<uint32_t> _joint_vel_shape;

    bool _bin_data_loaded;
    std::string _model_path;
    std::string _folder_path;
    std::string _motion_file_path;
    std::vector<float> _target_dof_pos;
    const int _frame_interval = 5;
    unsigned int _refer_idx = 0;
    unsigned int _last_refer_idx = 0;
    const int _anchor_idx = 0;
    bool _pause_flag = false;
    int _start_refer_idx = 0;
    int _pause_refer_idx = 350;
    int _end_refer_idx = -1;
    int _motion_frame_count = 0;
    const std::vector<float> _gravity_vec = {0.0f, 0.0f, -1.0f};
    float _anchor_terminate_thresh = 0.5f;
    bool _terminate_flag = false;
    bool _pause_curr_flag = false;

    const float _default_dof_pos[NUM_DOF] = {
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        -0.1, 0.0, 0.0, 0.5, -0.175, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0,
        0.0, 0.0, 0.0, -0.5, 0.0
    };

    // policy output index → motor bus index
    const int dof_mapping[NUM_DOF] = {
        0, 3, 8, 13, 17, 21,
        1, 4, 9, 14, 18, 22,
        2, 5, 10, 6, 11, 15,
        19, 23, 7, 12, 16,
        20, 24
    };

    // motor bus index → policy index (inverse of dof_mapping)
    const int motor_bus_to_policy[NUM_DOF] = {
        0, 6, 12, 1, 7, 13,
        15, 20, 2, 8, 14, 16,
        21, 3, 9, 17, 22,
        4, 10, 18, 23, 5,
        11, 19, 24
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

#endif // WBC_H
