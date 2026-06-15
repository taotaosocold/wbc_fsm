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
#define NUM_BODIES 14

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

    const std::vector<const char*> _input_names = {"obs", "time_step"};
    const std::vector<const char*> _output_names_all = {
        "actions", "joint_pos", "joint_vel",
        "body_pos_w", "body_quat_w",
        "body_lin_vel_w", "body_ang_vel_w"
    };

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
    const float action_beta = 0.7f;  // EMA smoothing

    float _joint_q[NUM_DOF];

    std::vector<float> _action;
    std::vector<float> _last_action;  // EMA smoothed action (model order)
    std::vector<float> _observation;

    // Data source mode
    std::string _data_source;  // "onnx" or "npz"
    int _total_frames;

    // Autoregressive reference (ONNX mode)
    std::vector<float> _ref_joint_pos;
    std::vector<float> _ref_joint_vel;
    std::vector<float> _ref_body_quat_w;  // 14*4 body quaternions from model output
    bool _warmup_done = false;
    int _anchor_body_idx = 7;  // waist_yaw_link is body index 7

    // Motion data from NPZ
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

    // Model joint order → motor bus index (dof_mapping[policy_idx] = bus_idx)
    // Model: left_leg_pelvic_pitch, right_leg_pelvic_pitch, waist_yaw,
    //        left_leg_pelvic_roll, right_leg_pelvic_roll, head_yaw,
    //        left_shoulder_pitch, right_shoulder_pitch,
    //        left_leg_pelvic_yaw, right_leg_pelvic_yaw, head_pitch,
    //        left_shoulder_roll, right_shoulder_roll,
    //        left_leg_knee_pitch, right_leg_knee_pitch,
    //        left_shoulder_yaw, right_shoulder_yaw,
    //        left_leg_ankle_pitch, right_leg_ankle_pitch,
    //        left_elbow_pitch, right_elbow_pitch,
    //        left_leg_ankle_roll, right_leg_ankle_roll,
    //        left_wrist_yaw, right_wrist_yaw
    const int dof_mapping[NUM_DOF] = {
        1,  7, 14,                            // policy 0-2
        0,  6, 12,                            // policy 3-5
        15, 20,                               // policy 6-7
        2,  8, 13,                            // policy 8-10
        16, 21,                               // policy 11-12
        3,  9,                                // policy 13-14
        17, 22,                               // policy 15-16
        4, 10,                                // policy 17-18
        18, 23,                               // policy 19-20
        5, 11,                                // policy 21-22
        19, 24                                // policy 23-24
    };

    // Motor bus index → model policy index (inverse of dof_mapping)
    const int motor_bus_to_policy[NUM_DOF] = {
        3,  0,  8, 13, 17, 21,               // bus 0-5: left leg
        4,  1,  9, 14, 18, 22,               // bus 6-11: right leg
        5, 10,  2,                            // bus 12-14: head, waist
        6, 11, 15, 19, 23,                    // bus 15-19: left arm
        7, 12, 16, 20, 24                     // bus 20-24: right arm
    };

    // Default joint positions in BUS order (from model metadata, rearranged)
    const float _default_dof_pos[NUM_DOF] = {
         0.0, -0.1,  0.0,  0.5, -0.175,  0.0,   // left leg
         0.0, -0.1,  0.0,  0.5, -0.175,  0.0,   // right leg
         0.0,  0.0,  0.0,                         // head_yaw, head_pitch, waist_yaw
         0.0,  0.0,  0.0, -0.5,  0.0,            // left arm
         0.0,  0.0,  0.0, -0.5,  0.0             // right arm
    };

    // Kp in BUS order (from model metadata joint_stiffness, rearranged)
    const double dof_Kps[NUM_DOF] = {
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,  // left leg
        276.311, 276.311, 156.310, 276.311, 156.310, 156.310,  // right leg
          0.0,     0.0,   276.311,                              // head, waist
        130.201, 130.201,  96.825, 130.201,  96.825,           // left arm
        130.201, 130.201,  96.825, 130.201,  96.825            // right arm
    };

    // Kd in BUS order (from model metadata joint_damping, rearranged)
    const double dof_Kds[NUM_DOF] = {
        17.591, 17.591,  9.951, 17.591,  9.951,  9.951,  // left leg
        17.591, 17.591,  9.951, 17.591,  9.951,  9.951,  // right leg
         0.0,    0.0,   17.591,                           // head, waist
         8.289,  8.289,  6.164,  8.289,  6.164,          // left arm
         8.289,  8.289,  6.164,  8.289,  6.164           // right arm
    };

    // Action scales in BUS order (from model metadata, rearranged)
    const float _action_scale[NUM_DOF] = {
        0.136, 0.136, 0.096, 0.136, 0.096, 0.096,  // left leg
        0.136, 0.136, 0.096, 0.136, 0.096, 0.096,  // right leg
        1.0,   1.0,   0.054,                         // head, waist
        0.144, 0.144, 0.093, 0.144, 0.093,          // left arm
        0.144, 0.144, 0.093, 0.144, 0.093           // right arm
    };
};

#endif // WBC_H
