#ifndef WBC_H
#define WBC_H

#include "FSM/FSMState.h"
#include "common/read_traj.h"
#include "common/npz_reader.h"
#include "common/mathTools.h"
#include "common/data_converter.h"
#include <onnxruntime_cxx_api.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <cmath>
#include <stdexcept>
#include <sstream>

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
    Ort::RunOptions _run_options;  // Reused for LSTM hidden state preservation
    Ort::AllocatorWithDefaultOptions _allocator;

    int64_t _obs_size_;
    int64_t _action_size_;

    // Data pipeline — matches deploy_beyondmimic reference exactly
    DataConverter _converter;
    bool _motion_initialized = false;   // dry run + init alignment done
    bool _warmup_done = false;

    // Model metadata read from ONNX (model order)
    std::vector<float> _model_kp;
    std::vector<float> _model_kd;
    std::vector<float> _model_action_scales;
    std::vector<float> _model_default_dof_pos;
    std::vector<std::string> _model_joint_names;
    std::vector<std::string> _model_body_names;
    std::string _model_anchor_body_name;

    // Bus-order values for motor commands (derived from model metadata + reordering)
    std::vector<float> _bus_kp;
    std::vector<float> _bus_kd;

    // Cached PD targets (bus order)
    float _targetPos_rl[NUM_DOF];
    float _last_targetPos_rl[NUM_DOF];

    // Last action for observation (model order, double precision)
    std::vector<double> _last_action_model;

    // Observation buffer
    std::vector<float> _observation;

    void _loadPolicy();

    // ONNX metadata helpers
    static std::vector<float> _parseFloatList(const std::string& s);
    static std::vector<std::string> _parseStringList(const std::string& s);

    // Extract bus-order values from MotorState struct array (safe: handles struct stride)
    static void _extractBusOrder(const MotorState* ms, float* q_out, float* dq_out, int N);

    // Bus-order CasBot joint names (matches XML actuator order = sim2real bus order)
    // Derived from axis comparison: leg_l1(axis 0 1 0) = left_leg_pelvic_pitch(axis 0 1 0)
    static constexpr const char* bus_joint_names[NUM_DOF] = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
        "head_yaw_joint", "head_pitch_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint", "right_wrist_yaw_joint"
    };

    // Computed in _loadPolicy: bus index → model index
    int _bus_to_model_idx[NUM_DOF];

    // ==================== NPZ mode (debug/playback) ====================
    std::string _data_source;
    int _total_frames;

    std::vector<float> _ref_joint_pos;
    std::vector<float> _ref_joint_vel;

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

    // NPZ data source helpers
    std::vector<float> _getNPZJointPos(int frame_idx) const;
    std::vector<float> _getNPZJointVel(int frame_idx) const;
    std::vector<float> _getNPZAnchorQuat(int frame_idx) const;
    void _observations_compute_npz();
    void _action_compute_npz();

    // ONNX data source
    void _observations_compute_onnx();
    void _action_compute_onnx();
};

#endif // WBC_H
