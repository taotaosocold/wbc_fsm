#include <iostream>
#include "FSM/State_WBC.h"
#include "common/read_traj.h"
#include "common/npz_reader.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ============================================================================
// Static helpers: parse comma-separated strings from ONNX metadata
// ============================================================================
std::vector<float> State_WBC::_parseFloatList(const std::string& s) {
    std::vector<float> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        out.push_back(std::stof(item));
    }
    return out;
}

std::vector<std::string> State_WBC::_parseStringList(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        out.push_back(item);
    }
    return out;
}

// ============================================================================
// Extract bus-order joint values from MotorState struct array
// MotorState layout: mode, q, dq, ddq, tauEst — not contiguous floats
// ============================================================================
void State_WBC::_extractBusOrder(const MotorState* ms, float* q_out, float* dq_out, int N) {
    for (int i = 0; i < N; i++) {
        q_out[i]  = ms[i].q;
        dq_out[i] = ms[i].dq;
    }
}

// ============================================================================
// NPZ helpers
// ============================================================================
std::vector<float> State_WBC::_getNPZJointPos(int frame_idx) const {
    int n = _joint_pos_shape[1];
    int base = frame_idx * n;
    std::vector<float> pos(n);
    for (int i = 0; i < n; i++) pos[i] = _joint_pos[base + i];
    return pos;
}

std::vector<float> State_WBC::_getNPZJointVel(int frame_idx) const {
    int n = _joint_vel_shape[1];
    int base = frame_idx * n;
    std::vector<float> vel(n);
    for (int i = 0; i < n; i++) vel[i] = _joint_vel[base + i];
    return vel;
}

std::vector<float> State_WBC::_getNPZAnchorQuat(int frame_idx) const {
    int num_links = _body_quat_w_shape[1];
    int base = frame_idx * num_links * 4 + _anchor_idx * 4;
    return {_body_quat_w[base], _body_quat_w[base + 1],
            _body_quat_w[base + 2], _body_quat_w[base + 3]};
}

// ============================================================================
// Constructor
// ============================================================================
State_WBC::State_WBC(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::WBC, "wbc")
{
    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/wbc.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
        throw std::runtime_error("Cannot open config file");
    }

    try {
        json config = json::parse(config_file);
        std::string base_path = std::string(PROJECT_ROOT_DIR) + "/";
        _model_path = base_path + config["model_path"].get<std::string>();

        _data_source = config.value("data_source", "npz");
        _anchor_terminate_thresh = config["safe_projgravity_threshold"].get<float>();
        _start_refer_idx = config["start_idx"].get<int>();
        _pause_refer_idx = config["pause_idx"].get<int>();
        _end_refer_idx = config["end_idx"].get<int>();
        _total_frames = config.value("total_frames", 0);
        _action_beta = config.value("action_beta", 1.0f);

        std::cout << "[Config] Model path: " << _model_path << std::endl;
        std::cout << "[Config] Data source: " << _data_source << std::endl;

        if (_data_source == "npz") {
            _motion_file_path = base_path + config["motion_path"].get<std::string>();
            std::cout << "[Config] Motion file: " << _motion_file_path << std::endl;

            _bin_data_loaded = NPZReader::loadNPZ(
                _motion_file_path,
                _joint_pos, _joint_pos_shape,
                _joint_vel, _joint_vel_shape,
                _body_pos_w, _body_pos_w_shape,
                _body_quat_w, _body_quat_w_shape,
                _body_ang_vel_w, _body_ang_vel_w_shape,
                _body_lin_vel_w, _body_lin_vel_w_shape,
                _fps, _fps_shape
            );
            _motion_frame_count = _joint_pos_shape[0];
            if (_bin_data_loaded) {
                std::cout << "[SUCCESS] Loaded motion data from NPZ. Frames: "
                          << _motion_frame_count << std::endl;
            } else {
                std::cerr << "[ERROR] Failed to load NPZ data!" << std::endl;
            }
        } else if (_data_source == "onnx") {
            _bin_data_loaded = true;
            _motion_frame_count = _total_frames;
            std::cout << "[Config] ONNX autoregressive mode, total_frames: "
                      << _total_frames << std::endl;
        } else {
            std::cerr << "[ERROR] Unknown data_source: " << _data_source << std::endl;
            throw std::runtime_error("Unknown data_source");
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
        throw;
    }
    config_file.close();

    _ref_joint_pos = std::vector<float>(NUM_DOF, 0.0f);
    _ref_joint_vel = std::vector<float>(NUM_DOF, 0.0f);

    _loadPolicy();
}

// ============================================================================
// ONNX model loading + metadata parsing + DataConverter configuration
// ============================================================================
void State_WBC::_loadPolicy()
{
    // --- Load ONNX session ---
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

    Ort::TypeInfo input0_type = _session->GetInputTypeInfo(0);
    auto input0_shapes = input0_type.GetTensorTypeAndShapeInfo().GetShape();
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape();

    _obs_size_ = input0_shapes[1];
    _action_size_ = output_shapes[1];

    std::cout << "[State_WBC] Model loaded. obs_size=" << _obs_size_
              << " action_size=" << _action_size_ << std::endl;

    // --- Parse ONNX model metadata ---
    Ort::ModelMetadata model_meta = _session->GetModelMetadata();
    auto keys = model_meta.GetCustomMetadataMapKeysAllocated(_allocator);

    std::map<std::string, std::string> meta_map;
    for (const auto& key_ptr : keys) {
        std::string key(key_ptr.get());
        auto val_ptr = model_meta.LookupCustomMetadataMapAllocated(key.c_str(), _allocator);
        std::string val(val_ptr.get());
        meta_map[key] = val;
    }

    auto get_meta = [&](const std::string& k) -> std::string {
        auto it = meta_map.find(k);
        return (it != meta_map.end()) ? it->second : "";
    };

    std::string joint_names_str = get_meta("joint_names");
    std::string default_pos_str = get_meta("default_joint_pos");
    std::string stiffness_str   = get_meta("joint_stiffness");
    std::string damping_str     = get_meta("joint_damping");
    std::string action_scale_str = get_meta("action_scale");
    std::string body_names_str  = get_meta("body_names");
    std::string anchor_str      = get_meta("anchor_body_name");

    _model_joint_names = _parseStringList(joint_names_str);
    _model_default_dof_pos = default_pos_str.empty()
        ? std::vector<float>(NUM_DOF, 0.0f) : _parseFloatList(default_pos_str);
    _model_kp = stiffness_str.empty()
        ? std::vector<float>(NUM_DOF, 100.0f) : _parseFloatList(stiffness_str);
    _model_kd = damping_str.empty()
        ? std::vector<float>(NUM_DOF, 5.0f) : _parseFloatList(damping_str);
    _model_action_scales = action_scale_str.empty()
        ? std::vector<float>(NUM_DOF, 1.0f) : _parseFloatList(action_scale_str);
    _model_body_names = _parseStringList(body_names_str);
    _model_anchor_body_name = anchor_str;

    std::cout << "[State_WBC] Metadata: joints=" << _model_joint_names.size()
              << " bodies=" << _model_body_names.size()
              << " anchor=" << _model_anchor_body_name << std::endl;

    // Print first few joint names for verification
    std::cout << "[State_WBC] Model joint names (first 5):";
    for (int i = 0; i < std::min(5, (int)_model_joint_names.size()); i++)
        std::cout << " " << _model_joint_names[i];
    std::cout << std::endl;

    // --- Configure DataConverter with name-based reordering ---
    // robot_joint_names: bus-ordered CasBot wire names
    // joint_name_map: maps bus wire names → RoboJuDo model names
    // This lets DoFAdapter handle ALL reordering robustly via name matching.

    // Build bus-ordered name list from the constant array
    std::vector<std::string> bus_names;
    for (int i = 0; i < NUM_DOF; i++) {
        bus_names.push_back(std::string(bus_joint_names[i]));
    }

    // Build joint_name_map: bus wire name → RoboJuDo model name
    // Derived from axis comparison of both XML models
    std::map<std::string, std::string> name_map;
    name_map["leg_l1_joint"] = "left_leg_pelvic_pitch_joint";
    name_map["leg_l2_joint"] = "left_leg_pelvic_roll_joint";
    name_map["leg_l3_joint"] = "left_leg_pelvic_yaw_joint";
    name_map["leg_l4_joint"] = "left_leg_knee_pitch_joint";
    name_map["leg_l5_joint"] = "left_leg_ankle_pitch_joint";
    name_map["leg_l6_joint"] = "left_leg_ankle_roll_joint";
    name_map["leg_r1_joint"] = "right_leg_pelvic_pitch_joint";
    name_map["leg_r2_joint"] = "right_leg_pelvic_roll_joint";
    name_map["leg_r3_joint"] = "right_leg_pelvic_yaw_joint";
    name_map["leg_r4_joint"] = "right_leg_knee_pitch_joint";
    name_map["leg_r5_joint"] = "right_leg_ankle_pitch_joint";
    name_map["leg_r6_joint"] = "right_leg_ankle_roll_joint";
    // head_yaw/pitch, waist_yaw, arm joints: same names in both conventions

    BeyondMimicConfig cfg;
    cfg.joint_names           = _model_joint_names;
    cfg.default_dof_pos       = _model_default_dof_pos;
    cfg.kp                    = _model_kp;
    cfg.kd                    = _model_kd;
    cfg.action_scales         = _model_action_scales;
    cfg.body_names            = _model_body_names;
    cfg.anchor_body_name      = _model_anchor_body_name;
    cfg.without_state_estimator = true;
    cfg.use_motion_from_model   = true;
    cfg.use_residual_action     = false;
    cfg.override_robot_anchor_pos = true;
    cfg.action_beta           = _action_beta;
    cfg.clip_actions          = 100.0f;
    cfg.robot_joint_names     = bus_names;
    cfg.joint_name_map        = name_map;

    _converter.configure(cfg);

    // Pre-compute bus-order Kp/Kd using DataConverter's name-based reordering
    _bus_kp = _converter.modelToControlled(_model_kp);
    _bus_kd = _converter.modelToControlled(_model_kd);

    // Pre-compute bus→model index mapping (for NPZ path and enter() default pos)
    for (int b = 0; b < NUM_DOF; b++) {
        std::string bus_name(bus_joint_names[b]);
        auto it = name_map.find(bus_name);
        std::string model_name = (it != name_map.end()) ? it->second : bus_name;
        int found = -1;
        for (int m = 0; m < NUM_DOF; m++) {
            if (_model_joint_names[m] == model_name) { found = m; break; }
        }
        _bus_to_model_idx[b] = (found >= 0) ? found : b;
    }

    _last_action_model.assign(NUM_DOF, 0.0);
}

// ============================================================================
// ONNX observation construction (matches DataConverter::buildObservation)
// ============================================================================
void State_WBC::_observations_compute_onnx()
{
    // Robot state from IMU + motor state
    Eigen::Quaterniond anchor_quat(
        _lowState->imu.quaternion[0],
        _lowState->imu.quaternion[1],
        _lowState->imu.quaternion[2],
        _lowState->imu.quaternion[3]);
    
    Eigen::Vector3d base_ang_vel(
        static_cast<double>(_lowState->imu.gyroscope[0]),
        static_cast<double>(_lowState->imu.gyroscope[1]),
        static_cast<double>(_lowState->imu.gyroscope[2]));

    // Extract bus-order joint values (handles MotorState struct stride safely)
    float bus_q[NUM_DOF], bus_dq[NUM_DOF];
    _extractBusOrder(_lowState->motorState, bus_q, bus_dq, NUM_DOF);

    // Convert to double in bus order — DataConverter's DoFAdapter
    // handles bus→model reordering via name-based matching
    std::vector<double> dof_pos_bus(NUM_DOF), dof_vel_bus(NUM_DOF);
    for (int i = 0; i < NUM_DOF; i++) {
        dof_pos_bus[i] = static_cast<double>(bus_q[i]);
        dof_vel_bus[i] = static_cast<double>(bus_dq[i]);
    }

    // DataConverter builds the 134-dim observation
    // robot2model_ adapter internally reorders bus→model via name matching
    auto obs = _converter.buildObservation(base_ang_vel, anchor_quat,
                                             dof_pos_bus, dof_vel_bus);

    if (!_motion_initialized) {
        std::cerr << "[State_WBC] ERROR: Motion not initialized before observation!" << std::endl;
    }

    _observation = std::move(obs);
}

// ============================================================================
// ONNX action computation (matches reference pipeline)
// ============================================================================
void State_WBC::_action_compute_onnx()
{
    try {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

        float time_step = static_cast<float>(_refer_idx);
        std::vector<int64_t> obs_shape = {1, _obs_size_};
        std::vector<int64_t> ts_shape = {1, 1};

        // input names must match ONNX model
        static const char* input_names_arr[2] = {"obs", "time_step"};
        static const char* output_names_arr[5] = {
            "actions", "joint_pos", "joint_vel", "body_pos_w", "body_quat_w"
        };

        std::vector<Ort::Value> input_tensors;
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _observation.data(), _observation.size(),
            obs_shape.data(), obs_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, &time_step, 1,
            ts_shape.data(), ts_shape.size()));

        // Reuse _run_options for LSTM hidden state preservation
        auto outputs = _session->Run(
            _run_options,
            input_names_arr, input_tensors.data(), 2,
            output_names_arr, 5);

        // --- Read model outputs ---
        // outputs[0]: actions [1, N]
        float* raw_actions = outputs[0].GetTensorMutableData<float>();
        std::vector<float> actions_vec(raw_actions, raw_actions + _action_size_);

        // outputs[1]: joint_pos [1, N]
        float* out_joint_pos = outputs[1].GetTensorMutableData<float>();
        std::vector<float> model_joint_pos(out_joint_pos, out_joint_pos + NUM_DOF);

        // outputs[2]: joint_vel [1, N]
        float* out_joint_vel = outputs[2].GetTensorMutableData<float>();
        std::vector<float> model_joint_vel(out_joint_vel, out_joint_vel + NUM_DOF);

        // outputs[3]: body_pos_w [1, num_bodies, 3]
        auto body_pos_shape = outputs[3].GetTensorTypeAndShapeInfo().GetShape();
        size_t num_body_pos = body_pos_shape[1] * body_pos_shape[2];
        float* out_body_pos_w = outputs[3].GetTensorMutableData<float>();
        std::vector<float> body_pos_w(out_body_pos_w, out_body_pos_w + num_body_pos);

        // outputs[4]: body_quat_w [1, num_bodies, 4]
        auto body_quat_shape = outputs[4].GetTensorTypeAndShapeInfo().GetShape();
        size_t num_body_quat = body_quat_shape[1] * body_quat_shape[2];
        float* out_body_quat_w = outputs[4].GetTensorMutableData<float>();
        std::vector<float> body_quat_w(out_body_quat_w, out_body_quat_w + num_body_quat);

        // --- Update motion command for NEXT step's observation ---
        MotionCommand cmd;
        cmd.joint_pos   = model_joint_pos;
        cmd.joint_vel   = model_joint_vel;
        cmd.body_pos_w  = body_pos_w;
        cmd.body_quat_w = body_quat_w;
        _converter.setMotionCommand(cmd);

        if (!_warmup_done) {
            _warmup_done = true;
            std::cout << "[State_WBC] ONNX warmup complete (first inference)." << std::endl;
        }

        // --- Process action: smooth → clamp → scale ---
        auto scaled_action = _converter.processAction(actions_vec);

        // --- Compute PD targets (already in bus order via model2robot_ adapter) ---
        auto pd_targets = _converter.computePdTarget(scaled_action);

        for (int b = 0; b < NUM_DOF; b++) {
            _targetPos_rl[b] = static_cast<float>(pd_targets[b]);
        }

    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in action compute" << std::endl;
    }
}

// ============================================================================
// NPZ observation (kept for debugging — uses existing logic)
// ============================================================================
void State_WBC::_observations_compute_npz()
{
    std::vector<float> base_quat = {
        _lowState->imu.quaternion[0],
        _lowState->imu.quaternion[1],
        _lowState->imu.quaternion[2],
        _lowState->imu.quaternion[3]
    };
    std::vector<float> projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec);

    std::vector<float> dof_pos_policy(NUM_DOF);
    std::vector<float> dof_vel_policy(NUM_DOF);
    // Fill in model order: find bus index for each model joint
    for (int m = 0; m < NUM_DOF; ++m) {
        int bus = -1;
        for (int b = 0; b < NUM_DOF; b++) {
            if (_bus_to_model_idx[b] == m) { bus = b; break; }
        }
        dof_pos_policy[m] = _lowState->motorState[bus].q;
        dof_vel_policy[m] = _lowState->motorState[bus].dq;
    }

    auto body_ang_vel = std::vector<float>({
        static_cast<float>(_lowState->imu.gyroscope[0]),
        static_cast<float>(_lowState->imu.gyroscope[1]),
        static_cast<float>(_lowState->imu.gyroscope[2])
    });

    std::vector<float> ref_joint_pos(NUM_DOF);
    std::vector<float> ref_joint_vel(NUM_DOF);
    std::vector<float> anchor_ori_b(6, 0.0f);

    int idx = _refer_idx;
    if (_pause_flag) idx = _refer_idx;
    if (idx >= _end_refer_idx) idx = _end_refer_idx;
    else if (idx <= 1) idx = 1;

    ref_joint_pos = _getNPZJointPos(idx);
    ref_joint_vel = _getNPZJointVel(idx);
    if (_pause_flag) {
        std::fill(ref_joint_vel.begin(), ref_joint_vel.end(), 0.0f);
    }

    auto anchor_quat_ref = _getNPZAnchorQuat(idx);
    auto base_yaw_quat = yaw_quat(base_quat);
    auto ref_yaw_quat = yaw_quat(anchor_quat_ref);
    auto ref_yaw_quat_conj = quat_conjugate(ref_yaw_quat);
    auto yaw_quat_delta = quat_multiply(base_yaw_quat, ref_yaw_quat_conj);
    auto aligned_anchor_quat = quat_multiply(yaw_quat_delta, anchor_quat_ref);

    Eigen::Matrix3f anchor_mat = matrix_from_quat(aligned_anchor_quat);
    anchor_ori_b[0] = anchor_mat(0, 0);
    anchor_ori_b[1] = anchor_mat(0, 1);
    anchor_ori_b[2] = anchor_mat(1, 0);
    anchor_ori_b[3] = anchor_mat(1, 1);
    anchor_ori_b[4] = anchor_mat(2, 0);
    anchor_ori_b[5] = anchor_mat(2, 1);

    auto motion_projected_gravity = quat_apply_inverse(aligned_anchor_quat, _gravity_vec);
    float anchor_proj_gravity_error = std::abs(motion_projected_gravity[2] - projected_gravity[2]);
    if (anchor_proj_gravity_error > _anchor_terminate_thresh) {
        _terminate_flag = true;
        std::cout << "[Warning] Large anchor projected gravity error: "
                  << anchor_proj_gravity_error << std::endl;
    }

    _observation.clear();
    _observation.insert(_observation.end(), ref_joint_pos.begin(), ref_joint_pos.end());
    _observation.insert(_observation.end(), ref_joint_vel.begin(), ref_joint_vel.end());
    _observation.insert(_observation.end(), anchor_ori_b.begin(), anchor_ori_b.end());
    _observation.insert(_observation.end(), body_ang_vel.begin(), body_ang_vel.end());
    _observation.insert(_observation.end(), dof_pos_policy.begin(), dof_pos_policy.end());
    _observation.insert(_observation.end(), dof_vel_policy.begin(), dof_vel_policy.end());
    _observation.insert(_observation.end(), _last_action_model.size(), 0.0f);
    for (size_t i = 0; i < _last_action_model.size() && i < NUM_DOF; i++)
        _observation[NUM_DOF*2 + 6 + 3 + NUM_DOF*2 + i] = static_cast<float>(_last_action_model[i]);
}

void State_WBC::_action_compute_npz()
{
    try {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
        float time_step = static_cast<float>(_refer_idx);

        std::vector<Ort::Value> input_tensors;
        std::vector<int64_t> obs_shape = {1, _obs_size_};
        std::vector<int64_t> ts_shape = {1, 1};

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _observation.data(), _observation.size(),
            obs_shape.data(), obs_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, &time_step, 1,
            ts_shape.data(), ts_shape.size()));

        static const char* out_names[] = {"actions", "joint_pos", "joint_vel",
                                           "body_pos_w", "body_quat_w"};
        auto output_tensors = _session->Run(
            _run_options,
            nullptr, input_tensors.data(), input_tensors.size(),
            out_names, 5);

        float* actions = output_tensors[0].GetTensorMutableData<float>();
        std::vector<float> raw_action(actions, actions + _action_size_);

        // Use same processing as reference
        auto scaled_action = _converter.processAction(raw_action);
        auto pd_targets = _converter.computePdTarget(scaled_action);

        // Sync last_action for next observation
        _last_action_model = _converter.lastAction();

        for (int b = 0; b < NUM_DOF; b++) {
            _targetPos_rl[b] = static_cast<float>(pd_targets[b]);
        }

    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    }
}

// ============================================================================
// FSM interface
// ============================================================================
void State_WBC::enter()
{
    _pause_flag = false;
    _terminate_flag = false;
    _pause_curr_flag = false;
    _refer_idx = _start_refer_idx;
    _last_refer_idx = _refer_idx;

    if (_pause_refer_idx < 0) _pause_refer_idx = 0;

    int total_frames = (_data_source == "onnx") ? _total_frames : _motion_frame_count;
    if (_end_refer_idx < 0 || _end_refer_idx < _start_refer_idx) {
        if (_end_refer_idx < _start_refer_idx) {
            std::cout << "[WARNING]: end_idx is smaller than start_idx, "
                         "defaulting to motion length." << std::endl;
        }
        _end_refer_idx = total_frames - 1;
    }

    // Initialize motor commands
    for (int i = 0; i < NUM_DOF; i++) {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = 0;
        _lowCmd->motorCmd[i].Kp = _bus_kp[i];
        _lowCmd->motorCmd[i].Kd = _bus_kd[i];
        _targetPos_rl[i] = _converter.modelToControlled(_model_default_dof_pos)[i];
        _last_targetPos_rl[i] = _lowState->motorState[i].q;
    }

    // ONNX mode: dry run for motion initialization and anchor alignment
    if (_data_source == "onnx") {
        _last_action_model.assign(NUM_DOF, 0.0);

        // Dry run: zero obs at timestep=0 to get initial model output
        try {
            auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

            // Build zero observation
            std::vector<float> zero_obs(_obs_size_, 0.0f);
            // Fill default joint pos offset = 0 (qpos == default at start)
            // Fill command with zeros (no prior command)
            // NOTE: anchor_ori stays all zeros → identity rotation

            float time_step = 0.0f;
            std::vector<int64_t> obs_shape = {1, _obs_size_};
            std::vector<int64_t> ts_shape = {1, 1};

            std::vector<Ort::Value> inputs;
            inputs.push_back(Ort::Value::CreateTensor<float>(
                memory_info, zero_obs.data(), zero_obs.size(),
                obs_shape.data(), obs_shape.size()));
            inputs.push_back(Ort::Value::CreateTensor<float>(
                memory_info, &time_step, 1,
                ts_shape.data(), ts_shape.size()));

            static const char* in_names[] = {"obs", "time_step"};
            static const char* out_names[] = {"actions", "joint_pos", "joint_vel",
                                               "body_pos_w", "body_quat_w"};

            auto outputs = _session->Run(
                _run_options,
                in_names, inputs.data(), 2,
                out_names, 5);

            // Parse model output for motion initialization
            std::vector<float> init_joint_pos(NUM_DOF);
            std::vector<float> init_joint_vel(NUM_DOF, 0.0f);
            std::vector<float> init_body_pos;
            std::vector<float> init_body_quat;
            std::vector<float> init_actions;

            if (outputs.size() > 0 && outputs[0].IsTensor()) {
                float* d = outputs[0].GetTensorMutableData<float>();
                auto s = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
                init_actions.assign(d, d + s[1]);
            }
            if (outputs.size() > 1 && outputs[1].IsTensor()) {
                float* d = outputs[1].GetTensorMutableData<float>();
                auto s = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
                init_joint_pos.assign(d, d + s[1]);
            }
            if (outputs.size() > 2 && outputs[2].IsTensor()) {
                float* d = outputs[2].GetTensorMutableData<float>();
                auto s = outputs[2].GetTensorTypeAndShapeInfo().GetShape();
                init_joint_vel.assign(d, d + s[1]);
            }
            if (outputs.size() > 3 && outputs[3].IsTensor()) {
                float* d = outputs[3].GetTensorMutableData<float>();
                auto s = outputs[3].GetTensorTypeAndShapeInfo().GetShape();
                size_t n = s[1] * s[2];
                init_body_pos.assign(d, d + n);
            }
            if (outputs.size() > 4 && outputs[4].IsTensor()) {
                float* d = outputs[4].GetTensorMutableData<float>();
                auto s = outputs[4].GetTensorTypeAndShapeInfo().GetShape();
                size_t n = s[1] * s[2];
                init_body_quat.assign(d, d + n);
            }

            // Set motion command for next step
            MotionCommand init_cmd;
            init_cmd.joint_pos   = init_joint_pos;
            init_cmd.joint_vel   = init_joint_vel;
            init_cmd.body_pos_w  = init_body_pos;
            init_cmd.body_quat_w = init_body_quat;
            _converter.setMotionCommand(init_cmd);

            // Setup anchor alignment from first model-predicted anchor pose
            int anchor_idx = _converter.config().anchor_body_index;
            if (anchor_idx >= 0 && anchor_idx * 3 + 2 < (int)init_body_pos.size()) {
                Eigen::Vector3f anchor_pos(
                    init_cmd.bodyPos(anchor_idx));
                Eigen::Quaternionf anchor_quat_ref(
                    init_cmd.bodyQuat(anchor_idx));
                _converter.initAlignment().setBase(anchor_quat_ref, anchor_pos, true, true);
                std::cout << "[State_WBC] Init alignment set from model anchor body idx="
                          << anchor_idx << std::endl;
            } else {
                std::cout << "[State_WBC] WARNING: anchor body not found in model output,"
                          << " using identity alignment" << std::endl;
            }

            // Seed last_action with initial actions
            _converter.processAction(init_actions);

            _motion_initialized = true;
            _warmup_done = false;  // Will be set after first real inference
            std::cout << "[State_WBC] Dry run complete, motion initialized." << std::endl;

        } catch (const Ort::Exception& e) {
            std::cerr << "[State_WBC] Dry run error: " << e.what() << std::endl;
        }
    }
}

void State_WBC::run()
{
    if (_pause_flag && !_pause_curr_flag) {
        _refer_idx = _pause_refer_idx;
    }

    if (_data_source == "onnx") {
        _observations_compute_onnx();
        _action_compute_onnx();
    } else {
        _observations_compute_npz();
        _action_compute_npz();
    }

    for (int j = 0; j < NUM_DOF; j++) {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = _targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].Kp = _bus_kp[j];
        _lowCmd->motorCmd[j].Kd = _bus_kd[j];
        _last_targetPos_rl[j] = _targetPos_rl[j];
    }
    _last_refer_idx = _refer_idx;

    if (!_pause_flag) {
        _refer_idx++;
    }
    if (_refer_idx >= _end_refer_idx) {
        _refer_idx = _end_refer_idx;
    }

    std::string pause_string = _pause_flag ? " | Press R1 to resume..." : " | Press R2 to pause...";
    std::cout << "\r[State_WBC] Running. Refer idx: " << _refer_idx
              << "/" << _end_refer_idx << pause_string << std::flush;
}

void State_WBC::exit()
{
    std::cout << "\n[State_WBC] Exiting WBC state." << std::endl;
}

FSMStateName State_WBC::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    else if (_terminate_flag) {
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R2_A) {
        return FSMStateName::FIXEDSTAND;
    }
    else if (_lowState->userCmd == UserCommand::R2 && !_pause_flag) {
        _pause_flag = true;
        std::cout << std::endl << "WBC Pause" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::L2 && !_pause_flag) {
        _pause_flag = true;
        _pause_curr_flag = true;
        std::cout << std::endl << "WBC Pause" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::R1 && _pause_flag) {
        _pause_flag = false;
        _pause_curr_flag = false;
        std::cout << std::endl << "WBC Resume" << std::endl;
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::SELECT) {
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else {
        return FSMStateName::WBC;
    }
}
