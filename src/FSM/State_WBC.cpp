#include <iostream>
#include "FSM/State_WBC.h"
#include "common/read_traj.h"
#include "common/npz_reader.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

State_WBC::State_WBC(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::WBC, "wbc"){

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
    _ref_body_quat_w = std::vector<float>(NUM_BODIES * 4, 0.0f);

    _loadPolicy();
}

void State_WBC::_loadPolicy()
{
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

    Ort::TypeInfo input0_type = _session->GetInputTypeInfo(0);
    auto input0_shapes = input0_type.GetTensorTypeAndShapeInfo().GetShape();
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape();

    _obs_size_ = input0_shapes[1];
    _action_size_ = output_shapes[1];
    _action = std::vector<float>(_action_size_, 0.0f);

    std::cout << "[State_WBC] Model loaded. obs_size=" << _obs_size_
              << " action_size=" << _action_size_ << std::endl;
}

void State_WBC::_observations_compute()
{
    // --- base quaternion and projected gravity ---
    std::vector<float> base_quat = {
        _lowState->imu.quaternion[0],
        _lowState->imu.quaternion[1],
        _lowState->imu.quaternion[2],
        _lowState->imu.quaternion[3]
    };
    std::vector<float> projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec);

    // --- robot joint state in POLICY order ---
    std::vector<float> dof_pos_policy(NUM_DOF);
    std::vector<float> dof_vel_policy(NUM_DOF);
    for (int p = 0; p < NUM_DOF; ++p) {
        int bus = dof_mapping[p];
        dof_pos_policy[p] = _lowState->motorState[bus].q - this->_default_dof_pos[bus];
        dof_vel_policy[p] = _lowState->motorState[bus].dq;
    }

    // --- base angular velocity ---
    auto body_ang_vel = std::vector<float>({
        static_cast<float>(_lowState->imu.gyroscope[0]),
        static_cast<float>(_lowState->imu.gyroscope[1]),
        static_cast<float>(_lowState->imu.gyroscope[2])
    });

    // --- command and anchor orientation: depends on data source ---
    std::vector<float> ref_joint_pos(NUM_DOF);
    std::vector<float> ref_joint_vel(NUM_DOF);
    std::vector<float> anchor_ori_b(6, 0.0f);

    if (_data_source == "npz") {
        auto get_ref_joint_pos = [this](int frame_idx) -> std::vector<float> {
            int num_dofs = _joint_pos_shape[1];
            int base = frame_idx * num_dofs;
            std::vector<float> pos(num_dofs);
            for (int i = 0; i < num_dofs; ++i)
                pos[i] = _joint_pos[base + i];
            return pos;
        };
        auto get_ref_joint_vel = [this](int frame_idx) -> std::vector<float> {
            int num_dofs = _joint_vel_shape[1];
            int base = frame_idx * num_dofs;
            std::vector<float> vel(num_dofs);
            for (int i = 0; i < num_dofs; ++i)
                vel[i] = _joint_vel[base + i];
            return vel;
        };
        auto get_anchor_quat = [this](int frame_idx) -> std::vector<float> {
            int num_links = _body_quat_w_shape[1];
            int base = frame_idx * num_links * 4 + _anchor_idx * 4;
            return {_body_quat_w[base], _body_quat_w[base + 1],
                    _body_quat_w[base + 2], _body_quat_w[base + 3]};
        };

        int idx = _refer_idx;
        if (_pause_flag) idx = _refer_idx;
        if (idx >= _end_refer_idx) idx = _end_refer_idx;
        else if (idx <= 1) idx = 1;

        ref_joint_pos = get_ref_joint_pos(idx);
        ref_joint_vel = get_ref_joint_vel(idx);
        if (_pause_flag) {
            std::fill(ref_joint_vel.begin(), ref_joint_vel.end(), 0.0f);
        }

        auto anchor_quat_ref = get_anchor_quat(idx);
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

        // safety check
        auto motion_projected_gravity = quat_apply_inverse(aligned_anchor_quat, _gravity_vec);
        float anchor_proj_gravity_error = std::abs(motion_projected_gravity[2] - projected_gravity[2]);
        if (anchor_proj_gravity_error > _anchor_terminate_thresh) {
            _terminate_flag = true;
            std::cout << "[Warning] Large anchor projected gravity error: "
                      << anchor_proj_gravity_error << std::endl;
        }

    } else if (_data_source == "onnx") {
        // autoregressive: use model's previous output as reference
        ref_joint_pos = _ref_joint_pos;
        ref_joint_vel = _ref_joint_vel;

        // Compute anchor orientation from model's body_quat_w output
        if (_warmup_done) {
            int bo = _anchor_body_idx * 4;
            std::vector<float> anchor_quat_ref = {
                _ref_body_quat_w[bo + 0], _ref_body_quat_w[bo + 1],
                _ref_body_quat_w[bo + 2], _ref_body_quat_w[bo + 3]
            };

            // Align reference anchor yaw with robot base yaw
            auto base_yaw_quat = yaw_quat(base_quat);
            auto ref_yaw_quat = yaw_quat(anchor_quat_ref);
            auto ref_yaw_quat_conj = quat_conjugate(ref_yaw_quat);
            auto yaw_quat_delta = quat_multiply(base_yaw_quat, ref_yaw_quat_conj);
            auto aligned_anchor_quat = quat_multiply(yaw_quat_delta, anchor_quat_ref);

            // Relative orientation: base_quat_inv * aligned_anchor_quat
            auto base_quat_inv = quat_inv(base_quat);
            auto q_rel = quat_mul(base_quat_inv, aligned_anchor_quat);
            Eigen::Matrix3f rel_mat = matrix_from_quat(q_rel);

            anchor_ori_b[0] = rel_mat(0, 0);
            anchor_ori_b[1] = rel_mat(0, 1);
            anchor_ori_b[2] = rel_mat(1, 0);
            anchor_ori_b[3] = rel_mat(1, 1);
            anchor_ori_b[4] = rel_mat(2, 0);
            anchor_ori_b[5] = rel_mat(2, 1);
        }
        // else: first step, anchor_ori_b stays all zeros (identity rotation)
    }

    // --- assemble observation: command(50) + anchor_ori_b(6) + ang_vel(3) + dof_pos(25) + dof_vel(25) + action(25) = 134 ---
    _observation.clear();
    _observation.insert(_observation.end(), ref_joint_pos.begin(), ref_joint_pos.end());   // 25
    _observation.insert(_observation.end(), ref_joint_vel.begin(), ref_joint_vel.end());   // 25 → 50
    _observation.insert(_observation.end(), anchor_ori_b.begin(), anchor_ori_b.end());     // 6  → 56
    _observation.insert(_observation.end(), body_ang_vel.begin(), body_ang_vel.end());     // 3  → 59
    _observation.insert(_observation.end(), dof_pos_policy.begin(), dof_pos_policy.end()); // 25 → 84
    _observation.insert(_observation.end(), dof_vel_policy.begin(), dof_vel_policy.end()); // 25 → 109
    _observation.insert(_observation.end(), _action.begin(), _action.end());               // 25 → 134

    for (size_t i = 0; i < _observation.size(); i++) {
        _observation[i] = std::max(-clip_observations,
                                   std::min(_observation[i], clip_observations));
    }
}

void State_WBC::_action_compute()
{
    try {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

        // time_step is the integer frame index (matching Python: int(self.timestep))
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

        auto output_tensors = _session->Run(
            Ort::RunOptions{nullptr},
            _input_names.data(), input_tensors.data(), input_tensors.size(),
            _output_names_all.data(), _output_names_all.size());

        // read actions (model order)
        float *actions = output_tensors[0].GetTensorMutableData<float>();
        std::memcpy(_action.data(), actions, _action.size() * sizeof(float));

        for (int p = 0; p < NUM_DOF; p++) {
            _action[p] = std::max(-clip_actions, std::min(_action[p], clip_actions));
            // EMA smoothing and scale with per-joint action_scale
            float smoothed = (1.0f - action_beta) * _last_action[p] + action_beta * _action[p];
            _action[p] = smoothed;
            _last_action[p] = smoothed;

            // Scale action and add residual (model output joint_pos as reference)
            int bus = dof_mapping[p];
            float scaled = smoothed * _action_scale[bus] + _ref_joint_pos[p];
            this->_joint_q[bus] = scaled + _default_dof_pos[bus];
        }

        // ONNX mode: store model-generated reference for next frame
        if (_data_source == "onnx" && output_tensors.size() >= 5) {
            float *out_joint_pos = output_tensors[1].GetTensorMutableData<float>();
            float *out_joint_vel = output_tensors[2].GetTensorMutableData<float>();
            // outputs[3]: body_pos_w [1, 14, 3]
            float *out_body_quat_w = output_tensors[4].GetTensorMutableData<float>(); // [1, 14, 4]

            std::memcpy(_ref_joint_pos.data(), out_joint_pos, NUM_DOF * sizeof(float));
            std::memcpy(_ref_joint_vel.data(), out_joint_vel, NUM_DOF * sizeof(float));
            std::memcpy(_ref_body_quat_w.data(), out_body_quat_w, NUM_BODIES * 4 * sizeof(float));

            if (!_warmup_done) {
                _warmup_done = true;
                std::cout << "[State_WBC] ONNX warmup complete." << std::endl;
            }
        }
    }
    catch (const Ort::Exception &e) {
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
    }
}

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

    // Initialize autoregressive reference (ONNX mode)
    if (_data_source == "onnx") {
        _action = std::vector<float>(_action_size_, 0.0f);
        _last_action.assign(NUM_DOF, 0.0f);
        for (int p = 0; p < NUM_DOF; ++p) {
            _ref_joint_pos[p] = 0.0f;
            _ref_joint_vel[p] = 0.0f;
        }
        _warmup_done = false;
    }

    for (int i = 0; i < NUM_DOF; i++) {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = 0;
        _lowCmd->motorCmd[i].Kp = this->dof_Kps[i];
        _lowCmd->motorCmd[i].Kd = this->dof_Kds[i];
        this->_targetPos_rl[i] = this->_default_dof_pos[i];
        this->_last_targetPos_rl[i] = _lowState->motorState[i].q;
        this->_joint_q[i] = this->_default_dof_pos[i];
    }
}

void State_WBC::run()
{
    if (!_pause_flag) {
        _refer_idx++;
    } else {
        if (!_pause_curr_flag)
            _refer_idx = _pause_refer_idx;
    }
    if (_refer_idx >= _end_refer_idx) {
        _refer_idx = _end_refer_idx;
    }
    _observations_compute();
    _action_compute();
    memcpy(this->_targetPos_rl, this->_joint_q, sizeof(this->_joint_q));

    for (int j = 0; j < NUM_DOF; j++) {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = _targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].Kp = this->dof_Kps[j];
        _lowCmd->motorCmd[j].Kd = this->dof_Kds[j];
        this->_last_targetPos_rl[j] = _targetPos_rl[j];
    }
    _last_refer_idx = _refer_idx;
    std::string pause_string = _pause_flag ? " | Press R1 to resume..." : " | Press R2 to pause...";
    std::cout << "\r[State_WBC] Running. Refer idx: " << _refer_idx
              << "/" << _end_refer_idx << pause_string << std::flush;
}

void State_WBC::exit()
{
    std::cout << "[State_WBC] Exiting WBC state." << std::endl;
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
