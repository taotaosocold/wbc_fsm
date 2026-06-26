#include "common/data_converter.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// ============================================================================
// DoFAdapter
// ============================================================================
void DoFAdapter::build(const std::vector<std::string>& src_names,
                        const std::vector<std::string>& tar_names,
                        const std::map<std::string, std::string>& name_map) {
  src_len_ = static_cast<int>(src_names.size());
  tar_len_ = static_cast<int>(tar_names.size());

  src_indices_.clear();
  tar_indices_.clear();

  std::map<std::string, int> tar_index;
  for (int j = 0; j < tar_len_; j++) {
    tar_index[tar_names[j]] = j;
  }

  for (int si = 0; si < src_len_; si++) {
    const std::string& src_name = src_names[si];

    auto map_it = name_map.find(src_name);
    if (map_it != name_map.end()) {
      auto ti = tar_index.find(map_it->second);
      if (ti != tar_index.end()) {
        src_indices_.push_back(si);
        tar_indices_.push_back(ti->second);
        continue;
      }
    }

    auto ti = tar_index.find(src_name);
    if (ti != tar_index.end()) {
      src_indices_.push_back(si);
      tar_indices_.push_back(ti->second);
    }
  }

  std::cout << "[DoFAdapter] Mapped " << src_indices_.size()
            << " / " << src_len_ << " -> " << tar_len_
            << " joints (name_map has " << name_map.size() << " entries)"
            << std::endl;
}

std::vector<float> DoFAdapter::fit(const std::vector<float>& data,
                                    const std::vector<float>& template_vals) const {
  std::vector<float> result;
  if (!template_vals.empty()) {
    result = template_vals;
  } else {
    result.assign(tar_len_, 0.0f);
  }
  int n = static_cast<int>(src_indices_.size());
  for (int k = 0; k < n; k++) {
    int si = src_indices_[k];
    int ti = tar_indices_[k];
    if (si < static_cast<int>(data.size())) {
      result[ti] = data[si];
    }
  }
  return result;
}

std::vector<double> DoFAdapter::fit(const std::vector<double>& data,
                                     const std::vector<double>& template_vals) const {
  std::vector<double> result;
  if (!template_vals.empty()) {
    result = template_vals;
  } else {
    result.assign(tar_len_, 0.0);
  }
  int n = static_cast<int>(src_indices_.size());
  for (int k = 0; k < n; k++) {
    int si = src_indices_[k];
    int ti = tar_indices_[k];
    if (si < static_cast<int>(data.size())) {
      result[ti] = data[si];
    }
  }
  return result;
}

// ============================================================================
// TransformAlignment
// ============================================================================
void TransformAlignment::setBase(const Eigen::Quaternionf& quat,
                                  const Eigen::Vector3f& pos,
                                  bool yaw_only, bool xy_only) {
  yaw_only_ = yaw_only;
  xy_only_  = xy_only;

  if (yaw_only_) {
    // Extract yaw via Eigen rotation matrix
    Eigen::Matrix3d R = quat.cast<double>().toRotationMatrix();
    double yaw = std::atan2(R(1, 0), R(0, 0));
    R_base_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  } else {
    R_base_ = quat.cast<double>();
  }

  if (xy_only_) {
    p_base_ = Eigen::Vector3d(pos.x(), pos.y(), 0.0);
  } else {
    p_base_ = pos.cast<double>();
  }
}

Eigen::Quaternionf TransformAlignment::alignQuat(const Eigen::Quaternionf& q) const {
  Eigen::Quaterniond qd = q.cast<double>();
  Eigen::Quaterniond result = R_base_.inverse() * qd;
  return result.cast<float>();
}

Eigen::Vector3f TransformAlignment::alignPos(const Eigen::Vector3f& p) const {
  Eigen::Vector3d pd = p.cast<double>();
  Eigen::Vector3d result = R_base_.inverse() * (pd - p_base_);
  return result.cast<float>();
}

void TransformAlignment::alignTransform(Eigen::Quaternionf& q,
                                         Eigen::Vector3f& p) const {
  q = alignQuat(q);
  p = alignPos(p);
}

Eigen::Quaterniond TransformAlignment::alignQuatD(const Eigen::Quaterniond& q) const {
  return R_base_.inverse() * q;
}

Eigen::Vector3d TransformAlignment::alignPosD(const Eigen::Vector3d& p) const {
  return R_base_.inverse() * (p - p_base_);
}

void TransformAlignment::alignTransformD(Eigen::Quaterniond& q,
                                          Eigen::Vector3d& p) const {
  q = alignQuatD(q);
  p = alignPosD(p);
}

// ============================================================================
// DataConverter
// ============================================================================
void DataConverter::configure(const BeyondMimicConfig& cfg) {
  cfg_ = cfg;
  cfg_.setAnchorBodyIndex();
  last_action_.assign(cfg.numDof(), 0.0);

  const auto& controlled_names = cfg_.robot_joint_names.empty()
                                     ? cfg_.joint_names
                                     : cfg_.robot_joint_names;

  if (cfg_.robot_joint_names.empty()) {
    std::cout << "[DataConverter] No robot_joint_names set, assuming 1:1 "
              << "mapping to model joint order (" << cfg_.numDof() << " DOF)"
              << std::endl;
  } else {
    // Build inverse map (model_name → bus_name) for model2robot direction
    std::map<std::string, std::string> inv_name_map;
    for (const auto& [k, v] : cfg_.joint_name_map) {
      inv_name_map[v] = k;
    }
    robot2model_.build(controlled_names, cfg_.joint_names, cfg_.joint_name_map);
    model2robot_.build(cfg_.joint_names, controlled_names, inv_name_map);
  }

  std::cout << "[DataConverter] Configured: " << cfg_.numDof() << " DOF, "
            << cfg_.numBodies() << " bodies, anchor=" << cfg_.anchor_body_name
            << " (idx=" << cfg_.anchor_body_index << ")"
            << ", without_state_estimator=" << cfg_.without_state_estimator
            << ", use_motion_from_model=" << cfg_.use_motion_from_model
            << ", use_residual_action=" << cfg_.use_residual_action
            << ", action_beta=" << cfg_.action_beta << std::endl;
}

std::vector<float> DataConverter::buildObservation(
    const Eigen::Vector3d& base_ang_vel,
    const Eigen::Quaterniond& anchor_quat,
    const std::vector<double>& dof_pos_ctrl,
    const std::vector<double>& dof_vel_ctrl,
    const Eigen::Vector3d& anchor_pos) {

  int N = cfg_.numDof();

  // Reorder from controlled -> model order
  std::vector<double> dof_pos_model = robot2model_.fit(dof_pos_ctrl);
  std::vector<double> dof_vel_model = robot2model_.fit(dof_vel_ctrl);

  std::vector<float> obs;

  // --- 1. Command (reference motion joint_pos + joint_vel, MODEL order) ---
  if (command_.empty()) {
    obs.insert(obs.end(), N * 2, 0.0f);
  } else {
    for (int i = 0; i < N; i++) obs.push_back(command_.joint_pos[i]);
    for (int i = 0; i < N; i++) obs.push_back(command_.joint_vel[i]);
  }

  // --- 2. Motion anchor orientation (6 values) ---
  if (!command_.empty() && cfg_.numBodies() > 0 && cfg_.anchor_body_index >= 0) {
    Eigen::Vector3d anchor_pos_w =
        command_.bodyPos(cfg_.anchor_body_index).cast<double>();
    Eigen::Quaterniond anchor_quat_w =
        command_.bodyQuat(cfg_.anchor_body_index).cast<double>();

    init_align_.alignTransformD(anchor_quat_w, anchor_pos_w);

    // override_robot_anchor_pos: use motion's anchor pos instead of robot's
    Eigen::Quaterniond robot_anchor_quat = anchor_quat;
    Eigen::Vector3d robot_anchor_pos =
        cfg_.override_robot_anchor_pos ? anchor_pos_w : anchor_pos;

    // rel_pos — skipped when without_state_estimator=true
    if (!cfg_.without_state_estimator) {
      obs.insert(obs.end(), 3, 0.0f);
    }

    // Relative rotation: q_rel = robot_anchor_quat.conjugate() * anchor_quat_w
    Eigen::Quaternionf q_rel = robot_anchor_quat.cast<float>().conjugate() * anchor_quat_w.cast<float>();
    Eigen::Matrix3f R = q_rel.toRotationMatrix();
    // First 2 columns in row-major order: R00,R01,R10,R11,R20,R21
    obs.push_back(static_cast<float>(R(0, 0)));
    obs.push_back(static_cast<float>(R(0, 1)));
    obs.push_back(static_cast<float>(R(1, 0)));
    obs.push_back(static_cast<float>(R(1, 1)));
    obs.push_back(static_cast<float>(R(2, 0)));
    obs.push_back(static_cast<float>(R(2, 1)));
  } else {
    if (!cfg_.without_state_estimator) {
      obs.insert(obs.end(), 3, 0.0f);
    }
    obs.insert(obs.end(), 6, 0.0f);
  }

  // --- 3. Base angular velocity (3) ---
  obs.push_back(static_cast<float>(base_ang_vel.x()));
  obs.push_back(static_cast<float>(base_ang_vel.y()));
  obs.push_back(static_cast<float>(base_ang_vel.z()));

  // --- 4. Joint position relative to default (N, MODEL order) ---
  for (int i = 0; i < N; i++) {
    double qpos = dof_pos_model[i];
    double def  = static_cast<double>(cfg_.default_dof_pos[i]);
    obs.push_back(static_cast<float>(qpos - def));
  }

  // --- 5. Joint velocity (N, MODEL order) ---
  for (int i = 0; i < N; i++) {
    obs.push_back(static_cast<float>(dof_vel_model[i]));
  }

  // --- 6. Last action (N, MODEL order) ---
  for (int i = 0; i < N; i++) {
    obs.push_back(static_cast<float>(last_action_[i]));
  }

  return obs;
}

std::vector<double> DataConverter::processAction(
    const std::vector<float>& raw_action) {
  int N = cfg_.numDof();
  float beta = cfg_.action_beta;
  float clip = cfg_.clip_actions;
  std::vector<float> smoothed(N);

  for (int i = 0; i < N; i++) {
    smoothed[i] = (1.0f - beta) * last_action_[i]
                  + beta * raw_action[i];
  }

  std::vector<double> scaled(N);
  for (int i = 0; i < N; i++) {
    float a = std::clamp(smoothed[i], -clip, clip);
    scaled[i] = static_cast<double>(a * cfg_.action_scales[i]);
  }

  last_action_ = std::move(smoothed);
  return scaled;
}

std::vector<double> DataConverter::computePdTarget(
    const std::vector<double>& scaled_action) {
  int N = cfg_.numDof();

  std::vector<double> target_model(N);
  if (cfg_.use_residual_action && !command_.empty()) {
    for (int i = 0; i < N; i++) {
      target_model[i] = scaled_action[i]
                      + static_cast<double>(command_.joint_pos[i]);
    }
  } else {
    for (int i = 0; i < N; i++) {
      target_model[i] = static_cast<double>(cfg_.default_dof_pos[i])
                      + scaled_action[i];
    }
  }

  if (cfg_.robot_joint_names.empty()) {
    return target_model;
  }
  return model2robot_.fit(target_model);
}
