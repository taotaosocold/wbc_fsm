#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Motion data snapshot — stored from model output, fed as "command" next step
// ---------------------------------------------------------------------------
struct MotionCommand {
  std::vector<float> joint_pos;
  std::vector<float> joint_vel;
  std::vector<float> body_pos_w;   // [num_bodies * 3]
  std::vector<float> body_quat_w;  // [num_bodies * 4]  w,x,y,z

  Eigen::Vector3f bodyPos(int idx) const {
    return Eigen::Vector3f(body_pos_w[idx * 3], body_pos_w[idx * 3 + 1],
                           body_pos_w[idx * 3 + 2]);
  }
  Eigen::Quaternionf bodyQuat(int idx) const {
    return Eigen::Quaternionf(body_quat_w[idx * 4], body_quat_w[idx * 4 + 1],
                              body_quat_w[idx * 4 + 2], body_quat_w[idx * 4 + 3]);
  }
  bool empty() const { return joint_pos.empty(); }
};

// ---------------------------------------------------------------------------
// DoFAdapter — name-based joint reordering
// ---------------------------------------------------------------------------
class DoFAdapter {
 public:
  DoFAdapter() = default;

  void build(const std::vector<std::string>& src_names,
             const std::vector<std::string>& tar_names,
             const std::map<std::string, std::string>& name_map = {});

  std::vector<float> fit(const std::vector<float>& data,
                         const std::vector<float>& template_vals = {}) const;
  std::vector<double> fit(const std::vector<double>& data,
                          const std::vector<double>& template_vals = {}) const;

  int srcLen() const { return src_len_; }
  int tarLen() const { return tar_len_; }

 private:
  std::vector<int> src_indices_;
  std::vector<int> tar_indices_;
  int src_len_ = 0;
  int tar_len_ = 0;
};

// ---------------------------------------------------------------------------
// Config for BeyondMimic data pipeline
// ---------------------------------------------------------------------------
struct BeyondMimicConfig {
  std::vector<std::string> joint_names;
  std::vector<float> default_dof_pos;
  std::vector<float> kp;
  std::vector<float> kd;
  std::vector<float> action_scales;

  std::vector<std::string> robot_joint_names;
  std::map<std::string, std::string> joint_name_map;

  std::string anchor_body_name;
  std::vector<std::string> body_names;
  int anchor_body_index = 0;

  bool without_state_estimator = true;
  bool use_motion_from_model = true;
  bool use_residual_action = false;
  bool override_robot_anchor_pos = true;

  float action_beta = 1.0f;
  float clip_actions = 100.0f;

  int start_timestep = 0;
  int max_timestep = -1;

  int numDof() const { return static_cast<int>(joint_names.size()); }
  int numBodies() const { return static_cast<int>(body_names.size()); }
  int numRobotDof() const {
    return robot_joint_names.empty() ? numDof()
                                     : static_cast<int>(robot_joint_names.size());
  }
  void setAnchorBodyIndex() {
    for (size_t i = 0; i < body_names.size(); i++) {
      if (body_names[i] == anchor_body_name) {
        anchor_body_index = static_cast<int>(i);
        return;
      }
    }
    anchor_body_index = 0;
  }
};

// ---------------------------------------------------------------------------
// TransformAlignment — yaw-only + xy-only alignment
// ---------------------------------------------------------------------------
class TransformAlignment {
 public:
  TransformAlignment() = default;

  void setBase(const Eigen::Quaternionf& quat, const Eigen::Vector3f& pos,
               bool yaw_only = true, bool xy_only = true);

  Eigen::Quaternionf alignQuat(const Eigen::Quaternionf& q) const;
  Eigen::Vector3f alignPos(const Eigen::Vector3f& p) const;
  void alignTransform(Eigen::Quaternionf& q, Eigen::Vector3f& p) const;

  // Double-precision overloads for observation construction
  Eigen::Quaterniond alignQuatD(const Eigen::Quaterniond& q) const;
  Eigen::Vector3d alignPosD(const Eigen::Vector3d& p) const;
  void alignTransformD(Eigen::Quaterniond& q, Eigen::Vector3d& p) const;

 private:
  Eigen::Quaterniond R_base_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_base_ = Eigen::Vector3d::Zero();
  bool yaw_only_ = true;
  bool xy_only_ = true;
};

// ---------------------------------------------------------------------------
// DataConverter — builds BeyondMimic observations, processes actions
// ---------------------------------------------------------------------------
class DataConverter {
 public:
  DataConverter() = default;

  void configure(const BeyondMimicConfig& cfg);

  /// Build observation from pre-extracted state in controlled-joint order.
  std::vector<float> buildObservation(const Eigen::Vector3d& base_ang_vel,
                                      const Eigen::Quaterniond& anchor_quat,
                                      const std::vector<double>& dof_pos_ctrl,
                                      const std::vector<double>& dof_vel_ctrl,
                                      const Eigen::Vector3d& anchor_pos =
                                          Eigen::Vector3d::Zero());

  /// Process raw model action through smoothing + per-joint scaling.
  std::vector<double> processAction(const std::vector<float>& raw_action);

  /// Compute PD target in controlled-joint order.
  std::vector<double> computePdTarget(const std::vector<double>& scaled_action);

  /// Store motion data from model output for next step's observation.
  void setMotionCommand(const MotionCommand& cmd) { command_ = cmd; }
  const MotionCommand& motionCommand() const { return command_; }

  /// Access last smoothed action (for manual observation construction, e.g. NPZ mode).
  const std::vector<float>& lastAction() const { return last_action_; }

  BeyondMimicConfig& config() { return cfg_; }
  const BeyondMimicConfig& config() const { return cfg_; }

  TransformAlignment& initAlignment() { return init_align_; }

  int numDof() const { return cfg_.numDof(); }

  std::vector<float> modelToControlled(const std::vector<float>& model_data) const {
    return model2robot_.fit(model_data);
  }
  std::vector<double> modelToControlled(const std::vector<double>& model_data) const {
    return model2robot_.fit(model_data);
  }

  int numControlled() const {
    return cfg_.robot_joint_names.empty()
               ? cfg_.numDof()
               : static_cast<int>(cfg_.robot_joint_names.size());
  }

 private:
  BeyondMimicConfig cfg_;
  MotionCommand command_;
  std::vector<float> last_action_;
  TransformAlignment init_align_;

  DoFAdapter robot2model_;
  DoFAdapter model2robot_;
};
