#include "omx_variable_stiffness_controller/omx_variable_stiffness_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace omx_variable_stiffness_controller
{

// Helper: Convert KDL::Vector to a string for debugging
static std::string vector_to_string(const KDL::Vector & v)
{
  std::stringstream ss;
  ss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return ss.str();
}

controller_interface::InterfaceConfiguration
OmxVariableStiffnessController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_POSITION));
    cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_VELOCITY));
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
OmxVariableStiffnessController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
  }
  return cfg;
}

controller_interface::CallbackReturn
OmxVariableStiffnessController::on_init()
{
  // Declare all parameters with defaults
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("root_link", "world");
  auto_declare<std::string>("tip_link", "");
  auto_declare<std::string>("robot_description_node", "/controller_manager");
  auto_declare<double>("torque_scale", 1.0);

  // Trajectory timing parameters
  auto_declare<double>("homing_duration", 5.0);
  auto_declare<double>("move_duration", 10.0);
  auto_declare<double>("wait_duration", 2.0);
  auto_declare<double>("max_rise_rate", 100.0);

  // Position waypoints (3-element arrays)
  auto_declare<std::vector<double>>("start_position", {0.2, 0.0, 0.15});
  auto_declare<std::vector<double>>("end_position", {0.25, 0.0, 0.15});
  auto_declare<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});

  // Stiffness/damping parameters (3-element arrays)
  auto_declare<std::vector<double>>("stiffness_homing", {200.0, 200.0, 200.0});
  auto_declare<std::vector<double>>("stiffness_rot", {50.0, 50.0, 50.0});
  auto_declare<std::vector<double>>("damping_rot", {5.0, 5.0, 5.0});
  auto_declare<std::vector<double>>("damping_default", {20.0, 20.0, 20.0});

  // Stiffness/damping profiles (loaded from CSV or parameter server)
  auto_declare<std::vector<double>>("stiffness_profile_x", {});
  auto_declare<std::vector<double>>("stiffness_profile_y", {});
  auto_declare<std::vector<double>>("stiffness_profile_z", {});
  auto_declare<std::vector<double>>("damping_profile_x", {});
  auto_declare<std::vector<double>>("damping_profile_y", {});
  auto_declare<std::vector<double>>("damping_profile_z", {});

  return controller_interface::CallbackReturn::SUCCESS;
}

bool OmxVariableStiffnessController::get_kdl_vector_param_(const std::string & name, KDL::Vector & vec)
{
  auto node = get_node();
  std::vector<double> params_vec;

  try {
    params_vec = node->get_parameter(name).as_double_array();
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "[INIT ERR] Failed to get parameter '%s'", name.c_str());
    return false;
  }

  if (params_vec.size() != 3) {
    RCLCPP_ERROR(node->get_logger(),
      "[INIT ERR] Parameter '%s' must have exactly 3 elements, got %zu",
      name.c_str(), params_vec.size());
    return false;
  }

  vec = KDL::Vector(params_vec[0], params_vec[1], params_vec[2]);
  RCLCPP_INFO(node->get_logger(), "[INIT DBG] Parameter %s loaded: %s",
    name.c_str(), vector_to_string(vec).c_str());
  return true;
}

std::string OmxVariableStiffnessController::get_robot_description_from_node_(
  const std::string & node_name) const
{
  auto node = get_node();
  if (!node) {
    return {};
  }

  // Derive topic name from node name
  std::string topic_name;
  size_t last_slash = node_name.rfind('/');
  if (last_slash != std::string::npos && last_slash > 0) {
    topic_name = node_name.substr(0, last_slash) + "/robot_description";
  } else {
    topic_name = "/robot_description";
  }

  auto sub = node->create_subscription<std_msgs::msg::String>(
    topic_name,
    rclcpp::QoS(1).transient_local(),
    [](const std_msgs::msg::String::SharedPtr) {});

  std_msgs::msg::String msg;
  if (rclcpp::wait_for_message(msg, sub, node->get_node_options().context(),
      std::chrono::seconds(3)))
  {
    RCLCPP_INFO(node->get_logger(), "Got robot_description from topic '%s'", topic_name.c_str());
    return msg.data;
  }

  RCLCPP_WARN(node->get_logger(),
    "Timeout waiting for robot_description on topic '%s'",
    topic_name.c_str());
  return {};
}

std::string OmxVariableStiffnessController::get_robot_description_xml_() const
{
  auto node = get_node();
  if (!node) {
    return {};
  }

  std::string urdf_xml;

  // Priority 1: Explicitly configured node
  if (!robot_description_node_.empty() && robot_description_node_ != "/controller_manager") {
    urdf_xml = get_robot_description_from_node_(robot_description_node_);
    if (!urdf_xml.empty()) {
      return urdf_xml;
    }
  }

  // Priority 2: Local parameter
  if (node->get_parameter("robot_description", urdf_xml) && !urdf_xml.empty()) {
    return urdf_xml;
  }

  // Priority 3: Default configured node
  if (robot_description_node_.empty() || robot_description_node_ == "/controller_manager") {
    urdf_xml = get_robot_description_from_node_(robot_description_node_);
    if (!urdf_xml.empty()) {
      return urdf_xml;
    }
  }

  // Priority 4: Common fallback
  urdf_xml = get_robot_description_from_node_("/robot_state_publisher");
  return urdf_xml;
}

bool OmxVariableStiffnessController::build_kdl_from_robot_description_(const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    return false;
  }
  return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
}

bool OmxVariableStiffnessController::build_chain_(const std::string & root, const std::string & tip)
{
  if (root.empty() || tip.empty()) {
    return false;
  }
  return kdl_tree_.getChain(root, tip, kdl_chain_);
}

controller_interface::CallbackReturn
OmxVariableStiffnessController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // Load basic parameters
  joints_ = node->get_parameter("joints").as_string_array();
  root_link_ = node->get_parameter("root_link").as_string();
  tip_link_ = node->get_parameter("tip_link").as_string();
  robot_description_node_ = node->get_parameter("robot_description_node").as_string();
  torque_scale_ = node->get_parameter("torque_scale").as_double();

  // Load timing parameters
  homing_duration_ = node->get_parameter("homing_duration").as_double();
  move_duration_ = node->get_parameter("move_duration").as_double();
  wait_duration_ = node->get_parameter("wait_duration").as_double();
  max_rise_rate_ = node->get_parameter("max_rise_rate").as_double();

  // Validate required parameters
  if (joints_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (tip_link_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'tip_link' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load KDL vector parameters
  if (!get_kdl_vector_param_("start_position", start_pos_) ||
      !get_kdl_vector_param_("end_position", end_pos_) ||
      !get_kdl_vector_param_("target_orientation", target_orientation_euler_) ||
      !get_kdl_vector_param_("stiffness_homing", stiffness_homing_) ||
      !get_kdl_vector_param_("stiffness_rot", K_rot_) ||
      !get_kdl_vector_param_("damping_rot", Kd_ang_) ||
      !get_kdl_vector_param_("damping_default", Kd_lin_default_))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Load stiffness/damping profiles
  stiffness_profile_x_ = node->get_parameter("stiffness_profile_x").as_double_array();
  stiffness_profile_y_ = node->get_parameter("stiffness_profile_y").as_double_array();
  stiffness_profile_z_ = node->get_parameter("stiffness_profile_z").as_double_array();
  damping_profile_x_ = node->get_parameter("damping_profile_x").as_double_array();
  damping_profile_y_ = node->get_parameter("damping_profile_y").as_double_array();
  damping_profile_z_ = node->get_parameter("damping_profile_z").as_double_array();

  if (!stiffness_profile_x_.empty()) {
    RCLCPP_INFO(node->get_logger(), "[PROFILE] Loaded stiffness profiles with %zu points",
      stiffness_profile_x_.size());
  } else {
    RCLCPP_WARN(node->get_logger(),
      "[PROFILE WARN] No stiffness profiles loaded. Using fixed homing stiffness.");
  }

  // Build KDL chain from robot_description
  const auto urdf_xml = get_robot_description_xml_();
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "robot_description not found. Tried local param, '%s', and '/robot_state_publisher'.",
      robot_description_node_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!build_kdl_from_robot_description_(urdf_xml)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF into KDL tree.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!build_chain_(root_link_, tip_link_)) {
    RCLCPP_ERROR(node->get_logger(),
      "Failed to extract KDL chain root='%s' tip='%s'.",
      root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kdl_chain_.getNrOfJoints() != joints_.size()) {
    RCLCPP_ERROR(node->get_logger(),
      "KDL chain DOFs (%u) != joints size (%zu). root='%s' tip='%s'.",
      kdl_chain_.getNrOfJoints(), joints_.size(),
      root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t n = joints_.size();

  // Initialize KDL solvers
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
  jnt_to_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // Initialize KDL arrays
  q_.resize(n);
  q_dot_.resize(n);
  tau_.resize(n);
  G_.resize(n);
  jacobian_.resize(n);

  // Initialize target orientation
  x_d_frame_.M = KDL::Rotation::RPY(
    target_orientation_euler_.x(),
    target_orientation_euler_.y(),
    target_orientation_euler_.z());

  // Initialize current stiffness/damping
  K_trans_current_ = stiffness_homing_;
  D_trans_current_ = Kd_lin_default_;
  current_state_ = State::INIT;

  // Clear cached interface pointers
  pos_if_.assign(n, nullptr);
  vel_if_.assign(n, nullptr);
  eff_cmd_if_.assign(n, nullptr);

  // Create publishers
  actual_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
    "~/cartesian_pose_actual", 100);
  desired_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
    "~/cartesian_pose_desired", 100);
  stiffness_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/stiffness_state", 100);
  torque_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/torque_values", 10);
  jacobian_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/jacobian_values", 10);
  ee_pos_pub_ = node->create_publisher<geometry_msgs::msg::Point>(
    "~/end_effector_position", 10);
  joint_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/joint_velocities", 10);
  ee_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/end_effector_velocities", 10);
  ee_orient_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
    "~/end_effector_orientation", 10);
  manipulability_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/manipulability_metrics", 10);

  // Create subscriber for runtime stiffness profile updates
  stiffness_profile_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/stiffness_profile_update",
    10,
    std::bind(&OmxVariableStiffnessController::stiffness_profile_callback_, this,
      std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(),
    "[INIT] Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f",
    n, root_link_.c_str(), tip_link_.c_str(), torque_scale_);
  RCLCPP_INFO(node->get_logger(),
    "[INIT] Trajectory: start=%s end=%s homing_dur=%.1f move_dur=%.1f",
    vector_to_string(start_pos_).c_str(),
    vector_to_string(end_pos_).c_str(),
    homing_duration_, move_duration_);

  return controller_interface::CallbackReturn::SUCCESS;
}

void OmxVariableStiffnessController::stiffness_profile_callback_(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Expected format: [n, kx_0, kx_1, ..., ky_0, ky_1, ..., kz_0, kz_1, ...]
  // First element is the number of profile points
  if (msg->data.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Received empty stiffness profile update");
    return;
  }

  size_t n = static_cast<size_t>(msg->data[0]);
  if (msg->data.size() != 1 + 3 * n) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Invalid stiffness profile size. Expected %zu, got %zu", 1 + 3 * n, msg->data.size());
    return;
  }

  stiffness_profile_x_.clear();
  stiffness_profile_y_.clear();
  stiffness_profile_z_.clear();

  for (size_t i = 0; i < n; ++i) {
    stiffness_profile_x_.push_back(msg->data[1 + i]);
    stiffness_profile_y_.push_back(msg->data[1 + n + i]);
    stiffness_profile_z_.push_back(msg->data[1 + 2 * n + i]);
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "Updated stiffness profiles with %zu points", n);
}

bool OmxVariableStiffnessController::cache_interfaces_()
{
  const size_t n = joints_.size();
  if (n == 0) {
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    const auto & j = joints_[i];

    // Position interface
    auto it_pos = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return (si.get_prefix_name() == j) &&
               (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
      });
    if (it_pos != state_interfaces_.end()) {
      pos_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_pos));
    }

    // Velocity interface
    auto it_vel = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return (si.get_prefix_name() == j) &&
               (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
      });
    if (it_vel != state_interfaces_.end()) {
      vel_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_vel));
    }

    // Effort command interface
    auto it_cmd = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & ci) {
        return (ci.get_prefix_name() == j) &&
               (ci.get_interface_name() == std::string(hardware_interface::HW_IF_EFFORT));
      });
    if (it_cmd != command_interfaces_.end()) {
      eff_cmd_if_[i] = const_cast<hardware_interface::LoanedCommandInterface *>(&(*it_cmd));
    }
  }

  // Validate required interfaces
  bool ok = true;
  for (size_t i = 0; i < n; ++i) {
    if (!pos_if_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing POSITION state interface for joint '%s'.", joints_[i].c_str());
      ok = false;
    }
    if (!eff_cmd_if_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing EFFORT command interface for joint '%s'.", joints_[i].c_str());
      ok = false;
    }
  }
  return ok;
}

controller_interface::CallbackReturn
OmxVariableStiffnessController::on_activate(const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  RCLCPP_INFO(node->get_logger(),
    "on_activate: %zu state interfaces, %zu command interfaces",
    state_interfaces_.size(), command_interfaces_.size());

  if (!cache_interfaces_()) {
    RCLCPP_ERROR(node->get_logger(),
      "Failed to cache required interfaces. Controller cannot run.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and compute starting Cartesian pose
  for (size_t i = 0; i < q_.rows(); ++i) {
    q_(i) = pos_if_[i]->get_value();
    q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
  }
  fk_solver_->JntToCart(q_, x_c_start_);

  // Set initial desired pose to current pose
  x_d_frame_ = x_c_start_;
  last_x_error_ = KDL::Twist::Zero();

  // Start state machine
  current_state_ = State::HOMING;
  move_start_time_ = node->get_clock()->now().seconds();

  // Zero command interfaces initially
  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }

  RCLCPP_INFO(node->get_logger(),
    "[STARTING] Initial EE pose: p=%s",
    vector_to_string(x_c_start_.p).c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmxVariableStiffnessController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "[STOPPING] Controller stopped.");
  return controller_interface::CallbackReturn::SUCCESS;
}

double OmxVariableStiffnessController::interpolate_profile_(
  double s, const std::vector<double> & profile) const
{
  if (profile.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return profile.front();
  }
  if (s >= 1.0) {
    return profile.back();
  }

  double scaled_s = s * (profile.size() - 1);
  size_t i = static_cast<size_t>(std::floor(scaled_s));
  size_t i_next = std::min(i + 1, profile.size() - 1);
  double s_local = scaled_s - i;

  return profile[i] * (1.0 - s_local) + profile[i_next] * s_local;
}

double OmxVariableStiffnessController::cos_interpolate_(double t, double T) const
{
  double s = std::min(t / T, 1.0);
  return 0.5 * (1.0 - std::cos(s * M_PI));
}

controller_interface::return_type
OmxVariableStiffnessController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto node = get_node();
  const size_t n = joints_.size();

  if (n == 0 || !dyn_param_ || !jnt_to_jac_solver_ || !fk_solver_) {
    return controller_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  double time_s = time.seconds();

  // 1. READ JOINT STATE & KINEMATICS/DYNAMICS
  for (size_t i = 0; i < n; ++i) {
    q_(i) = pos_if_[i]->get_value();
    q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
  }

  // Compute Forward Kinematics (actual pose)
  KDL::Frame x_c_frame;
  fk_solver_->JntToCart(q_, x_c_frame);

  // Compute Jacobian and Gravity torques
  jnt_to_jac_solver_->JntToJac(q_, jacobian_);
  dyn_param_->JntToGravity(q_, G_);

  // Compute Cartesian velocity
  Eigen::VectorXd x_c_dot_eigen = jacobian_.data * q_dot_.data;

  // 2. TRAJECTORY GENERATION & STATE MACHINE
  double s = 0.0;
  KDL::Vector x_d_new = x_d_frame_.p;

  switch (current_state_) {
    case State::INIT:
      // Should not reach here; immediately transitions in on_activate
      break;

    case State::HOMING:
    {
      double t = time_s - move_start_time_;
      double alpha = cos_interpolate_(t, homing_duration_);
      x_d_new = x_c_start_.p * (1.0 - alpha) + start_pos_ * alpha;
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (t >= homing_duration_) {
        current_state_ = State::MOVE_FORWARD;
        move_start_time_ = time_s;
        x_c_start_.p = x_d_new;
        RCLCPP_INFO(node->get_logger(), "[STATE] Homing complete, starting MOVE_FORWARD");
      }
      break;
    }

    case State::MOVE_FORWARD:
    case State::MOVE_RETURN:
    {
      double t = time_s - move_start_time_;
      double move_t_frac = std::min(t / move_duration_, 1.0);

      if (current_state_ == State::MOVE_FORWARD) {
        s = move_t_frac;
        double alpha = cos_interpolate_(t, move_duration_);
        x_d_new = start_pos_ * (1.0 - alpha) + end_pos_ * alpha;
      } else {  // MOVE_RETURN
        s = 1.0 - move_t_frac;
        double alpha = cos_interpolate_(t, move_duration_);
        x_d_new = end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
      }

      // STIFFNESS MODULATION: Sample & Filter
      if (!stiffness_profile_x_.empty()) {
        KDL::Vector K_trans_desired;
        K_trans_desired.x(interpolate_profile_(s, stiffness_profile_x_));
        K_trans_desired.y(interpolate_profile_(s, stiffness_profile_y_));
        K_trans_desired.z(interpolate_profile_(s, stiffness_profile_z_));

        // Damping profiles (if available)
        if (damping_profile_x_.size() == stiffness_profile_x_.size()) {
          D_trans_current_.x(interpolate_profile_(s, damping_profile_x_));
          D_trans_current_.y(interpolate_profile_(s, damping_profile_y_));
          D_trans_current_.z(interpolate_profile_(s, damping_profile_z_));
        } else {
          D_trans_current_ = Kd_lin_default_;
        }

        // STIFFNESS FILTER (SAFETY)
        double max_change = max_rise_rate_ * dt;
        for (int i = 0; i < 3; ++i) {
          double current_k = K_trans_current_(i);
          double desired_k = K_trans_desired(i);
          double delta_k = desired_k - current_k;
          double filtered_delta = std::clamp(delta_k, -max_change, max_change);
          K_trans_current_(i) = current_k + filtered_delta;
        }
      } else {
        K_trans_current_ = stiffness_homing_;
        D_trans_current_ = Kd_lin_default_;
      }

      // State Transition
      if (move_t_frac >= 1.0) {
        if (current_state_ == State::MOVE_FORWARD) {
          current_state_ = State::WAIT_AT_END;
          RCLCPP_INFO(node->get_logger(), "[STATE] Reached end, waiting");
        } else {
          current_state_ = State::WAIT_AT_START;
          RCLCPP_INFO(node->get_logger(), "[STATE] Returned to start, waiting");
        }
        move_start_time_ = time_s;
      }
      break;
    }

    case State::WAIT_AT_END:
    {
      x_d_new = end_pos_;
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (time_s - move_start_time_ >= wait_duration_) {
        current_state_ = State::MOVE_RETURN;
        move_start_time_ = time_s;
        RCLCPP_INFO(node->get_logger(), "[STATE] Starting MOVE_RETURN");
      }
      break;
    }

    case State::WAIT_AT_START:
    {
      x_d_new = start_pos_;
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (time_s - move_start_time_ >= wait_duration_) {
        current_state_ = State::MOVE_FORWARD;
        move_start_time_ = time_s;
        RCLCPP_INFO(node->get_logger(), "[STATE] Starting MOVE_FORWARD");
      }
      break;
    }
  }

  // Update desired position
  x_d_frame_.p = x_d_new;

  // Apply hardware safety limits before control law
  apply_safety_limits_();

  // 3. IMPEDANCE CONTROL LAW
  KDL::Twist x_error = KDL::diff(x_c_frame, x_d_frame_);
  KDL::Twist x_error_dot;
  if (dt > 1e-9) {
    x_error_dot.vel = (x_error.vel - last_x_error_.vel) / dt;
    x_error_dot.rot = (x_error.rot - last_x_error_.rot) / dt;
  } else {
    x_error_dot = KDL::Twist::Zero();
  }
  last_x_error_ = x_error;

  // Spring Force: F_s = K * X_err
  Eigen::Matrix<double, 6, 1> spring_force;
  spring_force << K_trans_current_.x() * x_error.vel.x(),
                  K_trans_current_.y() * x_error.vel.y(),
                  K_trans_current_.z() * x_error.vel.z(),
                  K_rot_.x() * x_error.rot.x(),
                  K_rot_.y() * x_error.rot.y(),
                  K_rot_.z() * x_error.rot.z();

  // Damping Force: F_d = Kd * X_dot_err
  Eigen::Matrix<double, 6, 1> damping_force;
  damping_force << D_trans_current_.x() * x_error_dot.vel.x(),
                   D_trans_current_.y() * x_error_dot.vel.y(),
                   D_trans_current_.z() * x_error_dot.vel.z(),
                   Kd_ang_.x() * x_error_dot.rot.x(),
                   Kd_ang_.y() * x_error_dot.rot.y(),
                   Kd_ang_.z() * x_error_dot.rot.z();

  Eigen::Matrix<double, 6, 1> F_total = spring_force + damping_force;

  // Convert Cartesian Force to Joint Torques: tau_impedance = J^T * F_total
  Eigen::MatrixXd jacobian_transpose = jacobian_.data.transpose();
  Eigen::VectorXd tau_impedance_vec = jacobian_transpose * F_total;

  // Final Control Torque: tau_total = tau_gravity + tau_impedance
  KDL::JntArray tau_total(n);
  for (size_t i = 0; i < n; ++i) {
    tau_total(i) = (G_(i) + tau_impedance_vec(i)) * torque_scale_;
  }

  // 4. PUBLISH DATA

  // Joint Velocities
  std_msgs::msg::Float64MultiArray joint_vel_msg;
  for (size_t i = 0; i < n; ++i) {
    joint_vel_msg.data.push_back(q_dot_(i));
  }
  joint_vel_pub_->publish(joint_vel_msg);

  // End Effector Position
  geometry_msgs::msg::Point ee_pos_msg;
  ee_pos_msg.x = x_c_frame.p.x();
  ee_pos_msg.y = x_c_frame.p.y();
  ee_pos_msg.z = x_c_frame.p.z();
  ee_pos_pub_->publish(ee_pos_msg);

  // End Effector Orientation (RPY)
  double roll, pitch, yaw;
  x_c_frame.M.GetRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 ee_orient_msg;
  ee_orient_msg.x = roll;
  ee_orient_msg.y = pitch;
  ee_orient_msg.z = yaw;
  ee_orient_pub_->publish(ee_orient_msg);

  // End Effector Cartesian Velocities (6D)
  std_msgs::msg::Float64MultiArray ee_vel_msg;
  for (int i = 0; i < 6; ++i) {
    ee_vel_msg.data.push_back(x_c_dot_eigen(i));
  }
  ee_vel_pub_->publish(ee_vel_msg);

  // Jacobian (Flattened 6xN matrix)
  std_msgs::msg::Float64MultiArray jacobian_msg;
  jacobian_msg.data.assign(jacobian_.data.data(),
    jacobian_.data.data() + jacobian_.data.size());
  jacobian_pub_->publish(jacobian_msg);

  // Manipulability metrics (JJ^T analysis)
  publish_manipulability_metrics_();

  // Torques
  std_msgs::msg::Float64MultiArray torque_msg;
  for (size_t i = 0; i < n; ++i) {
    torque_msg.data.push_back(tau_total(i));
  }
  torque_pub_->publish(torque_msg);

  // Stiffness State (K_trans, K_rot, D_trans, D_rot)
  std_msgs::msg::Float64MultiArray stiffness_msg;
  stiffness_msg.data.push_back(K_trans_current_.x());
  stiffness_msg.data.push_back(K_trans_current_.y());
  stiffness_msg.data.push_back(K_trans_current_.z());
  stiffness_msg.data.push_back(K_rot_.x());
  stiffness_msg.data.push_back(K_rot_.y());
  stiffness_msg.data.push_back(K_rot_.z());
  stiffness_msg.data.push_back(D_trans_current_.x());
  stiffness_msg.data.push_back(D_trans_current_.y());
  stiffness_msg.data.push_back(D_trans_current_.z());
  stiffness_msg.data.push_back(Kd_ang_.x());
  stiffness_msg.data.push_back(Kd_ang_.y());
  stiffness_msg.data.push_back(Kd_ang_.z());
  stiffness_pub_->publish(stiffness_msg);

  // Actual and Desired Pose
  geometry_msgs::msg::Pose actual_pose_msg;
  actual_pose_msg.position.x = x_c_frame.p.x();
  actual_pose_msg.position.y = x_c_frame.p.y();
  actual_pose_msg.position.z = x_c_frame.p.z();
  double x, y, z, w;
  x_c_frame.M.GetQuaternion(x, y, z, w);
  actual_pose_msg.orientation.x = x;
  actual_pose_msg.orientation.y = y;
  actual_pose_msg.orientation.z = z;
  actual_pose_msg.orientation.w = w;
  actual_pose_pub_->publish(actual_pose_msg);

  geometry_msgs::msg::Pose desired_pose_msg;
  desired_pose_msg.position.x = x_d_frame_.p.x();
  desired_pose_msg.position.y = x_d_frame_.p.y();
  desired_pose_msg.position.z = x_d_frame_.p.z();
  x_d_frame_.M.GetQuaternion(x, y, z, w);
  desired_pose_msg.orientation.x = x;
  desired_pose_msg.orientation.y = y;
  desired_pose_msg.orientation.z = z;
  desired_pose_msg.orientation.w = w;
  desired_pose_pub_->publish(desired_pose_msg);

  // 5. COMMAND TORQUES TO HARDWARE
  for (size_t i = 0; i < n; ++i) {
    (void)eff_cmd_if_[i]->set_value(tau_total(i));
  }

  return controller_interface::return_type::OK;
}

void OmxVariableStiffnessController::apply_safety_limits_()
{
  // Clamp translational stiffness to hardware limits
  K_trans_current_.x(std::clamp(K_trans_current_.x(), 0.0, MAX_CARTESIAN_STIFFNESS));
  K_trans_current_.y(std::clamp(K_trans_current_.y(), 0.0, MAX_CARTESIAN_STIFFNESS));
  K_trans_current_.z(std::clamp(K_trans_current_.z(), 0.0, MAX_CARTESIAN_STIFFNESS));

  // Clamp translational damping to hardware limits
  D_trans_current_.x(std::clamp(D_trans_current_.x(), 0.0, MAX_CARTESIAN_DAMPING));
  D_trans_current_.y(std::clamp(D_trans_current_.y(), 0.0, MAX_CARTESIAN_DAMPING));
  D_trans_current_.z(std::clamp(D_trans_current_.z(), 0.0, MAX_CARTESIAN_DAMPING));

  // Clamp rotational stiffness to hardware limits
  K_rot_.x(std::clamp(K_rot_.x(), 0.0, MAX_ROTATIONAL_STIFFNESS));
  K_rot_.y(std::clamp(K_rot_.y(), 0.0, MAX_ROTATIONAL_STIFFNESS));
  K_rot_.z(std::clamp(K_rot_.z(), 0.0, MAX_ROTATIONAL_STIFFNESS));

  // Clamp rotational damping to hardware limits
  Kd_ang_.x(std::clamp(Kd_ang_.x(), 0.0, MAX_ROTATIONAL_DAMPING));
  Kd_ang_.y(std::clamp(Kd_ang_.y(), 0.0, MAX_ROTATIONAL_DAMPING));
  Kd_ang_.z(std::clamp(Kd_ang_.z(), 0.0, MAX_ROTATIONAL_DAMPING));
}

void OmxVariableStiffnessController::publish_manipulability_metrics_()
{
  // Compute JJ^T (6x6 matrix) for manipulability analysis
  // This gives insight into stiffness transformation and singularity proximity
  Eigen::MatrixXd J = jacobian_.data;                    // 6xN Jacobian
  Eigen::MatrixXd JJT = J * J.transpose();               // 6x6 JJ^T

  // Compute eigenvalues of JJ^T using SelfAdjointEigenSolver (JJ^T is symmetric positive semi-definite)
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJT);
  Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();      // Sorted ascending

  // Compute determinant (product of eigenvalues, squared Yoshikawa manipulability)
  double det = JJT.determinant();

  // Compute condition number = sqrt(lambda_max / lambda_min)
  // High condition number indicates proximity to singularity
  double lambda_min = eigenvalues(0);
  double lambda_max = eigenvalues(eigenvalues.size() - 1);
  double condition_number = 0.0;
  if (lambda_min > 1e-10) {
    condition_number = std::sqrt(lambda_max / lambda_min);
  } else {
    condition_number = std::numeric_limits<double>::infinity();
  }

  // Yoshikawa manipulability measure: w = sqrt(det(JJ^T))
  double yoshikawa_manipulability = std::sqrt(std::max(det, 0.0));

  // Minimum singular value (sqrt of min eigenvalue) - indicates distance to singularity
  double min_singular_value = std::sqrt(std::max(lambda_min, 0.0));
  double max_singular_value = std::sqrt(std::max(lambda_max, 0.0));

  // Publish manipulability metrics
  // Format: [condition_number, determinant, yoshikawa, min_sv, max_sv, eigenvalue_0, ..., eigenvalue_5]
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(condition_number);
  msg.data.push_back(det);
  msg.data.push_back(yoshikawa_manipulability);
  msg.data.push_back(min_singular_value);
  msg.data.push_back(max_singular_value);

  // Add all eigenvalues (6 values for 6x6 JJ^T)
  for (int i = 0; i < eigenvalues.size(); ++i) {
    msg.data.push_back(eigenvalues(i));
  }

  manipulability_pub_->publish(msg);
}

}  // namespace omx_variable_stiffness_controller

PLUGINLIB_EXPORT_CLASS(
  omx_variable_stiffness_controller::OmxVariableStiffnessController,
  controller_interface::ControllerInterface)
