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
#include <urdf/model.h>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/msg/joint_state.hpp>

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

// Helper: State enum to readable string
static const char * state_name(State s)
{
  switch (s) {
    case State::INIT:          return "INIT";
    case State::HOMING:        return "HOMING";
    case State::MOVE_FORWARD:  return "MOVE_FORWARD";
    case State::WAIT_AT_END:   return "WAIT_AT_END";
    case State::MOVE_RETURN:   return "MOVE_RETURN";
    case State::WAIT_AT_START: return "WAIT_AT_START";
    default:                   return "UNKNOWN";
  }
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
  auto_declare<double>("wait_duration", 3.0);
  auto_declare<double>("max_rise_rate", 100.0);

  // Trajectory safety parameters
  auto_declare<bool>("use_joint_space_trajectory", true);
  auto_declare<double>("min_manipulability_threshold", 0.01);
  auto_declare<int>("num_trajectory_samples", 50);
  auto_declare<double>("dls_damping_factor", 0.05);

  // Position waypoints (3-element arrays)
  auto_declare<std::vector<double>>("start_position", {0.2, 0.0, 0.15});
  auto_declare<std::vector<double>>("end_position", {0.25, 0.0, 0.15});
  auto_declare<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});

  // Stiffness/damping parameters (3-element arrays)
  auto_declare<std::vector<double>>("stiffness_homing", {20.0, 20.0, 20.0});
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

  // Singularity escape parameters
  // q_preferred: joint configuration the arm is pulled toward when escaping a singularity.
  // Elbow-bent default for OpenManipulator-X: base neutral, shoulder down, elbow bent, wrist up.
  auto_declare<std::vector<double>>(
    "singularity_preferred_joints", {0.0, -0.5, 1.0, 0.5});
  auto_declare<double>("singularity_escape_K",  3.0);   // Nm/rad  proportional gain
  auto_declare<double>("singularity_escape_D",  0.3);   // Nms/rad velocity damping
  auto_declare<double>("singularity_exit_hysteresis", 2.5);  // exit when sigma_min > threshold * this

  // Contact force estimator
  // Lower alpha = more filtering = smoother but laggier force estimate.
  // 0.02 gives ~1.6 Hz cutoff at 500 Hz (appropriate for contact detection).
  // Raise to 0.05-0.1 if you need faster response at the cost of more noise.
  auto_declare<double>("contact_force_filter_alpha", 0.02);

  // How far (meters) the EE must diverge from desired before we publish a
  // deviated waypoint for external tools (GUI) to consume.
  auto_declare<double>("deviation_publish_threshold", 0.01);

  // Waypoint blending duration (seconds)
  auto_declare<double>("waypoint_blend_duration", 2.0);

  // Joint-space homing / regularization (prevents singularity traversal)
  auto_declare<double>("homing_joint_K", 15.0);
  auto_declare<double>("homing_joint_D", 2.0);
  auto_declare<double>("trajectory_joint_reg_K", 3.0);
  auto_declare<double>("trajectory_joint_reg_D", 0.5);

  return controller_interface::CallbackReturn::SUCCESS;
}

bool OmxVariableStiffnessController::get_kdl_vector_param_(
  const std::string & name, KDL::Vector & vec)
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

bool OmxVariableStiffnessController::build_kdl_from_robot_description_(
  const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    return false;
  }
  return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
}

bool OmxVariableStiffnessController::build_chain_(
  const std::string & root, const std::string & tip)
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

  // Set velocity filter alpha for error-derivative low-pass filter
  velocity_filter_alpha_ = 0.1;

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

  // Load trajectory safety parameters
  use_joint_space_trajectory_ = node->get_parameter("use_joint_space_trajectory").as_bool();
  min_manipulability_thresh_ = node->get_parameter("min_manipulability_threshold").as_double();
  num_trajectory_samples_ = static_cast<size_t>(
    node->get_parameter("num_trajectory_samples").as_int());
  dls_damping_factor_ = node->get_parameter("dls_damping_factor").as_double();

  // Load singularity escape parameters
  K_escape_ = node->get_parameter("singularity_escape_K").as_double();
  D_escape_ = node->get_parameter("singularity_escape_D").as_double();
  singularity_exit_hysteresis_ = node->get_parameter("singularity_exit_hysteresis").as_double();
  contact_force_filter_alpha_  = node->get_parameter("contact_force_filter_alpha").as_double();

  // Joint-space homing / regularization gains
  homing_joint_K_ = node->get_parameter("homing_joint_K").as_double();
  homing_joint_D_ = node->get_parameter("homing_joint_D").as_double();
  trajectory_joint_reg_K_ = node->get_parameter("trajectory_joint_reg_K").as_double();
  trajectory_joint_reg_D_ = node->get_parameter("trajectory_joint_reg_D").as_double();

  // q_preferred_ is sized once we know the joint count (after chain is built), but
  // we stash the raw vector here so we can populate it after resize.
  const auto q_pref_vec =
    node->get_parameter("singularity_preferred_joints").as_double_array();

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

  // Load stiffness/damping profiles (optional - may not be set)
  auto get_optional_double_array = [&node](const std::string & name) -> std::vector<double> {
    try {
      auto param = node->get_parameter(name);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return param.as_double_array();
      }
    } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
      // Parameter not set - return empty
    } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
      // Parameter has no value - return empty
    }
    return {};
  };

  stiffness_profile_x_ = get_optional_double_array("stiffness_profile_x");
  stiffness_profile_y_ = get_optional_double_array("stiffness_profile_y");
  stiffness_profile_z_ = get_optional_double_array("stiffness_profile_z");
  damping_profile_x_ = get_optional_double_array("damping_profile_x");
  damping_profile_y_ = get_optional_double_array("damping_profile_y");
  damping_profile_z_ = get_optional_double_array("damping_profile_z");

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


  // FIX: KDL chain check moved to AFTER build_chain_() — chain is now valid here
  if (kdl_chain_.getNrOfJoints() == 0) {
    RCLCPP_ERROR(node->get_logger(), "KDL chain has zero joints after build.");
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

  // Load joint limits from URDF
  if (!load_joint_limits_(urdf_xml)) {
    RCLCPP_WARN(node->get_logger(),
      "[SAFETY] Could not load joint limits from URDF. Joint limit checking disabled.");
  }
  gravity_ = KDL::Vector(0.0, 0.0, -9.81);
  // Initialize KDL solvers
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
  jnt_to_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // Position-priority IK weights for 4-DOF arm (under-actuated for full 6D).
  // Weight position (1.0) >> orientation (0.01) so IK converges on position
  // reliably; the orientation residual is acceptable for a 4-DOF manipulator.
  // Use relaxed eps (1e-4) and more iterations (2000) to handle edge cases.
  {
    Eigen::Matrix<double,6,1> L;
    L << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_, L, 1e-4, 2000);
  }

  // Initialize KDL arrays
  q_.resize(n);
  q_dot_.resize(n);
  tau_.resize(n);
  G_.resize(n);
  jacobian_.resize(n);
  q_min_.resize(n);
  q_max_.resize(n);


  // Populate preferred (escape) joint configuration.
  // Must be done after n is known and q_preferred_ can be sized.
  q_preferred_.resize(n);
  KDL::SetToZero(q_preferred_);
  if (q_pref_vec.size() == n) {
    for (size_t i = 0; i < n; ++i) {
      q_preferred_(i) = q_pref_vec[i];
    }
    RCLCPP_INFO(node->get_logger(),
      "[ESCAPE] Preferred joints loaded: [%.3f, %.3f, %.3f, %.3f]",
      q_preferred_(0), q_preferred_(1), q_preferred_(2), q_preferred_(3));
  } else {
    RCLCPP_WARN(node->get_logger(),
      "[ESCAPE] singularity_preferred_joints has %zu elements but expected %zu. "
      "Using zero configuration (may not escape well).",
      q_pref_vec.size(), n);
  }

  // Reset singularity escape state
  in_singularity_      = false;
  singularity_time_debt_ = 0.0;

  // Initialize target orientation from Euler angles
  x_d_frame_.M = KDL::Rotation::RPY(
    target_orientation_euler_.x(),
    target_orientation_euler_.y(),
    target_orientation_euler_.z());

  // x_d_frame_.p will be synced to actual robot pose in on_activate().
  // Setting it to start_pos_ here is just a sane pre-activation default.
  x_d_frame_.p = start_pos_;

  // Initialize current stiffness/damping
  K_trans_current_ = stiffness_homing_;
  D_trans_current_ = Kd_lin_default_;
  current_state_ = State::INIT;

  // Initialize error filter
  x_error_dot_filtered_ = KDL::Twist::Zero();
  last_x_error_ = KDL::Twist::Zero();

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

  // Subscriber for runtime stiffness profile updates
  stiffness_profile_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/stiffness_profile_update",
    10,
    std::bind(&OmxVariableStiffnessController::stiffness_profile_callback_, this,
      std::placeholders::_1));

  // Subscriber for runtime waypoint commands
  waypoint_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/waypoint_command",
    10,
    std::bind(&OmxVariableStiffnessController::waypoint_callback_, this,
      std::placeholders::_1));

  // Publisher for waypoint active status
  waypoint_active_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "~/waypoint_active", 10);

  // Contact force estimate publishers
  // ~/contact_wrench  — WrenchStamped in root_link_ frame (deflection-based)
  // ~/contact_valid   — Bool, false during singularity escape (discard those rows)
  contact_wrench_pub_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/contact_wrench", 10);
  contact_valid_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "~/contact_valid", 10);

  // Deviated waypoint publisher
  deviated_waypoint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
    "~/deviated_waypoint", 10);

  RCLCPP_INFO(node->get_logger(),
    "[INIT] Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f",
    n, root_link_.c_str(), tip_link_.c_str(), torque_scale_);
  RCLCPP_INFO(node->get_logger(),
    "[INIT] Trajectory: start=%s end=%s homing_dur=%.1f move_dur=%.1f",
    vector_to_string(start_pos_).c_str(),
    vector_to_string(end_pos_).c_str(),
    homing_duration_, move_duration_);

  // Validate and pre-compute joint-space trajectory if enabled
  if (use_joint_space_trajectory_) {
    RCLCPP_INFO(node->get_logger(),
      "[SAFETY] Validating trajectory via IK with %zu samples...",
      num_trajectory_samples_);
    if (!validate_and_compute_trajectory_()) {
      RCLCPP_ERROR(node->get_logger(),
        "[SAFETY ERROR] Trajectory validation failed! "
        "The straight-line path is not IK-feasible or passes through singularities. "
        "Adjust start_position/end_position or set use_joint_space_trajectory:=false");
      return controller_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(node->get_logger(),
      "[SAFETY] Trajectory validated successfully with %zu waypoints",
      trajectory_joint_waypoints_.size());
  } else {
    RCLCPP_WARN(node->get_logger(),
      "[SAFETY WARN] Joint-space trajectory validation DISABLED. "
      "Using raw Cartesian interpolation - may be unsafe!");
  }

  RCLCPP_INFO(node->get_logger(),
    "Configuration successful. Filter alpha: %.2f", velocity_filter_alpha_);
  // Load deviation publish threshold (meters)
  if (node->get_parameter("deviation_publish_threshold", deviation_publish_threshold_)) {
    RCLCPP_INFO(node->get_logger(), "Deviation publish threshold: %.4f m", deviation_publish_threshold_);
  }
  if (node->get_parameter("waypoint_blend_duration", waypoint_blend_duration_)) {
    RCLCPP_INFO(node->get_logger(), "Waypoint blend duration: %.2f s", waypoint_blend_duration_);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void OmxVariableStiffnessController::stiffness_profile_callback_(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Expected format: [n, kx_0, kx_1, ..., ky_0, ky_1, ..., kz_0, kz_1, ...]
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

// FIX: on_activate completely rewritten.
//   - Calls cache_interfaces_() so pos_if_/vel_if_/eff_cmd_if_ are valid before update().
//   - Reads joint state through the correct cached interfaces (pos_if_[]).
//   - Syncs x_d_frame_ to actual robot pose to prevent a torque step at activation.
//   - Resets error filter and state machine correctly.
//   - Removes all references to non-existent members (joint_positions_, joint_state_interfaces_,
//     x_dot_error_filtered_, first_update_).
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

  // Read initial joint positions through the validated cached interfaces.
  // Single get_value() call per interface — a second read could theoretically
  // return a different value and silently bypass the NaN guard.
  for (size_t i = 0; i < q_.rows(); ++i) {
    double val = pos_if_[i]->get_value();
    if (!std::isfinite(val)) {
      RCLCPP_ERROR(node->get_logger(),
        "Joint '%s' reports non-finite position (%.6f) at activation.", joints_[i].c_str(), val);
      return controller_interface::CallbackReturn::ERROR;
    }
    q_(i) = val;

    double vel = 0.0;
    if (vel_if_[i]) {
      vel = vel_if_[i]->get_value();
      if (!std::isfinite(vel)) {
        // Gazebo returns NaN for velocity until first physics step; safe to zero.
        vel = 0.0;
      }
    }
    q_dot_(i) = vel;
  }

  // Store joint positions at activation for joint-space homing
  q_at_activation_.resize(q_.rows());
  for (size_t i = 0; i < q_.rows(); ++i) {
    q_at_activation_(i) = q_(i);
  }

  // Compute actual Cartesian pose and sync desired pose to it.
  // This eliminates the torque step that would otherwise occur if x_d_frame_
  // was already set to start_pos_ while the robot is somewhere else.
  fk_solver_->JntToCart(q_, x_c_start_);
  x_d_frame_   = x_c_start_;

  // Reset error history and filter so damping term starts at zero
  last_x_error_          = KDL::Twist::Zero();
  x_error_dot_filtered_  = KDL::Twist::Zero();

  // Reset singularity escape state
  in_singularity_        = false;
  singularity_time_debt_ = 0.0;

  // Reset contact force estimator
  contact_wrench_filtered_.setZero();
  contact_valid_ = false;

  // Reset stiffness to safe homing values
  K_trans_current_ = stiffness_homing_;
  D_trans_current_ = Kd_lin_default_;

  // Start state machine
  clear_waypoints_();  // discard any leftover waypoints from prior activation
  current_state_   = State::HOMING;
  move_start_time_ = node->get_clock()->now().seconds();

  // // Zero all command interfaces initially
  // for (auto & ci : command_interfaces_) {
  //   ci.set_value(0.0);
  // }

  // Compute gravity torque at current configuration
dyn_param_->JntToGravity(q_, G_);

for (size_t i = 0; i < q_.rows(); ++i) {
  eff_cmd_if_[i]->set_value(G_(i) * torque_scale_);
}

  RCLCPP_INFO(node->get_logger(),
    "[ACTIVATE] Initial EE pose: [%.4f, %.4f, %.4f]",
    x_c_start_.p.x(), x_c_start_.p.y(), x_c_start_.p.z());

  // ── TRAJECTORY CONFIG SUMMARY (paste this block when reporting issues) ─────
  RCLCPP_INFO(node->get_logger(), "┌─────────────────────────────────────────────────────────────────");
  RCLCPP_INFO(node->get_logger(), "│ TRAJECTORY CONFIG");
  RCLCPP_INFO(node->get_logger(), "│  start_pos   : [%.4f, %.4f, %.4f]",
    start_pos_.x(), start_pos_.y(), start_pos_.z());
  RCLCPP_INFO(node->get_logger(), "│  end_pos     : [%.4f, %.4f, %.4f]",
    end_pos_.x(), end_pos_.y(), end_pos_.z());
  RCLCPP_INFO(node->get_logger(), "│  orient_euler: [%.4f, %.4f, %.4f]",
    target_orientation_euler_.x(),
    target_orientation_euler_.y(),
    target_orientation_euler_.z());
  RCLCPP_INFO(node->get_logger(), "│  homing_dur  : %.2f s", homing_duration_);
  RCLCPP_INFO(node->get_logger(), "│  move_dur    : %.2f s", move_duration_);
  RCLCPP_INFO(node->get_logger(), "│  wait_dur    : %.2f s", wait_duration_);
  RCLCPP_INFO(node->get_logger(), "│  K_homing    : [%.2f, %.2f, %.2f]",
    stiffness_homing_.x(), stiffness_homing_.y(), stiffness_homing_.z());
  RCLCPP_INFO(node->get_logger(), "│  K_rot       : [%.2f, %.2f, %.2f]",
    K_rot_.x(), K_rot_.y(), K_rot_.z());
  RCLCPP_INFO(node->get_logger(), "│  D_lin       : [%.2f, %.2f, %.2f]",
    Kd_lin_default_.x(), Kd_lin_default_.y(), Kd_lin_default_.z());
  RCLCPP_INFO(node->get_logger(), "│  dls_lambda  : %.4f", dls_damping_factor_);
  RCLCPP_INFO(node->get_logger(), "│  manip_thresh: %.4f", min_manipulability_thresh_);
  RCLCPP_INFO(node->get_logger(), "│  torque_scale: %.4f", torque_scale_);
  RCLCPP_INFO(node->get_logger(), "│  jspace_traj : %s  (%zu waypoints)",
    use_joint_space_trajectory_ ? "YES" : "NO",
    trajectory_joint_waypoints_.size());
  RCLCPP_INFO(node->get_logger(), "│  homing_K/D  : %.1f / %.1f  reg_K/D: %.1f / %.1f",
    homing_joint_K_, homing_joint_D_, trajectory_joint_reg_K_, trajectory_joint_reg_D_);
  RCLCPP_INFO(node->get_logger(), "│  filter_alpha: %.3f", velocity_filter_alpha_);
  RCLCPP_INFO(node->get_logger(), "│  contact_alpha: %.3f  (~%.1f Hz cutoff at 500 Hz)",
    contact_force_filter_alpha_,
    contact_force_filter_alpha_ * 500.0 / (2.0 * M_PI));
  RCLCPP_INFO(node->get_logger(), "│");
  RCLCPP_INFO(node->get_logger(), "│  INITIAL JOINT POSITIONS");
  for (size_t i = 0; i < q_.rows(); ++i) {
    RCLCPP_INFO(node->get_logger(), "│    %s : q=%.4f rad  (%.2f deg)",
      joints_[i].c_str(), q_(i), q_(i) * 180.0 / M_PI);
  }
  RCLCPP_INFO(node->get_logger(), "│  INITIAL EE : [%.4f, %.4f, %.4f]",
    x_c_start_.p.x(), x_c_start_.p.y(), x_c_start_.p.z());
  RCLCPP_INFO(node->get_logger(), "└─────────────────────────────────────────────────────────────────");

  debug_counter_ = 0;
  traj_s_        = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmxVariableStiffnessController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  clear_waypoints_();  // discard pending waypoints on stop
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

  double scaled_s = s * static_cast<double>(profile.size() - 1);
  size_t i        = static_cast<size_t>(std::floor(scaled_s));
  size_t i_next   = std::min(i + 1, profile.size() - 1);
  double s_local  = scaled_s - static_cast<double>(i);

  return profile[i] * (1.0 - s_local) + profile[i_next] * s_local;
}

double OmxVariableStiffnessController::cos_interpolate_(double t, double T) const
{
  double s = std::min(t / T, 1.0);
  return 0.5 * (1.0 - std::cos(s * M_PI));
}

controller_interface::return_type
OmxVariableStiffnessController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto node = get_node();
  const size_t n = joints_.size();

  if (n == 0 || !dyn_param_ || !jnt_to_jac_solver_ || !fk_solver_) {
    return controller_interface::return_type::ERROR;
  }

  double dt     = period.seconds();
  double time_s = time.seconds();

  // ── 1. READ JOINT STATE ──────────────────────────────────────────────────
  // for (size_t i = 0; i < n; ++i) {
  //   q_(i)     = pos_if_[i]->get_value();
  //   q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
  // }
 
  for (size_t i = 0; i < n; ++i) {

  double q_val = pos_if_[i]->get_value();
  if (!std::isfinite(q_val)) {
    RCLCPP_ERROR(node->get_logger(),
      "NaN POSITION on joint %zu. Holding last value.", i);
    q_val = q_(i);  // hold previous
  }
  q_(i) = q_val;

  double qd_val = 0.0;
  if (vel_if_[i]) {
    qd_val = vel_if_[i]->get_value();
    if (!std::isfinite(qd_val)) {
      RCLCPP_WARN_THROTTLE(node->get_logger(),
        *node->get_clock(), 1000,
        "NaN VELOCITY on joint %zu. Forcing 0.", i);
      qd_val = 0.0;
    }
  }
  q_dot_(i) = qd_val;
}


  // Forward kinematics → actual Cartesian pose
  KDL::Frame x_c_frame;
  fk_solver_->JntToCart(q_, x_c_frame);

  // Jacobian and gravity
  jnt_to_jac_solver_->JntToJac(q_, jacobian_);
  dyn_param_->JntToGravity(q_, G_);

  // Manipulability check (minimum singular value of J)
  double current_manipulability = compute_manipulability_();
  if (current_manipulability < min_manipulability_thresh_) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "[SAFETY] Near singularity! sigma_min=%.4f (threshold=%.4f), DLS active",
      current_manipulability, min_manipulability_thresh_);
  }

  // Cartesian velocity via Jacobian
  Eigen::VectorXd x_c_dot_eigen = jacobian_.data * q_dot_.data;

  // ── SINGULARITY ESCAPE: TRAJECTORY FREEZE ───────────────────────────────
  // When in escape mode, the trajectory timer must not advance — otherwise
  // the desired waypoint moves away from the arm while it's stuck.
  // We accumulate the "frozen" time as a debt and subtract it from the
  // effective time, so s resumes from exactly where it left off once the
  // arm recovers.
  if (in_singularity_) {
    singularity_time_debt_ += dt;  // Every cycle stuck = one dt of debt
  }
  // Effective time used for s computation (paused during escape)
  const double effective_time_s = time_s - singularity_time_debt_;

  // ── 2. TRAJECTORY STATE MACHINE ─────────────────────────────────────────
  double s = 0.0;
  KDL::Vector x_d_new = x_d_frame_.p;

  switch (current_state_) {
    case State::INIT:
      // Transitioned in on_activate(); should not reach here during normal operation
      break;

    case State::HOMING:
    {
      double t     = effective_time_s - move_start_time_;
      double alpha = cos_interpolate_(t, homing_duration_);
      // When joint-space waypoints exist, compute homing target via FK of
      // the interpolated joint path.  x_d_new is for display/logging only —
      // the actual torque uses joint-space PD (see torque section below).
      if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()) {
        KDL::JntArray q_homing(n);
        for (size_t i = 0; i < n; ++i) {
          q_homing(i) = q_at_activation_(i) * (1.0 - alpha) +
                         trajectory_joint_waypoints_[0](i) * alpha;
        }
        KDL::Frame x_homing_frame;
        fk_solver_->JntToCart(q_homing, x_homing_frame);
        x_d_new = x_homing_frame.p;
      } else {
        x_d_new = x_c_start_.p * (1.0 - alpha) + start_pos_ * alpha;
      }
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (t >= homing_duration_) {
        current_state_   = State::MOVE_FORWARD;
        move_start_time_ = effective_time_s;
        x_c_start_.p     = x_d_new;
        RCLCPP_INFO(node->get_logger(),
          "[STATE] HOMING complete → MOVE_FORWARD | EE_actual=[%.4f,%.4f,%.4f] EE_desired=[%.4f,%.4f,%.4f] err=[%.4f,%.4f,%.4f]",
          x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z(),
          x_d_new.x(), x_d_new.y(), x_d_new.z(),
          x_d_new.x() - x_c_frame.p.x(),
          x_d_new.y() - x_c_frame.p.y(),
          x_d_new.z() - x_c_frame.p.z());
      }
      break;
    }

    case State::MOVE_FORWARD:
    case State::MOVE_RETURN:
    {
      double t            = effective_time_s - move_start_time_;
      double move_t_frac  = std::min(t / move_duration_, 1.0);

      s = (current_state_ == State::MOVE_FORWARD) ? move_t_frac : (1.0 - move_t_frac);
      traj_s_ = s;

      // Use IK-validated joint-space trajectory when available
      if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()) {
        KDL::JntArray q_desired(n);
        if (get_trajectory_joints_(s, q_desired)) {
          KDL::Frame x_desired_frame;
          fk_solver_->JntToCart(q_desired, x_desired_frame);
          x_d_new = x_desired_frame.p;
        } else {
          // Fallback: raw Cartesian interpolation
          double alpha = cos_interpolate_(t, move_duration_);
          x_d_new = (current_state_ == State::MOVE_FORWARD)
            ? start_pos_ * (1.0 - alpha) + end_pos_ * alpha
            : end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
        }
      } else {
        double alpha = cos_interpolate_(t, move_duration_);
        x_d_new = (current_state_ == State::MOVE_FORWARD)
          ? start_pos_ * (1.0 - alpha) + end_pos_ * alpha
          : end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
      }

      // Stiffness modulation from profile
      if (!stiffness_profile_x_.empty()) {
        KDL::Vector K_trans_desired;
        K_trans_desired.x(interpolate_profile_(s, stiffness_profile_x_));
        K_trans_desired.y(interpolate_profile_(s, stiffness_profile_y_));
        K_trans_desired.z(interpolate_profile_(s, stiffness_profile_z_));

        if (damping_profile_x_.size() == stiffness_profile_x_.size()) {
          D_trans_current_.x(interpolate_profile_(s, damping_profile_x_));
          D_trans_current_.y(interpolate_profile_(s, damping_profile_y_));
          D_trans_current_.z(interpolate_profile_(s, damping_profile_z_));
        } else {
          D_trans_current_ = Kd_lin_default_;
        }

        // Rate-limited stiffness update (safety)
        double max_change = max_rise_rate_ * dt;
        for (int i = 0; i < 3; ++i) {
          double delta = std::clamp(
            K_trans_desired(i) - K_trans_current_(i), -max_change, max_change);
          K_trans_current_(i) += delta;
        }
      } else {
        K_trans_current_ = stiffness_homing_;
        D_trans_current_ = Kd_lin_default_;
      }

      // State transitions
      if (move_t_frac >= 1.0) {
        if (current_state_ == State::MOVE_FORWARD) {
          current_state_ = State::WAIT_AT_END;
          RCLCPP_INFO(node->get_logger(),
            "[STATE] MOVE_FORWARD done → WAIT_AT_END | s=%.4f EE_actual=[%.4f,%.4f,%.4f] EE_desired=[%.4f,%.4f,%.4f] err=[%.4f,%.4f,%.4f]",
            s,
            x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z(),
            x_d_new.x(), x_d_new.y(), x_d_new.z(),
            x_d_new.x() - x_c_frame.p.x(),
            x_d_new.y() - x_c_frame.p.y(),
            x_d_new.z() - x_c_frame.p.z());
        } else {
          current_state_ = State::WAIT_AT_START;
          RCLCPP_INFO(node->get_logger(),
            "[STATE] MOVE_RETURN done → WAIT_AT_START | s=%.4f EE_actual=[%.4f,%.4f,%.4f] EE_desired=[%.4f,%.4f,%.4f] err=[%.4f,%.4f,%.4f]",
            s,
            x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z(),
            x_d_new.x(), x_d_new.y(), x_d_new.z(),
            x_d_new.x() - x_c_frame.p.x(),
            x_d_new.y() - x_c_frame.p.y(),
            x_d_new.z() - x_c_frame.p.z());
        }
        move_start_time_ = effective_time_s;
      }
      break;
    }

    case State::WAIT_AT_END:
    {
      x_d_new          = end_pos_;
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (effective_time_s - move_start_time_ >= wait_duration_) {
        current_state_   = State::MOVE_RETURN;
        move_start_time_ = effective_time_s;
        RCLCPP_INFO(node->get_logger(),
          "[STATE] WAIT_AT_END done → MOVE_RETURN | EE_actual=[%.4f,%.4f,%.4f]",
          x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z());
      }
      break;
    }

    case State::WAIT_AT_START:
    {
      x_d_new          = start_pos_;
      K_trans_current_ = stiffness_homing_;
      D_trans_current_ = Kd_lin_default_;

      if (effective_time_s - move_start_time_ >= wait_duration_) {
        current_state_   = State::MOVE_FORWARD;
        move_start_time_ = effective_time_s;
        RCLCPP_INFO(node->get_logger(),
          "[STATE] WAIT_AT_START done → MOVE_FORWARD | EE_actual=[%.4f,%.4f,%.4f]",
          x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z());
      }
      break;
    }
  }

  x_d_frame_.p = x_d_new;

  // Apply external waypoint override if active
  x_d_frame_.p = process_waypoints_(x_d_frame_.p, time_s);

  // Publish deviated waypoint (joint state) when EE position deviates
  // from desired by more than the configured threshold. This allows the
  // GUI or other tools to capture the actual configuration at the moment
  // of deviation. Rate-limited to ~10 Hz (every 50th cycle at 500 Hz).
  {
    KDL::Vector err = KDL::Vector(
      x_c_frame.p.x() - x_d_frame_.p.x(),
      x_c_frame.p.y() - x_d_frame_.p.y(),
      x_c_frame.p.z() - x_d_frame_.p.z());
    double err_norm = std::sqrt(err.x() * err.x() + err.y() * err.y() + err.z() * err.z());
    if (err_norm > deviation_publish_threshold_ && deviated_waypoint_pub_ &&
        (debug_counter_ % 50 == 0)) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = node->get_clock()->now();
      js.name = std::vector<std::string>(joints_.begin(), joints_.end());
      js.position.resize(n);
      for (size_t i = 0; i < n; ++i) {
        js.position[i] = q_(i);
      }
      deviated_waypoint_pub_->publish(js);
    }
  }

  // Clamp stiffness/damping to hardware limits
  apply_safety_limits_();

  // ── 3. SINGULARITY DETECTION & MODE SELECTION ───────────────────────────
  //
  // Two modes:
  //   NORMAL  — Cartesian impedance + DLS projection (standard operation)
  //   ESCAPE  — Joint-space PD toward q_preferred_ + gravity comp only
  //
  // Entry: sigma_min < min_manipulability_thresh_
  // Exit:  sigma_min > min_manipulability_thresh_ * singularity_exit_hysteresis_
  //        (hysteresis prevents flicker at the boundary)

  const double exit_threshold = min_manipulability_thresh_ * singularity_exit_hysteresis_;

  if (!in_singularity_ && current_manipulability < min_manipulability_thresh_) {
    in_singularity_        = true;
    singularity_entry_time_ = time_s;
    RCLCPP_WARN(node->get_logger(),
      "[ESCAPE] *** SINGULARITY ENTERED *** sigma_min=%.5f < thresh=%.5f | "
      "q=[%.3f,%.3f,%.3f,%.3f] | EE=[%.4f,%.4f,%.4f] | state=%s s=%.3f",
      current_manipulability, min_manipulability_thresh_,
      q_(0), q_(1), q_(2), q_(3),
      x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z(),
      state_name(current_state_), traj_s_);
  }

  if (in_singularity_ && current_manipulability > exit_threshold) {
    in_singularity_ = false;
    RCLCPP_WARN(node->get_logger(),
      "[ESCAPE] *** SINGULARITY EXITED *** sigma_min=%.5f > exit_thresh=%.5f | "
      "time_in_singularity=%.2f s | frozen_time_debt=%.2f s | "
      "q=[%.3f,%.3f,%.3f,%.3f] | EE=[%.4f,%.4f,%.4f]",
      current_manipulability, exit_threshold,
      time_s - singularity_entry_time_,
      singularity_time_debt_,
      q_(0), q_(1), q_(2), q_(3),
      x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z());
    // NOTE: singularity_time_debt_ is NOT reset here.
    // It keeps accumulating as a permanent offset to effective_time_s so that
    // s resumes from the correct position even across multiple escape episodes.
  }

  // Throttled status reminder while stuck
  if (in_singularity_) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
      "[ESCAPE] In singularity escape for %.1f s | sigma_min=%.5f | "
      "pulling toward q_pref=[%.2f,%.2f,%.2f,%.2f] | frozen_debt=%.1f s",
      time_s - singularity_entry_time_, current_manipulability,
      q_preferred_(0), q_preferred_(1), q_preferred_(2), q_preferred_(3),
      singularity_time_debt_);
  }

  // ── 4. TORQUE COMPUTATION ────────────────────────────────────────────────

  KDL::JntArray tau_total(n);

  if (in_singularity_) {
    // ── ESCAPE MODE: Joint-space PD toward preferred configuration ──────────
    //
    // τ = G + K_escape * (q_pref - q) - D_escape * q_dot
    //
    // Why this works: the preferred config is chosen to have high manipulability
    // (elbow bent, shoulder down). The joint-space PD torque is well-defined
    // even at singularities because it doesn't go through the Jacobian.
    // The arm moves in joint space, manipulability rises, and the Cartesian
    // impedance controller regains authority.
    //
    // The desired Cartesian pose (x_d_frame_) is NOT updated during escape —
    // it stays frozen at the last trajectory point so that when normal control
    // resumes the arm converges to the right place.
    for (size_t i = 0; i < n; ++i) {
      double tau_escape =
        K_escape_ * (q_preferred_(i) - q_(i)) -
        D_escape_ * q_dot_(i);
      tau_total(i) = (G_(i) + tau_escape) * torque_scale_;
    }

    // Keep impedance error history valid so there's no derivative spike on exit
    // (recompute x_error silently but don't use it for torques)
    KDL::Twist x_error_silent = KDL::diff(x_c_frame, x_d_frame_);
    x_error_dot_filtered_.vel = KDL::Vector::Zero();
    x_error_dot_filtered_.rot = KDL::Vector::Zero();
    last_x_error_ = x_error_silent;

    // Contact estimate is invalid during escape.
    // x_d is frozen while the arm moves to q_preferred, so x_err reflects
    // arm motion toward q_pref — not contact deflection.
    // Hold the filter state (do not zero it) so there is no step on re-entry.
    contact_valid_ = false;

  } else if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()
             && current_state_ == State::HOMING) {
    // ── JOINT-SPACE HOMING: safe PD toward first IK-validated waypoint ──────
    //
    // Pure joint-space PD prevents the Cartesian impedance from driving the
    // arm through singularities on the way to the start position.
    // τ = G + K_homing * (q_target - q) - D_homing * q_dot
    double t_homing = effective_time_s - move_start_time_;
    double alpha_homing = std::clamp(cos_interpolate_(t_homing, homing_duration_), 0.0, 1.0);
    for (size_t i = 0; i < n; ++i) {
      double q_target = q_at_activation_(i) * (1.0 - alpha_homing) +
                         trajectory_joint_waypoints_[0](i) * alpha_homing;
      double tau_jspace = homing_joint_K_ * (q_target - q_(i)) -
                          homing_joint_D_ * q_dot_(i);
      tau_total(i) = (G_(i) + tau_jspace) * torque_scale_;
    }
    // Keep impedance error history valid for smooth transition to Cartesian mode
    KDL::Twist x_error_silent = KDL::diff(x_c_frame, x_d_frame_);
    x_error_dot_filtered_.vel = KDL::Vector::Zero();
    x_error_dot_filtered_.rot = KDL::Vector::Zero();
    last_x_error_ = x_error_silent;
    contact_wrench_filtered_.setZero();
    contact_valid_ = false;

  } else {
    // ── NORMAL MODE: Cartesian impedance + DLS projection ───────────────────

    // ── 4a. IMPEDANCE CONTROL LAW ──────────────────────────────────────────
    KDL::Twist x_error = KDL::diff(x_c_frame, x_d_frame_);

    // Low-pass filtered error derivative — suppresses 500 Hz noise amplification
    KDL::Twist x_error_dot_raw;
    if (dt > 1e-9) {
      x_error_dot_raw.vel = (x_error.vel - last_x_error_.vel) * (1.0 / dt);
      x_error_dot_raw.rot = (x_error.rot - last_x_error_.rot) * (1.0 / dt);
    } else {
      x_error_dot_raw = KDL::Twist::Zero();
    }
    x_error_dot_filtered_.vel =
      velocity_filter_alpha_ * x_error_dot_raw.vel +
      (1.0 - velocity_filter_alpha_) * x_error_dot_filtered_.vel;
    x_error_dot_filtered_.rot =
      velocity_filter_alpha_ * x_error_dot_raw.rot +
      (1.0 - velocity_filter_alpha_) * x_error_dot_filtered_.rot;
    last_x_error_ = x_error;

    // Spring force: F_s = K * x_err
    Eigen::Matrix<double, 6, 1> spring_force;
    spring_force << K_trans_current_.x() * x_error.vel.x(),
                    K_trans_current_.y() * x_error.vel.y(),
                    K_trans_current_.z() * x_error.vel.z(),
                    K_rot_.x()           * x_error.rot.x(),
                    K_rot_.y()           * x_error.rot.y(),
                    K_rot_.z()           * x_error.rot.z();

    // Damping force: F_d = D * x_error_dot (filtered)
    Eigen::Matrix<double, 6, 1> damping_force;
    damping_force << D_trans_current_.x() * x_error_dot_filtered_.vel.x(),
                     D_trans_current_.y() * x_error_dot_filtered_.vel.y(),
                     D_trans_current_.z() * x_error_dot_filtered_.vel.z(),
                     Kd_ang_.x()          * x_error_dot_filtered_.rot.x(),
                     Kd_ang_.y()          * x_error_dot_filtered_.rot.y(),
                     Kd_ang_.z()          * x_error_dot_filtered_.rot.z();

    Eigen::Matrix<double, 6, 1> F_total = spring_force + damping_force;

    // ── CONTACT FORCE ESTIMATE (NORMAL MODE) ──────────────────────────────
    // F_total is the raw impedance wrench BEFORE DLS projection.
    // Under quasi-static contact: F_contact ≈ F_total (the spring deflection
    // equals the contact force that's pushing the EE off the desired pose).
    // Under free motion: x_err is small, so F_total ≈ 0 — the estimate is
    // naturally near zero without any baseline subtraction needed.
    //
    // MUST use F_total here, not F_filtered (post-DLS). DLS attenuates forces
    // in singular Jacobian directions — applying it here would suppress the
    // contact estimate exactly near joint-limit/singular configurations.
    contact_wrench_filtered_ =
      contact_force_filter_alpha_ * F_total +
      (1.0 - contact_force_filter_alpha_) * contact_wrench_filtered_;
    contact_valid_ = true;

    // ── 4b. DLS PROJECTION ─────────────────────────────────────────────────
    // τ = J^T (JJ^T + λ²I)^{-1} JJ^T F
    // DLS is still active in the normal mode onset region (between onset and
    // hard threshold) to smooth the transition in/out of escape mode.
    // ── DLS PROJECTION (numerically stable) ─────────────────────────────

Eigen::MatrixXd J   = jacobian_.data;          // 6×n
Eigen::MatrixXd JJT = J * J.transpose();       // 6×6

// Variable DLS: zero when well-conditioned, rises as sigma_min approaches
// the singularity threshold.  This matches the formula shown in the debug
// display and avoids the ~60% force attenuation the previous inverted formula
// caused at normal operating manipulability (~0.066).
//
//   onset   = 5 × min_thresh  (~0.050 for default thresh=0.010)
//   p       = 1 - sigma_min/onset  (0 far away, 1 at threshold)
//   λ²      = λ_max² × p²          (quadratic roll-in)
//
// At sigma_min = 0.066 (normal):  p < 0  → λ² = 0   (DLS off, full force)
// At sigma_min = 0.020 (onset):   p = 0.6 → λ² = 0.0009  (light damping)
// At sigma_min = 0.010 (thresh):  p = 1.0 → λ² = 0.0025  (full damping)
const double dls_onset = min_manipulability_thresh_ * 5.0;
double lambda_sq = 0.0;
if (current_manipulability < dls_onset) {
  double p = 1.0 - (current_manipulability / dls_onset);
  lambda_sq = dls_damping_factor_ * dls_damping_factor_ * p * p;
}

// ── Torque computation (handles under-actuated robots correctly) ────────
//
// OpenManipulator-X has 4 DOF → J is 6×4, JJT is 6×6 rank ≤ 4.
// When DLS is off (λ²=0), JJT is always singular — LDLT will fail.
// Solution: when DLS is inactive, use the simple Jacobian transpose
// τ = J^T F which is the natural impedance control law and is always
// numerically stable regardless of DOF count.  Only when DLS is active
// (near singularity) do we need the damped pseudoinverse projection
// τ = J^T (JJ^T + λ²I)^{-1} F to prevent torque amplification.

Eigen::VectorXd tau_impedance;
if (lambda_sq < 1e-10) {
  // DLS off — Jacobian transpose (stable for any DOF count)
  tau_impedance = J.transpose() * F_total;
} else {
  // DLS on — damped pseudoinverse (λ² > 0 guarantees SPD)
  Eigen::Matrix<double, 6, 6> JJT_damped =
    JJT + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::LDLT<Eigen::Matrix<double,6,6>> ldlt(JJT_damped);
  if (ldlt.info() != Eigen::Success) {
    // Should not happen with λ²>0, fallback to J^T
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
      "JJT_damped LDLT failed despite lambda_sq=%.6f — using J^T fallback.",
      lambda_sq);
    tau_impedance = J.transpose() * F_total;
  } else {
    Eigen::Matrix<double,6,1> F_dls = ldlt.solve(F_total);
    tau_impedance = J.transpose() * F_dls;
  }
}

for (size_t i = 0; i < n; ++i) {
  double tau = G_(i) + tau_impedance(i);

  if (!std::isfinite(tau)) {
    RCLCPP_ERROR(node->get_logger(),
      "NaN torque detected on joint %zu. Zeroing.", i);
    tau = G_(i);
  }

  tau_total(i) = tau * torque_scale_;
}

    // ── Joint-space regularization: keep arm near planned joint path ──────
    // Without this, the Cartesian impedance alone can drive the 4-DOF arm
    // into IK branches with x<0, even though the desired Cartesian path
    // stays at x>0.  The regularization term gently guides the arm back
    // toward the validated joint-space solution.
    if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()) {
      KDL::JntArray q_reg_target(n);
      bool have_reg_target = false;
      if (current_state_ == State::MOVE_FORWARD || current_state_ == State::MOVE_RETURN) {
        have_reg_target = get_trajectory_joints_(traj_s_, q_reg_target);
      } else if (current_state_ == State::WAIT_AT_END) {
        q_reg_target = trajectory_joint_waypoints_.back();
        have_reg_target = true;
      } else if (current_state_ == State::WAIT_AT_START) {
        q_reg_target = trajectory_joint_waypoints_.front();
        have_reg_target = true;
      }
      if (have_reg_target) {
        for (size_t i = 0; i < n; ++i) {
          double tau_reg = trajectory_joint_reg_K_ * (q_reg_target(i) - q_(i))
                           - trajectory_joint_reg_D_ * q_dot_(i);
          tau_total(i) += tau_reg * torque_scale_;
        }
      }
    }
  }

  // ── 4. PUBLISH DIAGNOSTICS ───────────────────────────────────────────────

  // Contact force estimate — publish before other diagnostics so it's timestamped
  // as close as possible to the control cycle.
  // force.x/y/z: estimated contact force in root_link_ frame (N)
  // torque.x/y/z: estimated contact moment in root_link_ frame (Nm)
  // contact_valid=false means the arm was in singularity escape this cycle;
  // the wrench holds the last valid filtered value. Discard these rows in analysis.
  {
    geometry_msgs::msg::WrenchStamped cw_msg;
    cw_msg.header.stamp    = time;
    cw_msg.header.frame_id = contact_valid_ ? root_link_ : (root_link_ + "_escape_invalid");
    cw_msg.wrench.force.x  = contact_wrench_filtered_(0);
    cw_msg.wrench.force.y  = contact_wrench_filtered_(1);
    cw_msg.wrench.force.z  = contact_wrench_filtered_(2);
    cw_msg.wrench.torque.x = contact_wrench_filtered_(3);
    cw_msg.wrench.torque.y = contact_wrench_filtered_(4);
    cw_msg.wrench.torque.z = contact_wrench_filtered_(5);
    contact_wrench_pub_->publish(cw_msg);

    std_msgs::msg::Bool cv_msg;
    cv_msg.data = contact_valid_;
    contact_valid_pub_->publish(cv_msg);
  }

  // Joint velocities
  std_msgs::msg::Float64MultiArray joint_vel_msg;
  for (size_t i = 0; i < n; ++i) {
    joint_vel_msg.data.push_back(q_dot_(i));
  }
  joint_vel_pub_->publish(joint_vel_msg);

  // End-effector position
  geometry_msgs::msg::Point ee_pos_msg;
  ee_pos_msg.x = x_c_frame.p.x();
  ee_pos_msg.y = x_c_frame.p.y();
  ee_pos_msg.z = x_c_frame.p.z();
  ee_pos_pub_->publish(ee_pos_msg);

  // End-effector orientation (RPY)
  double roll, pitch, yaw;
  x_c_frame.M.GetRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 ee_orient_msg;
  ee_orient_msg.x = roll;
  ee_orient_msg.y = pitch;
  ee_orient_msg.z = yaw;
  ee_orient_pub_->publish(ee_orient_msg);

  // End-effector Cartesian velocities (6D)
  std_msgs::msg::Float64MultiArray ee_vel_msg;
  for (int i = 0; i < 6; ++i) {
    ee_vel_msg.data.push_back(x_c_dot_eigen(i));
  }
  ee_vel_pub_->publish(ee_vel_msg);

  // Jacobian (flattened 6×N)
  std_msgs::msg::Float64MultiArray jacobian_msg;
  jacobian_msg.data.assign(
    jacobian_.data.data(),
    jacobian_.data.data() + jacobian_.data.size());
  jacobian_pub_->publish(jacobian_msg);

  // Manipulability metrics
  publish_manipulability_metrics_();

  // Torque values
  std_msgs::msg::Float64MultiArray torque_msg;
  for (size_t i = 0; i < n; ++i) {
    torque_msg.data.push_back(tau_total(i));
  }
  torque_pub_->publish(torque_msg);

  // Stiffness state [Ktx,Kty,Ktz, Krx,Kry,Krz, Dtx,Dty,Dtz, Drx,Dry,Drz]
  std_msgs::msg::Float64MultiArray stiffness_msg;
  stiffness_msg.data = {
    K_trans_current_.x(), K_trans_current_.y(), K_trans_current_.z(),
    K_rot_.x(),           K_rot_.y(),           K_rot_.z(),
    D_trans_current_.x(), D_trans_current_.y(), D_trans_current_.z(),
    Kd_ang_.x(),          Kd_ang_.y(),          Kd_ang_.z()
  };
  stiffness_pub_->publish(stiffness_msg);

  // Actual Cartesian pose
  geometry_msgs::msg::Pose actual_pose_msg;
  actual_pose_msg.position.x = x_c_frame.p.x();
  actual_pose_msg.position.y = x_c_frame.p.y();
  actual_pose_msg.position.z = x_c_frame.p.z();
  {
    double qx, qy, qz, qw;
    x_c_frame.M.GetQuaternion(qx, qy, qz, qw);
    actual_pose_msg.orientation.x = qx;
    actual_pose_msg.orientation.y = qy;
    actual_pose_msg.orientation.z = qz;
    actual_pose_msg.orientation.w = qw;
  }
  actual_pose_pub_->publish(actual_pose_msg);

  // Desired Cartesian pose
  geometry_msgs::msg::Pose desired_pose_msg;
  desired_pose_msg.position.x = x_d_frame_.p.x();
  desired_pose_msg.position.y = x_d_frame_.p.y();
  desired_pose_msg.position.z = x_d_frame_.p.z();
  {
    double qx, qy, qz, qw;
    x_d_frame_.M.GetQuaternion(qx, qy, qz, qw);
    desired_pose_msg.orientation.x = qx;
    desired_pose_msg.orientation.y = qy;
    desired_pose_msg.orientation.z = qz;
    desired_pose_msg.orientation.w = qw;
  }
  desired_pose_pub_->publish(desired_pose_msg);

  // ── TRAJECTORY DEBUG SNAPSHOT ────────────────────────────────────────────
  // Printed every ~500 ms. Copy the ┌─ … └─ block and paste to report issues.
  ++debug_counter_;
  constexpr size_t DBG_INTERVAL = 250;
  if (debug_counter_ % DBG_INTERVAL == 0) {
    double ex = x_d_frame_.p.x() - x_c_frame.p.x();
    double ey = x_d_frame_.p.y() - x_c_frame.p.y();
    double ez = x_d_frame_.p.z() - x_c_frame.p.z();
    double pos_err_norm = std::sqrt(ex*ex + ey*ey + ez*ez);
    double tau_max = 0.0;
    for (size_t i = 0; i < n; ++i) {
      tau_max = std::max(tau_max, std::abs(tau_total(i)));
    }

    RCLCPP_INFO(node->get_logger(), "┌─── TRAJ_DBG  t=%.3f s ────────────────────────────────────────", time_s);
    RCLCPP_INFO(node->get_logger(), "│  mode        : %s",
      in_singularity_ ? "ESCAPE (joint-space PD)" : "NORMAL (Cartesian impedance)");
    RCLCPP_INFO(node->get_logger(), "│  state       : %s", state_name(current_state_));
    RCLCPP_INFO(node->get_logger(), "│  traj_s      : %.4f  (frozen_debt=%.3f s)",
      traj_s_, singularity_time_debt_);
    RCLCPP_INFO(node->get_logger(), "│  EE_actual   : [%.4f, %.4f, %.4f]",
      x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z());
    RCLCPP_INFO(node->get_logger(), "│  EE_desired  : [%.4f, %.4f, %.4f]",
      x_d_frame_.p.x(), x_d_frame_.p.y(), x_d_frame_.p.z());
    RCLCPP_INFO(node->get_logger(), "│  pos_error   : [%.4f, %.4f, %.4f]  norm=%.4f m",
      ex, ey, ez, pos_err_norm);
    RCLCPP_INFO(node->get_logger(), "│  joint_pos   : [%.4f, %.4f, %.4f, %.4f]  (rad)",
      q_(0), q_(1), q_(2), q_(3));
    RCLCPP_INFO(node->get_logger(), "│  joint_vel   : [%.4f, %.4f, %.4f, %.4f]  (rad/s)",
      q_dot_(0), q_dot_(1), q_dot_(2), q_dot_(3));
    RCLCPP_INFO(node->get_logger(), "│  torques     : [%.4f, %.4f, %.4f, %.4f]  (Nm scaled) peak=%.4f",
      tau_total(0), tau_total(1), tau_total(2), tau_total(3), tau_max);
    RCLCPP_INFO(node->get_logger(), "│  sigma_min   : %.5f  (enter=%.5f  exit=%.5f)",
      current_manipulability, min_manipulability_thresh_, exit_threshold);
    RCLCPP_INFO(node->get_logger(), "│  contact_F   : [%.4f, %.4f, %.4f] N  [%.4f, %.4f, %.4f] Nm  valid=%s",
      contact_wrench_filtered_(0), contact_wrench_filtered_(1), contact_wrench_filtered_(2),
      contact_wrench_filtered_(3), contact_wrench_filtered_(4), contact_wrench_filtered_(5),
      contact_valid_ ? "YES" : "NO (escape)");
    if (in_singularity_) {
      RCLCPP_INFO(node->get_logger(), "│  q_preferred : [%.3f, %.3f, %.3f, %.3f]",
        q_preferred_(0), q_preferred_(1), q_preferred_(2), q_preferred_(3));
      RCLCPP_INFO(node->get_logger(), "│  q_err_pref  : [%.3f, %.3f, %.3f, %.3f]",
        q_preferred_(0)-q_(0), q_preferred_(1)-q_(1),
        q_preferred_(2)-q_(2), q_preferred_(3)-q_(3));
      RCLCPP_INFO(node->get_logger(), "│  escape_gains: K=%.2f  D=%.2f  hysteresis=%.1fx",
        K_escape_, D_escape_, singularity_exit_hysteresis_);
    } else {
      RCLCPP_INFO(node->get_logger(), "│  K_trans     : [%.2f, %.2f, %.2f]  D_trans: [%.2f, %.2f, %.2f]",
        K_trans_current_.x(), K_trans_current_.y(), K_trans_current_.z(),
        D_trans_current_.x(), D_trans_current_.y(), D_trans_current_.z());
      // Debug display uses the same formula as the actual computation above.
      double lambda_sq_dbg = 0.0;
      const double dbg_onset = min_manipulability_thresh_ * 5.0;
      if (current_manipulability < dbg_onset) {
        double p = 1.0 - (current_manipulability / dbg_onset);
        lambda_sq_dbg = dls_damping_factor_ * dls_damping_factor_ * p * p;
      }
      RCLCPP_INFO(node->get_logger(), "│  lambda_sq   : %.6f  (DLS %s)",
        lambda_sq_dbg,
        (current_manipulability < dbg_onset) ? "ACTIVE" : "off");
    }
    RCLCPP_INFO(node->get_logger(), "└───────────────────────────────────────────────────────────────");
  }

  // ── 5. COMMAND TORQUES TO HARDWARE ──────────────────────────────────────
  // FIX: Use eff_cmd_if_[] (correct cached handles) and tau_total (correct variable).
  // torque_scale_ is already folded into tau_total above — do NOT apply it again here.
  for (size_t i = 0; i < n; ++i) {
    eff_cmd_if_[i]->set_value(tau_total(i));
  }

  return controller_interface::return_type::OK;
}

void OmxVariableStiffnessController::apply_safety_limits_()
{
  K_trans_current_.x(std::clamp(K_trans_current_.x(), 0.0, MAX_CARTESIAN_STIFFNESS));
  K_trans_current_.y(std::clamp(K_trans_current_.y(), 0.0, MAX_CARTESIAN_STIFFNESS));
  K_trans_current_.z(std::clamp(K_trans_current_.z(), 0.0, MAX_CARTESIAN_STIFFNESS));

  D_trans_current_.x(std::clamp(D_trans_current_.x(), 0.0, MAX_CARTESIAN_DAMPING));
  D_trans_current_.y(std::clamp(D_trans_current_.y(), 0.0, MAX_CARTESIAN_DAMPING));
  D_trans_current_.z(std::clamp(D_trans_current_.z(), 0.0, MAX_CARTESIAN_DAMPING));

  K_rot_.x(std::clamp(K_rot_.x(), 0.0, MAX_ROTATIONAL_STIFFNESS));
  K_rot_.y(std::clamp(K_rot_.y(), 0.0, MAX_ROTATIONAL_STIFFNESS));
  K_rot_.z(std::clamp(K_rot_.z(), 0.0, MAX_ROTATIONAL_STIFFNESS));

  Kd_ang_.x(std::clamp(Kd_ang_.x(), 0.0, MAX_ROTATIONAL_DAMPING));
  Kd_ang_.y(std::clamp(Kd_ang_.y(), 0.0, MAX_ROTATIONAL_DAMPING));
  Kd_ang_.z(std::clamp(Kd_ang_.z(), 0.0, MAX_ROTATIONAL_DAMPING));
}

void OmxVariableStiffnessController::publish_manipulability_metrics_()
{
  Eigen::MatrixXd J   = jacobian_.data;
  Eigen::MatrixXd JJT = J * J.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJT);
  Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();  // ascending

  double det            = JJT.determinant();
  double lambda_min     = eigenvalues(0);
  double lambda_max     = eigenvalues(eigenvalues.size() - 1);
  double condition_number = (lambda_min > 1e-10)
    ? std::sqrt(lambda_max / lambda_min)
    : std::numeric_limits<double>::infinity();

  double yoshikawa      = std::sqrt(std::max(det, 0.0));
  double min_sv         = std::sqrt(std::max(lambda_min, 0.0));
  double max_sv         = std::sqrt(std::max(lambda_max, 0.0));

  // Format: [condition_number, det, yoshikawa, min_sv, max_sv, eigenvalue_0..5]
  std_msgs::msg::Float64MultiArray msg;
  msg.data.push_back(condition_number);
  msg.data.push_back(det);
  msg.data.push_back(yoshikawa);
  msg.data.push_back(min_sv);
  msg.data.push_back(max_sv);
  for (int i = 0; i < eigenvalues.size(); ++i) {
    msg.data.push_back(eigenvalues(i));
  }
  manipulability_pub_->publish(msg);
}

void OmxVariableStiffnessController::waypoint_callback_(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  WaypointCommand wp;
  wp.position  = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  wp.timestamp = get_node()->now().seconds();

  std::string frame = msg->header.frame_id;
  wp.is_offset = (frame == "offset" || frame == "relative");

  {
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
    waypoint_queue_.push(wp);
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "[WAYPOINT] Received %s waypoint: [%.3f, %.3f, %.3f], queue size: %zu",
    wp.is_offset ? "OFFSET" : "ABSOLUTE",
    wp.position.x(), wp.position.y(), wp.position.z(),
    waypoint_queue_.size());
}

KDL::Vector OmxVariableStiffnessController::process_waypoints_(
  const KDL::Vector & trajectory_target, double current_time)
{
  {
    std::lock_guard<std::mutex> lock(waypoint_mutex_);

    if (!has_active_waypoint_ && !waypoint_queue_.empty()) {
      current_waypoint_     = waypoint_queue_.front();
      waypoint_queue_.pop();
      has_active_waypoint_  = true;
      waypoint_mode_active_ = true;
      waypoint_start_time_  = current_time;

      if (current_waypoint_.is_offset) {
        current_waypoint_.position = trajectory_target + current_waypoint_.position;
      }
      waypoint_blend_start_ = trajectory_target;

      RCLCPP_INFO(get_node()->get_logger(),
        "[WAYPOINT] Activating waypoint → [%.3f, %.3f, %.3f]",
        current_waypoint_.position.x(),
        current_waypoint_.position.y(),
        current_waypoint_.position.z());
    }
  }

  std_msgs::msg::Bool status_msg;
  status_msg.data = has_active_waypoint_;
  waypoint_active_pub_->publish(status_msg);

  if (!has_active_waypoint_) {
    waypoint_mode_active_ = false;
    return trajectory_target;
  }

  double elapsed        = current_time - waypoint_start_time_;
  double blend_progress = std::min(elapsed / waypoint_blend_duration_, 1.0);
  double smooth         = 0.5 * (1.0 - std::cos(M_PI * blend_progress));

  KDL::Vector blended;
  blended.x(waypoint_blend_start_.x() +
    smooth * (current_waypoint_.position.x() - waypoint_blend_start_.x()));
  blended.y(waypoint_blend_start_.y() +
    smooth * (current_waypoint_.position.y() - waypoint_blend_start_.y()));
  blended.z(waypoint_blend_start_.z() +
    smooth * (current_waypoint_.position.z() - waypoint_blend_start_.z()));

  if (blend_progress >= 1.0) {
    std::lock_guard<std::mutex> lock(waypoint_mutex_);
    if (waypoint_queue_.empty()) {
      RCLCPP_INFO(get_node()->get_logger(),
        "[WAYPOINT] Reached waypoint, returning to trajectory");
      has_active_waypoint_  = false;
      waypoint_mode_active_ = false;
      return trajectory_target;
    } else {
      current_waypoint_    = waypoint_queue_.front();
      waypoint_queue_.pop();
      waypoint_start_time_ = current_time;
      waypoint_blend_start_ = blended;

      if (current_waypoint_.is_offset) {
        current_waypoint_.position = blended + current_waypoint_.position;
      }

      RCLCPP_INFO(get_node()->get_logger(),
        "[WAYPOINT] Next waypoint → [%.3f, %.3f, %.3f], remaining: %zu",
        current_waypoint_.position.x(),
        current_waypoint_.position.y(),
        current_waypoint_.position.z(),
        waypoint_queue_.size());
    }
  }

  return blended;
}

void OmxVariableStiffnessController::clear_waypoints_()
{
  std::lock_guard<std::mutex> lock(waypoint_mutex_);
  while (!waypoint_queue_.empty()) {
    waypoint_queue_.pop();
  }
  has_active_waypoint_  = false;
  waypoint_mode_active_ = false;
  RCLCPP_INFO(get_node()->get_logger(),
    "[WAYPOINT] Cleared all waypoints, returning to trajectory");
}

bool OmxVariableStiffnessController::load_joint_limits_(const std::string & urdf_xml)
{
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_xml)) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to parse URDF for joint limits");
    return false;
  }

  size_t n = joints_.size();
  q_min_.resize(n);
  q_max_.resize(n);

  for (size_t i = 0; i < n; ++i) {
    auto joint = urdf_model.getJoint(joints_[i]);
    if (!joint) {
      RCLCPP_WARN(get_node()->get_logger(),
        "Joint '%s' not found in URDF", joints_[i].c_str());
      return false;
    }

    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
      if (joint->limits) {
        q_min_(i) = joint->limits->lower;
        q_max_(i) = joint->limits->upper;
        RCLCPP_DEBUG(get_node()->get_logger(),
          "[LIMITS] Joint '%s': [%.3f, %.3f]",
          joints_[i].c_str(), q_min_(i), q_max_(i));
      } else {
        q_min_(i) = -M_PI;
        q_max_(i) =  M_PI;
      }
    } else if (joint->type == urdf::Joint::CONTINUOUS) {
      q_min_(i) = -M_PI;
      q_max_(i) =  M_PI;
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
        "Joint '%s' has unsupported type", joints_[i].c_str());
      return false;
    }
  }

  has_joint_limits_ = true;
  return true;
}

bool OmxVariableStiffnessController::is_within_joint_limits_(const KDL::JntArray & q) const
{
  if (!has_joint_limits_) {
    return true;
  }
  for (size_t i = 0; i < q.rows(); ++i) {
    if (q(i) < q_min_(i) || q(i) > q_max_(i)) {
      return false;
    }
  }
  return true;
}

double OmxVariableStiffnessController::compute_manipulability_() const
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_.data);
  Eigen::VectorXd sv = svd.singularValues();
  return sv(sv.size() - 1);  // minimum singular value
}

bool OmxVariableStiffnessController::validate_and_compute_trajectory_()
{
  auto node = get_node();
  const size_t n = joints_.size();

  trajectory_joint_waypoints_.clear();
  trajectory_joint_waypoints_.reserve(num_trajectory_samples_ + 1);

  // ── Position-only IK using Jacobian pseudo-inverse ──────────────────────
  // The KDL LMA solver cannot reliably solve for a 4-DOF arm with a 6-DOF
  // target: it converges to wrong arm configurations.  Analytical IK with
  // hardcoded link lengths is fragile.  Instead we implement a simple
  // position-only Jacobian pseudo-inverse IK that uses the actual FK and
  // Jacobian solvers (which know the exact chain geometry from the URDF).
  //
  // This iterates: dq = J_pos^+ * (p_desired - p_current) with damping,
  // starting from a reasonable seed.  Since we only minimize the 3-DOF
  // position error (not 6-DOF), the 4-DOF arm has 1 redundant DOF which
  // we resolve by preferring the seed configuration.
  auto solve_position_ik = [&](
    const KDL::Vector & target_pos,
    const KDL::JntArray & seed,
    KDL::JntArray & q_out,
    double & pos_err_out) -> bool
  {
    const int max_iter = 300;
    const double tol = 0.002;     // 2 mm
    const double alpha = 0.5;     // step size
    const double lambda = 0.01;   // damping for pseudo-inverse
    const double null_gain = 1.0; // null-space bias toward seed (strong to prevent arm-flip)
    const size_t nj = seed.rows();

    q_out = seed;

    for (int iter = 0; iter < max_iter; ++iter) {
      KDL::Frame fk_frame;
      fk_solver_->JntToCart(q_out, fk_frame);

      Eigen::Vector3d err;
      err(0) = target_pos.x() - fk_frame.p.x();
      err(1) = target_pos.y() - fk_frame.p.y();
      err(2) = target_pos.z() - fk_frame.p.z();
      pos_err_out = err.norm();

      if (pos_err_out < tol) {
        return true;
      }

      KDL::Jacobian jac(nj);
      jnt_to_jac_solver_->JntToJac(q_out, jac);

      // Extract 3×nj position rows of the Jacobian
      Eigen::Matrix<double, 3, Eigen::Dynamic> Jp = jac.data.topRows(3);

      // Damped pseudo-inverse: J^+ = J^T (J J^T + lambda² I)^{-1}
      Eigen::Matrix3d JJt = Jp * Jp.transpose() + lambda * lambda * Eigen::Matrix3d::Identity();
      Eigen::Vector3d v = JJt.ldlt().solve(err);
      Eigen::VectorXd dq_task = Jp.transpose() * v;

      // Null-space projection: (I - J^+ J) * (q_seed - q)
      // This uses the redundant DOF (4 joints, 3 task DOFs) to keep
      // joint angles close to the seed, preventing arm-flipping.
      Eigen::MatrixXd Jpinv = Jp.transpose() * JJt.ldlt().solve(
        Eigen::Matrix3d::Identity());
      Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nj, nj) - Jpinv * Jp;
      Eigen::VectorXd q_bias(nj);
      for (size_t j = 0; j < nj; ++j) {
        q_bias(j) = seed(j) - q_out(j);
      }
      Eigen::VectorXd dq = dq_task + null_gain * N * q_bias;

      for (size_t j = 0; j < nj; ++j) {
        q_out(j) += alpha * dq(j);
      }
    }
    return pos_err_out < 0.005;  // slightly relaxed final check
  };

  // ----- Backward-walk seeding strategy -----
  // The end position (closer to base, more articulated) reliably produces an
  // elbow-down IK solution.  We solve the trajectory in REVERSE (end→start),
  // warm-starting each waypoint from its neighbor, then reverse the vector.
  // This ensures the start-position seed comes from a known-good elbow-down
  // configuration instead of the potentially ambiguous q_preferred_.

  // 1. Solve IK for the *end* position first using q_preferred_ seed
  KDL::JntArray q_end_seed = q_preferred_;
  {
    KDL::JntArray q_test(n);
    double err_test;
    if (!solve_position_ik(end_pos_, q_preferred_, q_test, err_test)) {
      RCLCPP_WARN(node->get_logger(),
        "[SAFETY] q_preferred_ failed for end pos (err=%.4f), trying zeros", err_test);
      KDL::JntArray q_zero(n);
      for (size_t j = 0; j < n; ++j) q_zero(j) = 0.0;
      if (solve_position_ik(end_pos_, q_zero, q_test, err_test)) {
        q_end_seed = q_test;
      }
    } else {
      q_end_seed = q_test;
    }
  }
  RCLCPP_INFO(node->get_logger(),
    "[SAFETY] Position-IK for end (s=1): q=[%.3f, %.3f, %.3f, %.3f]",
    q_end_seed(0), q_end_seed(1), q_end_seed(2), q_end_seed(3));

  KDL::Rotation target_rot = KDL::Rotation::RPY(
    target_orientation_euler_.x(),
    target_orientation_euler_.y(),
    target_orientation_euler_.z());

  double min_manip = std::numeric_limits<double>::max();

  // 2. Walk BACKWARD from s=1 (end) to s=0 (start), warm-starting each from
  //    the previous (higher-s) solution.
  KDL::JntArray q_init = q_end_seed;
  std::vector<KDL::JntArray> reverse_waypoints;

  for (int sample = static_cast<int>(num_trajectory_samples_); sample >= 0; --sample) {
    double s   = static_cast<double>(sample) / static_cast<double>(num_trajectory_samples_);
    KDL::Vector pos = start_pos_ * (1.0 - s) + end_pos_ * s;

    // --- POSITION-ONLY IK using Jacobian pseudo-inverse ---
    KDL::JntArray q_result(n);
    double fk_pos_err = 0.0;
    bool ik_ok = solve_position_ik(pos, q_init, q_result, fk_pos_err);

    if (!ik_ok) {
      RCLCPP_ERROR(node->get_logger(),
        "[SAFETY ERROR] Position-IK failed at s=%.2f, pos=[%.3f, %.3f, %.3f], err=%.4f m",
        s, pos.x(), pos.y(), pos.z(), fk_pos_err);
      return false;
    }

    if (!is_within_joint_limits_(q_result)) {
      RCLCPP_ERROR(node->get_logger(),
        "[SAFETY ERROR] Position-IK violates joint limits at s=%.2f: q=[%.3f, %.3f, %.3f, %.3f]",
        s, q_result(0), q_result(1), q_result(2), q_result(3));
      return false;
    }

    KDL::Jacobian jac(n);
    jnt_to_jac_solver_->JntToJac(q_result, jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      jac.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sv    = svd.singularValues();
    double sigma_min      = sv(sv.size() - 1);

    if (sample == static_cast<int>(num_trajectory_samples_)) {
      RCLCPP_INFO(node->get_logger(),
        "[SAFETY] s=1 (end) pos-IK: q=[%.3f, %.3f, %.3f, %.3f], FK_err=%.4f, sigma_min=%.6f",
        q_result(0), q_result(1), q_result(2), q_result(3), fk_pos_err, sigma_min);
    }
    if (sample == 0) {
      RCLCPP_INFO(node->get_logger(),
        "[SAFETY] s=0 (start) pos-IK: q=[%.3f, %.3f, %.3f, %.3f], FK_err=%.4f, sigma_min=%.6f",
        q_result(0), q_result(1), q_result(2), q_result(3), fk_pos_err, sigma_min);
    }

    if (sigma_min < min_manip) {
      min_manip = sigma_min;
    }

    if (sigma_min < min_manipulability_thresh_) {
      RCLCPP_ERROR(node->get_logger(),
        "[SAFETY ERROR] Near singularity at s=%.2f (sigma_min=%.4f < thresh=%.4f)",
        s, sigma_min, min_manipulability_thresh_);
      return false;
    }

    reverse_waypoints.push_back(q_result);
    q_init = q_result;  // warm-start next IK from this solution
  }

  // 3. Reverse to get forward order (s=0 → s=1)
  for (auto it = reverse_waypoints.rbegin(); it != reverse_waypoints.rend(); ++it) {
    trajectory_joint_waypoints_.push_back(*it);
  }

  RCLCPP_INFO(node->get_logger(),
    "[SAFETY] Trajectory validated. Min sigma=%.4f (thresh=%.4f)",
    min_manip, min_manipulability_thresh_);

  // Export validated joint-space waypoints to CSV for inspection/regeneration
  try {
    std::ofstream csv("/tmp/omx_trajectory_waypoints.csv");
    if (csv.is_open()) {
      // Header
      csv << std::setprecision(6) << std::fixed;
      csv << "# waypoint_index";
      for (size_t i = 0; i < n; ++i) csv << ",joint" << (i+1);
      csv << "\n";
      for (size_t wi = 0; wi < trajectory_joint_waypoints_.size(); ++wi) {
        const KDL::JntArray & q = trajectory_joint_waypoints_[wi];
        csv << wi;
        for (size_t j = 0; j < q.rows(); ++j) {
          csv << "," << q(j);
        }
        csv << "\n";
      }
      csv.close();
      RCLCPP_INFO(node->get_logger(), "[EXPORT] Wrote %zu joint-waypoints to /tmp/omx_trajectory_waypoints.csv",
        trajectory_joint_waypoints_.size());
    } else {
      RCLCPP_WARN(node->get_logger(), "[EXPORT] Failed to open /tmp/omx_trajectory_waypoints.csv for writing");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "[EXPORT] Exception while writing waypoints: %s", e.what());
  }
  return true;
}

bool OmxVariableStiffnessController::get_trajectory_joints_(
  double s, KDL::JntArray & q_out) const
{
  if (trajectory_joint_waypoints_.empty()) {
    return false;
  }

  s = std::clamp(s, 0.0, 1.0);

  double idx_f  = s * static_cast<double>(trajectory_joint_waypoints_.size() - 1);
  size_t idx_lo = static_cast<size_t>(std::floor(idx_f));
  size_t idx_hi = std::min(idx_lo + 1, trajectory_joint_waypoints_.size() - 1);
  double t      = idx_f - static_cast<double>(idx_lo);

  const KDL::JntArray & q_lo = trajectory_joint_waypoints_[idx_lo];
  const KDL::JntArray & q_hi = trajectory_joint_waypoints_[idx_hi];

  q_out.resize(q_lo.rows());
  for (size_t i = 0; i < q_out.rows(); ++i) {
    q_out(i) = q_lo(i) * (1.0 - t) + q_hi(i) * t;
  }
  return true;
}

}  // namespace omx_variable_stiffness_controller

PLUGINLIB_EXPORT_CLASS(
  omx_variable_stiffness_controller::OmxVariableStiffnessController,
  controller_interface::ControllerInterface)

// Note: compute_escape_torque_() is declared in the header for potential future
// use by derived classes or unit tests, but the escape torque is computed inline
// in update() for simplicity (avoids a function call on the hot path).
// If you prefer the helper form, here it is:
//
// KDL::JntArray OmxVariableStiffnessController::compute_escape_torque_() const
// {
//   KDL::JntArray tau(q_.rows());
//   for (size_t i = 0; i < q_.rows(); ++i) {
//     tau(i) = K_escape_ * (q_preferred_(i) - q_(i)) - D_escape_ * q_dot_(i);
//   }
//   return tau;
// }
// #include "omx_variable_stiffness_controller/omx_variable_stiffness_controller.hpp"

// #include <algorithm>
// #include <cmath>
// #include <string>
// #include <vector>
// #include <sstream>
// #include <chrono>

// #include <kdl/chaindynparam.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <urdf/model.h>
// #include <std_msgs/msg/string.hpp>
// #include <rclcpp/wait_for_message.hpp>
// #include <tf2_kdl/tf2_kdl.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

// #include "pluginlib/class_list_macros.hpp"

// namespace omx_variable_stiffness_controller
// {

// // Helper: Convert KDL::Vector to a string for debugging
// static std::string vector_to_string(const KDL::Vector & v)
// {
//   std::stringstream ss;
//   ss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
//   return ss.str();
// }

// controller_interface::InterfaceConfiguration
// OmxVariableStiffnessController::state_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration cfg;
//   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

//   for (const auto & j : joints_) {
//     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_POSITION));
//     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_VELOCITY));
//   }
//   return cfg;
// }

// controller_interface::InterfaceConfiguration
// OmxVariableStiffnessController::command_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration cfg;
//   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

//   for (const auto & j : joints_) {
//     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
//   }
//   return cfg;
// }

// controller_interface::CallbackReturn
// OmxVariableStiffnessController::on_init()
// {
//   // Declare all parameters with defaults
//   auto_declare<std::vector<std::string>>("joints", {});
//   auto_declare<std::string>("root_link", "world");
//   auto_declare<std::string>("tip_link", "");
//   auto_declare<std::string>("robot_description_node", "/controller_manager");
//   auto_declare<double>("torque_scale", 1.0);

//   // Trajectory timing parameters
//   auto_declare<double>("homing_duration", 5.0);
//   auto_declare<double>("move_duration", 10.0);
//   auto_declare<double>("wait_duration", 3.0);
//   auto_declare<double>("max_rise_rate", 100.0);

//   // Trajectory safety parameters
//   auto_declare<bool>("use_joint_space_trajectory", true);
//   auto_declare<double>("min_manipulability_threshold", 0.01);
//   auto_declare<int>("num_trajectory_samples", 50);
//   auto_declare<double>("dls_damping_factor", 0.05);

//   // Position waypoints (3-element arrays)
//   auto_declare<std::vector<double>>("start_position", {0.2, 0.0, 0.15});
//   auto_declare<std::vector<double>>("end_position", {0.25, 0.0, 0.15});
//   auto_declare<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});

//   // Stiffness/damping parameters (3-element arrays)
//   auto_declare<std::vector<double>>("stiffness_homing", {20.0, 20.0, 20.0});
//   auto_declare<std::vector<double>>("stiffness_rot", {50.0, 50.0, 50.0});
//   auto_declare<std::vector<double>>("damping_rot", {5.0, 5.0, 5.0});
//   auto_declare<std::vector<double>>("damping_default", {20.0, 20.0, 20.0});

//   // Stiffness/damping profiles (loaded from CSV or parameter server)
//   auto_declare<std::vector<double>>("stiffness_profile_x", {});
//   auto_declare<std::vector<double>>("stiffness_profile_y", {});
//   auto_declare<std::vector<double>>("stiffness_profile_z", {});
//   auto_declare<std::vector<double>>("damping_profile_x", {});
//   auto_declare<std::vector<double>>("damping_profile_y", {});
//   auto_declare<std::vector<double>>("damping_profile_z", {});

//   return controller_interface::CallbackReturn::SUCCESS;
// }

// bool OmxVariableStiffnessController::get_kdl_vector_param_(
//   const std::string & name, KDL::Vector & vec)
// {
//   auto node = get_node();
//   std::vector<double> params_vec;

//   try {
//     params_vec = node->get_parameter(name).as_double_array();
//   } catch (...) {
//     RCLCPP_ERROR(node->get_logger(), "[INIT ERR] Failed to get parameter '%s'", name.c_str());
//     return false;
//   }

//   if (params_vec.size() != 3) {
//     RCLCPP_ERROR(node->get_logger(),
//       "[INIT ERR] Parameter '%s' must have exactly 3 elements, got %zu",
//       name.c_str(), params_vec.size());
//     return false;
//   }

//   vec = KDL::Vector(params_vec[0], params_vec[1], params_vec[2]);
//   RCLCPP_INFO(node->get_logger(), "[INIT DBG] Parameter %s loaded: %s",
//     name.c_str(), vector_to_string(vec).c_str());
//   return true;
// }

// std::string OmxVariableStiffnessController::get_robot_description_from_node_(
//   const std::string & node_name) const
// {
//   auto node = get_node();
//   if (!node) {
//     return {};
//   }

//   // Derive topic name from node name
//   std::string topic_name;
//   size_t last_slash = node_name.rfind('/');
//   if (last_slash != std::string::npos && last_slash > 0) {
//     topic_name = node_name.substr(0, last_slash) + "/robot_description";
//   } else {
//     topic_name = "/robot_description";
//   }

//   auto sub = node->create_subscription<std_msgs::msg::String>(
//     topic_name,
//     rclcpp::QoS(1).transient_local(),
//     [](const std_msgs::msg::String::SharedPtr) {});

//   std_msgs::msg::String msg;
//   if (rclcpp::wait_for_message(msg, sub, node->get_node_options().context(),
//       std::chrono::seconds(3)))
//   {
//     RCLCPP_INFO(node->get_logger(), "Got robot_description from topic '%s'", topic_name.c_str());
//     return msg.data;
//   }

//   RCLCPP_WARN(node->get_logger(),
//     "Timeout waiting for robot_description on topic '%s'",
//     topic_name.c_str());
//   return {};
// }

// std::string OmxVariableStiffnessController::get_robot_description_xml_() const
// {
//   auto node = get_node();
//   if (!node) {
//     return {};
//   }

//   std::string urdf_xml;

//   // Priority 1: Explicitly configured node
//   if (!robot_description_node_.empty() && robot_description_node_ != "/controller_manager") {
//     urdf_xml = get_robot_description_from_node_(robot_description_node_);
//     if (!urdf_xml.empty()) {
//       return urdf_xml;
//     }
//   }

//   // Priority 2: Local parameter
//   if (node->get_parameter("robot_description", urdf_xml) && !urdf_xml.empty()) {
//     return urdf_xml;
//   }

//   // Priority 3: Default configured node
//   if (robot_description_node_.empty() || robot_description_node_ == "/controller_manager") {
//     urdf_xml = get_robot_description_from_node_(robot_description_node_);
//     if (!urdf_xml.empty()) {
//       return urdf_xml;
//     }
//   }

//   // Priority 4: Common fallback
//   urdf_xml = get_robot_description_from_node_("/robot_state_publisher");
//   return urdf_xml;
// }

// bool OmxVariableStiffnessController::build_kdl_from_robot_description_(
//   const std::string & urdf_xml)
// {
//   if (urdf_xml.empty()) {
//     return false;
//   }
//   return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
// }

// bool OmxVariableStiffnessController::build_chain_(
//   const std::string & root, const std::string & tip)
// {
//   if (root.empty() || tip.empty()) {
//     return false;
//   }
//   return kdl_tree_.getChain(root, tip, kdl_chain_);
// }

// controller_interface::CallbackReturn
// OmxVariableStiffnessController::on_configure(const rclcpp_lifecycle::State &)
// {
//   auto node = get_node();

//   // Set velocity filter alpha for error-derivative low-pass filter
//   velocity_filter_alpha_ = 0.1;

//   // Load basic parameters
//   joints_ = node->get_parameter("joints").as_string_array();
//   root_link_ = node->get_parameter("root_link").as_string();
//   tip_link_ = node->get_parameter("tip_link").as_string();
//   robot_description_node_ = node->get_parameter("robot_description_node").as_string();
//   torque_scale_ = node->get_parameter("torque_scale").as_double();

//   // Load timing parameters
//   homing_duration_ = node->get_parameter("homing_duration").as_double();
//   move_duration_ = node->get_parameter("move_duration").as_double();
//   wait_duration_ = node->get_parameter("wait_duration").as_double();
//   max_rise_rate_ = node->get_parameter("max_rise_rate").as_double();

//   // Load trajectory safety parameters
//   use_joint_space_trajectory_ = node->get_parameter("use_joint_space_trajectory").as_bool();
//   min_manipulability_thresh_ = node->get_parameter("min_manipulability_threshold").as_double();
//   num_trajectory_samples_ = static_cast<size_t>(
//     node->get_parameter("num_trajectory_samples").as_int());
//   dls_damping_factor_ = node->get_parameter("dls_damping_factor").as_double();

//   // Validate required parameters
//   if (joints_.empty()) {
//     RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
//     return controller_interface::CallbackReturn::ERROR;
//   }
//   if (tip_link_.empty()) {
//     RCLCPP_ERROR(node->get_logger(), "Parameter 'tip_link' is empty.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // Load KDL vector parameters
//   if (!get_kdl_vector_param_("start_position", start_pos_) ||
//       !get_kdl_vector_param_("end_position", end_pos_) ||
//       !get_kdl_vector_param_("target_orientation", target_orientation_euler_) ||
//       !get_kdl_vector_param_("stiffness_homing", stiffness_homing_) ||
//       !get_kdl_vector_param_("stiffness_rot", K_rot_) ||
//       !get_kdl_vector_param_("damping_rot", Kd_ang_) ||
//       !get_kdl_vector_param_("damping_default", Kd_lin_default_))
//   {
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // Load stiffness/damping profiles (optional - may not be set)
//   auto get_optional_double_array = [&node](const std::string & name) -> std::vector<double> {
//     try {
//       auto param = node->get_parameter(name);
//       if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
//         return param.as_double_array();
//       }
//     } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
//       // Parameter not set - return empty
//     } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
//       // Parameter has no value - return empty
//     }
//     return {};
//   };

//   stiffness_profile_x_ = get_optional_double_array("stiffness_profile_x");
//   stiffness_profile_y_ = get_optional_double_array("stiffness_profile_y");
//   stiffness_profile_z_ = get_optional_double_array("stiffness_profile_z");
//   damping_profile_x_ = get_optional_double_array("damping_profile_x");
//   damping_profile_y_ = get_optional_double_array("damping_profile_y");
//   damping_profile_z_ = get_optional_double_array("damping_profile_z");

//   if (!stiffness_profile_x_.empty()) {
//     RCLCPP_INFO(node->get_logger(), "[PROFILE] Loaded stiffness profiles with %zu points",
//       stiffness_profile_x_.size());
//   } else {
//     RCLCPP_WARN(node->get_logger(),
//       "[PROFILE WARN] No stiffness profiles loaded. Using fixed homing stiffness.");
//   }

//   // Build KDL chain from robot_description
//   const auto urdf_xml = get_robot_description_xml_();
//   if (urdf_xml.empty()) {
//     RCLCPP_ERROR(node->get_logger(),
//       "robot_description not found. Tried local param, '%s', and '/robot_state_publisher'.",
//       robot_description_node_.c_str());
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   if (!build_kdl_from_robot_description_(urdf_xml)) {
//     RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF into KDL tree.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   if (!build_chain_(root_link_, tip_link_)) {
//     RCLCPP_ERROR(node->get_logger(),
//       "Failed to extract KDL chain root='%s' tip='%s'.",
//       root_link_.c_str(), tip_link_.c_str());
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // FIX: KDL chain check moved to AFTER build_chain_() — chain is now valid here
//   if (kdl_chain_.getNrOfJoints() == 0) {
//     RCLCPP_ERROR(node->get_logger(), "KDL chain has zero joints after build.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   if (kdl_chain_.getNrOfJoints() != joints_.size()) {
//     RCLCPP_ERROR(node->get_logger(),
//       "KDL chain DOFs (%u) != joints size (%zu). root='%s' tip='%s'.",
//       kdl_chain_.getNrOfJoints(), joints_.size(),
//       root_link_.c_str(), tip_link_.c_str());
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   const size_t n = joints_.size();

//   // Load joint limits from URDF
//   if (!load_joint_limits_(urdf_xml)) {
//     RCLCPP_WARN(node->get_logger(),
//       "[SAFETY] Could not load joint limits from URDF. Joint limit checking disabled.");
//   }

//   // Initialize KDL solvers
//   dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
//   jnt_to_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
//   fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
//   ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);

//   // Initialize KDL arrays
//   q_.resize(n);
//   q_dot_.resize(n);
//   tau_.resize(n);
//   G_.resize(n);
//   jacobian_.resize(n);
//   q_min_.resize(n);
//   q_max_.resize(n);

//   // Initialize target orientation from Euler angles
//   x_d_frame_.M = KDL::Rotation::RPY(
//     target_orientation_euler_.x(),
//     target_orientation_euler_.y(),
//     target_orientation_euler_.z());

//   // x_d_frame_.p will be synced to actual robot pose in on_activate().
//   // Setting it to start_pos_ here is just a sane pre-activation default.
//   x_d_frame_.p = start_pos_;

//   // Initialize current stiffness/damping
//   K_trans_current_ = stiffness_homing_;
//   D_trans_current_ = Kd_lin_default_;
//   current_state_ = State::INIT;

//   // Initialize error filter
//   x_error_dot_filtered_ = KDL::Twist::Zero();
//   last_x_error_ = KDL::Twist::Zero();

//   // Clear cached interface pointers
//   pos_if_.assign(n, nullptr);
//   vel_if_.assign(n, nullptr);
//   eff_cmd_if_.assign(n, nullptr);

//   // Create publishers
//   actual_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
//     "~/cartesian_pose_actual", 100);
//   desired_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
//     "~/cartesian_pose_desired", 100);
//   stiffness_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/stiffness_state", 100);
//   torque_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/torque_values", 10);
//   jacobian_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/jacobian_values", 10);
//   ee_pos_pub_ = node->create_publisher<geometry_msgs::msg::Point>(
//     "~/end_effector_position", 10);
//   joint_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/joint_velocities", 10);
//   ee_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/end_effector_velocities", 10);
//   ee_orient_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
//     "~/end_effector_orientation", 10);
//   manipulability_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
//     "~/manipulability_metrics", 10);

//   // Subscriber for runtime stiffness profile updates
//   stiffness_profile_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
//     "~/stiffness_profile_update",
//     10,
//     std::bind(&OmxVariableStiffnessController::stiffness_profile_callback_, this,
//       std::placeholders::_1));

//   // Subscriber for runtime waypoint commands
//   waypoint_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
//     "~/waypoint_command",
//     10,
//     std::bind(&OmxVariableStiffnessController::waypoint_callback_, this,
//       std::placeholders::_1));

//   // Publisher for waypoint active status
//   waypoint_active_pub_ = node->create_publisher<std_msgs::msg::Bool>(
//     "~/waypoint_active", 10);

//   RCLCPP_INFO(node->get_logger(),
//     "[INIT] Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f",
//     n, root_link_.c_str(), tip_link_.c_str(), torque_scale_);
//   RCLCPP_INFO(node->get_logger(),
//     "[INIT] Trajectory: start=%s end=%s homing_dur=%.1f move_dur=%.1f",
//     vector_to_string(start_pos_).c_str(),
//     vector_to_string(end_pos_).c_str(),
//     homing_duration_, move_duration_);

//   // Validate and pre-compute joint-space trajectory if enabled
//   if (use_joint_space_trajectory_) {
//     RCLCPP_INFO(node->get_logger(),
//       "[SAFETY] Validating trajectory via IK with %zu samples...",
//       num_trajectory_samples_);
//     if (!validate_and_compute_trajectory_()) {
//       RCLCPP_ERROR(node->get_logger(),
//         "[SAFETY ERROR] Trajectory validation failed! "
//         "The straight-line path is not IK-feasible or passes through singularities. "
//         "Adjust start_position/end_position or set use_joint_space_trajectory:=false");
//       return controller_interface::CallbackReturn::ERROR;
//     }
//     RCLCPP_INFO(node->get_logger(),
//       "[SAFETY] Trajectory validated successfully with %zu waypoints",
//       trajectory_joint_waypoints_.size());
//   } else {
//     RCLCPP_WARN(node->get_logger(),
//       "[SAFETY WARN] Joint-space trajectory validation DISABLED. "
//       "Using raw Cartesian interpolation - may be unsafe!");
//   }

//   RCLCPP_INFO(node->get_logger(),
//     "Configuration successful. Filter alpha: %.2f", velocity_filter_alpha_);
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// void OmxVariableStiffnessController::stiffness_profile_callback_(
//   const std_msgs::msg::Float64MultiArray::SharedPtr msg)
// {
//   // Expected format: [n, kx_0, kx_1, ..., ky_0, ky_1, ..., kz_0, kz_1, ...]
//   if (msg->data.empty()) {
//     RCLCPP_WARN(get_node()->get_logger(), "Received empty stiffness profile update");
//     return;
//   }

//   size_t n = static_cast<size_t>(msg->data[0]);
//   if (msg->data.size() != 1 + 3 * n) {
//     RCLCPP_ERROR(get_node()->get_logger(),
//       "Invalid stiffness profile size. Expected %zu, got %zu", 1 + 3 * n, msg->data.size());
//     return;
//   }

//   stiffness_profile_x_.clear();
//   stiffness_profile_y_.clear();
//   stiffness_profile_z_.clear();

//   for (size_t i = 0; i < n; ++i) {
//     stiffness_profile_x_.push_back(msg->data[1 + i]);
//     stiffness_profile_y_.push_back(msg->data[1 + n + i]);
//     stiffness_profile_z_.push_back(msg->data[1 + 2 * n + i]);
//   }

//   RCLCPP_INFO(get_node()->get_logger(),
//     "Updated stiffness profiles with %zu points", n);
// }

// bool OmxVariableStiffnessController::cache_interfaces_()
// {
//   const size_t n = joints_.size();
//   if (n == 0) {
//     return false;
//   }

//   for (size_t i = 0; i < n; ++i) {
//     const auto & j = joints_[i];

//     // Position interface
//     auto it_pos = std::find_if(
//       state_interfaces_.begin(), state_interfaces_.end(),
//       [&](const auto & si) {
//         return (si.get_prefix_name() == j) &&
//                (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
//       });
//     if (it_pos != state_interfaces_.end()) {
//       pos_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_pos));
//     }

//     // Velocity interface
//     auto it_vel = std::find_if(
//       state_interfaces_.begin(), state_interfaces_.end(),
//       [&](const auto & si) {
//         return (si.get_prefix_name() == j) &&
//                (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
//       });
//     if (it_vel != state_interfaces_.end()) {
//       vel_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_vel));
//     }

//     // Effort command interface
//     auto it_cmd = std::find_if(
//       command_interfaces_.begin(), command_interfaces_.end(),
//       [&](const auto & ci) {
//         return (ci.get_prefix_name() == j) &&
//                (ci.get_interface_name() == std::string(hardware_interface::HW_IF_EFFORT));
//       });
//     if (it_cmd != command_interfaces_.end()) {
//       eff_cmd_if_[i] = const_cast<hardware_interface::LoanedCommandInterface *>(&(*it_cmd));
//     }
//   }

//   // Validate required interfaces
//   bool ok = true;
//   for (size_t i = 0; i < n; ++i) {
//     if (!pos_if_[i]) {
//       RCLCPP_ERROR(get_node()->get_logger(),
//         "Missing POSITION state interface for joint '%s'.", joints_[i].c_str());
//       ok = false;
//     }
//     if (!eff_cmd_if_[i]) {
//       RCLCPP_ERROR(get_node()->get_logger(),
//         "Missing EFFORT command interface for joint '%s'.", joints_[i].c_str());
//       ok = false;
//     }
//   }
//   return ok;
// }

// // FIX: on_activate completely rewritten.
// //   - Calls cache_interfaces_() so pos_if_/vel_if_/eff_cmd_if_ are valid before update().
// //   - Reads joint state through the correct cached interfaces (pos_if_[]).
// //   - Syncs x_d_frame_ to actual robot pose to prevent a torque step at activation.
// //   - Resets error filter and state machine correctly.
// //   - Removes all references to non-existent members (joint_positions_, joint_state_interfaces_,
// //     x_dot_error_filtered_, first_update_).
// controller_interface::CallbackReturn
// OmxVariableStiffnessController::on_activate(const rclcpp_lifecycle::State &)
// {
//   auto node = get_node();

//   RCLCPP_INFO(node->get_logger(),
//     "on_activate: %zu state interfaces, %zu command interfaces",
//     state_interfaces_.size(), command_interfaces_.size());

//   if (!cache_interfaces_()) {
//     RCLCPP_ERROR(node->get_logger(),
//       "Failed to cache required interfaces. Controller cannot run.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // Read initial joint positions through the validated cached interfaces
//   for (size_t i = 0; i < q_.rows(); ++i) {
//     double val = pos_if_[i]->get_value();
//     if (std::isnan(val)) {
//       RCLCPP_ERROR(node->get_logger(),
//         "Joint '%s' reports NaN position at activation.", joints_[i].c_str());
//       return controller_interface::CallbackReturn::ERROR;
//     }
//     q_(i)     = val;
//     q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
//   }

//   // Compute actual Cartesian pose and sync desired pose to it.
//   // This eliminates the torque step that would otherwise occur if x_d_frame_
//   // was already set to start_pos_ while the robot is somewhere else.
//   fk_solver_->JntToCart(q_, x_c_start_);
//   x_d_frame_   = x_c_start_;

//   // Reset error history and filter so damping term starts at zero
//   last_x_error_          = KDL::Twist::Zero();
//   x_error_dot_filtered_  = KDL::Twist::Zero();

//   // Reset stiffness to safe homing values
//   K_trans_current_ = stiffness_homing_;
//   D_trans_current_ = Kd_lin_default_;

//   // Start state machine
//   current_state_   = State::HOMING;
//   move_start_time_ = node->get_clock()->now().seconds();

//   // Zero all command interfaces initially
//   for (auto & ci : command_interfaces_) {
//     ci.set_value(0.0);
//   }

//   RCLCPP_INFO(node->get_logger(),
//     "[ACTIVATE] Initial EE pose: [%.4f, %.4f, %.4f]",
//     x_c_start_.p.x(), x_c_start_.p.y(), x_c_start_.p.z());

//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn
// OmxVariableStiffnessController::on_deactivate(const rclcpp_lifecycle::State &)
// {
//   for (auto & ci : command_interfaces_) {
//     ci.set_value(0.0);
//   }
//   RCLCPP_INFO(get_node()->get_logger(), "[STOPPING] Controller stopped.");
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// double OmxVariableStiffnessController::interpolate_profile_(
//   double s, const std::vector<double> & profile) const
// {
//   if (profile.empty()) {
//     return 0.0;
//   }
//   if (s <= 0.0) {
//     return profile.front();
//   }
//   if (s >= 1.0) {
//     return profile.back();
//   }

//   double scaled_s = s * static_cast<double>(profile.size() - 1);
//   size_t i        = static_cast<size_t>(std::floor(scaled_s));
//   size_t i_next   = std::min(i + 1, profile.size() - 1);
//   double s_local  = scaled_s - static_cast<double>(i);

//   return profile[i] * (1.0 - s_local) + profile[i_next] * s_local;
// }

// double OmxVariableStiffnessController::cos_interpolate_(double t, double T) const
// {
//   double s = std::min(t / T, 1.0);
//   return 0.5 * (1.0 - std::cos(s * M_PI));
// }

// controller_interface::return_type
// OmxVariableStiffnessController::update(
//   const rclcpp::Time & time, const rclcpp::Duration & period)
// {
//   auto node = get_node();
//   const size_t n = joints_.size();

//   if (n == 0 || !dyn_param_ || !jnt_to_jac_solver_ || !fk_solver_) {
//     return controller_interface::return_type::ERROR;
//   }

//   double dt     = period.seconds();
//   double time_s = time.seconds();

//   // ── 1. READ JOINT STATE ──────────────────────────────────────────────────
//   for (size_t i = 0; i < n; ++i) {
//     q_(i)     = pos_if_[i]->get_value();
//     q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
//   }

//   // Forward kinematics → actual Cartesian pose
//   KDL::Frame x_c_frame;
//   fk_solver_->JntToCart(q_, x_c_frame);

//   // Jacobian and gravity
//   jnt_to_jac_solver_->JntToJac(q_, jacobian_);
//   dyn_param_->JntToGravity(q_, G_);

//   // Manipulability check (minimum singular value of J)
//   double current_manipulability = compute_manipulability_();
//   if (current_manipulability < min_manipulability_thresh_) {
//     RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
//       "[SAFETY] Near singularity! sigma_min=%.4f (threshold=%.4f), DLS active",
//       current_manipulability, min_manipulability_thresh_);
//   }

//   // Cartesian velocity via Jacobian
//   Eigen::VectorXd x_c_dot_eigen = jacobian_.data * q_dot_.data;

//   // ── 2. TRAJECTORY STATE MACHINE ─────────────────────────────────────────
//   double s = 0.0;
//   KDL::Vector x_d_new = x_d_frame_.p;

//   switch (current_state_) {
//     case State::INIT:
//       // Transitioned in on_activate(); should not reach here during normal operation
//       break;

//     case State::HOMING:
//     {
//       double t     = time_s - move_start_time_;
//       double alpha = cos_interpolate_(t, homing_duration_);
//       x_d_new          = x_c_start_.p * (1.0 - alpha) + start_pos_ * alpha;
//       K_trans_current_ = stiffness_homing_;
//       D_trans_current_ = Kd_lin_default_;

//       if (t >= homing_duration_) {
//         current_state_   = State::MOVE_FORWARD;
//         move_start_time_ = time_s;
//         x_c_start_.p     = x_d_new;
//         RCLCPP_INFO(node->get_logger(), "[STATE] Homing complete → MOVE_FORWARD");
//       }
//       break;
//     }

//     case State::MOVE_FORWARD:
//     case State::MOVE_RETURN:
//     {
//       double t            = time_s - move_start_time_;
//       double move_t_frac  = std::min(t / move_duration_, 1.0);

//       s = (current_state_ == State::MOVE_FORWARD) ? move_t_frac : (1.0 - move_t_frac);

//       // Use IK-validated joint-space trajectory when available
//       if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()) {
//         KDL::JntArray q_desired(n);
//         if (get_trajectory_joints_(s, q_desired)) {
//           KDL::Frame x_desired_frame;
//           fk_solver_->JntToCart(q_desired, x_desired_frame);
//           x_d_new = x_desired_frame.p;
//         } else {
//           // Fallback: raw Cartesian interpolation
//           double alpha = cos_interpolate_(t, move_duration_);
//           x_d_new = (current_state_ == State::MOVE_FORWARD)
//             ? start_pos_ * (1.0 - alpha) + end_pos_ * alpha
//             : end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
//         }
//       } else {
//         double alpha = cos_interpolate_(t, move_duration_);
//         x_d_new = (current_state_ == State::MOVE_FORWARD)
//           ? start_pos_ * (1.0 - alpha) + end_pos_ * alpha
//           : end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
//       }

//       // Stiffness modulation from profile
//       if (!stiffness_profile_x_.empty()) {
//         KDL::Vector K_trans_desired;
//         K_trans_desired.x(interpolate_profile_(s, stiffness_profile_x_));
//         K_trans_desired.y(interpolate_profile_(s, stiffness_profile_y_));
//         K_trans_desired.z(interpolate_profile_(s, stiffness_profile_z_));

//         if (damping_profile_x_.size() == stiffness_profile_x_.size()) {
//           D_trans_current_.x(interpolate_profile_(s, damping_profile_x_));
//           D_trans_current_.y(interpolate_profile_(s, damping_profile_y_));
//           D_trans_current_.z(interpolate_profile_(s, damping_profile_z_));
//         } else {
//           D_trans_current_ = Kd_lin_default_;
//         }

//         // Rate-limited stiffness update (safety)
//         double max_change = max_rise_rate_ * dt;
//         for (int i = 0; i < 3; ++i) {
//           double delta = std::clamp(
//             K_trans_desired(i) - K_trans_current_(i), -max_change, max_change);
//           K_trans_current_(i) += delta;
//         }
//       } else {
//         K_trans_current_ = stiffness_homing_;
//         D_trans_current_ = Kd_lin_default_;
//       }

//       // State transitions
//       if (move_t_frac >= 1.0) {
//         if (current_state_ == State::MOVE_FORWARD) {
//           current_state_ = State::WAIT_AT_END;
//           RCLCPP_INFO(node->get_logger(), "[STATE] Reached end → WAIT_AT_END");
//         } else {
//           current_state_ = State::WAIT_AT_START;
//           RCLCPP_INFO(node->get_logger(), "[STATE] Returned to start → WAIT_AT_START");
//         }
//         move_start_time_ = time_s;
//       }
//       break;
//     }

//     case State::WAIT_AT_END:
//     {
//       x_d_new          = end_pos_;
//       K_trans_current_ = stiffness_homing_;
//       D_trans_current_ = Kd_lin_default_;

//       if (time_s - move_start_time_ >= wait_duration_) {
//         current_state_   = State::MOVE_RETURN;
//         move_start_time_ = time_s;
//         RCLCPP_INFO(node->get_logger(), "[STATE] → MOVE_RETURN");
//       }
//       break;
//     }

//     case State::WAIT_AT_START:
//     {
//       x_d_new          = start_pos_;
//       K_trans_current_ = stiffness_homing_;
//       D_trans_current_ = Kd_lin_default_;

//       if (time_s - move_start_time_ >= wait_duration_) {
//         current_state_   = State::MOVE_FORWARD;
//         move_start_time_ = time_s;
//         RCLCPP_INFO(node->get_logger(), "[STATE] → MOVE_FORWARD");
//       }
//       break;
//     }
//   }

//   x_d_frame_.p = x_d_new;

//   // Apply external waypoint override if active
//   x_d_frame_.p = process_waypoints_(x_d_frame_.p, time_s);

//   // Clamp stiffness/damping to hardware limits
//   apply_safety_limits_();

//   // ── 3. IMPEDANCE CONTROL LAW ─────────────────────────────────────────────
//   KDL::Twist x_error = KDL::diff(x_c_frame, x_d_frame_);

//   // FIX: Low-pass filtered error derivative to suppress noise amplification at 500 Hz.
//   // Raw finite-difference of error at 500 Hz produces large, noisy damping torques.
//   // alpha=0.1 is conservative; raise toward 0.3 if damping response feels sluggish.
//   KDL::Twist x_error_dot_raw;
//   if (dt > 1e-9) {
//     x_error_dot_raw.vel = (x_error.vel - last_x_error_.vel) * (1.0 / dt);
//     x_error_dot_raw.rot = (x_error.rot - last_x_error_.rot) * (1.0 / dt);
//   } else {
//     x_error_dot_raw = KDL::Twist::Zero();
//   }
//   // One-pole IIR low-pass filter
//   x_error_dot_filtered_.vel =
//     velocity_filter_alpha_ * x_error_dot_raw.vel +
//     (1.0 - velocity_filter_alpha_) * x_error_dot_filtered_.vel;
//   x_error_dot_filtered_.rot =
//     velocity_filter_alpha_ * x_error_dot_raw.rot +
//     (1.0 - velocity_filter_alpha_) * x_error_dot_filtered_.rot;

//   last_x_error_ = x_error;

//   // Spring force: F_s = K * x_err
//   Eigen::Matrix<double, 6, 1> spring_force;
//   spring_force << K_trans_current_.x() * x_error.vel.x(),
//                   K_trans_current_.y() * x_error.vel.y(),
//                   K_trans_current_.z() * x_error.vel.z(),
//                   K_rot_.x()           * x_error.rot.x(),
//                   K_rot_.y()           * x_error.rot.y(),
//                   K_rot_.z()           * x_error.rot.z();

//   // Damping force: F_d = D * x_error_dot (filtered)
//   Eigen::Matrix<double, 6, 1> damping_force;
//   damping_force << D_trans_current_.x() * x_error_dot_filtered_.vel.x(),
//                    D_trans_current_.y() * x_error_dot_filtered_.vel.y(),
//                    D_trans_current_.z() * x_error_dot_filtered_.vel.z(),
//                    Kd_ang_.x()          * x_error_dot_filtered_.rot.x(),
//                    Kd_ang_.y()          * x_error_dot_filtered_.rot.y(),
//                    Kd_ang_.z()          * x_error_dot_filtered_.rot.z();

//   Eigen::Matrix<double, 6, 1> F_total = spring_force + damping_force;

//   // Damped Least Squares (DLS) torque projection
//   // τ = J^T (JJ^T + λ²I)^{-1} JJ^T F
//   // Prevents torque blow-up near singularities by progressively attenuating
//   // forces in singular directions as λ² grows.
//   Eigen::MatrixXd J   = jacobian_.data;
//   Eigen::MatrixXd JJT = J * J.transpose();  // 6×6

//   // Adaptive λ²: scales from 0 up to dls_damping_factor_² as we approach singularity
//   double lambda_sq = 0.0;
//   const double onset_threshold = min_manipulability_thresh_ * 5.0;
//   if (current_manipulability < onset_threshold) {
//     double proximity = 1.0 - (current_manipulability / onset_threshold);
//     lambda_sq = dls_damping_factor_ * dls_damping_factor_ * proximity * proximity;
//   }

//   Eigen::Matrix<double, 6, 6> JJT_damped =
//     JJT + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
//   Eigen::Matrix<double, 6, 6> JJT_damped_inv = JJT_damped.inverse();

//   Eigen::Matrix<double, 6, 1> F_filtered     = JJT_damped_inv * JJT * F_total;
//   Eigen::VectorXd              tau_impedance  = J.transpose() * F_filtered;

//   // Final torque = gravity compensation + impedance torque, scaled for hardware
//   // torque_scale_ converts Nm → raw servo units (tune via YAML)
//   KDL::JntArray tau_total(n);
//   for (size_t i = 0; i < n; ++i) {
//     tau_total(i) = (G_(i) + tau_impedance(i)) * torque_scale_;
//   }

//   // ── 4. PUBLISH DIAGNOSTICS ───────────────────────────────────────────────

//   // Joint velocities
//   std_msgs::msg::Float64MultiArray joint_vel_msg;
//   for (size_t i = 0; i < n; ++i) {
//     joint_vel_msg.data.push_back(q_dot_(i));
//   }
//   joint_vel_pub_->publish(joint_vel_msg);

//   // End-effector position
//   geometry_msgs::msg::Point ee_pos_msg;
//   ee_pos_msg.x = x_c_frame.p.x();
//   ee_pos_msg.y = x_c_frame.p.y();
//   ee_pos_msg.z = x_c_frame.p.z();
//   ee_pos_pub_->publish(ee_pos_msg);

//   // End-effector orientation (RPY)
//   double roll, pitch, yaw;
//   x_c_frame.M.GetRPY(roll, pitch, yaw);
//   geometry_msgs::msg::Vector3 ee_orient_msg;
//   ee_orient_msg.x = roll;
//   ee_orient_msg.y = pitch;
//   ee_orient_msg.z = yaw;
//   ee_orient_pub_->publish(ee_orient_msg);

//   // End-effector Cartesian velocities (6D)
//   std_msgs::msg::Float64MultiArray ee_vel_msg;
//   for (int i = 0; i < 6; ++i) {
//     ee_vel_msg.data.push_back(x_c_dot_eigen(i));
//   }
//   ee_vel_pub_->publish(ee_vel_msg);

//   // Jacobian (flattened 6×N)
//   std_msgs::msg::Float64MultiArray jacobian_msg;
//   jacobian_msg.data.assign(
//     jacobian_.data.data(),
//     jacobian_.data.data() + jacobian_.data.size());
//   jacobian_pub_->publish(jacobian_msg);

//   // Manipulability metrics
//   publish_manipulability_metrics_();

//   // Torque values
//   std_msgs::msg::Float64MultiArray torque_msg;
//   for (size_t i = 0; i < n; ++i) {
//     torque_msg.data.push_back(tau_total(i));
//   }
//   torque_pub_->publish(torque_msg);

//   // Stiffness state [Ktx,Kty,Ktz, Krx,Kry,Krz, Dtx,Dty,Dtz, Drx,Dry,Drz]
//   std_msgs::msg::Float64MultiArray stiffness_msg;
//   stiffness_msg.data = {
//     K_trans_current_.x(), K_trans_current_.y(), K_trans_current_.z(),
//     K_rot_.x(),           K_rot_.y(),           K_rot_.z(),
//     D_trans_current_.x(), D_trans_current_.y(), D_trans_current_.z(),
//     Kd_ang_.x(),          Kd_ang_.y(),          Kd_ang_.z()
//   };
//   stiffness_pub_->publish(stiffness_msg);

//   // Actual Cartesian pose
//   geometry_msgs::msg::Pose actual_pose_msg;
//   actual_pose_msg.position.x = x_c_frame.p.x();
//   actual_pose_msg.position.y = x_c_frame.p.y();
//   actual_pose_msg.position.z = x_c_frame.p.z();
//   {
//     double qx, qy, qz, qw;
//     x_c_frame.M.GetQuaternion(qx, qy, qz, qw);
//     actual_pose_msg.orientation.x = qx;
//     actual_pose_msg.orientation.y = qy;
//     actual_pose_msg.orientation.z = qz;
//     actual_pose_msg.orientation.w = qw;
//   }
//   actual_pose_pub_->publish(actual_pose_msg);

//   // Desired Cartesian pose
//   geometry_msgs::msg::Pose desired_pose_msg;
//   desired_pose_msg.position.x = x_d_frame_.p.x();
//   desired_pose_msg.position.y = x_d_frame_.p.y();
//   desired_pose_msg.position.z = x_d_frame_.p.z();
//   {
//     double qx, qy, qz, qw;
//     x_d_frame_.M.GetQuaternion(qx, qy, qz, qw);
//     desired_pose_msg.orientation.x = qx;
//     desired_pose_msg.orientation.y = qy;
//     desired_pose_msg.orientation.z = qz;
//     desired_pose_msg.orientation.w = qw;
//   }
//   desired_pose_pub_->publish(desired_pose_msg);

//   // ── 5. COMMAND TORQUES TO HARDWARE ──────────────────────────────────────
//   // FIX: Use eff_cmd_if_[] (correct cached handles) and tau_total (correct variable).
//   // torque_scale_ is already folded into tau_total above — do NOT apply it again here.
//   for (size_t i = 0; i < n; ++i) {
//     eff_cmd_if_[i]->set_value(tau_total(i));
//   }

//   return controller_interface::return_type::OK;
// }

// void OmxVariableStiffnessController::apply_safety_limits_()
// {
//   K_trans_current_.x(std::clamp(K_trans_current_.x(), 0.0, MAX_CARTESIAN_STIFFNESS));
//   K_trans_current_.y(std::clamp(K_trans_current_.y(), 0.0, MAX_CARTESIAN_STIFFNESS));
//   K_trans_current_.z(std::clamp(K_trans_current_.z(), 0.0, MAX_CARTESIAN_STIFFNESS));

//   D_trans_current_.x(std::clamp(D_trans_current_.x(), 0.0, MAX_CARTESIAN_DAMPING));
//   D_trans_current_.y(std::clamp(D_trans_current_.y(), 0.0, MAX_CARTESIAN_DAMPING));
//   D_trans_current_.z(std::clamp(D_trans_current_.z(), 0.0, MAX_CARTESIAN_DAMPING));

//   K_rot_.x(std::clamp(K_rot_.x(), 0.0, MAX_ROTATIONAL_STIFFNESS));
//   K_rot_.y(std::clamp(K_rot_.y(), 0.0, MAX_ROTATIONAL_STIFFNESS));
//   K_rot_.z(std::clamp(K_rot_.z(), 0.0, MAX_ROTATIONAL_STIFFNESS));

//   Kd_ang_.x(std::clamp(Kd_ang_.x(), 0.0, MAX_ROTATIONAL_DAMPING));
//   Kd_ang_.y(std::clamp(Kd_ang_.y(), 0.0, MAX_ROTATIONAL_DAMPING));
//   Kd_ang_.z(std::clamp(Kd_ang_.z(), 0.0, MAX_ROTATIONAL_DAMPING));
// }

// void OmxVariableStiffnessController::publish_manipulability_metrics_()
// {
//   Eigen::MatrixXd J   = jacobian_.data;
//   Eigen::MatrixXd JJT = J * J.transpose();

//   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJT);
//   Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();  // ascending

//   double det            = JJT.determinant();
//   double lambda_min     = eigenvalues(0);
//   double lambda_max     = eigenvalues(eigenvalues.size() - 1);
//   double condition_number = (lambda_min > 1e-10)
//     ? std::sqrt(lambda_max / lambda_min)
//     : std::numeric_limits<double>::infinity();

//   double yoshikawa      = std::sqrt(std::max(det, 0.0));
//   double min_sv         = std::sqrt(std::max(lambda_min, 0.0));
//   double max_sv         = std::sqrt(std::max(lambda_max, 0.0));

//   // Format: [condition_number, det, yoshikawa, min_sv, max_sv, eigenvalue_0..5]
//   std_msgs::msg::Float64MultiArray msg;
//   msg.data.push_back(condition_number);
//   msg.data.push_back(det);
//   msg.data.push_back(yoshikawa);
//   msg.data.push_back(min_sv);
//   msg.data.push_back(max_sv);
//   for (int i = 0; i < eigenvalues.size(); ++i) {
//     msg.data.push_back(eigenvalues(i));
//   }
//   manipulability_pub_->publish(msg);
// }

// void OmxVariableStiffnessController::waypoint_callback_(
//   const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   WaypointCommand wp;
//   wp.position  = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//   wp.timestamp = get_node()->now().seconds();

//   std::string frame = msg->header.frame_id;
//   wp.is_offset = (frame == "offset" || frame == "relative");

//   {
//     std::lock_guard<std::mutex> lock(waypoint_mutex_);
//     waypoint_queue_.push(wp);
//   }

//   RCLCPP_INFO(get_node()->get_logger(),
//     "[WAYPOINT] Received %s waypoint: [%.3f, %.3f, %.3f], queue size: %zu",
//     wp.is_offset ? "OFFSET" : "ABSOLUTE",
//     wp.position.x(), wp.position.y(), wp.position.z(),
//     waypoint_queue_.size());
// }

// KDL::Vector OmxVariableStiffnessController::process_waypoints_(
//   const KDL::Vector & trajectory_target, double current_time)
// {
//   {
//     std::lock_guard<std::mutex> lock(waypoint_mutex_);

//     if (!has_active_waypoint_ && !waypoint_queue_.empty()) {
//       current_waypoint_     = waypoint_queue_.front();
//       waypoint_queue_.pop();
//       has_active_waypoint_  = true;
//       waypoint_mode_active_ = true;
//       waypoint_start_time_  = current_time;

//       if (current_waypoint_.is_offset) {
//         current_waypoint_.position = trajectory_target + current_waypoint_.position;
//       }
//       waypoint_blend_start_ = trajectory_target;

//       RCLCPP_INFO(get_node()->get_logger(),
//         "[WAYPOINT] Activating waypoint → [%.3f, %.3f, %.3f]",
//         current_waypoint_.position.x(),
//         current_waypoint_.position.y(),
//         current_waypoint_.position.z());
//     }
//   }

//   std_msgs::msg::Bool status_msg;
//   status_msg.data = has_active_waypoint_;
//   waypoint_active_pub_->publish(status_msg);

//   if (!has_active_waypoint_) {
//     waypoint_mode_active_ = false;
//     return trajectory_target;
//   }

//   double elapsed        = current_time - waypoint_start_time_;
//   double blend_progress = std::min(elapsed / waypoint_blend_duration_, 1.0);
//   double smooth         = 0.5 * (1.0 - std::cos(M_PI * blend_progress));

//   KDL::Vector blended;
//   blended.x(waypoint_blend_start_.x() +
//     smooth * (current_waypoint_.position.x() - waypoint_blend_start_.x()));
//   blended.y(waypoint_blend_start_.y() +
//     smooth * (current_waypoint_.position.y() - waypoint_blend_start_.y()));
//   blended.z(waypoint_blend_start_.z() +
//     smooth * (current_waypoint_.position.z() - waypoint_blend_start_.z()));

//   if (blend_progress >= 1.0) {
//     std::lock_guard<std::mutex> lock(waypoint_mutex_);
//     if (waypoint_queue_.empty()) {
//       RCLCPP_INFO(get_node()->get_logger(), "[WAYPOINT] Reached waypoint, holding position");
//     } else {
//       current_waypoint_    = waypoint_queue_.front();
//       waypoint_queue_.pop();
//       waypoint_start_time_ = current_time;
//       waypoint_blend_start_ = blended;

//       if (current_waypoint_.is_offset) {
//         current_waypoint_.position = blended + current_waypoint_.position;
//       }

//       RCLCPP_INFO(get_node()->get_logger(),
//         "[WAYPOINT] Next waypoint → [%.3f, %.3f, %.3f], remaining: %zu",
//         current_waypoint_.position.x(),
//         current_waypoint_.position.y(),
//         current_waypoint_.position.z(),
//         waypoint_queue_.size());
//     }
//   }

//   return blended;
// }

// void OmxVariableStiffnessController::clear_waypoints_()
// {
//   std::lock_guard<std::mutex> lock(waypoint_mutex_);
//   while (!waypoint_queue_.empty()) {
//     waypoint_queue_.pop();
//   }
//   has_active_waypoint_  = false;
//   waypoint_mode_active_ = false;
//   RCLCPP_INFO(get_node()->get_logger(),
//     "[WAYPOINT] Cleared all waypoints, returning to trajectory");
// }

// bool OmxVariableStiffnessController::load_joint_limits_(const std::string & urdf_xml)
// {
//   urdf::Model urdf_model;
//   if (!urdf_model.initString(urdf_xml)) {
//     RCLCPP_WARN(get_node()->get_logger(), "Failed to parse URDF for joint limits");
//     return false;
//   }

//   size_t n = joints_.size();
//   q_min_.resize(n);
//   q_max_.resize(n);

//   for (size_t i = 0; i < n; ++i) {
//     auto joint = urdf_model.getJoint(joints_[i]);
//     if (!joint) {
//       RCLCPP_WARN(get_node()->get_logger(),
//         "Joint '%s' not found in URDF", joints_[i].c_str());
//       return false;
//     }

//     if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::PRISMATIC) {
//       if (joint->limits) {
//         q_min_(i) = joint->limits->lower;
//         q_max_(i) = joint->limits->upper;
//         RCLCPP_DEBUG(get_node()->get_logger(),
//           "[LIMITS] Joint '%s': [%.3f, %.3f]",
//           joints_[i].c_str(), q_min_(i), q_max_(i));
//       } else {
//         q_min_(i) = -M_PI;
//         q_max_(i) =  M_PI;
//       }
//     } else if (joint->type == urdf::Joint::CONTINUOUS) {
//       q_min_(i) = -M_PI;
//       q_max_(i) =  M_PI;
//     } else {
//       RCLCPP_WARN(get_node()->get_logger(),
//         "Joint '%s' has unsupported type", joints_[i].c_str());
//       return false;
//     }
//   }

//   has_joint_limits_ = true;
//   return true;
// }

// bool OmxVariableStiffnessController::is_within_joint_limits_(const KDL::JntArray & q) const
// {
//   if (!has_joint_limits_) {
//     return true;
//   }
//   for (size_t i = 0; i < q.rows(); ++i) {
//     if (q(i) < q_min_(i) || q(i) > q_max_(i)) {
//       return false;
//     }
//   }
//   return true;
// }

// double OmxVariableStiffnessController::compute_manipulability_() const
// {
//   Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_.data);
//   Eigen::VectorXd sv = svd.singularValues();
//   return sv(sv.size() - 1);  // minimum singular value
// }

// bool OmxVariableStiffnessController::validate_and_compute_trajectory_()
// {
//   auto node = get_node();
//   const size_t n = joints_.size();

//   trajectory_joint_waypoints_.clear();
//   trajectory_joint_waypoints_.reserve(num_trajectory_samples_ + 1);

//   // Non-singular "elbow-bent" seed for OpenManipulator-X
//   KDL::JntArray q_init(n);
//   q_init(0) =  0.0;   // joint1: base rotation
//   q_init(1) = -0.5;   // joint2: shoulder down
//   q_init(2) =  1.0;   // joint3: elbow bent
//   q_init(3) =  0.5;   // joint4: wrist compensating

//   KDL::Rotation target_rot = KDL::Rotation::RPY(
//     target_orientation_euler_.x(),
//     target_orientation_euler_.y(),
//     target_orientation_euler_.z());

//   double min_manip = std::numeric_limits<double>::max();

//   for (size_t sample = 0; sample <= num_trajectory_samples_; ++sample) {
//     double s   = static_cast<double>(sample) / static_cast<double>(num_trajectory_samples_);
//     KDL::Vector pos = start_pos_ * (1.0 - s) + end_pos_ * s;
//     KDL::Frame  target_frame(target_rot, pos);

//     KDL::JntArray q_result(n);
//     int ik_ret = ik_solver_->CartToJnt(q_init, target_frame, q_result);

//     if (ik_ret < 0) {
//       RCLCPP_ERROR(node->get_logger(),
//         "[SAFETY ERROR] IK failed at s=%.2f, pos=[%.3f, %.3f, %.3f]",
//         s, pos.x(), pos.y(), pos.z());
//       return false;
//     }

//     if (!is_within_joint_limits_(q_result)) {
//       RCLCPP_ERROR(node->get_logger(),
//         "[SAFETY ERROR] IK violates joint limits at s=%.2f", s);
//       return false;
//     }

//     KDL::Jacobian jac(n);
//     jnt_to_jac_solver_->JntToJac(q_result, jac);

//     Eigen::JacobiSVD<Eigen::MatrixXd> svd(
//       jac.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
//     Eigen::VectorXd sv    = svd.singularValues();
//     double sigma_min      = sv(sv.size() - 1);

//     if (sample == 0) {
//       RCLCPP_INFO(node->get_logger(),
//         "[SAFETY] s=0 IK: q=[%.3f, %.3f, %.3f, %.3f], sigma_min=%.6f",
//         q_result(0), q_result(1), q_result(2), q_result(3), sigma_min);
//     }

//     if (sigma_min < min_manip) {
//       min_manip = sigma_min;
//     }

//     if (sigma_min < min_manipulability_thresh_) {
//       RCLCPP_ERROR(node->get_logger(),
//         "[SAFETY ERROR] Near singularity at s=%.2f (sigma_min=%.4f < thresh=%.4f)",
//         s, sigma_min, min_manipulability_thresh_);
//       return false;
//     }

//     trajectory_joint_waypoints_.push_back(q_result);
//     q_init = q_result;  // warm-start next IK from previous solution
//   }

//   RCLCPP_INFO(node->get_logger(),
//     "[SAFETY] Trajectory validated. Min sigma=%.4f (thresh=%.4f)",
//     min_manip, min_manipulability_thresh_);
//   return true;
// }

// bool OmxVariableStiffnessController::get_trajectory_joints_(
//   double s, KDL::JntArray & q_out) const
// {
//   if (trajectory_joint_waypoints_.empty()) {
//     return false;
//   }

//   s = std::clamp(s, 0.0, 1.0);

//   double idx_f  = s * static_cast<double>(trajectory_joint_waypoints_.size() - 1);
//   size_t idx_lo = static_cast<size_t>(std::floor(idx_f));
//   size_t idx_hi = std::min(idx_lo + 1, trajectory_joint_waypoints_.size() - 1);
//   double t      = idx_f - static_cast<double>(idx_lo);

//   const KDL::JntArray & q_lo = trajectory_joint_waypoints_[idx_lo];
//   const KDL::JntArray & q_hi = trajectory_joint_waypoints_[idx_hi];

//   q_out.resize(q_lo.rows());
//   for (size_t i = 0; i < q_out.rows(); ++i) {
//     q_out(i) = q_lo(i) * (1.0 - t) + q_hi(i) * t;
//   }
//   return true;
// }

// }  // namespace omx_variable_stiffness_controller

// PLUGINLIB_EXPORT_CLASS(
//   omx_variable_stiffness_controller::OmxVariableStiffnessController,
//   controller_interface::ControllerInterface)


// // #include "omx_variable_stiffness_controller/omx_variable_stiffness_controller.hpp"

// // #include <algorithm>
// // #include <cmath>
// // #include <string>
// // #include <vector>
// // #include <sstream>
// // #include <chrono>

// // #include <kdl/chaindynparam.hpp>
// // #include <kdl/jntarray.hpp>
// // #include <kdl_parser/kdl_parser.hpp>
// // #include <urdf/model.h>
// // #include <std_msgs/msg/string.hpp>
// // #include <rclcpp/wait_for_message.hpp>
// // #include <tf2_kdl/tf2_kdl.hpp>
// // #include <tf2_eigen/tf2_eigen.hpp>

// // #include "pluginlib/class_list_macros.hpp"

// // namespace omx_variable_stiffness_controller
// // {

// // // Helper: Convert KDL::Vector to a string for debugging
// // static std::string vector_to_string(const KDL::Vector & v)
// // {
// //   std::stringstream ss;
// //   ss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
// //   return ss.str();
// // }

// // controller_interface::InterfaceConfiguration
// // OmxVariableStiffnessController::state_interface_configuration() const
// // {
// //   controller_interface::InterfaceConfiguration cfg;
// //   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

// //   for (const auto & j : joints_) {
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_POSITION));
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_VELOCITY));
// //   }
// //   return cfg;
// // }

// // controller_interface::InterfaceConfiguration
// // OmxVariableStiffnessController::command_interface_configuration() const
// // {
// //   controller_interface::InterfaceConfiguration cfg;
// //   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

// //   for (const auto & j : joints_) {
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
// //   }
// //   return cfg;
// // }

// // controller_interface::CallbackReturn
// // OmxVariableStiffnessController::on_init()
// // {
// //   // Declare all parameters with defaults
// //   auto_declare<std::vector<std::string>>("joints", {});
// //   auto_declare<std::string>("root_link", "world");
// //   auto_declare<std::string>("tip_link", "");
// //   auto_declare<std::string>("robot_description_node", "/controller_manager");
// //   auto_declare<double>("torque_scale", 1.0);

// //   // Trajectory timing parameters
// //   auto_declare<double>("homing_duration", 5.0);
// //   auto_declare<double>("move_duration", 10.0);
// //   auto_declare<double>("wait_duration", 3.0);
// //   auto_declare<double>("max_rise_rate", 100.0);

// //   // Trajectory safety parameters
// //   auto_declare<bool>("use_joint_space_trajectory", true);
// //   auto_declare<double>("min_manipulability_threshold", 0.01);
// //   auto_declare<int>("num_trajectory_samples", 50);
// //   auto_declare<double>("dls_damping_factor", 0.05);

// //   // Position waypoints (3-element arrays)
// //   auto_declare<std::vector<double>>("start_position", {0.2, 0.0, 0.15});
// //   auto_declare<std::vector<double>>("end_position", {0.25, 0.0, 0.15});
// //   auto_declare<std::vector<double>>("target_orientation", {0.0, 0.0, 0.0});

// //   // Stiffness/damping parameters (3-element arrays)
// //   auto_declare<std::vector<double>>("stiffness_homing", {20.0, 20.0, 20.0});
// //   auto_declare<std::vector<double>>("stiffness_rot", {50.0, 50.0, 50.0});
// //   auto_declare<std::vector<double>>("damping_rot", {5.0, 5.0, 5.0});
// //   auto_declare<std::vector<double>>("damping_default", {20.0, 20.0, 20.0});

// //   // Stiffness/damping profiles (loaded from CSV or parameter server)
// //   auto_declare<std::vector<double>>("stiffness_profile_x", {});
// //   auto_declare<std::vector<double>>("stiffness_profile_y", {});
// //   auto_declare<std::vector<double>>("stiffness_profile_z", {});
// //   auto_declare<std::vector<double>>("damping_profile_x", {});
// //   auto_declare<std::vector<double>>("damping_profile_y", {});
// //   auto_declare<std::vector<double>>("damping_profile_z", {});

// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // bool OmxVariableStiffnessController::get_kdl_vector_param_(const std::string & name, KDL::Vector & vec)
// // {
// //   auto node = get_node();
// //   std::vector<double> params_vec;

// //   try {
// //     params_vec = node->get_parameter(name).as_double_array();
// //   } catch (...) {
// //     RCLCPP_ERROR(node->get_logger(), "[INIT ERR] Failed to get parameter '%s'", name.c_str());
// //     return false;
// //   }

// //   if (params_vec.size() != 3) {
// //     RCLCPP_ERROR(node->get_logger(),
// //       "[INIT ERR] Parameter '%s' must have exactly 3 elements, got %zu",
// //       name.c_str(), params_vec.size());
// //     return false;
// //   }

// //   vec = KDL::Vector(params_vec[0], params_vec[1], params_vec[2]);
// //   RCLCPP_INFO(node->get_logger(), "[INIT DBG] Parameter %s loaded: %s",
// //     name.c_str(), vector_to_string(vec).c_str());
// //   return true;
// // }

// // std::string OmxVariableStiffnessController::get_robot_description_from_node_(
// //   const std::string & node_name) const
// // {
// //   auto node = get_node();
// //   if (!node) {
// //     return {};
// //   }

// //   // Derive topic name from node name
// //   std::string topic_name;
// //   size_t last_slash = node_name.rfind('/');
// //   if (last_slash != std::string::npos && last_slash > 0) {
// //     topic_name = node_name.substr(0, last_slash) + "/robot_description";
// //   } else {
// //     topic_name = "/robot_description";
// //   }

// //   auto sub = node->create_subscription<std_msgs::msg::String>(
// //     topic_name,
// //     rclcpp::QoS(1).transient_local(),
// //     [](const std_msgs::msg::String::SharedPtr) {});

// //   std_msgs::msg::String msg;
// //   if (rclcpp::wait_for_message(msg, sub, node->get_node_options().context(),
// //       std::chrono::seconds(3)))
// //   {
// //     RCLCPP_INFO(node->get_logger(), "Got robot_description from topic '%s'", topic_name.c_str());
// //     return msg.data;
// //   }

// //   RCLCPP_WARN(node->get_logger(),
// //     "Timeout waiting for robot_description on topic '%s'",
// //     topic_name.c_str());
// //   return {};
// // }

// // std::string OmxVariableStiffnessController::get_robot_description_xml_() const
// // {
// //   auto node = get_node();
// //   if (!node) {
// //     return {};
// //   }

// //   std::string urdf_xml;

// //   // Priority 1: Explicitly configured node
// //   if (!robot_description_node_.empty() && robot_description_node_ != "/controller_manager") {
// //     urdf_xml = get_robot_description_from_node_(robot_description_node_);
// //     if (!urdf_xml.empty()) {
// //       return urdf_xml;
// //     }
// //   }

// //   // Priority 2: Local parameter
// //   if (node->get_parameter("robot_description", urdf_xml) && !urdf_xml.empty()) {
// //     return urdf_xml;
// //   }

// //   // Priority 3: Default configured node
// //   if (robot_description_node_.empty() || robot_description_node_ == "/controller_manager") {
// //     urdf_xml = get_robot_description_from_node_(robot_description_node_);
// //     if (!urdf_xml.empty()) {
// //       return urdf_xml;
// //     }
// //   }

// //   // Priority 4: Common fallback
// //   urdf_xml = get_robot_description_from_node_("/robot_state_publisher");
// //   return urdf_xml;
// // }

// // bool OmxVariableStiffnessController::build_kdl_from_robot_description_(const std::string & urdf_xml)
// // {
// //   if (urdf_xml.empty()) {
// //     return false;
// //   }
// //   return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
// // }

// // bool OmxVariableStiffnessController::build_chain_(const std::string & root, const std::string & tip)
// // {
// //   if (root.empty() || tip.empty()) {
// //     return false;
// //   }
// //   return kdl_tree_.getChain(root, tip, kdl_chain_);
// // }

// // controller_interface::CallbackReturn
// // OmxVariableStiffnessController::on_configure(const rclcpp_lifecycle::State &)
// // {
// //   auto node = get_node();

// //   velocity_filter_alpha_ = 0.1; 
  
// //   // Initialize KDL solvers (Keep your existing solver initialization)
// //   if (!kdl_chain_.getNrOfJoints()) {
// //       RCLCPP_ERROR(get_node()->get_logger(), "KDL chain empty!");
// //       return CallbackReturn::ERROR;
// //   }

// //   // Load basic parameters
// //   joints_ = node->get_parameter("joints").as_string_array();
// //   root_link_ = node->get_parameter("root_link").as_string();
// //   tip_link_ = node->get_parameter("tip_link").as_string();
// //   robot_description_node_ = node->get_parameter("robot_description_node").as_string();
// //   torque_scale_ = node->get_parameter("torque_scale").as_double();

// //   // Load timing parameters
// //   homing_duration_ = node->get_parameter("homing_duration").as_double();
// //   move_duration_ = node->get_parameter("move_duration").as_double();
// //   wait_duration_ = node->get_parameter("wait_duration").as_double();
// //   max_rise_rate_ = node->get_parameter("max_rise_rate").as_double();

// //   // Load trajectory safety parameters
// //   use_joint_space_trajectory_ = node->get_parameter("use_joint_space_trajectory").as_bool();
// //   min_manipulability_thresh_ = node->get_parameter("min_manipulability_threshold").as_double();
// //   num_trajectory_samples_ = static_cast<size_t>(
// //     node->get_parameter("num_trajectory_samples").as_int());
// //   dls_damping_factor_ = node->get_parameter("dls_damping_factor").as_double();

// //   // Validate required parameters
// //   if (joints_.empty()) {
// //     RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }
// //   if (tip_link_.empty()) {
// //     RCLCPP_ERROR(node->get_logger(), "Parameter 'tip_link' is empty.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   // Load KDL vector parameters
// //   if (!get_kdl_vector_param_("start_position", start_pos_) ||
// //       !get_kdl_vector_param_("end_position", end_pos_) ||
// //       !get_kdl_vector_param_("target_orientation", target_orientation_euler_) ||
// //       !get_kdl_vector_param_("stiffness_homing", stiffness_homing_) ||
// //       !get_kdl_vector_param_("stiffness_rot", K_rot_) ||
// //       !get_kdl_vector_param_("damping_rot", Kd_ang_) ||
// //       !get_kdl_vector_param_("damping_default", Kd_lin_default_))
// //   {
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   // Load stiffness/damping profiles (optional - may not be set)
// //   auto get_optional_double_array = [&node](const std::string & name) -> std::vector<double> {
// //     try {
// //       auto param = node->get_parameter(name);
// //       if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
// //         return param.as_double_array();
// //       }
// //     } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
// //       // Parameter not set - return empty
// //     } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
// //       // Parameter has no value - return empty
// //     }
// //     return {};
// //   };

// //   stiffness_profile_x_ = get_optional_double_array("stiffness_profile_x");
// //   stiffness_profile_y_ = get_optional_double_array("stiffness_profile_y");
// //   stiffness_profile_z_ = get_optional_double_array("stiffness_profile_z");
// //   damping_profile_x_ = get_optional_double_array("damping_profile_x");
// //   damping_profile_y_ = get_optional_double_array("damping_profile_y");
// //   damping_profile_z_ = get_optional_double_array("damping_profile_z");

// //   if (!stiffness_profile_x_.empty()) {
// //     RCLCPP_INFO(node->get_logger(), "[PROFILE] Loaded stiffness profiles with %zu points",
// //       stiffness_profile_x_.size());
// //   } else {
// //     RCLCPP_WARN(node->get_logger(),
// //       "[PROFILE WARN] No stiffness profiles loaded. Using fixed homing stiffness.");
// //   }

// //   // Build KDL chain from robot_description
// //   const auto urdf_xml = get_robot_description_xml_();
// //   if (urdf_xml.empty()) {
// //     RCLCPP_ERROR(node->get_logger(),
// //       "robot_description not found. Tried local param, '%s', and '/robot_state_publisher'.",
// //       robot_description_node_.c_str());
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   if (!build_kdl_from_robot_description_(urdf_xml)) {
// //     RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF into KDL tree.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   if (!build_chain_(root_link_, tip_link_)) {
// //     RCLCPP_ERROR(node->get_logger(),
// //       "Failed to extract KDL chain root='%s' tip='%s'.",
// //       root_link_.c_str(), tip_link_.c_str());
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   if (kdl_chain_.getNrOfJoints() != joints_.size()) {
// //     RCLCPP_ERROR(node->get_logger(),
// //       "KDL chain DOFs (%u) != joints size (%zu). root='%s' tip='%s'.",
// //       kdl_chain_.getNrOfJoints(), joints_.size(),
// //       root_link_.c_str(), tip_link_.c_str());
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   const size_t n = joints_.size();

// //   // Load joint limits from URDF
// //   if (!load_joint_limits_(urdf_xml)) {
// //     RCLCPP_WARN(node->get_logger(),
// //       "[SAFETY] Could not load joint limits from URDF. Joint limit checking disabled.");
// //   }

// //   // Initialize KDL solvers
// //   dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);
// //   jnt_to_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
// //   fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
// //   ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);

// //   // Initialize KDL arrays
// //   q_.resize(n);
// //   q_dot_.resize(n);
// //   tau_.resize(n);
// //   G_.resize(n);
// //   jacobian_.resize(n);
// //   q_min_.resize(n);
// //   q_max_.resize(n);

// //   // Initialize target orientation
// //   x_d_frame_.M = KDL::Rotation::RPY(
// //     target_orientation_euler_.x(),
// //     target_orientation_euler_.y(),
// //     target_orientation_euler_.z());

// //   // Initialize current stiffness/damping
// //   K_trans_current_ = stiffness_homing_;
// //   D_trans_current_ = Kd_lin_default_;
// //   current_state_ = State::INIT;

// //   // Clear cached interface pointers
// //   pos_if_.assign(n, nullptr);
// //   vel_if_.assign(n, nullptr);
// //   eff_cmd_if_.assign(n, nullptr);

// //   // Create publishers
// //   actual_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
// //     "~/cartesian_pose_actual", 100);
// //   desired_pose_pub_ = node->create_publisher<geometry_msgs::msg::Pose>(
// //     "~/cartesian_pose_desired", 100);
// //   stiffness_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/stiffness_state", 100);
// //   torque_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/torque_values", 10);
// //   jacobian_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/jacobian_values", 10);
// //   ee_pos_pub_ = node->create_publisher<geometry_msgs::msg::Point>(
// //     "~/end_effector_position", 10);
// //   joint_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/joint_velocities", 10);
// //   ee_vel_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/end_effector_velocities", 10);
// //   ee_orient_pub_ = node->create_publisher<geometry_msgs::msg::Vector3>(
// //     "~/end_effector_orientation", 10);
// //   manipulability_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
// //     "~/manipulability_metrics", 10);

// //   // Create subscriber for runtime stiffness profile updates
// //   stiffness_profile_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
// //     "~/stiffness_profile_update",
// //     10,
// //     std::bind(&OmxVariableStiffnessController::stiffness_profile_callback_, this,
// //       std::placeholders::_1));

// //   // Create subscriber for runtime waypoint commands
// //   // Allows external nodes to send target poses as deviations or absolute targets
// //   waypoint_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
// //     "~/waypoint_command",
// //     10,
// //     std::bind(&OmxVariableStiffnessController::waypoint_callback_, this,
// //       std::placeholders::_1));

// //   // Publisher for waypoint active status
// //   waypoint_active_pub_ = node->create_publisher<std_msgs::msg::Bool>(
// //     "~/waypoint_active", 10);

// //   RCLCPP_INFO(node->get_logger(),
// //     "[INIT] Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f",
// //     n, root_link_.c_str(), tip_link_.c_str(), torque_scale_);
// //   RCLCPP_INFO(node->get_logger(),
// //     "[INIT] Trajectory: start=%s end=%s homing_dur=%.1f move_dur=%.1f",
// //     vector_to_string(start_pos_).c_str(),
// //     vector_to_string(end_pos_).c_str(),
// //     homing_duration_, move_duration_);

// //     std::vector<double> start_pos_vec;
// //   if (get_node()->get_parameter("start_position", start_pos_vec) && start_pos_vec.size() == 3) {
// //       x_d_frame_.p = KDL::Vector(start_pos_vec[0], start_pos_vec[1], start_pos_vec[2]);
// //   }else { RCLCPP_ERROR(get_node()->get_logger(), "Failed to load 'start_position' parameter or it has incorrect size. Using default."); x_d_frame_.p = start_pos_; }


// //   // Validate and pre-compute joint-space trajectory if enabled
// //   if (use_joint_space_trajectory_) {
// //     RCLCPP_INFO(node->get_logger(),
// //       "[SAFETY] Validating trajectory via IK with %zu samples...",
// //       num_trajectory_samples_);
// //     if (!validate_and_compute_trajectory_()) {
// //       RCLCPP_ERROR(node->get_logger(),
// //         "[SAFETY ERROR] Trajectory validation failed! "
// //         "The straight-line path is not IK-feasible or passes through singularities. "
// //         "Adjust start_position/end_position or set use_joint_space_trajectory:=false");
// //       return controller_interface::CallbackReturn::ERROR;
// //     }
// //     RCLCPP_INFO(node->get_logger(),
// //       "[SAFETY] Trajectory validated successfully with %zu waypoints",
// //       trajectory_joint_waypoints_.size());
// //   } else {
// //     RCLCPP_WARN(node->get_logger(),
// //       "[SAFETY WARN] Joint-space trajectory validation DISABLED. "
// //       "Using raw Cartesian interpolation - may be unsafe!");
// //   }


// //   RCLCPP_INFO(get_node()->get_logger(), "Configuration successful. Filter Alpha: %.2f", velocity_filter_alpha_);
// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // void OmxVariableStiffnessController::stiffness_profile_callback_(
// //   const std_msgs::msg::Float64MultiArray::SharedPtr msg)
// // {
// //   // Expected format: [n, kx_0, kx_1, ..., ky_0, ky_1, ..., kz_0, kz_1, ...]
// //   // First element is the number of profile points
// //   if (msg->data.empty()) {
// //     RCLCPP_WARN(get_node()->get_logger(), "Received empty stiffness profile update");
// //     return;
// //   }

// //   size_t n = static_cast<size_t>(msg->data[0]);
// //   if (msg->data.size() != 1 + 3 * n) {
// //     RCLCPP_ERROR(get_node()->get_logger(),
// //       "Invalid stiffness profile size. Expected %zu, got %zu", 1 + 3 * n, msg->data.size());
// //     return;
// //   }

// //   stiffness_profile_x_.clear();
// //   stiffness_profile_y_.clear();
// //   stiffness_profile_z_.clear();

// //   for (size_t i = 0; i < n; ++i) {
// //     stiffness_profile_x_.push_back(msg->data[1 + i]);
// //     stiffness_profile_y_.push_back(msg->data[1 + n + i]);
// //     stiffness_profile_z_.push_back(msg->data[1 + 2 * n + i]);
// //   }

// //   RCLCPP_INFO(get_node()->get_logger(),
// //     "Updated stiffness profiles with %zu points", n);
// // }

// // bool OmxVariableStiffnessController::cache_interfaces_()
// // {
// //   const size_t n = joints_.size();
// //   if (n == 0) {
// //     return false;
// //   }

// //   for (size_t i = 0; i < n; ++i) {
// //     const auto & j = joints_[i];

// //     // Position interface
// //     auto it_pos = std::find_if(
// //       state_interfaces_.begin(), state_interfaces_.end(),
// //       [&](const auto & si) {
// //         return (si.get_prefix_name() == j) &&
// //                (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
// //       });
// //     if (it_pos != state_interfaces_.end()) {
// //       pos_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_pos));
// //     }

// //     // Velocity interface
// //     auto it_vel = std::find_if(
// //       state_interfaces_.begin(), state_interfaces_.end(),
// //       [&](const auto & si) {
// //         return (si.get_prefix_name() == j) &&
// //                (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
// //       });
// //     if (it_vel != state_interfaces_.end()) {
// //       vel_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_vel));
// //     }

// //     // Effort command interface
// //     auto it_cmd = std::find_if(
// //       command_interfaces_.begin(), command_interfaces_.end(),
// //       [&](const auto & ci) {
// //         return (ci.get_prefix_name() == j) &&
// //                (ci.get_interface_name() == std::string(hardware_interface::HW_IF_EFFORT));
// //       });
// //     if (it_cmd != command_interfaces_.end()) {
// //       eff_cmd_if_[i] = const_cast<hardware_interface::LoanedCommandInterface *>(&(*it_cmd));
// //     }
// //   }

// //   // Validate required interfaces
// //   bool ok = true;
// //   for (size_t i = 0; i < n; ++i) {
// //     if (!pos_if_[i]) {
// //       RCLCPP_ERROR(get_node()->get_logger(),
// //         "Missing POSITION state interface for joint '%s'.", joints_[i].c_str());
// //       ok = false;
// //     }
// //     if (!eff_cmd_if_[i]) {
// //       RCLCPP_ERROR(get_node()->get_logger(),
// //         "Missing EFFORT command interface for joint '%s'.", joints_[i].c_str());
// //       ok = false;
// //     }
// //   }
// //   return ok;
// // }

// // controller_interface::CallbackReturn
// // // OmxVariableStiffnessController::on_activate(const rclcpp_lifecycle::State &)
// // // {
// // //   auto node = get_node();

// // //   RCLCPP_INFO(node->get_logger(),
// // //     "on_activate: %zu state interfaces, %zu command interfaces",
// // //     state_interfaces_.size(), command_interfaces_.size());

// // //   if (!cache_interfaces_()) {
// // //     RCLCPP_ERROR(node->get_logger(),
// // //       "Failed to cache required interfaces. Controller cannot run.");
// // //     return controller_interface::CallbackReturn::ERROR;
// // //   }

// // //   // Read initial joint positions and compute starting Cartesian pose
// // //   for (size_t i = 0; i < q_.rows(); ++i) {
// // //     q_(i) = pos_if_[i]->get_value();
// // //     q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
// // //   }
// // //   fk_solver_->JntToCart(q_, x_c_start_);

// // //   // Set initial desired pose to current pose
// // //   x_d_frame_ = x_c_start_;
// // //   last_x_error_ = KDL::Twist::Zero();

// // //   // Start state machine
// // //   current_state_ = State::HOMING;
// // //   move_start_time_ = node->get_clock()->now().seconds();

// // //   // Zero command interfaces initially
// // //   for (auto & ci : command_interfaces_) {
// // //     (void)ci.set_value(0.0);
// // //   }

// // //   RCLCPP_INFO(node->get_logger(),
// // //     "[STARTING] Initial EE pose: p=%s",
// // //     vector_to_string(x_c_start_.p).c_str());

// // //   return controller_interface::CallbackReturn::SUCCESS;
// // // }

// // CallbackReturn OmxVariableStiffnessController::on_activate(const rclcpp_lifecycle::State &)
// // {
// //   // 1. Clear buffers
// //   std::fill(joint_positions_.begin(), joint_positions_.end(), 0.0);
// //   std::fill(joint_velocities_.begin(), joint_velocities_.end(), 0.0);
  
// //   // 2. Read initial hardware state
// //   for (size_t i = 0; i < joints_.size(); ++i) {
// //     if (std::isnan(joint_state_interfaces_[i].get_value())) {
// //       RCLCPP_ERROR(get_node()->get_logger(), "Joint %s has NaN value!", joints_[i].c_str());
// //       return CallbackReturn::ERROR;
// //     }
// //     q_(i) = joint_state_interfaces_[i].get_value();
// //   }

// //   // 3. [CRITICAL FIX] Sync Desired Pose to Actual Pose
// //   fk_solver_->JntToCart(q_, x_c_frame); 
// //   x_d_frame_ = x_c_frame;  // <--- The Magic Line. Target = Current.
  
// //   // 4. Reset Filters and History
// //   x_dot_error_filtered_ = KDL::Twist::Zero();
// //   last_x_error_ = KDL::Twist::Zero();
// //   first_update_ = true;

// //   RCLCPP_INFO(get_node()->get_logger(), "Activated. Holding current pose: [%f, %f, %f]", 
// //     x_c_frame.p.x(), x_c_frame.p.y(), x_c_frame.p.z());
    
// //   return CallbackReturn::SUCCESS;
// // }

// // controller_interface::CallbackReturn
// // OmxVariableStiffnessController::on_deactivate(const rclcpp_lifecycle::State &)
// // {
// //   for (auto & ci : command_interfaces_) {
// //     (void)ci.set_value(0.0);
// //   }
// //   RCLCPP_INFO(get_node()->get_logger(), "[STOPPING] Controller stopped.");
// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // double OmxVariableStiffnessController::interpolate_profile_(
// //   double s, const std::vector<double> & profile) const
// // {
// //   if (profile.empty()) {
// //     return 0.0;
// //   }
// //   if (s <= 0.0) {
// //     return profile.front();
// //   }
// //   if (s >= 1.0) {
// //     return profile.back();
// //   }

// //   double scaled_s = s * (profile.size() - 1);
// //   size_t i = static_cast<size_t>(std::floor(scaled_s));
// //   size_t i_next = std::min(i + 1, profile.size() - 1);
// //   double s_local = scaled_s - i;

// //   return profile[i] * (1.0 - s_local) + profile[i_next] * s_local;
// // }

// // double OmxVariableStiffnessController::cos_interpolate_(double t, double T) const
// // {
// //   double s = std::min(t / T, 1.0);
// //   return 0.5 * (1.0 - std::cos(s * M_PI));
// // }

// // controller_interface::return_type
// // OmxVariableStiffnessController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
// // {
// //   auto node = get_node();
// //   const size_t n = joints_.size();

// //   if (n == 0 || !dyn_param_ || !jnt_to_jac_solver_ || !fk_solver_) {
// //     return controller_interface::return_type::ERROR;
// //   }

// //   double dt = period.seconds();
// //   double time_s = time.seconds();

// //   // 1. READ JOINT STATE & KINEMATICS/DYNAMICS
// //   for (size_t i = 0; i < n; ++i) {
// //     q_(i) = pos_if_[i]->get_value();
// //     q_dot_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
// //   }

// //   // Compute Forward Kinematics (actual pose)
// //   KDL::Frame x_c_frame;
// //   fk_solver_->JntToCart(q_, x_c_frame);

// //   // Compute Jacobian and Gravity torques
// //   jnt_to_jac_solver_->JntToJac(q_, jacobian_);
// //   dyn_param_->JntToGravity(q_, G_);

// //   // Compute manipulability for DLS damping (sigma_min of Jacobian)
// //   double current_manipulability = compute_manipulability_();
// //   if (current_manipulability < min_manipulability_thresh_) {
// //     RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
// //       "[SAFETY] Near singularity! sigma_min=%.4f (threshold=%.4f), DLS active",
// //       current_manipulability, min_manipulability_thresh_);
// //   }

// //   // Compute Cartesian velocity
// //   Eigen::VectorXd x_c_dot_eigen = jacobian_.data * q_dot_.data;

// //   // 2. TRAJECTORY GENERATION & STATE MACHINE
// //   double s = 0.0;
// //   KDL::Vector x_d_new = x_d_frame_.p;

// //   switch (current_state_) {
// //     case State::INIT:
// //       // Should not reach here; immediately transitions in on_activate
// //       break;

// //     case State::HOMING:
// //     {
// //       double t = time_s - move_start_time_;
// //       double alpha = cos_interpolate_(t, homing_duration_);
// //       x_d_new = x_c_start_.p * (1.0 - alpha) + start_pos_ * alpha;
// //       K_trans_current_ = stiffness_homing_;
// //       D_trans_current_ = Kd_lin_default_;

// //       if (t >= homing_duration_) {
// //         current_state_ = State::MOVE_FORWARD;
// //         move_start_time_ = time_s;
// //         x_c_start_.p = x_d_new;
// //         RCLCPP_INFO(node->get_logger(), "[STATE] Homing complete, starting MOVE_FORWARD");
// //       }
// //       break;
// //     }

// //     case State::MOVE_FORWARD:
// //     case State::MOVE_RETURN:
// //     {
// //       double t = time_s - move_start_time_;
// //       double move_t_frac = std::min(t / move_duration_, 1.0);

// //       if (current_state_ == State::MOVE_FORWARD) {
// //         s = move_t_frac;
// //       } else {  // MOVE_RETURN
// //         s = 1.0 - move_t_frac;
// //       }

// //       // Use validated joint-space trajectory if available
// //       if (use_joint_space_trajectory_ && !trajectory_joint_waypoints_.empty()) {
// //         // Get interpolated joint position and compute FK for desired Cartesian pose
// //         KDL::JntArray q_desired(joints_.size());
// //         if (get_trajectory_joints_(s, q_desired)) {
// //           KDL::Frame x_desired_frame;
// //           fk_solver_->JntToCart(q_desired, x_desired_frame);
// //           x_d_new = x_desired_frame.p;
// //         } else {
// //           // Fallback to Cartesian interpolation
// //           double alpha = cos_interpolate_(t, move_duration_);
// //           if (current_state_ == State::MOVE_FORWARD) {
// //             x_d_new = start_pos_ * (1.0 - alpha) + end_pos_ * alpha;
// //           } else {
// //             x_d_new = end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
// //           }
// //         }
// //       } else {
// //         // Raw Cartesian interpolation (less safe)
// //         double alpha = cos_interpolate_(t, move_duration_);
// //         if (current_state_ == State::MOVE_FORWARD) {
// //           x_d_new = start_pos_ * (1.0 - alpha) + end_pos_ * alpha;
// //         } else {
// //           x_d_new = end_pos_ * (1.0 - alpha) + start_pos_ * alpha;
// //         }
// //       }

// //       // STIFFNESS MODULATION: Sample & Filter
// //       if (!stiffness_profile_x_.empty()) {
// //         KDL::Vector K_trans_desired;
// //         K_trans_desired.x(interpolate_profile_(s, stiffness_profile_x_));
// //         K_trans_desired.y(interpolate_profile_(s, stiffness_profile_y_));
// //         K_trans_desired.z(interpolate_profile_(s, stiffness_profile_z_));

// //         // Damping profiles (if available)
// //         if (damping_profile_x_.size() == stiffness_profile_x_.size()) {
// //           D_trans_current_.x(interpolate_profile_(s, damping_profile_x_));
// //           D_trans_current_.y(interpolate_profile_(s, damping_profile_y_));
// //           D_trans_current_.z(interpolate_profile_(s, damping_profile_z_));
// //         } else {
// //           D_trans_current_ = Kd_lin_default_;
// //         }

// //         // STIFFNESS FILTER (SAFETY)
// //         double max_change = max_rise_rate_ * dt;
// //         for (int i = 0; i < 3; ++i) {
// //           double current_k = K_trans_current_(i);
// //           double desired_k = K_trans_desired(i);
// //           double delta_k = desired_k - current_k;
// //           double filtered_delta = std::clamp(delta_k, -max_change, max_change);
// //           K_trans_current_(i) = current_k + filtered_delta;
// //         }
// //       } else {
// //         K_trans_current_ = stiffness_homing_;
// //         D_trans_current_ = Kd_lin_default_;
// //       }

// //       // State Transition
// //       if (move_t_frac >= 1.0) {
// //         if (current_state_ == State::MOVE_FORWARD) {
// //           current_state_ = State::WAIT_AT_END;
// //           RCLCPP_INFO(node->get_logger(), "[STATE] Reached end, waiting");
// //         } else {
// //           current_state_ = State::WAIT_AT_START;
// //           RCLCPP_INFO(node->get_logger(), "[STATE] Returned to start, waiting");
// //         }
// //         move_start_time_ = time_s;
// //       }
// //       break;
// //     }

// //     case State::WAIT_AT_END:
// //     {
// //       x_d_new = end_pos_;
// //       K_trans_current_ = stiffness_homing_;
// //       D_trans_current_ = Kd_lin_default_;

// //       if (time_s - move_start_time_ >= wait_duration_) {
// //         current_state_ = State::MOVE_RETURN;
// //         move_start_time_ = time_s;
// //         RCLCPP_INFO(node->get_logger(), "[STATE] Starting MOVE_RETURN");
// //       }
// //       break;
// //     }

// //     case State::WAIT_AT_START:
// //     {
// //       x_d_new = start_pos_;
// //       K_trans_current_ = stiffness_homing_;
// //       D_trans_current_ = Kd_lin_default_;

// //       if (time_s - move_start_time_ >= wait_duration_) {
// //         current_state_ = State::MOVE_FORWARD;
// //         move_start_time_ = time_s;
// //         RCLCPP_INFO(node->get_logger(), "[STATE] Starting MOVE_FORWARD");
// //       }
// //       break;
// //     }
// //   }

// //   // Update desired position from trajectory
// //   x_d_frame_.p = x_d_new;

// //   // WAYPOINT OVERRIDE: If waypoints are queued, blend towards them
// //   // This allows external nodes to command deviations from the trajectory
// //   x_d_frame_.p = process_waypoints_(x_d_frame_.p, time_s);

// //   // Apply hardware safety limits before control law
// //   apply_safety_limits_();

// //   // 3. IMPEDANCE CONTROL LAW
// //   KDL::Twist x_error = KDL::diff(x_c_frame, x_d_frame_);
// //   KDL::Twist x_error_dot;
// //   if (dt > 1e-9) {
// //     x_error_dot.vel = (x_error.vel - last_x_error_.vel) / dt;
// //     x_error_dot.rot = (x_error.rot - last_x_error_.rot) / dt;
// //   } else {
// //     x_error_dot = KDL::Twist::Zero();
// //   }
// //   last_x_error_ = x_error;

// //   // Spring Force: F_s = K * X_err
// //   Eigen::Matrix<double, 6, 1> spring_force;
// //   spring_force << K_trans_current_.x() * x_error.vel.x(),
// //                   K_trans_current_.y() * x_error.vel.y(),
// //                   K_trans_current_.z() * x_error.vel.z(),
// //                   K_rot_.x() * x_error.rot.x(),
// //                   K_rot_.y() * x_error.rot.y(),
// //                   K_rot_.z() * x_error.rot.z();

// //   // Damping Force: F_d = Kd * X_dot_err
// //   Eigen::Matrix<double, 6, 1> damping_force;
// //   damping_force << D_trans_current_.x() * x_error_dot.vel.x(),
// //                    D_trans_current_.y() * x_error_dot.vel.y(),
// //                    D_trans_current_.z() * x_error_dot.vel.z(),
// //                    Kd_ang_.x() * x_error_dot.rot.x(),
// //                    Kd_ang_.y() * x_error_dot.rot.y(),
// //                    Kd_ang_.z() * x_error_dot.rot.z();

// //   Eigen::Matrix<double, 6, 1> F_total = spring_force + damping_force;

// //   // Convert Cartesian Force to Joint Torques using Damped Least Squares (DLS)
// //   // Standard: τ = J^T * F
// //   // DLS: τ = J^T (JJ^T + λ²I)^{-1} JJ^T * F  (filters singular directions)
// //   // This prevents torque amplification near singularities
// //   Eigen::MatrixXd J = jacobian_.data;
// //   Eigen::MatrixXd JJT = J * J.transpose();  // 6x6
  
// //   // Adaptive damping: λ² increases as manipulability decreases
// //   double lambda_sq = 0.0;
// //   if (current_manipulability < min_manipulability_thresh_ * 5.0) {
// //     // Damping factor grows as we approach singularity
// //     double proximity = 1.0 - (current_manipulability / (min_manipulability_thresh_ * 5.0));
// //     lambda_sq = dls_damping_factor_ * dls_damping_factor_ * proximity * proximity;
// //   }
  
// //   // Regularized inverse: (JJ^T + λ²I)^{-1}
// //   Eigen::Matrix<double, 6, 6> JJT_damped = JJT + lambda_sq * Eigen::Matrix<double, 6, 6>::Identity();
// //   Eigen::Matrix<double, 6, 6> JJT_damped_inv = JJT_damped.inverse();
  
// //   // DLS force projection: filters out singular components
// //   Eigen::Matrix<double, 6, 1> F_filtered = JJT_damped_inv * JJT * F_total;
  
// //   // Joint torques via Jacobian transpose
// //   Eigen::VectorXd tau_impedance_vec = J.transpose() * F_filtered;

// //   // Final Control Torque: tau_total = tau_gravity + tau_impedance
// //   // Gravity compensation is maintained even near singularities for safety
// //   KDL::JntArray tau_total(n);
// //   for (size_t i = 0; i < n; ++i) {
// //     tau_total(i) = (G_(i) + tau_impedance_vec(i)) * torque_scale_;
// //   }

// //   // 4. PUBLISH DATA

// //   // Joint Velocities
// //   std_msgs::msg::Float64MultiArray joint_vel_msg;
// //   for (size_t i = 0; i < n; ++i) {
// //     joint_vel_msg.data.push_back(q_dot_(i));
// //   }
// //   joint_vel_pub_->publish(joint_vel_msg);

// //   // End Effector Position
// //   geometry_msgs::msg::Point ee_pos_msg;
// //   ee_pos_msg.x = x_c_frame.p.x();
// //   ee_pos_msg.y = x_c_frame.p.y();
// //   ee_pos_msg.z = x_c_frame.p.z();
// //   ee_pos_pub_->publish(ee_pos_msg);

// //   // End Effector Orientation (RPY)
// //   double roll, pitch, yaw;
// //   x_c_frame.M.GetRPY(roll, pitch, yaw);
// //   geometry_msgs::msg::Vector3 ee_orient_msg;
// //   ee_orient_msg.x = roll;
// //   ee_orient_msg.y = pitch;
// //   ee_orient_msg.z = yaw;
// //   ee_orient_pub_->publish(ee_orient_msg);

// //   // End Effector Cartesian Velocities (6D)
// //   std_msgs::msg::Float64MultiArray ee_vel_msg;
// //   for (int i = 0; i < 6; ++i) {
// //     ee_vel_msg.data.push_back(x_c_dot_eigen(i));
// //   }
// //   ee_vel_pub_->publish(ee_vel_msg);

// //   // Jacobian (Flattened 6xN matrix)
// //   std_msgs::msg::Float64MultiArray jacobian_msg;
// //   jacobian_msg.data.assign(jacobian_.data.data(),
// //     jacobian_.data.data() + jacobian_.data.size());
// //   jacobian_pub_->publish(jacobian_msg);

// //   // Manipulability metrics (JJ^T analysis)
// //   publish_manipulability_metrics_();

// //   // Torques
// //   std_msgs::msg::Float64MultiArray torque_msg;
// //   for (size_t i = 0; i < n; ++i) {
// //     torque_msg.data.push_back(tau_total(i));
// //   }
// //   torque_pub_->publish(torque_msg);

// //   // Stiffness State (K_trans, K_rot, D_trans, D_rot)
// //   std_msgs::msg::Float64MultiArray stiffness_msg;
// //   stiffness_msg.data.push_back(K_trans_current_.x());
// //   stiffness_msg.data.push_back(K_trans_current_.y());
// //   stiffness_msg.data.push_back(K_trans_current_.z());
// //   stiffness_msg.data.push_back(K_rot_.x());
// //   stiffness_msg.data.push_back(K_rot_.y());
// //   stiffness_msg.data.push_back(K_rot_.z());
// //   stiffness_msg.data.push_back(D_trans_current_.x());
// //   stiffness_msg.data.push_back(D_trans_current_.y());
// //   stiffness_msg.data.push_back(D_trans_current_.z());
// //   stiffness_msg.data.push_back(Kd_ang_.x());
// //   stiffness_msg.data.push_back(Kd_ang_.y());
// //   stiffness_msg.data.push_back(Kd_ang_.z());
// //   stiffness_pub_->publish(stiffness_msg);

// //   // Actual and Desired Pose
// //   geometry_msgs::msg::Pose actual_pose_msg;
// //   actual_pose_msg.position.x = x_c_frame.p.x();
// //   actual_pose_msg.position.y = x_c_frame.p.y();
// //   actual_pose_msg.position.z = x_c_frame.p.z();
// //   double x, y, z, w;
// //   x_c_frame.M.GetQuaternion(x, y, z, w);
// //   actual_pose_msg.orientation.x = x;
// //   actual_pose_msg.orientation.y = y;
// //   actual_pose_msg.orientation.z = z;
// //   actual_pose_msg.orientation.w = w;
// //   actual_pose_pub_->publish(actual_pose_msg);

// //   geometry_msgs::msg::Pose desired_pose_msg;
// //   desired_pose_msg.position.x = x_d_frame_.p.x();
// //   desired_pose_msg.position.y = x_d_frame_.p.y();
// //   desired_pose_msg.position.z = x_d_frame_.p.z();
// //   x_d_frame_.M.GetQuaternion(x, y, z, w);
// //   desired_pose_msg.orientation.x = x;
// //   desired_pose_msg.orientation.y = y;
// //   desired_pose_msg.orientation.z = z;
// //   desired_pose_msg.orientation.w = w;
// //   desired_pose_pub_->publish(desired_pose_msg);

// //   // 5. COMMAND TORQUES TO HARDWARE
// //   // for (size_t i = 0; i < n; ++i) {
// //   //   (void)eff_cmd_if_[i]->set_value(tau_total(i));
// //   // }
// //   for (size_t i = 0; i < joints_.size(); ++i) {
// //      // Add Gravity Compensation (if not already handled by hardware/firmware)
// //      // tau_cmd_interfaces_[i].set_value(tau_eigen(i) + G_(i)); 
     
// //      // Note: If using OpenManipulator-X, ensure 'torque_scale' is applied 
// //      // if the hardware interface expects raw currents instead of Nm.
// // double scaled_torque = tau_eigen(i) * torque_scale_; tau_cmd_interfaces_[i].set_value(scaled_torque); } 
// // return controller_interface::return_type::OK; 
// // } 

// // void OmxVariableStiffnessController::apply_safety_limits_()
// // {
// //   // Clamp translational stiffness to hardware limits
// //   K_trans_current_.x(std::clamp(K_trans_current_.x(), 0.0, MAX_CARTESIAN_STIFFNESS));
// //   K_trans_current_.y(std::clamp(K_trans_current_.y(), 0.0, MAX_CARTESIAN_STIFFNESS));
// //   K_trans_current_.z(std::clamp(K_trans_current_.z(), 0.0, MAX_CARTESIAN_STIFFNESS));

// //   // Clamp translational damping to hardware limits
// //   D_trans_current_.x(std::clamp(D_trans_current_.x(), 0.0, MAX_CARTESIAN_DAMPING));
// //   D_trans_current_.y(std::clamp(D_trans_current_.y(), 0.0, MAX_CARTESIAN_DAMPING));
// //   D_trans_current_.z(std::clamp(D_trans_current_.z(), 0.0, MAX_CARTESIAN_DAMPING));

// //   // Clamp rotational stiffness to hardware limits
// //   K_rot_.x(std::clamp(K_rot_.x(), 0.0, MAX_ROTATIONAL_STIFFNESS));
// //   K_rot_.y(std::clamp(K_rot_.y(), 0.0, MAX_ROTATIONAL_STIFFNESS));
// //   K_rot_.z(std::clamp(K_rot_.z(), 0.0, MAX_ROTATIONAL_STIFFNESS));

// //   // Clamp rotational damping to hardware limits
// //   Kd_ang_.x(std::clamp(Kd_ang_.x(), 0.0, MAX_ROTATIONAL_DAMPING));
// //   Kd_ang_.y(std::clamp(Kd_ang_.y(), 0.0, MAX_ROTATIONAL_DAMPING));
// //   Kd_ang_.z(std::clamp(Kd_ang_.z(), 0.0, MAX_ROTATIONAL_DAMPING));
// // }

// // void OmxVariableStiffnessController::publish_manipulability_metrics_()
// // {
// //   // Compute JJ^T (6x6 matrix) for manipulability analysis
// //   // This gives insight into stiffness transformation and singularity proximity
// //   Eigen::MatrixXd J = jacobian_.data;                    // 6xN Jacobian
// //   Eigen::MatrixXd JJT = J * J.transpose();               // 6x6 JJ^T

// //   // Compute eigenvalues of JJ^T using SelfAdjointEigenSolver (JJ^T is symmetric positive semi-definite)
// //   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(JJT);
// //   Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();      // Sorted ascending

// //   // Compute determinant (product of eigenvalues, squared Yoshikawa manipulability)
// //   double det = JJT.determinant();

// //   // Compute condition number = sqrt(lambda_max / lambda_min)
// //   // High condition number indicates proximity to singularity
// //   double lambda_min = eigenvalues(0);
// //   double lambda_max = eigenvalues(eigenvalues.size() - 1);
// //   double condition_number = 0.0;
// //   if (lambda_min > 1e-10) {
// //     condition_number = std::sqrt(lambda_max / lambda_min);
// //   } else {
// //     condition_number = std::numeric_limits<double>::infinity();
// //   }

// //   // Yoshikawa manipulability measure: w = sqrt(det(JJ^T))
// //   double yoshikawa_manipulability = std::sqrt(std::max(det, 0.0));

// //   // Minimum singular value (sqrt of min eigenvalue) - indicates distance to singularity
// //   double min_singular_value = std::sqrt(std::max(lambda_min, 0.0));
// //   double max_singular_value = std::sqrt(std::max(lambda_max, 0.0));

// //   // Publish manipulability metrics
// //   // Format: [condition_number, determinant, yoshikawa, min_sv, max_sv, eigenvalue_0, ..., eigenvalue_5]
// //   std_msgs::msg::Float64MultiArray msg;
// //   msg.data.push_back(condition_number);
// //   msg.data.push_back(det);
// //   msg.data.push_back(yoshikawa_manipulability);
// //   msg.data.push_back(min_singular_value);
// //   msg.data.push_back(max_singular_value);

// //   // Add all eigenvalues (6 values for 6x6 JJ^T)
// //   for (int i = 0; i < eigenvalues.size(); ++i) {
// //     msg.data.push_back(eigenvalues(i));
// //   }

// //   manipulability_pub_->publish(msg);
// // }

// // void OmxVariableStiffnessController::waypoint_callback_(
// //   const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// // {
// //   // Create waypoint command from message
// //   WaypointCommand wp;
// //   wp.position = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
// //   wp.timestamp = get_node()->now().seconds();

// //   // Check frame_id for offset mode
// //   // "offset" or "relative" = offset from current trajectory target
// //   // "absolute", "world", or empty = absolute position
// //   std::string frame = msg->header.frame_id;
// //   wp.is_offset = (frame == "offset" || frame == "relative");

// //   // Add to queue (thread-safe)
// //   {
// //     std::lock_guard<std::mutex> lock(waypoint_mutex_);
// //     waypoint_queue_.push(wp);
// //   }

// //   RCLCPP_INFO(get_node()->get_logger(),
// //     "[WAYPOINT] Received %s waypoint: [%.3f, %.3f, %.3f], queue size: %zu",
// //     wp.is_offset ? "OFFSET" : "ABSOLUTE",
// //     wp.position.x(), wp.position.y(), wp.position.z(),
// //     waypoint_queue_.size());
// // }

// // KDL::Vector OmxVariableStiffnessController::process_waypoints_(
// //   const KDL::Vector & trajectory_target, double current_time)
// // {
// //   // Check for new waypoints in queue
// //   {
// //     std::lock_guard<std::mutex> lock(waypoint_mutex_);

// //     // If current waypoint is done and queue has more, get next
// //     if (!has_active_waypoint_ && !waypoint_queue_.empty()) {
// //       current_waypoint_ = waypoint_queue_.front();
// //       waypoint_queue_.pop();
// //       has_active_waypoint_ = true;
// //       waypoint_mode_active_ = true;
// //       waypoint_start_time_ = current_time;

// //       // Capture current position for smooth blending
// //       // Use trajectory target as starting point for the blend
// //       if (current_waypoint_.is_offset) {
// //         current_waypoint_.position = trajectory_target + current_waypoint_.position;
// //       }
// //       waypoint_blend_start_ = trajectory_target;

// //       RCLCPP_INFO(get_node()->get_logger(),
// //         "[WAYPOINT] Activating waypoint -> [%.3f, %.3f, %.3f]",
// //         current_waypoint_.position.x(),
// //         current_waypoint_.position.y(),
// //         current_waypoint_.position.z());
// //     }
// //   }

// //   // Publish waypoint active status
// //   std_msgs::msg::Bool status_msg;
// //   status_msg.data = has_active_waypoint_;
// //   waypoint_active_pub_->publish(status_msg);

// //   // If no active waypoint, return trajectory target unchanged
// //   if (!has_active_waypoint_) {
// //     waypoint_mode_active_ = false;
// //     return trajectory_target;
// //   }

// //   // Blend from waypoint_blend_start_ to current_waypoint_.position
// //   double elapsed = current_time - waypoint_start_time_;
// //   double blend_progress = std::min(elapsed / waypoint_blend_duration_, 1.0);

// //   // Smooth cosine interpolation for blending
// //   double smooth_progress = 0.5 * (1.0 - std::cos(M_PI * blend_progress));

// //   KDL::Vector blended_target;
// //   blended_target.x(waypoint_blend_start_.x() +
// //     smooth_progress * (current_waypoint_.position.x() - waypoint_blend_start_.x()));
// //   blended_target.y(waypoint_blend_start_.y() +
// //     smooth_progress * (current_waypoint_.position.y() - waypoint_blend_start_.y()));
// //   blended_target.z(waypoint_blend_start_.z() +
// //     smooth_progress * (current_waypoint_.position.z() - waypoint_blend_start_.z()));

// //   // Check if waypoint reached (blend complete)
// //   if (blend_progress >= 1.0) {
// //     // Check if more waypoints in queue
// //     std::lock_guard<std::mutex> lock(waypoint_mutex_);
// //     if (waypoint_queue_.empty()) {
// //       // Waypoint completed, no more in queue
// //       // Stay at waypoint position until new command or clear
// //       RCLCPP_INFO(get_node()->get_logger(),
// //         "[WAYPOINT] Reached waypoint, holding position");
// //     } else {
// //       // Grab next waypoint
// //       current_waypoint_ = waypoint_queue_.front();
// //       waypoint_queue_.pop();
// //       waypoint_start_time_ = current_time;
// //       waypoint_blend_start_ = blended_target;

// //       if (current_waypoint_.is_offset) {
// //         current_waypoint_.position = blended_target + current_waypoint_.position;
// //       }

// //       RCLCPP_INFO(get_node()->get_logger(),
// //         "[WAYPOINT] Next waypoint -> [%.3f, %.3f, %.3f], remaining: %zu",
// //         current_waypoint_.position.x(),
// //         current_waypoint_.position.y(),
// //         current_waypoint_.position.z(),
// //         waypoint_queue_.size());
// //     }
// //   }

// //   return blended_target;
// // }

// // void OmxVariableStiffnessController::clear_waypoints_()
// // {
// //   std::lock_guard<std::mutex> lock(waypoint_mutex_);

// //   // Clear queue
// //   while (!waypoint_queue_.empty()) {
// //     waypoint_queue_.pop();
// //   }

// //   has_active_waypoint_ = false;
// //   waypoint_mode_active_ = false;

// //   RCLCPP_INFO(get_node()->get_logger(), "[WAYPOINT] Cleared all waypoints, returning to trajectory");
// // }

// // bool OmxVariableStiffnessController::load_joint_limits_(const std::string & urdf_xml)
// // {
// //   // Parse URDF to extract joint limits
// //   urdf::Model urdf_model;
// //   if (!urdf_model.initString(urdf_xml)) {
// //     RCLCPP_WARN(get_node()->get_logger(), "Failed to parse URDF for joint limits");
// //     return false;
// //   }

// //   size_t n = joints_.size();
// //   q_min_.resize(n);
// //   q_max_.resize(n);

// //   for (size_t i = 0; i < n; ++i) {
// //     auto joint = urdf_model.getJoint(joints_[i]);
// //     if (!joint) {
// //       RCLCPP_WARN(get_node()->get_logger(),
// //         "Joint '%s' not found in URDF", joints_[i].c_str());
// //       return false;
// //     }

// //     if (joint->type == urdf::Joint::REVOLUTE ||
// //         joint->type == urdf::Joint::PRISMATIC)
// //     {
// //       if (joint->limits) {
// //         q_min_(i) = joint->limits->lower;
// //         q_max_(i) = joint->limits->upper;
// //         RCLCPP_DEBUG(get_node()->get_logger(),
// //           "[LIMITS] Joint '%s': [%.3f, %.3f]",
// //           joints_[i].c_str(), q_min_(i), q_max_(i));
// //       } else {
// //         // No limits specified - use large defaults
// //         q_min_(i) = -M_PI;
// //         q_max_(i) = M_PI;
// //       }
// //     } else if (joint->type == urdf::Joint::CONTINUOUS) {
// //       q_min_(i) = -M_PI;
// //       q_max_(i) = M_PI;
// //     } else {
// //       RCLCPP_WARN(get_node()->get_logger(),
// //         "Joint '%s' has unsupported type", joints_[i].c_str());
// //       return false;
// //     }
// //   }

// //   has_joint_limits_ = true;
// //   return true;
// // }

// // bool OmxVariableStiffnessController::is_within_joint_limits_(const KDL::JntArray & q) const
// // {
// //   if (!has_joint_limits_) {
// //     return true;  // No limits to check
// //   }

// //   for (size_t i = 0; i < q.rows(); ++i) {
// //     if (q(i) < q_min_(i) || q(i) > q_max_(i)) {
// //       return false;
// //     }
// //   }
// //   return true;
// // }

// // double OmxVariableStiffnessController::compute_manipulability_() const
// // {
// //   // Compute manipulability using minimum singular value (robust singularity measure)
// //   // sigma_min → 0 indicates proximity to singularity
// //   Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_.data);
// //   Eigen::VectorXd sv = svd.singularValues();
  
// //   // Return minimum singular value
// //   return sv(sv.size() - 1);
// // }

// // bool OmxVariableStiffnessController::validate_and_compute_trajectory_()
// // {
// //   auto node = get_node();
// //   const size_t n = joints_.size();

// //   trajectory_joint_waypoints_.clear();
// //   trajectory_joint_waypoints_.reserve(num_trajectory_samples_);

// //   // Initial guess for IK - use a non-singular "elbow-bent" configuration
// //   // For OpenManipulator-X: joint2 down, joint3 bent up, joint4 compensating
// //   KDL::JntArray q_init(n);
// //   q_init(0) = 0.0;       // joint1: base rotation
// //   q_init(1) = -0.5;      // joint2: shoulder down
// //   q_init(2) = 1.0;       // joint3: elbow bent
// //   q_init(3) = 0.5;       // joint4: wrist compensating

// //   // Build target orientation from euler angles
// //   KDL::Rotation target_rot = KDL::Rotation::RPY(
// //     target_orientation_euler_.x(),
// //     target_orientation_euler_.y(),
// //     target_orientation_euler_.z());

// //   double min_manip = std::numeric_limits<double>::max();

// //   for (size_t sample = 0; sample <= num_trajectory_samples_; ++sample) {
// //     double s = static_cast<double>(sample) / static_cast<double>(num_trajectory_samples_);

// //     // Interpolate Cartesian position along trajectory
// //     KDL::Vector pos = start_pos_ * (1.0 - s) + end_pos_ * s;
// //     KDL::Frame target_frame(target_rot, pos);

// //     // Solve IK
// //     KDL::JntArray q_result(n);
// //     int ik_ret = ik_solver_->CartToJnt(q_init, target_frame, q_result);

// //     if (ik_ret < 0) {
// //       RCLCPP_ERROR(node->get_logger(),
// //         "[SAFETY ERROR] IK failed at s=%.2f, position=[%.3f, %.3f, %.3f]",
// //         s, pos.x(), pos.y(), pos.z());
// //       return false;
// //     }

// //     // Check joint limits
// //     if (!is_within_joint_limits_(q_result)) {
// //       RCLCPP_ERROR(node->get_logger(),
// //         "[SAFETY ERROR] IK solution violates joint limits at s=%.2f", s);
// //       return false;
// //     }

// //     // Compute manipulability using minimum singular value (robust singularity measure)
// //     // For 4-DOF arm in 6D task space, σ_min indicates proximity to singularity
// //     KDL::Jacobian jac(n);
// //     jnt_to_jac_solver_->JntToJac(q_result, jac);
    
// //     // SVD of J (6xn) - use JacobiSVD for small matrices
// //     Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
// //     Eigen::VectorXd sv = svd.singularValues();
    
// //     // Use minimum singular value as singularity measure (more robust than product)
// //     double sigma_min = sv(sv.size() - 1);  // SVD returns in descending order
// //     double manip = sigma_min;

// //     if (sample == 0) {
// //       RCLCPP_INFO(node->get_logger(),
// //         "[SAFETY] First waypoint IK: q=[%.3f, %.3f, %.3f, %.3f], sigma_min=%.6f, sv=[%.4f, %.4f, %.4f, %.4f]",
// //         q_result(0), q_result(1), q_result(2), q_result(3), sigma_min,
// //         sv(0), sv(1), sv(2), sv.size() > 3 ? sv(3) : 0.0);
// //     }

// //     if (manip < min_manip) {
// //       min_manip = manip;
// //     }

// //     if (manip < min_manipulability_thresh_) {
// //       RCLCPP_ERROR(node->get_logger(),
// //         "[SAFETY ERROR] Near singularity at s=%.2f (sigma_min=%.4f < threshold=%.4f)",
// //         s, manip, min_manipulability_thresh_);
// //       return false;
// //     }

// //     trajectory_joint_waypoints_.push_back(q_result);

// //     // Use this solution as seed for next point
// //     q_init = q_result;
// //   }

// //   RCLCPP_INFO(node->get_logger(),
// //     "[SAFETY] Trajectory validation passed. Min manipulability=%.4f (threshold=%.4f)",
// //     min_manip, min_manipulability_thresh_);

// //   return true;
// // }

// // bool OmxVariableStiffnessController::get_trajectory_joints_(double s, KDL::JntArray & q_out) const
// // {
// //   if (trajectory_joint_waypoints_.empty()) {
// //     return false;
// //   }

// //   // Clamp s to [0, 1]
// //   s = std::clamp(s, 0.0, 1.0);

// //   // Find the two waypoints to interpolate between
// //   double idx_f = s * static_cast<double>(trajectory_joint_waypoints_.size() - 1);
// //   size_t idx_lo = static_cast<size_t>(std::floor(idx_f));
// //   size_t idx_hi = std::min(idx_lo + 1, trajectory_joint_waypoints_.size() - 1);
// //   double t = idx_f - static_cast<double>(idx_lo);

// //   // Linear interpolation in joint space
// //   const KDL::JntArray & q_lo = trajectory_joint_waypoints_[idx_lo];
// //   const KDL::JntArray & q_hi = trajectory_joint_waypoints_[idx_hi];

// //   q_out.resize(q_lo.rows());
// //   for (size_t i = 0; i < q_out.rows(); ++i) {
// //     q_out(i) = q_lo(i) * (1.0 - t) + q_hi(i) * t;
// //   }

// //   return true;
// // }

// // }  // namespace omx_variable_stiffness_controller

// // PLUGINLIB_EXPORT_CLASS(
// //   omx_variable_stiffness_controller::OmxVariableStiffnessController,
// //   controller_interface::ControllerInterface)