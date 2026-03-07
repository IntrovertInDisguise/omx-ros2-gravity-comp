#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <queue>
#include <mutex>

namespace omx_variable_stiffness_controller
{

/**
 * @brief State machine states for the controller trajectory
 */
enum class State
{
  INIT,
  HOMING,
  MOVE_FORWARD,
  WAIT_AT_END,
  MOVE_RETURN,
  WAIT_AT_START
};

/**
 * @brief Variable Cartesian Impedance Controller with time-varying stiffness profiles
 *
 * This controller implements a Cartesian impedance control law with the ability to
 * modulate translational stiffness along a trajectory using pre-loaded profiles.
 * It supports cyclic motion between start and end positions with configurable
 * stiffness/damping profiles.
 */
class OmxVariableStiffnessController : public controller_interface::ControllerInterface
{
public:
  OmxVariableStiffnessController() = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::vector<std::string> joints_;
  std::string root_link_;
  std::string tip_link_;
  std::string robot_description_node_{"/controller_manager"};
  double torque_scale_{1.0};
  double max_joint_torque_command_{0.0};  // Per-joint saturation; 0 = disabled (Gazebo)

  // Torque ramp: on hardware, ramp output from 0→1 over torque_ramp_duration_
  // seconds after activation to prevent impulse from gravity comp model mismatch.
  double torque_ramp_duration_{2.0};   // seconds (0 = disabled i.e. Gazebo)
  double activation_time_{0.0};        // wall time of activation

  // Stale-data detector: if all joint positions are identical for N consecutive
  // cycles, the Dynamixel bus has crashed. Zero torques to prevent blind driving.
  static constexpr size_t STALE_THRESHOLD = 50;  // cycles (~100ms at 500Hz)
  size_t stale_position_count_{0};
  KDL::JntArray q_prev_;  // previous cycle's positions

  // Trajectory parameters
  double homing_duration_{5.0};
  double move_duration_{10.0};
  double wait_duration_{2.0};
  double max_rise_rate_{100.0};  // Max stiffness change rate per second

  // Waypoint command mode
  bool waypoint_mode_active_{false};  // True when processing external waypoints
  double waypoint_blend_duration_{2.0};  // Seconds to blend to new waypoint

  // Hardware safety limits for Robotis servos
  static constexpr double MAX_CARTESIAN_STIFFNESS = 65.0;   // N/m max
  static constexpr double MAX_CARTESIAN_DAMPING = 10.0;     // Ns/m max (raised for z-axis)
  static constexpr double MAX_ROTATIONAL_STIFFNESS = 20.0;  // Nm/rad max
  static constexpr double MAX_ROTATIONAL_DAMPING = 1.0;     // Nms/rad max

  // KDL structures
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  KDL::Vector gravity_{0.0, 0.0, -9.81};

  // Joint limits (loaded from URDF)
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  bool has_joint_limits_{false};

  // Pre-computed joint-space trajectory (IK-validated waypoints)
  std::vector<KDL::JntArray> trajectory_joint_waypoints_;
  size_t num_trajectory_samples_{50};  // Number of points to sample along trajectory

  // Safety parameters
  double min_manipulability_thresh_{0.01};  // Pause/slow if below this
  bool use_joint_space_trajectory_{true};   // Use validated joint-space path
  double dls_damping_factor_{0.05};         // λ for Damped Least Squares near singularities

  // Joint state buffers
  KDL::JntArray q_;
  KDL::JntArray q_dot_;
  KDL::JntArray tau_;
  KDL::JntArray G_;
  KDL::Jacobian jacobian_;

  // Cartesian state
  KDL::Frame x_c_start_;       // Starting pose (captured at activation)
  KDL::Frame x_d_frame_;       // Desired pose frame

  // Trajectory waypoints
  KDL::Vector start_pos_;
  KDL::Vector end_pos_;
  KDL::Vector target_orientation_euler_;

  // Stiffness/damping parameters
  KDL::Vector stiffness_homing_;   // Stiffness during homing
  KDL::Vector K_rot_;              // Rotational stiffness
  KDL::Vector Kd_ang_;             // Rotational damping
  KDL::Vector Kd_lin_default_;     // Default linear damping

  // Current stiffness/damping (modulated during motion)
  KDL::Vector K_trans_current_;
  KDL::Vector D_trans_current_;
  KDL::Vector K_rot_current_;    // Per-state rotational stiffness (zeroed during HOMING)
  KDL::Vector Kd_ang_current_;   // Per-state rotational damping

  // Stiffness profiles (loaded from parameter server)
  std::vector<double> stiffness_profile_x_;
  std::vector<double> stiffness_profile_y_;
  std::vector<double> stiffness_profile_z_;
  std::vector<double> damping_profile_x_;
  std::vector<double> damping_profile_y_;
  std::vector<double> damping_profile_z_;

  // State machine
  State current_state_{State::INIT};
  double move_start_time_{0.0};

  // Waypoint queue for runtime target commands
  struct WaypointCommand {
    KDL::Vector position;
    bool is_offset;  // True = offset from trajectory, False = absolute position
    double timestamp;
  };
  std::queue<WaypointCommand> waypoint_queue_;
  std::mutex waypoint_mutex_;
  WaypointCommand current_waypoint_;
  KDL::Vector waypoint_blend_start_;  // Position when waypoint tracking started
  double waypoint_start_time_{0.0};
  bool has_active_waypoint_{false};

  // Error tracking for damping
  KDL::Twist last_x_error_;

  // ── SINGULARITY ESCAPE ───────────────────────────────────────────────────
  //
  // Problem: DLS attenuates Cartesian forces near singularities, but that
  // means it also attenuates the corrective torque needed to *leave* the
  // singular config. The arm locks in place while the trajectory timer keeps
  // advancing, so the desired pose drifts away and the arm can never recover.
  //
  // Solution:
  //  1. Detect entry via sigma_min < threshold (with hysteresis on exit).
  //  2. Freeze trajectory time so s does not advance while stuck.
  //  3. Bypass Cartesian impedance entirely; instead apply a joint-space PD
  //     torque that drives joints toward a known non-singular configuration
  //     (q_preferred_). Gravity comp is still applied for safety.
  //  4. Resume normal impedance control only after sigma_min recovers above
  //     threshold * singularity_exit_hysteresis_.

  bool   in_singularity_{false};         // True while escape mode is active
  double singularity_time_debt_{0.0};    // Accumulated frozen time (added back to move_start_time_ on exit so s resumes from the right point)
  double singularity_entry_time_{0.0};   // wall time when escape mode began (for logging)

  // Escape gains (joint-space PD toward q_preferred_)
  double K_escape_{3.0};    // Nm/rad — proportional gain toward preferred config
  double D_escape_{0.3};    // Nms/rad — derivative (velocity) damping during escape

  // Known non-singular configuration for OMX (elbow-bent, away from full extension)
  // Loaded from YAML so users can tune without recompiling.
  KDL::JntArray q_preferred_;

  // Hysteresis multiplier: exit escape mode only when sigma_min > threshold * this.
  // Prevents flicker at the singularity boundary.
  double singularity_exit_hysteresis_{2.5};

  // Debug throttle counter (incremented every update() call, ~500 Hz)
  size_t debug_counter_{0};
  // Current trajectory progress s ∈ [0,1] — kept as member so state transitions can log it
  double traj_s_{0.0};

  // FIX: Low-pass filtered error derivative to suppress noise at 500 Hz
  KDL::Twist x_error_dot_filtered_;

  // FIX: Velocity filter alpha (set in on_configure)
  double velocity_filter_alpha_{0.1};

  // ── CONTACT FORCE ESTIMATE ───────────────────────────────────────────────
  //
  // Estimator: deflection-based spring+damping wrench.
  //
  //   F_contact_raw = K * x_err + D * xdot_err_filtered   (6D: linear + angular)
  //
  // Uses F_total computed BEFORE DLS projection. DLS attenuates forces in
  // singular Jacobian directions to protect motors — must NOT be applied here,
  // or the contact force estimate is suppressed exactly when pushing hardest.
  //
  // During escape mode the arm moves to q_preferred while x_d is frozen,
  // making x_err meaningless as a contact proxy. The filter state is held
  // (not zeroed — zeroing creates a step on re-entry) and contact_valid_ goes
  // false. Logger consumers should discard rows where contact_valid == 0.
  //
  // Topics:  ~/contact_wrench  (WrenchStamped, in root_link_ frame)
  //          ~/contact_valid   (Bool)
  Eigen::Matrix<double, 6, 1> contact_wrench_filtered_;
  double contact_force_filter_alpha_{0.02};   // ~1.6 Hz at 500 Hz; tune via YAML
  bool   contact_valid_{false};
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr contact_wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_valid_pub_;

  // Publisher for deviated waypoint joint states (when EE deviates from desired)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr deviated_waypoint_pub_;

  // Threshold (meters) above which we publish a deviated waypoint
  double deviation_publish_threshold_{0.01};

  // Joint-space homing and regularization (prevents singularity traversal)
  KDL::JntArray q_at_activation_;       // Joint positions when controller activated
  double homing_joint_K_{15.0};         // Joint-space PD gain during HOMING (Gazebo default)
  double homing_joint_D_{2.0};          // Joint-space damping during HOMING (Gazebo default)
  double trajectory_joint_reg_K_{3.0};  // Joint-space regularization gain during MOVE
  double trajectory_joint_reg_D_{0.5};  // Joint-space regularization damping
  double homing_converge_thresh_{0.05}; // EE position convergence threshold for homing (m)

  // Cached interface handles
  std::vector<hardware_interface::LoanedStateInterface *> pos_if_;
  std::vector<hardware_interface::LoanedStateInterface *> vel_if_;
  std::vector<hardware_interface::LoanedCommandInterface *> eff_cmd_if_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr actual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ee_orient_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr manipulability_pub_;

  // Subscriber for stiffness profile updates
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_profile_sub_;

  // Subscriber for runtime waypoint commands
  // frame_id = "offset" -> offset from current trajectory target
  // frame_id = "absolute" or empty -> absolute position in world frame
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

  // Publisher for waypoint status (queue size, active waypoint, etc.)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_active_pub_;

  // Helpers
  std::string get_robot_description_xml_() const;
  std::string get_robot_description_from_node_(const std::string & node_name) const;
  bool build_kdl_from_robot_description_(const std::string & urdf_xml);
  bool build_chain_(const std::string & root, const std::string & tip);
  bool cache_interfaces_();

  // Profile helpers
  double interpolate_profile_(double s, const std::vector<double> & profile) const;
  double cos_interpolate_(double t, double T) const;

  // Callback for profile updates
  void stiffness_profile_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Helper to load KDL::Vector from 3-element parameter
  bool get_kdl_vector_param_(const std::string & name, KDL::Vector & vec);

  // Apply hardware safety limits to stiffness/damping
  void apply_safety_limits_();

  // Compute and publish manipulability metrics (JJ^T analysis)
  void publish_manipulability_metrics_();

  // Compute manipulability at current configuration
  double compute_manipulability_() const;

  // Validate and pre-compute joint-space trajectory via IK
  bool validate_and_compute_trajectory_();

  // Get interpolated joint position from pre-computed trajectory
  bool get_trajectory_joints_(double s, KDL::JntArray & q_out) const;

  // Load joint limits from URDF
  bool load_joint_limits_(const std::string & urdf_xml);

  // Check if joint array is within limits
  bool is_within_joint_limits_(const KDL::JntArray & q) const;

  // Waypoint command callback
  void waypoint_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Process waypoint queue - returns modified target position if waypoint active
  KDL::Vector process_waypoints_(const KDL::Vector & trajectory_target, double current_time);

  // Clear all waypoints and return to trajectory following
  void clear_waypoints_();

  // Compute joint-space escape torque toward q_preferred_ (used during singularity escape)
  // Returns tau_escape as a KDL::JntArray. Caller adds gravity comp on top.
  KDL::JntArray compute_escape_torque_() const;
};

}  // namespace omx_variable_stiffness_controller

// #pragma once

// #include <string>
// #include <vector>
// #include <memory>
// #include <chrono>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include <kdl/tree.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/jacobian.hpp>
// #include <kdl/frames.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chaindynparam.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolverpos_lma.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>

// #include <Eigen/Dense>

// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/vector3.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <std_msgs/msg/bool.hpp>

// #include <queue>
// #include <mutex>

// namespace omx_variable_stiffness_controller
// {

// /**
//  * @brief State machine states for the controller trajectory
//  */
// enum class State
// {
//   INIT,
//   HOMING,
//   MOVE_FORWARD,
//   WAIT_AT_END,
//   MOVE_RETURN,
//   WAIT_AT_START
// };

// /**
//  * @brief Variable Cartesian Impedance Controller with time-varying stiffness profiles
//  *
//  * This controller implements a Cartesian impedance control law with the ability to
//  * modulate translational stiffness along a trajectory using pre-loaded profiles.
//  * It supports cyclic motion between start and end positions with configurable
//  * stiffness/damping profiles.
//  */
// class OmxVariableStiffnessController : public controller_interface::ControllerInterface
// {
// public:
//   OmxVariableStiffnessController() = default;

//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;

//   controller_interface::CallbackReturn on_init() override;
//   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
//   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
//   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//   controller_interface::return_type update(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   // Parameters
//   std::vector<std::string> joints_;
//   std::string root_link_;
//   std::string tip_link_;
//   std::string robot_description_node_{"/controller_manager"};
//   double torque_scale_{1.0};

//   // Trajectory parameters
//   double homing_duration_{5.0};
//   double move_duration_{10.0};
//   double wait_duration_{2.0};
//   double max_rise_rate_{100.0};  // Max stiffness change rate per second

//   // Waypoint command mode
//   bool waypoint_mode_active_{false};  // True when processing external waypoints
//   double waypoint_blend_duration_{2.0};  // Seconds to blend to new waypoint

//   // Hardware safety limits for Robotis servos
//   static constexpr double MAX_CARTESIAN_STIFFNESS = 65.0;   // N/m max
//   static constexpr double MAX_CARTESIAN_DAMPING = 3.0;      // Ns/m max
//   static constexpr double MAX_ROTATIONAL_STIFFNESS = 20.0;  // Nm/rad max
//   static constexpr double MAX_ROTATIONAL_DAMPING = 1.0;     // Nms/rad max

//   // KDL structures
//   KDL::Tree kdl_tree_;
//   KDL::Chain kdl_chain_;
//   std::unique_ptr<KDL::ChainDynParam> dyn_param_;
//   std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
//   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
//   std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
//   KDL::Vector gravity_{0.0, 0.0, -9.81};

//   // Joint limits (loaded from URDF)
//   KDL::JntArray q_min_;
//   KDL::JntArray q_max_;
//   bool has_joint_limits_{false};

//   // Pre-computed joint-space trajectory (IK-validated waypoints)
//   std::vector<KDL::JntArray> trajectory_joint_waypoints_;
//   size_t num_trajectory_samples_{50};  // Number of points to sample along trajectory

//   // Safety parameters
//   double min_manipulability_thresh_{0.01};  // Pause/slow if below this
//   bool use_joint_space_trajectory_{true};   // Use validated joint-space path
//   double dls_damping_factor_{0.05};         // λ for Damped Least Squares near singularities

//   // Joint state buffers
//   KDL::JntArray q_;
//   KDL::JntArray q_dot_;
//   KDL::JntArray tau_;
//   KDL::JntArray G_;
//   KDL::Jacobian jacobian_;

//   // Cartesian state
//   KDL::Frame x_c_start_;       // Starting pose (captured at activation)
//   KDL::Frame x_d_frame_;       // Desired pose frame

//   // Trajectory waypoints
//   KDL::Vector start_pos_;
//   KDL::Vector end_pos_;
//   KDL::Vector target_orientation_euler_;

//   // Stiffness/damping parameters
//   KDL::Vector stiffness_homing_;   // Stiffness during homing
//   KDL::Vector K_rot_;              // Rotational stiffness
//   KDL::Vector Kd_ang_;             // Rotational damping
//   KDL::Vector Kd_lin_default_;     // Default linear damping

//   // Current stiffness/damping (modulated during motion)
//   KDL::Vector K_trans_current_;
//   KDL::Vector D_trans_current_;

//   // Stiffness profiles (loaded from parameter server)
//   std::vector<double> stiffness_profile_x_;
//   std::vector<double> stiffness_profile_y_;
//   std::vector<double> stiffness_profile_z_;
//   std::vector<double> damping_profile_x_;
//   std::vector<double> damping_profile_y_;
//   std::vector<double> damping_profile_z_;

//   // State machine
//   State current_state_{State::INIT};
//   double move_start_time_{0.0};

//   // Waypoint queue for runtime target commands
//   struct WaypointCommand {
//     KDL::Vector position;
//     bool is_offset;  // True = offset from trajectory, False = absolute position
//     double timestamp;
//   };
//   std::queue<WaypointCommand> waypoint_queue_;
//   std::mutex waypoint_mutex_;
//   WaypointCommand current_waypoint_;
//   KDL::Vector waypoint_blend_start_;  // Position when waypoint tracking started
//   double waypoint_start_time_{0.0};
//   bool has_active_waypoint_{false};

//   // Error tracking for damping
//   KDL::Twist last_x_error_;

//   // FIX: Low-pass filtered error derivative to suppress noise at 500 Hz
//   KDL::Twist x_error_dot_filtered_;

//   // FIX: Velocity filter alpha (set in on_configure)
//   double velocity_filter_alpha_{0.1};

//   // Cached interface handles
//   std::vector<hardware_interface::LoanedStateInterface *> pos_if_;
//   std::vector<hardware_interface::LoanedStateInterface *> vel_if_;
//   std::vector<hardware_interface::LoanedCommandInterface *> eff_cmd_if_;

//   // Publishers
//   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr actual_pose_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_pos_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_pub_;
//   rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ee_orient_pub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr manipulability_pub_;

//   // Subscriber for stiffness profile updates
//   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_profile_sub_;

//   // Subscriber for runtime waypoint commands
//   // frame_id = "offset" -> offset from current trajectory target
//   // frame_id = "absolute" or empty -> absolute position in world frame
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

//   // Publisher for waypoint status (queue size, active waypoint, etc.)
//   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_active_pub_;

//   // Helpers
//   std::string get_robot_description_xml_() const;
//   std::string get_robot_description_from_node_(const std::string & node_name) const;
//   bool build_kdl_from_robot_description_(const std::string & urdf_xml);
//   bool build_chain_(const std::string & root, const std::string & tip);
//   bool cache_interfaces_();

//   // Profile helpers
//   double interpolate_profile_(double s, const std::vector<double> & profile) const;
//   double cos_interpolate_(double t, double T) const;

//   // Callback for profile updates
//   void stiffness_profile_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

//   // Helper to load KDL::Vector from 3-element parameter
//   bool get_kdl_vector_param_(const std::string & name, KDL::Vector & vec);

//   // Apply hardware safety limits to stiffness/damping
//   void apply_safety_limits_();

//   // Compute and publish manipulability metrics (JJ^T analysis)
//   void publish_manipulability_metrics_();

//   // Compute manipulability at current configuration
//   double compute_manipulability_() const;

//   // Validate and pre-compute joint-space trajectory via IK
//   bool validate_and_compute_trajectory_();

//   // Get interpolated joint position from pre-computed trajectory
//   bool get_trajectory_joints_(double s, KDL::JntArray & q_out) const;

//   // Load joint limits from URDF
//   bool load_joint_limits_(const std::string & urdf_xml);

//   // Check if joint array is within limits
//   bool is_within_joint_limits_(const KDL::JntArray & q) const;

//   // Waypoint command callback
//   void waypoint_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

//   // Process waypoint queue - returns modified target position if waypoint active
//   KDL::Vector process_waypoints_(const KDL::Vector & trajectory_target, double current_time);

//   // Clear all waypoints and return to trajectory following
//   void clear_waypoints_();
// };

// }  // namespace omx_variable_stiffness_controller
// // #pragma once

// // #include <string>
// // #include <vector>
// // #include <memory>
// // #include <chrono>

// // #include "rclcpp/rclcpp.hpp"
// // #include "rclcpp_lifecycle/lifecycle_node.hpp"
// // #include "controller_interface/controller_interface.hpp"
// // #include "hardware_interface/types/hardware_interface_type_values.hpp"

// // #include <kdl/tree.hpp>
// // #include <kdl/chain.hpp>
// // #include <kdl/jntarray.hpp>
// // #include <kdl/jacobian.hpp>
// // #include <kdl/frames.hpp>
// // #include <kdl_parser/kdl_parser.hpp>
// // #include <kdl/chaindynparam.hpp>
// // #include <kdl/chainjnttojacsolver.hpp>
// // #include <kdl/chainfksolverpos_recursive.hpp>
// // #include <kdl/chainiksolverpos_lma.hpp>
// // #include <kdl/chainiksolvervel_pinv.hpp>

// // #include <Eigen/Dense>

// // #include <geometry_msgs/msg/pose.hpp>
// // #include <geometry_msgs/msg/point.hpp>
// // #include <geometry_msgs/msg/vector3.hpp>
// // #include <geometry_msgs/msg/pose_stamped.hpp>
// // #include <std_msgs/msg/float64_multi_array.hpp>
// // #include <std_msgs/msg/string.hpp>
// // #include <std_msgs/msg/bool.hpp>

// // #include <queue>
// // #include <mutex>

// // namespace omx_variable_stiffness_controller
// // {

// // /**
// //  * @brief State machine states for the controller trajectory
// //  */
// // enum class State
// // {
// //   INIT,
// //   HOMING,
// //   MOVE_FORWARD,
// //   WAIT_AT_END,
// //   MOVE_RETURN,
// //   WAIT_AT_START
// // };

// // /**
// //  * @brief Variable Cartesian Impedance Controller with time-varying stiffness profiles
// //  *
// //  * This controller implements a Cartesian impedance control law with the ability to
// //  * modulate translational stiffness along a trajectory using pre-loaded profiles.
// //  * It supports cyclic motion between start and end positions with configurable
// //  * stiffness/damping profiles.
// //  */
// // class OmxVariableStiffnessController : public controller_interface::ControllerInterface
// // {
// // public:
// //   OmxVariableStiffnessController() = default;

// //   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
// //   controller_interface::InterfaceConfiguration command_interface_configuration() const override;

// //   controller_interface::CallbackReturn on_init() override;
// //   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
// //   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
// //   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

// //   controller_interface::return_type update(
// //     const rclcpp::Time & time, const rclcpp::Duration & period) override;

// // private:
// //   // Parameters
// //   std::vector<std::string> joints_;
// //   std::string root_link_;
// //   std::string tip_link_;
// //   std::string robot_description_node_{"/controller_manager"};
// //   double torque_scale_{1.0};

// //   // Trajectory parameters
// //   double homing_duration_{5.0};
// //   double move_duration_{10.0};
// //   double wait_duration_{2.0};
// //   double max_rise_rate_{100.0};  // Max stiffness change rate per second

// //   // Waypoint command mode
// //   bool waypoint_mode_active_{false};  // True when processing external waypoints
// //   double waypoint_blend_duration_{2.0};  // Seconds to blend to new waypoint

// //   // Hardware safety limits for Robotis servos
// //   static constexpr double MAX_CARTESIAN_STIFFNESS = 65.0;   // N/m max
// //   static constexpr double MAX_CARTESIAN_DAMPING = 3.0;      // Ns/m max
// //   static constexpr double MAX_ROTATIONAL_STIFFNESS = 20.0;  // Nm/rad max
// //   static constexpr double MAX_ROTATIONAL_DAMPING = 1.0;     // Nms/rad max

// //   // KDL structures
// //   KDL::Tree kdl_tree_;
// //   KDL::Chain kdl_chain_;
// //   std::unique_ptr<KDL::ChainDynParam> dyn_param_;
// //   std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
// //   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
// //   std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
// //   KDL::Vector gravity_{0.0, 0.0, -9.81};

// //   // Joint limits (loaded from URDF)
// //   KDL::JntArray q_min_;
// //   KDL::JntArray q_max_;
// //   bool has_joint_limits_{false};

// //   // Pre-computed joint-space trajectory (IK-validated waypoints)
// //   std::vector<KDL::JntArray> trajectory_joint_waypoints_;
// //   size_t num_trajectory_samples_{50};  // Number of points to sample along trajectory

// //   // Safety parameters
// //   double min_manipulability_thresh_{0.01};  // Pause/slow if below this
// //   bool use_joint_space_trajectory_{true};   // Use validated joint-space path
// //   double dls_damping_factor_{0.05};         // λ for Damped Least Squares near singularities

// //   // Joint state buffers
// //   KDL::JntArray q_;
// //   KDL::JntArray q_dot_;
// //   KDL::JntArray tau_;
// //   KDL::JntArray G_;
// //   KDL::Jacobian jacobian_;

// //   // Cartesian state
// //   KDL::Frame x_c_start_;       // Starting pose (captured at activation)
// //   KDL::Frame x_d_frame_;       // Desired pose frame

// //   // Trajectory waypoints
// //   KDL::Vector start_pos_;
// //   KDL::Vector end_pos_;
// //   KDL::Vector target_orientation_euler_;

// //   // Stiffness/damping parameters
// //   KDL::Vector stiffness_homing_;   // Stiffness during homing
// //   KDL::Vector K_rot_;              // Rotational stiffness
// //   KDL::Vector Kd_ang_;             // Rotational damping
// //   KDL::Vector Kd_lin_default_;     // Default linear damping

// //   // Current stiffness/damping (modulated during motion)
// //   KDL::Vector K_trans_current_;
// //   KDL::Vector D_trans_current_;

// //   // Stiffness profiles (loaded from parameter server)
// //   std::vector<double> stiffness_profile_x_;
// //   std::vector<double> stiffness_profile_y_;
// //   std::vector<double> stiffness_profile_z_;
// //   std::vector<double> damping_profile_x_;
// //   std::vector<double> damping_profile_y_;
// //   std::vector<double> damping_profile_z_;

// //   // State machine
// //   State current_state_{State::INIT};
// //   double move_start_time_{0.0};

// //   // Waypoint queue for runtime target commands
// //   struct WaypointCommand {
// //     KDL::Vector position;
// //     bool is_offset;  // True = offset from trajectory, False = absolute position
// //     double timestamp;
// //   };
// //   std::queue<WaypointCommand> waypoint_queue_;
// //   std::mutex waypoint_mutex_;
// //   WaypointCommand current_waypoint_;
// //   KDL::Vector waypoint_blend_start_;  // Position when waypoint tracking started
// //   double waypoint_start_time_{0.0};
// //   bool has_active_waypoint_{false};

// //   // Error tracking for damping
// //   KDL::Twist last_x_error_;

// //   // Cached interface handles
// //   std::vector<hardware_interface::LoanedStateInterface *> pos_if_;
// //   std::vector<hardware_interface::LoanedStateInterface *> vel_if_;
// //   std::vector<hardware_interface::LoanedCommandInterface *> eff_cmd_if_;

// //   // Publishers
// //   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr actual_pose_pub_;
// //   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;
// //   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_pos_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_pub_;
// //   rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ee_orient_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr manipulability_pub_;

// //   // Subscriber for stiffness profile updates
// //   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_profile_sub_;

// //   // Subscriber for runtime waypoint commands
// //   // frame_id = "offset" -> offset from current trajectory target
// //   // frame_id = "absolute" or empty -> absolute position in world frame
// //   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

// //   // Publisher for waypoint status (queue size, active waypoint, etc.)
// //   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_active_pub_;

// //   // Helpers
// //   std::string get_robot_description_xml_() const;
// //   std::string get_robot_description_from_node_(const std::string & node_name) const;
// //   bool build_kdl_from_robot_description_(const std::string & urdf_xml);
// //   bool build_chain_(const std::string & root, const std::string & tip);
// //   bool cache_interfaces_();

// //   // Profile helpers
// //   double interpolate_profile_(double s, const std::vector<double> & profile) const;
// //   double cos_interpolate_(double t, double T) const;

// //   // Callback for profile updates
// //   void stiffness_profile_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

// //   // Helper to load KDL::Vector from 3-element parameter
// //   bool get_kdl_vector_param_(const std::string & name, KDL::Vector & vec);

// //   // Apply hardware safety limits to stiffness/damping
// //   void apply_safety_limits_();

// //   // Compute and publish manipulability metrics (JJ^T analysis)
// //   void publish_manipulability_metrics_();

// //   // Compute manipulability at current configuration
// //   double compute_manipulability_() const;

// //   // Validate and pre-compute joint-space trajectory via IK
// //   bool validate_and_compute_trajectory_();

// //   // Get interpolated joint position from pre-computed trajectory
// //   bool get_trajectory_joints_(double s, KDL::JntArray & q_out) const;

// //   // Load joint limits from URDF
// //   bool load_joint_limits_(const std::string & urdf_xml);

// //   // Check if joint array is within limits
// //   bool is_within_joint_limits_(const KDL::JntArray & q) const;

// //   // Waypoint command callback
// //   void waypoint_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

// //   // Process waypoint queue - returns modified target position if waypoint active
// //   KDL::Vector process_waypoints_(const KDL::Vector & trajectory_target, double current_time);

// //   // Clear all waypoints and return to trajectory following
// //   void clear_waypoints_();
// // };

// // }  // namespace omx_variable_stiffness_controller
