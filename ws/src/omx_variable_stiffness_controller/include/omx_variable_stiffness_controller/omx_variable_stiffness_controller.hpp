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

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

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
  static constexpr double MAX_CARTESIAN_DAMPING = 3.0;      // Ns/m max
  static constexpr double MAX_ROTATIONAL_STIFFNESS = 20.0;  // Nm/rad max
  static constexpr double MAX_ROTATIONAL_DAMPING = 1.0;     // Nms/rad max

  // KDL structures
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::Vector gravity_{0.0, 0.0, -9.81};

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

  // Waypoint command callback
  void waypoint_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Process waypoint queue - returns modified target position if waypoint active
  KDL::Vector process_waypoints_(const KDL::Vector & trajectory_target, double current_time);

  // Clear all waypoints and return to trajectory following
  void clear_waypoints_();
};

}  // namespace omx_variable_stiffness_controller
