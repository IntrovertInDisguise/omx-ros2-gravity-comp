// Move all includes outside the namespace to avoid incorrect scoping
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <functional>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

namespace omx_variable_stiffness_controller
{

class OmxVariableStiffnessController : public controller_interface::ControllerInterface
{
public:
  OmxVariableStiffnessController() = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

  // Joint state containers
  KDL::JntArray q_;
  KDL::JntArray q_dot_;
  KDL::JntArray tau_;
  KDL::Jacobian jacobian_;
  KDL::JntArray G_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;
  bool has_joint_limits_ = false;

  // Parameters
  std::vector<std::string> joints_;
  std::string root_link_;
  std::string tip_link_;
  std::string robot_description_node_;
  double torque_scale_ = 1.0;

  double homing_duration_ = 5.0;
  double move_duration_ = 10.0;
  double wait_duration_ = 3.0;
  double max_rise_rate_ = 100.0;

  // Trajectory & stiffness/damping
  KDL::Vector start_pos_;
  KDL::Vector end_pos_;
  KDL::Vector target_orientation_euler_;
  KDL::Vector stiffness_homing_;
  KDL::Vector K_rot_;
  KDL::Vector Kd_ang_;
  KDL::Vector Kd_lin_default_;

  std::vector<double> stiffness_profile_x_;
  std::vector<double> stiffness_profile_y_;
  std::vector<double> stiffness_profile_z_;
  std::vector<double> damping_profile_x_;
  std::vector<double> damping_profile_y_;
  std::vector<double> damping_profile_z_;

  // Current stiffness/damping
  KDL::Vector K_trans_current_;
  KDL::Vector D_trans_current_;

  // Trajectory / safety
  bool use_joint_space_trajectory_ = true;
  double min_manipulability_thresh_ = 0.01;
  size_t num_trajectory_samples_ = 50;
  double dls_damping_factor_ = 0.05;
  std::vector<KDL::JntArray> trajectory_joint_waypoints_;

  // Hardware interface caches
  std::vector<hardware_interface::LoanedStateInterface *> pos_if_;
  std::vector<hardware_interface::LoanedStateInterface *> vel_if_;
  std::vector<hardware_interface::LoanedCommandInterface *> eff_cmd_if_;

  // Cartesian frames and history
  KDL::Frame x_d_frame_;
  KDL::Frame x_c_start_;          // EE pose at start of homing
  KDL::Frame x_c_frame_;          // current EE pose
  KDL::Twist last_x_error_;
  double move_start_time_ = 0.0;

  // ---- New orientation variables for smooth homing ----
  KDL::Rotation start_orientation_;   // orientation at activation
  KDL::Rotation target_orientation_;  // target orientation from parameters
  // ------------------------------------------------------

  // Waypoint system
  struct WaypointCommand {
    KDL::Vector position;
    double timestamp = 0.0;
    bool is_offset = false;
  };

  std::queue<WaypointCommand> waypoint_queue_;
  std::mutex waypoint_mutex_;
  bool has_active_waypoint_ = false;
  WaypointCommand current_waypoint_;
  double waypoint_start_time_ = 0.0;
  double waypoint_blend_duration_ = 2.0;
  KDL::Vector waypoint_blend_start_;

  // Publishers/subscribers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ee_orient_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr manipulability_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr actual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr desired_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr jacobian_pub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_profile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;

  // State machine
  enum class State { INIT, HOMING, WAIT_AT_START, MOVE_FORWARD, WAIT_AT_END, MOVE_RETURN };
  State current_state_ = State::INIT;

  // Gravity vector for dynamics
  KDL::Vector gravity_{0.0, 0.0, -9.81};

  // Safety limits (adjusted for hardware, can be overridden via parameters if needed)
  static constexpr double MAX_CARTESIAN_STIFFNESS = 65.0;   // N/m
  static constexpr double MAX_CARTESIAN_DAMPING = 3.0;      // Ns/m
  static constexpr double MAX_ROTATIONAL_STIFFNESS = 20.0;  // Nm/rad
  static constexpr double MAX_ROTATIONAL_DAMPING = 1.0;     // Nms/rad

  // Helpers / callbacks
  bool get_kdl_vector_param_(const std::string & name, KDL::Vector & vec);
  std::string get_robot_description_from_node_(const std::string & node_name) const;
  std::string get_robot_description_xml_() const;
  bool build_kdl_from_robot_description_(const std::string & urdf_xml);
  bool build_chain_(const std::string & root, const std::string & tip);
  bool load_stiffness_damping_profiles_from_csv_(const std::string & csv_filename);
  void stiffness_profile_callback_(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool cache_interfaces_();
  double interpolate_profile_(double s, const std::vector<double> & profile) const;
  double cos_interpolate_(double t, double T) const;
  void apply_safety_limits_();
  void publish_manipulability_metrics_();
  void waypoint_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  KDL::Vector process_waypoints_(const KDL::Vector & trajectory_target, double current_time);
  void clear_waypoints_();
  bool load_joint_limits_(const std::string & urdf_xml);
  bool is_within_joint_limits_(const KDL::JntArray & q) const;
  double compute_manipulability_() const;
  bool validate_and_compute_trajectory_();
  bool get_trajectory_joints_(double s, KDL::JntArray & q_out) const;

}; // class OmxVariableStiffnessController

} // namespace omx_variable_stiffness_controller