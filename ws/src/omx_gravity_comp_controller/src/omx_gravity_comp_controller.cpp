#include "omx_gravity_comp_controller/omx_gravity_comp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/wait_for_message.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace omx_gravity_comp_controller
{

controller_interface::InterfaceConfiguration
OmxGravityCompController::state_interface_configuration() const
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
OmxGravityCompController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
  }
  return cfg;
}

controller_interface::CallbackReturn
OmxGravityCompController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("root_link", "world");
  auto_declare<std::string>("tip_link", "");
  auto_declare<double>("torque_scale", 1.0);
  auto_declare<int>("debug_print_ms", 500);

  // NEW: where robot_description is expected to live
  auto_declare<std::string>("robot_description_node", "/controller_manager");

  return controller_interface::CallbackReturn::SUCCESS;
}

std::string OmxGravityCompController::get_robot_description_from_node_(const std::string & node_name) const
{
  auto node = get_node();
  if (!node) {
    return {};
  }

  // Derive topic name from node name (e.g., /robot1/robot_state_publisher -> /robot1/robot_description)
  std::string topic_name;
  size_t last_slash = node_name.rfind('/');
  if (last_slash != std::string::npos && last_slash > 0) {
    topic_name = node_name.substr(0, last_slash) + "/robot_description";
  } else {
    topic_name = "/robot_description";
  }

  // Create subscription with TRANSIENT_LOCAL durability to get the latched message
  auto sub = node->create_subscription<std_msgs::msg::String>(
    topic_name,
    rclcpp::QoS(1).transient_local(),
    [](const std_msgs::msg::String::SharedPtr) {});

  // Use wait_for_message with WaitSet - doesn't require executor spinning
  std_msgs::msg::String msg;
  if (rclcpp::wait_for_message(msg, sub, node->get_node_options().context(), std::chrono::seconds(3))) {
    RCLCPP_INFO(node->get_logger(), "Got robot_description from topic '%s'", topic_name.c_str());
    return msg.data;
  }

  RCLCPP_WARN(node->get_logger(),
    "Timeout waiting for robot_description on topic '%s'",
    topic_name.c_str());
  return {};
}

std::string OmxGravityCompController::get_robot_description_xml_() const
{
  auto node = get_node();
  if (!node) {
    return {};
  }

  std::string urdf_xml;

  // Priority 1: If robot_description_node is explicitly set to a non-default value,
  // use that first. This is critical for multi-robot setups with Gazebo Classic
  // where the controller node may inherit the wrong robot_description parameter.
  if (!robot_description_node_.empty() && robot_description_node_ != "/controller_manager") {
    urdf_xml = get_robot_description_from_node_(robot_description_node_);
    if (!urdf_xml.empty()) {
      return urdf_xml;
    }
  }

  // Priority 2: Try local parameter (only if robot_description_node wasn't explicitly set)
  if (node->get_parameter("robot_description", urdf_xml) && !urdf_xml.empty()) {
    return urdf_xml;
  }

  // Priority 3: Try default configured node
  if (robot_description_node_.empty() || robot_description_node_ == "/controller_manager") {
    urdf_xml = get_robot_description_from_node_(robot_description_node_);
    if (!urdf_xml.empty()) {
      return urdf_xml;
    }
  }

  // Priority 4: Common fallback
  urdf_xml = get_robot_description_from_node_("/robot_state_publisher");
  if (!urdf_xml.empty()) {
    return urdf_xml;
  }

  return {};
}

bool OmxGravityCompController::build_kdl_from_robot_description_(const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    return false;
  }
  return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
}

bool OmxGravityCompController::build_chain_(const std::string & root, const std::string & tip)
{
  if (root.empty() || tip.empty()) {
    return false;
  }
  return kdl_tree_.getChain(root, tip, kdl_chain_);
}

controller_interface::CallbackReturn
OmxGravityCompController::on_configure(const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  root_link_ = get_node()->get_parameter("root_link").as_string();
  tip_link_ = get_node()->get_parameter("tip_link").as_string();
  torque_scale_ = get_node()->get_parameter("torque_scale").as_double();
  debug_print_ms_ = get_node()->get_parameter("debug_print_ms").as_int();
  robot_description_node_ = get_node()->get_parameter("robot_description_node").as_string();

  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (tip_link_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'tip_link' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto urdf_xml = get_robot_description_xml_();
  if (urdf_xml.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "robot_description not found. Tried local param, '%s', and '/robot_state_publisher'.",
      robot_description_node_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!build_kdl_from_robot_description_(urdf_xml)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF into KDL tree.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!build_chain_(root_link_, tip_link_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to extract KDL chain root='%s' tip='%s'.",
      root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kdl_chain_.getNrOfJoints() != joints_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "KDL chain DOFs (%u) != joints size (%zu). root='%s' tip='%s'.",
      kdl_chain_.getNrOfJoints(), joints_.size(),
      root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  gravity_ = KDL::Vector(0.0, 0.0, -9.81);
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);

  const size_t n = joints_.size();
  q_ = KDL::JntArray(n);
  qd_ = KDL::JntArray(n);
  tau_g_ = KDL::JntArray(n);

  // Clear cached pointers; they will be rebuilt in on_activate
  pos_if_.assign(n, nullptr);
  vel_if_.assign(n, nullptr);
  eff_cmd_if_.assign(n, nullptr);

  RCLCPP_INFO(get_node()->get_logger(),
    "Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f, robot_description_node='%s'",
    n, root_link_.c_str(), tip_link_.c_str(), torque_scale_, robot_description_node_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

bool OmxGravityCompController::cache_interfaces_()
{
  const size_t n = joints_.size();
  if (n == 0) {
    return false;
  }

  // Map by name/interface
  for (size_t i = 0; i < n; ++i) {
    const auto & j = joints_[i];

    // position - use get_prefix_name() for joint name, get_interface_name() for type
    auto it_pos = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return (si.get_prefix_name() == j) &&
               (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
      });
    if (it_pos != state_interfaces_.end()) {
      pos_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_pos));
    }

    // velocity (optional)
    auto it_vel = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si) {
        return (si.get_prefix_name() == j) &&
               (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
      });
    if (it_vel != state_interfaces_.end()) {
      vel_if_[i] = const_cast<hardware_interface::LoanedStateInterface *>(&(*it_vel));
    }

    // effort command
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

  // Hard-fail if any position or effort command interface is missing
  bool ok = true;
  for (size_t i = 0; i < n; ++i) {
    if (!pos_if_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing POSITION state interface for joint '%s'.", joints_[i].c_str());
      ok = false;
    }
    if (!eff_cmd_if_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Missing EFFORT command interface for joint '%s'. "
        "If Gazebo exports only position commands, effort writes will be ignored.",
        joints_[i].c_str());
      ok = false;
    }
  }
  return ok;
}

controller_interface::CallbackReturn
OmxGravityCompController::on_activate(const rclcpp_lifecycle::State &)
{
  // Debug: Log what interfaces we have
  RCLCPP_INFO(get_node()->get_logger(), "on_activate: %zu state interfaces, %zu command interfaces",
    state_interfaces_.size(), command_interfaces_.size());
  for (const auto & si : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  State: %s/%s", si.get_name().c_str(), si.get_interface_name().c_str());
  }
  for (const auto & ci : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  Cmd:   %s/%s", ci.get_name().c_str(), ci.get_interface_name().c_str());
  }

  // Cache interfaces now that we have them loaned
  if (!cache_interfaces_()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Failed to cache required interfaces. Controller cannot run.");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmxGravityCompController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & ci : command_interfaces_) {
    (void)ci.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
OmxGravityCompController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t n = joints_.size();
  if (n == 0 || !dyn_param_) {
    return controller_interface::return_type::ERROR;
  }

  // Read q, qd (qd optional)
  for (size_t i = 0; i < n; ++i) {
    q_(i) = pos_if_[i]->get_value();
    qd_(i) = vel_if_[i] ? vel_if_[i]->get_value() : 0.0;
  }

  // Gravity torques
  KDL::JntArray g(n);
  g.data.setZero();

  const int ret = dyn_param_->JntToGravity(q_, g);
  if (ret != 0) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "KDL JntToGravity failed (ret=%d)", ret);
    return controller_interface::return_type::ERROR;
  }

  // Heuristic warning: if g is ~zero, likely missing inertials in URDF
  double g_norm = 0.0;
  for (size_t i = 0; i < n; ++i) {
    g_norm += g(i) * g(i);
  }
  g_norm = std::sqrt(g_norm);

  if (g_norm < 1e-6) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Computed gravity torques are ~0 (||g||=%.3e). Common causes: URDF inertials missing/zero, "
      "wrong root/tip chain, or robot_description not matching the simulated model.",
      g_norm);
  }

  // Command effort = scale * gravity
  for (size_t i = 0; i < n; ++i) {
    const double tau = torque_scale_ * g(i);
    (void)eff_cmd_if_[i]->set_value(tau);
  }

  if (n >= 4) {
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), debug_print_ms_,
      "q=[%.3f %.3f %.3f %.3f] g=[%.3f %.3f %.3f %.3f] ||g||=%.3e scale=%.2f",
      q_(0), q_(1), q_(2), q_(3),
      g(0), g(1), g(2), g(3),
      g_norm, torque_scale_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace omx_gravity_comp_controller

PLUGINLIB_EXPORT_CLASS(
  omx_gravity_comp_controller::OmxGravityCompController,
  controller_interface::ControllerInterface)

// #include "omx_gravity_comp_controller/omx_gravity_comp_controller.hpp"

// #include <algorithm>
// #include <string>
// #include <vector>

// #include <kdl/chaindynparam.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl_parser/kdl_parser.hpp>

// #include "pluginlib/class_list_macros.hpp"

// namespace omx_gravity_comp_controller
// {

// controller_interface::InterfaceConfiguration
// OmxGravityCompController::state_interface_configuration() const
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
// OmxGravityCompController::command_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration cfg;
//   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

//   for (const auto & j : joints_) {
//     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
//   }
//   return cfg;
// }

// controller_interface::CallbackReturn
// OmxGravityCompController::on_init()
// {
//   auto_declare<std::vector<std::string>>("joints", {});
//   auto_declare<std::string>("root_link", "world");
//   auto_declare<std::string>("tip_link", "");
//   auto_declare<double>("torque_scale", 1.0);

//   // Debug printing throttle (ms)
//   auto_declare<int>("debug_print_ms", 500);

//   return controller_interface::CallbackReturn::SUCCESS;
// }

// std::string OmxGravityCompController::get_robot_description_xml_() const
// {
//   std::string urdf_xml;
//   if (!get_node()->get_parameter("robot_description", urdf_xml)) {
//     return std::string{};
//   }
//   return urdf_xml;
// }

// bool OmxGravityCompController::build_kdl_from_robot_description_(const std::string & urdf_xml)
// {
//   if (urdf_xml.empty()) {
//     return false;
//   }
//   return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
// }

// bool OmxGravityCompController::build_chain_(const std::string & root, const std::string & tip)
// {
//   if (root.empty() || tip.empty()) {
//     return false;
//   }
//   return kdl_tree_.getChain(root, tip, kdl_chain_);
// }

// controller_interface::CallbackReturn
// OmxGravityCompController::on_configure(const rclcpp_lifecycle::State &)
// {
//   joints_ = get_node()->get_parameter("joints").as_string_array();
//   root_link_ = get_node()->get_parameter("root_link").as_string();
//   tip_link_ = get_node()->get_parameter("tip_link").as_string();
//   torque_scale_ = get_node()->get_parameter("torque_scale").as_double();
//   debug_print_ms_ = get_node()->get_parameter("debug_print_ms").as_int();

//   if (joints_.empty()) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
//     return controller_interface::CallbackReturn::ERROR;
//   }
//   if (tip_link_.empty()) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'tip_link' is empty.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   const auto urdf_xml = get_robot_description_xml_();
//   if (urdf_xml.empty()) {
//     RCLCPP_ERROR(get_node()->get_logger(),
//       "robot_description parameter not found on this node.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   if (!build_kdl_from_robot_description_(urdf_xml)) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF into KDL tree.");
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   if (!build_chain_(root_link_, tip_link_)) {
//     RCLCPP_ERROR(get_node()->get_logger(),
//       "Failed to extract KDL chain root='%s' tip='%s'.",
//       root_link_.c_str(), tip_link_.c_str());
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // If this mismatches, you will compute gravity for the wrong chain.
//   if (kdl_chain_.getNrOfJoints() != joints_.size()) {
//     RCLCPP_ERROR(get_node()->get_logger(),
//       "KDL chain DOFs (%u) != joints size (%zu). root='%s' tip='%s'.",
//       kdl_chain_.getNrOfJoints(), joints_.size(),
//       root_link_.c_str(), tip_link_.c_str());
//     return controller_interface::CallbackReturn::ERROR;
//   }

//   // IMPORTANT: set gravity before creating ChainDynParam
//   gravity_ = KDL::Vector(0.0, 0.0, -9.81);
//   dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);

//   const size_t n = joints_.size();
//   q_ = KDL::JntArray(n);
//   qd_ = KDL::JntArray(n);
//   tau_g_ = KDL::JntArray(n);

//   RCLCPP_INFO(get_node()->get_logger(),
//     "Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f, debug_print_ms=%d",
//     n, root_link_.c_str(), tip_link_.c_str(), torque_scale_, debug_print_ms_);

//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn
// OmxGravityCompController::on_activate(const rclcpp_lifecycle::State &)
// {
//   for (auto & ci : command_interfaces_) {
//     (void)ci.set_value(0.0);
//   }
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn
// OmxGravityCompController::on_deactivate(const rclcpp_lifecycle::State &)
// {
//   for (auto & ci : command_interfaces_) {
//     (void)ci.set_value(0.0);
//   }
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::return_type
// OmxGravityCompController::update(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   const size_t n = joints_.size();
//   if (n == 0 || !dyn_param_) {
//     return controller_interface::return_type::ERROR;
//   }
//   if (state_interfaces_.empty() || command_interfaces_.empty()) {
//     return controller_interface::return_type::ERROR;
//   }

//   // Read q, qd
//   for (size_t i = 0; i < n; ++i) {
//     const auto & j = joints_[i];

//     auto it_pos = std::find_if(
//       state_interfaces_.begin(), state_interfaces_.end(),
//       [&](const auto & si) {
//         return (si.get_name() == j) &&
//                (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
//       });
//     if (it_pos == state_interfaces_.end()) {
//       RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
//         "Missing position state interface for joint '%s'", j.c_str());
//       return controller_interface::return_type::ERROR;
//     }
//     q_(i) = it_pos->get_value();

//     auto it_vel = std::find_if(
//       state_interfaces_.begin(), state_interfaces_.end(),
//       [&](const auto & si) {
//         return (si.get_name() == j) &&
//                (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
//       });
//     qd_(i) = (it_vel == state_interfaces_.end()) ? 0.0 : it_vel->get_value();
//   }

//   // Gravity torques
//   KDL::JntArray g(n);
//   g.data.setZero();

//   const int ret = dyn_param_->JntToGravity(q_, g);
//   if (ret != 0) {
//     RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
//       "KDL JntToGravity failed (ret=%d)", ret);
//     return controller_interface::return_type::ERROR;
//   }

//   // Command effort = scale * gravity
//   for (size_t i = 0; i < n; ++i) {
//     const double tau = torque_scale_ * g(i);

//     auto it_cmd = std::find_if(
//       command_interfaces_.begin(), command_interfaces_.end(),
//       [&](const auto & ci) {
//         return (ci.get_name() == joints_[i]) &&
//                (ci.get_interface_name() == std::string(hardware_interface::HW_IF_EFFORT));
//       });
//     if (it_cmd == command_interfaces_.end()) {
//       RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
//         "Missing effort command interface for joint '%s'", joints_[i].c_str());
//       return controller_interface::return_type::ERROR;
//     }
//     (void)it_cmd->set_value(tau);
//   }

//   if (n >= 4) {
//     RCLCPP_INFO_THROTTLE(
//       get_node()->get_logger(), *get_node()->get_clock(), debug_print_ms_,
//       "q=[%.3f %.3f %.3f %.3f] g=[%.3f %.3f %.3f %.3f] scale=%.2f",
//       q_(0), q_(1), q_(2), q_(3),
//       g(0), g(1), g(2), g(3),
//       torque_scale_);
//   }

//   return controller_interface::return_type::OK;
// }

// }  // namespace omx_gravity_comp_controller

// PLUGINLIB_EXPORT_CLASS(
//   omx_gravity_comp_controller::OmxGravityCompController,
//   controller_interface::ControllerInterface)


// // #include "omx_gravity_comp_controller/omx_gravity_comp_controller.hpp"

// // #include <algorithm>
// // #include <stdexcept>

// // #include <kdl/chaindynparam.hpp>
// // #include <kdl/jntarray.hpp>

// // #include "pluginlib/class_list_macros.hpp"

// // namespace omx_gravity_comp_controller
// // {

// // controller_interface::InterfaceConfiguration
// // OmxGravityCompController::state_interface_configuration() const
// // {
// //   controller_interface::InterfaceConfiguration cfg;
// //   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

// //   // Need joint position for gravity torque computation.
// //   for (const auto & j : joints_) {
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_POSITION));
// //     // Optional; not strictly needed for gravity, but keeps future extension easy.
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_VELOCITY));
// //   }
// //   return cfg;
// // }

// // controller_interface::InterfaceConfiguration
// // OmxGravityCompController::command_interface_configuration() const
// // {
// //   controller_interface::InterfaceConfiguration cfg;
// //   cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

// //   for (const auto & j : joints_) {
// //     cfg.names.push_back(j + "/" + std::string(hardware_interface::HW_IF_EFFORT));
// //   }
// //   return cfg;
// // }

// // controller_interface::CallbackReturn
// // OmxGravityCompController::on_init()
// // {
// //   // Declare parameters (no assumptions about launch files)
// //   auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
// //   auto_declare<std::string>("root_link", "world");
// //   auto_declare<std::string>("tip_link", "");
// //   auto_declare<double>("torque_scale", 1.0);

// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // std::string OmxGravityCompController::get_robot_description_xml_() const
// // {
// //   // Standard: robot_description is usually on the same node.
// //   // If user remaps or pushes via param file, this still works.
// //   std::string urdf_xml;
// //   if (!get_node()->get_parameter("robot_description", urdf_xml)) {
// //     // Some setups put it under a namespace; user must provide it in that case.
// //     return std::string{};
// //   }
// //   return urdf_xml;
// // }

// // bool OmxGravityCompController::build_kdl_from_robot_description_(const std::string & urdf_xml)
// // {
// //   if (urdf_xml.empty()) {
// //     return false;
// //   }
// //   return kdl_parser::treeFromString(urdf_xml, kdl_tree_);
// // }

// // bool OmxGravityCompController::build_chain_(const std::string & root, const std::string & tip)
// // {
// //   if (root.empty() || tip.empty()) {
// //     return false;
// //   }
// //   return kdl_tree_.getChain(root, tip, kdl_chain_);
// // }

// // controller_interface::CallbackReturn
// // OmxGravityCompController::on_configure(const rclcpp_lifecycle::State &)
// // {
// //   joints_ = get_node()->get_parameter("joints").as_string_array();
// //   root_link_ = get_node()->get_parameter("root_link").as_string();
// //   tip_link_ = get_node()->get_parameter("tip_link").as_string();
// //   torque_scale_ = get_node()->get_parameter("torque_scale").as_double();

// //   if (joints_.empty()) {
// //     RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }
// //   if (tip_link_.empty()) {
// //     RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'tip_link' is empty.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   const auto urdf_xml = get_robot_description_xml_();
// //   if (urdf_xml.empty()) {
// //     RCLCPP_ERROR(
// //       get_node()->get_logger(),
// //       "robot_description parameter not found on this node. Provide it via param file.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   if (!build_kdl_from_robot_description_(urdf_xml)) {
// //     RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF into KDL tree.");
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   if (!build_chain_(root_link_, tip_link_)) {
// //     RCLCPP_ERROR(
// //       get_node()->get_logger(),
// //       "Failed to extract KDL chain root='%s' tip='%s'.",
// //       root_link_.c_str(), tip_link_.c_str());
// //     return controller_interface::CallbackReturn::ERROR;
// //   }

// //   // ChainDynParam needs inertia; URDF->KDL includes inertias if present in URDF.
// //   dyn_param_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, gravity_);

// //   const size_t n = joints_.size();
// //   q_ = KDL::JntArray(n);
// //   qd_ = KDL::JntArray(n);
// //   tau_g_ = KDL::JntArray(n);

// //   RCLCPP_INFO(
// //     get_node()->get_logger(),
// //     "Configured: %zu joints, root='%s', tip='%s', torque_scale=%.3f",
// //     n, root_link_.c_str(), tip_link_.c_str(), torque_scale_);

// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // controller_interface::CallbackReturn
// // OmxGravityCompController::on_activate(const rclcpp_lifecycle::State &)
// // {
// //   // Zero commands on activate for safety
// //   for (auto & ci : command_interfaces_) {
// //     (void)ci.set_value(0.0);
// //   }
// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // controller_interface::CallbackReturn
// // OmxGravityCompController::on_deactivate(const rclcpp_lifecycle::State &)
// // {
// //   for (auto & ci : command_interfaces_) {
// //     (void)ci.set_value(0.0);
// //   }
// //   return controller_interface::CallbackReturn::SUCCESS;
// // }

// // controller_interface::return_type
// // OmxGravityCompController::update(const rclcpp::Time &, const rclcpp::Duration &)
// // {
// //   const size_t n = joints_.size();
// //   if (state_interfaces_.empty() || command_interfaces_.empty()) {
// //     return controller_interface::return_type::ERROR;
// //   }

// //   // Extract joint position/velocity from state interfaces (order: joints_, pos+vel per joint)
// //   // We requested both position and velocity; if velocity isn't provided by hardware, it may be 0.
// //   for (size_t i = 0; i < n; ++i) {
// //     const auto & j = joints_[i];

// //     auto it_pos = std::find_if(
// //       state_interfaces_.begin(), state_interfaces_.end(),
// //       [&](const auto & si) {
// //         return (si.get_name() == j) &&
// //                (si.get_interface_name() == std::string(hardware_interface::HW_IF_POSITION));
// //       });
// //     if (it_pos == state_interfaces_.end()) {
// //       RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
// //                            "Missing position state interface for joint '%s'", j.c_str());
// //       return controller_interface::return_type::ERROR;
// //     }
// //     q_(i) = it_pos->get_value();

// //     auto it_vel = std::find_if(
// //       state_interfaces_.begin(), state_interfaces_.end(),
// //       [&](const auto & si) {
// //         return (si.get_name() == j) &&
// //                (si.get_interface_name() == std::string(hardware_interface::HW_IF_VELOCITY));
// //       });
// //     qd_(i) = (it_vel == state_interfaces_.end()) ? 0.0 : it_vel->get_value();
// //   }

// //   // Compute gravity torques
// //   KDL::JntArray coriolis(n);
// //   KDL::JntArray gravity(n);
// //   coriolis.data.setZero();
// //   gravity.data.setZero();

// //   const int ret = dyn_param_->JntToGravity(q_, gravity);
// //   if (ret != 0) {
// //     RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
// //                          "KDL JntToGravity failed (ret=%d)", ret);
// //     return controller_interface::return_type::ERROR;
// //   }

// //   // Write effort commands in the same joint order
// //   for (size_t i = 0; i < n; ++i) {
// //     const double tau = torque_scale_ * gravity(i);

// //     auto it_cmd = std::find_if(
// //       command_interfaces_.begin(), command_interfaces_.end(),
// //       [&](const auto & ci) {
// //         return (ci.get_name() == joints_[i]) &&
// //                (ci.get_interface_name() == std::string(hardware_interface::HW_IF_EFFORT));
// //       });

// //     if (it_cmd == command_interfaces_.end()) {
// //       RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
// //                            "Missing effort command interface for joint '%s'", joints_[i].c_str());
// //       return controller_interface::return_type::ERROR;
// //     }

// //     (void)it_cmd->set_value(tau);
// //   }

// //   RCLCPP_INFO_THROTTLE(
// //     get_node()->get_logger(), *get_node()->get_clock(), debug_print_ms_,
// //     "q=[%.3f %.3f %.3f %.3f] g=[%.3f %.3f %.3f %.3f] scale=%.2f",
// //     q_(0), q_(1), q_(2), q_(3),
// //     gravity(0), gravity(1), gravity(2), gravity(3),
// //     torque_scale_);

// //   return controller_interface::return_type::OK;
// // }

// // }  // namespace omx_gravity_comp_controller

// // PLUGINLIB_EXPORT_CLASS(
// //   omx_gravity_comp_controller::OmxGravityCompController,
// //   controller_interface::ControllerInterface)
