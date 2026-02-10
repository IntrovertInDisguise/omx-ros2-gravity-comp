#pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

namespace omx_gravity_comp_controller
{

class OmxGravityCompController : public controller_interface::ControllerInterface
{
public:
  OmxGravityCompController() = default;

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
  double torque_scale_{1.0};
  int debug_print_ms_{500};

  // Where to fetch robot_description from (default: /controller_manager)
  std::string robot_description_node_{"/controller_manager"};

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  KDL::Vector gravity_{0.0, 0.0, -9.81};

  // Buffers
  KDL::JntArray q_;
  KDL::JntArray qd_;
  KDL::JntArray tau_g_;

  // Cached interface handles (set in on_activate)
  std::vector<hardware_interface::LoanedStateInterface *> pos_if_;
  std::vector<hardware_interface::LoanedStateInterface *> vel_if_;
  std::vector<hardware_interface::LoanedCommandInterface *> eff_cmd_if_;

  // Helpers
  std::string get_robot_description_xml_() const;
  std::string get_robot_description_from_node_(const std::string & node_name) const;

  bool build_kdl_from_robot_description_(const std::string & urdf_xml);
  bool build_chain_(const std::string & root, const std::string & tip);

  bool cache_interfaces_();
};

}  // namespace omx_gravity_comp_controller


// #pragma once

// #include <string>
// #include <vector>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include <kdl/tree.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chaindynparam.hpp>

// #include <urdf/model.h>

// #include <Eigen/Core>

// namespace omx_gravity_comp_controller
// {

// class OmxGravityCompController : public controller_interface::ControllerInterface
// {
// public:
//   OmxGravityCompController() = default;

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
//   double torque_scale_{1.0};
//   int debug_print_ms_{500};

//   // KDL
//   KDL::Tree kdl_tree_;
//   KDL::Chain kdl_chain_;
//   std::unique_ptr<KDL::ChainDynParam> dyn_param_;
//   KDL::Vector gravity_{0.0, 0.0, -9.81};

//   // Buffers
//   KDL::JntArray q_;
//   KDL::JntArray qd_;
//   KDL::JntArray tau_g_;

//   // Helpers
//   bool build_kdl_from_robot_description_(const std::string & urdf_xml);
//   bool build_chain_(const std::string & root, const std::string & tip);
//   std::string get_robot_description_xml_() const;
// };

// }  // namespace omx_gravity_comp_controller
