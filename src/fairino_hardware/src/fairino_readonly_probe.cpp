#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libfairino/include/robot.h"
#include "pluginlib/class_list_macros.hpp"

namespace fairino_hardware {
class FairinoReadOnlyProbe : public hardware_interface::SystemInterface {
  std::unique_ptr<FRRobot> robot_;
  std::array<double, 6> position_;
  std::string ip_;
  static constexpr double radians_per_degree = 0.017453292519943295;
  void close() {
    if (robot_) {
      const auto rc = robot_->CloseRPC();
      if (rc != 0) RCLCPP_WARN(rclcpp::get_logger("fr5_readonly_probe"),
                              "CloseRPC failed: %d", static_cast<int>(rc));
      robot_.reset();
    }
  }
  bool sample(uint8_t flag) {
    JointPos joint{};
    const auto rc = robot_->GetActualJointPosDegree(flag, &joint);
    if (rc != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("fr5_readonly_probe"),
                   "GetActualJointPosDegree failed: %d", static_cast<int>(rc));
      return false;
    }
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(joint.jPos[i])) {
        RCLCPP_ERROR(rclcpp::get_logger("fr5_readonly_probe"),
                     "Non-finite feedback for joint %zu", i + 1);
        return false;
      }
    }
    for (size_t i = 0; i < 6; ++i) position_[i] = joint.jPos[i] * radians_per_degree;
    return true;
  }
public:
  ~FairinoReadOnlyProbe() override { close(); }
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    const auto ip = info_.hardware_parameters.find("robot_ip");
    if (ip == info_.hardware_parameters.end() || ip->second.empty() || info_.joints.size() != 6) {
      RCLCPP_ERROR(rclcpp::get_logger("fr5_readonly_probe"),
                   "Explicit robot_ip and exactly six joints required");
      return hardware_interface::CallbackReturn::ERROR;
    }
    ip_ = ip->second;
    for (size_t i = 0; i < 6; ++i) {
      const auto &joint = info_.joints[i];
      if (joint.name != "j" + std::to_string(i + 1) ||
          !joint.command_interfaces.empty() || joint.state_interfaces.size() != 1 ||
          joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        return hardware_interface::CallbackReturn::ERROR;
    }
    position_.fill(std::numeric_limits<double>::quiet_NaN());
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i = 0; i < 6; ++i)
      result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    return {};
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    close();
    robot_ = std::make_unique<FRRobot>();
    const auto rc = robot_->RPC(ip_.c_str());
    if (rc != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("fr5_readonly_probe"), "RPC failed: %d", static_cast<int>(rc));
      close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    if (!sample(0)) {
      close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("fr5_readonly_probe"),
                "READ ONLY connected to %s; feedback rad: %.6f %.6f %.6f %.6f %.6f %.6f",
                ip_.c_str(), position_[0], position_[1], position_[2],
                position_[3], position_[4], position_[5]);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    close();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (!robot_ || !sample(1)) return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    return hardware_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoReadOnlyProbe, hardware_interface::SystemInterface)
