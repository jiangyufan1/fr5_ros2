#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libfairino/include/robot.h"
#include "fairino_hardware/fr5_servo_session.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fairino_hardware {
// Separate from the vendor plugin. All ROS angles are radians; SDK angles degrees.
class FairinoMoveItHardware : public hardware_interface::SystemInterface {
  using Clock = std::chrono::steady_clock;
  static constexpr double RAD = 0.017453292519943295;
  static constexpr double PERIOD = 0.008;
  std::unique_ptr<FRRobot> robot_;
  std::array<double, 6> position_{}, command_{}, previous_{}, lower_{}, upper_{}, velocity_{};
  std::string ip_;
  bool servo_ = false;
  bool fault_ = false;
  Clock::time_point last_write_;
  ServoSettlingWindow settling_;

  rclcpp::Logger logger() const { return rclcpp::get_logger("fr5_moveit_hardware"); }
  void stop_servo() {
    if (robot_ && servo_) {
      // Best-effort stopping; a lost network connection can also prevent these calls.
      const auto stop_rc = robot_->StopMotion();
      const auto end_rc = robot_->ServoMoveEnd();
      if (stop_rc != 0 || end_rc != 0)
        RCLCPP_ERROR(logger(), "StopMotion=%d ServoMoveEnd=%d",
                     static_cast<int>(stop_rc), static_cast<int>(end_rc));
      servo_ = false;
    }
  }
  void close() {
    stop_servo();
    if (robot_) {
      const auto rc = robot_->CloseRPC();
      if (rc != 0) RCLCPP_WARN(logger(), "CloseRPC=%d", static_cast<int>(rc));
      robot_.reset();
    }
  }
  hardware_interface::return_type fail(const char *reason, int code = 0) {
    if (!fault_) RCLCPP_ERROR(logger(), "%s (code=%d). Restart required.", reason, code);
    fault_ = true;
    stop_servo();
    return hardware_interface::return_type::ERROR;
  }
  bool sample(uint8_t flag) {
    JointPos joint{};
    const auto rc = robot_->GetActualJointPosDegree(flag, &joint);
    if (rc != 0) { fail("Joint feedback failed", static_cast<int>(rc)); return false; }
    std::array<double, 6> next{};
    for (size_t i = 0; i < 6; ++i) {
      next[i] = joint.jPos[i] * RAD;
      if (!std::isfinite(next[i]) || next[i] < lower_[i] - 1e-6 || next[i] > upper_[i] + 1e-6) {
        RCLCPP_ERROR(logger(), "j%zu feedback=%.9f; limits=[%.9f, %.9f]",
                     i + 1, next[i], lower_[i], upper_[i]);
        fail("Invalid/out-of-bounds feedback"); return false;
      }
    }
    position_ = next;
    return true;
  }
public:
  ~FairinoMoveItHardware() override { close(); }
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      if (info_.hardware_parameters.at("enable_motion") != "true")
        throw std::runtime_error("Explicit enable_motion:=true is required");
      ip_ = info_.hardware_parameters.at("robot_ip");
      if (ip_.empty() || info_.joints.size() != 6)
        throw std::runtime_error("Explicit IP and six joints required");
      for (size_t i = 0; i < 6; ++i) {
        const auto &j = info_.joints[i];
        if (j.name != "j" + std::to_string(i + 1) || j.command_interfaces.size() != 1 ||
            j.state_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
            j.state_interfaces[0].name != "position")
          throw std::runtime_error("Expected ordered j1..j6, with position command/state only");
        lower_[i] = std::stod(j.parameters.at("min_position"));
        upper_[i] = std::stod(j.parameters.at("max_position"));
        velocity_[i] = std::stod(j.parameters.at("max_velocity"));
        if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i] ||
            !std::isfinite(velocity_[i]) || velocity_[i] <= 0)
          throw std::runtime_error("Invalid joint limits");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger(), "Configuration: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    position_.fill(std::numeric_limits<double>::quiet_NaN());
    command_ = position_;
    previous_ = position_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (size_t i = 0; i < 6; ++i) result.emplace_back(info_.joints[i].name, "position", &position_[i]);
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (size_t i = 0; i < 6; ++i) result.emplace_back(info_.joints[i].name, "position", &command_[i]);
    return result;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    close();
    fault_ = false;
    robot_ = std::make_unique<FRRobot>();
    const auto rc = robot_->RPC(ip_.c_str());
    if (rc != 0) {
      fail("RPC failed", static_cast<int>(rc)); close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    if (!sample(0)) { close(); return hardware_interface::CallbackReturn::ERROR; }
    command_ = position_;
    previous_ = position_;
    last_write_ = Clock::now();
    RCLCPP_INFO(logger(), "Connected %s; commands initialized from actual positions. Waiting for trajectory.", ip_.c_str());
    RCLCPP_INFO(logger(), "Feedback rad: %.6f %.6f %.6f %.6f %.6f %.6f",
                position_[0], position_[1], position_[2], position_[3], position_[4], position_[5]);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    close();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
    fault_ = true; close();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (fault_ || !robot_) return hardware_interface::return_type::ERROR;
    return sample(1) ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
  }
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override {
    if (fault_ || !robot_) return hardware_interface::return_type::ERROR;
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_write_).count();
    if (servo_cycle_invalid(servo_, elapsed, period.seconds())) {
      RCLCPP_ERROR(logger(), "Control timing: elapsed=%.6f s, period=%.6f s, streaming=%d",
                   elapsed, period.seconds(), static_cast<int>(servo_));
      return fail("Invalid control cycle or stall (over 50 ms while streaming)");
    }
    last_write_ = now;
    bool changed = false;
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(command_[i]) || command_[i] < lower_[i] || command_[i] > upper_[i])
        return fail("Non-finite/out-of-bounds position command");
      const double step = std::abs(command_[i] - previous_[i]);
      if (step > velocity_[i] * PERIOD * 1.25 + 1e-6)
        return fail("Command step exceeds model velocity bound");
      if (std::abs(command_[i] - position_[i]) > 0.35)
        return fail("Command/feedback deviation exceeds 0.35 rad");
      changed = changed || step > 1e-7;
    }
    // No ServoJ on launch: start only when a controller requests a changed target.
    if (!servo_ && !changed) return hardware_interface::return_type::OK;
    if (!servo_) {
      servo_ = true;  // Also clean up if the SDK start call partially succeeds.
      const auto rc = robot_->ServoMoveStart();
      if (rc != 0) return fail("ServoMoveStart failed; check auto mode/enable/alarm", static_cast<int>(rc));
      settling_.reset(command_, position_, std::chrono::duration<double>(now.time_since_epoch()).count());
      RCLCPP_INFO(logger(), "Servo stream started, nominal period 8 ms");
    }
    // End normal holding only after a full second of stable, in-tolerance feedback.
    // The streaming watchdog above still runs first: a stall is never treated as idle.
    if (settling_.ready(command_, position_, std::chrono::duration<double>(now.time_since_epoch()).count())) {
      const auto rc = robot_->ServoMoveEnd();
      if (rc != 0) return fail("ServoMoveEnd failed after settling", static_cast<int>(rc));
      servo_ = false;
      previous_ = command_;
      RCLCPP_INFO(logger(), "Servo stream ended: target settled. Ready for next trajectory.");
      return hardware_interface::return_type::OK;
    }
    JointPos target{};
    ExaxisPos external{0, 0, 0, 0};
    for (size_t i = 0; i < 6; ++i) target.jPos[i] = command_[i] / RAD;
    const auto rc = robot_->ServoJ(&target, &external, 0, 0, static_cast<float>(PERIOD), 0, 0);
    if (rc != 0) return fail("ServoJ failed", static_cast<int>(rc));
    previous_ = command_;
    return hardware_interface::return_type::OK;
  }
};
}
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoMoveItHardware, hardware_interface::SystemInterface)
