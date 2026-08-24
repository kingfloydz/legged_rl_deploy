#include "dex1_device.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace legged_rl_deploy {

namespace {

constexpr std::array<const char*, kDex1Dof> kDefaultCommandTopics{
    "dex1/left/cmd", "dex1/right/cmd"};
constexpr std::array<const char*, kDex1Dof> kDefaultStateTopics{
    "dex1/left/state", "dex1/right/state"};

}  // namespace

Dex1Device::Dex1Device(const YAML::Node& config, rclcpp::Node& node) {
  const std::string action_mode =
      config["action_mode"].as<std::string>("torque");
  if (action_mode == "position") {
    action_mode_ = ActionMode::Position;
  } else if (action_mode != "torque") {
    throw std::runtime_error(
        "dex1.action_mode must be either position or torque");
  }

  if (config["kp"] || config["kd"]) {
    const auto kp = config["kp"].as<std::vector<float>>();
    const auto kd = config["kd"].as<std::vector<float>>();
    if (kp.size() != kDex1Dof || kd.size() != kDex1Dof) {
      throw std::runtime_error(
          "dex1.kp and dex1.kd must contain two values");
    }
    std::copy(kp.begin(), kp.end(), kp_.begin());
    std::copy(kd.begin(), kd.end(), kd_.begin());
  }

  for (std::size_t side = 0; side < kDex1Dof; ++side) {
    std::string command_topic = kDefaultCommandTopics[side];
    std::string state_topic = kDefaultStateTopics[side];
    const char* side_name = side == 0 ? "left" : "right";
    if (config["command_topics"] && config["command_topics"][side_name]) {
      command_topic = config["command_topics"][side_name].as<std::string>();
    }
    if (config["state_topics"] && config["state_topics"][side_name]) {
      state_topic = config["state_topics"][side_name].as<std::string>();
    }

    publishers_[side] = node.create_publisher<Command>(command_topic, 1);
    subscriptions_[side] = node.create_subscription<State>(
        state_topic, 10,
        [this, side](const State::ConstSharedPtr message) {
          if (message->states.empty()) {
            return;
          }
          const auto& motor = message->states[0];
          std::lock_guard<std::mutex> lock(state_mutex_);
          position_[side] = motor.q;
          velocity_[side] = motor.dq;
          torque_[side] = motor.tau_est;
        });
    auto& message = command_messages_[side];
    message.cmds.resize(1);
    auto& command = message.cmds[0];
    command.mode = 1;
    command.q = 0.0f;
    command.dq = 0.0f;
    command.tau = 0.0f;
    command.kp = 0.0f;
    command.kd = 0.0f;
    command.reserve.fill(0);
  }
}

void Dex1Device::read(Dex1StateValues& destination) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  destination[0] = position_[0];
  destination[1] = position_[1];
  destination[2] = velocity_[0];
  destination[3] = velocity_[1];
  destination[4] = torque_[0];
  destination[5] = torque_[1];
}

void Dex1Device::publishPosition(const std::vector<float>& values) {
  for (std::size_t side = 0; side < kDex1Dof; ++side) {
    auto& motor = command_messages_[side].cmds[0];
    motor.q = values[side];
    motor.dq = 0.0f;
    motor.tau = 0.0f;
    motor.kp = kp_[side];
    motor.kd = kd_[side];
    publishers_[side]->publish(command_messages_[side]);
  }
}

void Dex1Device::publishTorque(const std::vector<float>& values) {
  for (std::size_t side = 0; side < kDex1Dof; ++side) {
    auto& motor = command_messages_[side].cmds[0];
    motor.q = 0.0f;
    motor.dq = 0.0f;
    motor.tau = values[side];
    motor.kp = 0.0f;
    motor.kd = 0.0f;
    publishers_[side]->publish(command_messages_[side]);
  }
}

bool Dex1Device::isPositionMode() const noexcept {
  return action_mode_ == ActionMode::Position;
}

}  // namespace legged_rl_deploy
