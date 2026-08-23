#pragma once

#include <array>
#include <cstddef>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/motor_cmds.hpp>
#include <unitree_go/msg/motor_states.hpp>
#include <yaml-cpp/yaml.h>

namespace legged_rl_deploy {

inline constexpr std::size_t kDex1Dof = 2;
inline constexpr std::size_t kDex1InputDim = 6;
using Dex1StateValues = std::array<float, kDex1InputDim>;

class Dex1Device final {
 public:
  Dex1Device(const YAML::Node& config, rclcpp::Node& node);

  void read(Dex1StateValues& destination) const;
  void publishPosition(const std::vector<float>& values);
  void publishTorque(const std::vector<float>& values);
  bool isPositionMode() const noexcept;

 private:
  enum class ActionMode { Position, Torque };

  using Command = unitree_go::msg::MotorCmds;
  using State = unitree_go::msg::MotorStates;

  mutable std::mutex state_mutex_;
  std::array<float, kDex1Dof> position_{};
  std::array<float, kDex1Dof> velocity_{};
  std::array<float, kDex1Dof> torque_{};
  ActionMode action_mode_ = ActionMode::Torque;
  float torque_limit_nm_ = 125.0f;
  std::array<float, kDex1Dof> kp_{5.0f, 5.0f};
  std::array<float, kDex1Dof> kd_{0.1f, 0.1f};
  std::array<rclcpp::Publisher<Command>::SharedPtr, kDex1Dof> publishers_{};
  std::array<Command, kDex1Dof> command_messages_;
  std::array<rclcpp::Subscription<State>::SharedPtr, kDex1Dof> subscriptions_{};
};

}  // namespace legged_rl_deploy
