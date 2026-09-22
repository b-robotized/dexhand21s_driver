// Copyright (c) 2025, b-robotized Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEXHAND21S_HARDWARE_INTERFACE__DEXHAND21S_HARDWARE_INTERFACE_HPP_
#define DEXHAND21S_HARDWARE_INTERFACE__DEXHAND21S_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "DexHand.h"

namespace dexhand21s_hardware_interface
{
class DexHand21sHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DexHand21sHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  static constexpr size_t DEXHAND21S_JOINT_COUNT = 3;
  static constexpr int16_t MIN_HALL_POSITION = 0;
  static constexpr int16_t MAX_HALL_POSITION = 1000;
  static constexpr double MIN_RAD_POSITION = 0.0;
  static constexpr double MAX_RAD_POSITION = 1.3;

private:
  // Converts between hall sensor value and radians for a given finger
  double hallToRad(int finger_id, int16_t hall_value);
  int16_t radToHall(int finger_id, double rad_value);

  void stateCallbackFunc(const DexRobot::Dex021::DX21StatusRxData * status);

  // Hardware handle
  std::shared_ptr<DexRobot::Dex021::DexHand_021S> hand_;
  uint8_t device_id_ = 0x01;
  int16_t finger_speed_deg_s_ = 10;  // SDK takes degrees per second (times 100)
  uint16_t sampling_rate_ = 50;
  double status_timeout_s_ = 0.5;

  // Steady-clock time of the last status frame from the hand, written by the SDK thread.
  std::atomic<int64_t> last_status_ns_{0};

  // Consecutive write() cycles in which the SDK refused to send a command.
  unsigned int failed_writes_ = 0;
  static constexpr unsigned int MAX_FAILED_WRITES = 5;
  static int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  std::array<std::string, DEXHAND21S_JOINT_COUNT> joint_position_itfs_;
  std::array<std::string, DEXHAND21S_JOINT_COUNT> joint_velocity_itfs_;
  std::array<std::string, DEXHAND21S_JOINT_COUNT> joint_temperature_itfs_;
  std::array<std::string, DEXHAND21S_JOINT_COUNT> joint_current_itfs_;

  std::array<double, DEXHAND21S_JOINT_COUNT> joint_position_states_{{0.0, 0.0, 0.0}};
  std::array<double, DEXHAND21S_JOINT_COUNT> joint_velocity_states_{{0.0, 0.0, 0.0}};
  std::array<double, DEXHAND21S_JOINT_COUNT> joint_temperature_states_{{0.0, 0.0, 0.0}};
  std::array<double, DEXHAND21S_JOINT_COUNT> joint_current_states_{{0.0, 0.0, 0.0}};
};

}  // namespace dexhand21s_hardware_interface

#endif  // DEXHAND21S_HARDWARE_INTERFACE__DEXHAND21S_HARDWARE_INTERFACE_HPP_
