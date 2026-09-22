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
//
// Author: Yara Shahin
//

#include "dexhand21s_hardware_interface/dexhand21s_hardware_interface.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dexhand21s_hardware_interface
{

// Converts hall sensor value to radians for a given finger
double DexHand21sHardwareInterface::hallToRad(int finger_id, int16_t hall_value)
{
  // if finger 1, then 0 - 1000 is 0.0 to -1.3 (clipped)
  // else, then 0 - 1000 is 0.0 to 1.3 (clipped)

  if (hall_value < MIN_HALL_POSITION)
  {
    hall_value = MIN_HALL_POSITION;
  }
  else if (hall_value > MAX_HALL_POSITION)
  {
    hall_value = MAX_HALL_POSITION;
  }

  if (finger_id == 1)
  {
    // scale (0-1000) to (0.0 to -1.3)
    double rad = (static_cast<double>(hall_value) / 1000.0) * (-1 * MAX_RAD_POSITION);
    return rad;
  }
  else
  {
    // scale (0-1000) to (0.0 to 1.3)
    double rad = (static_cast<double>(hall_value) / 1000.0) * MAX_RAD_POSITION;
    return rad;
  }

  return 0.0;
}

int16_t DexHand21sHardwareInterface::radToHall(int finger_id, double rad_value)
{
  // Finger 1 closes towards negative angles, fingers 2 and 3 towards positive ones.
  const double lo = finger_id == 1 ? -MAX_RAD_POSITION : MIN_RAD_POSITION;
  const double hi = finger_id == 1 ? MIN_RAD_POSITION : MAX_RAD_POSITION;
  const double clipped = std::clamp(rad_value, lo, hi);
  if (clipped != rad_value)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Finger %d command %.3f rad is outside [%.2f, %.2f] rad, clipping.", finger_id, rad_value, lo,
      hi);
  }
  return static_cast<int16_t>(
    std::lround(std::abs(clipped) / MAX_RAD_POSITION * MAX_HALL_POSITION));
}

void DexHand21sHardwareInterface::stateCallbackFunc(
  const DexRobot::Dex021::DX21StatusRxData * status)
{
  // Velocity, temperature and current are raw SDK values; units are undocumented (see README).
  last_status_ns_ = now_ns();
  joint_position_states_ = {
    {hallToRad(1, status->MotorHallValue(1)), hallToRad(2, status->MotorHallValue(2)),
     hallToRad(3, status->MotorHallValue(3))}};
  joint_velocity_states_ = {
    {static_cast<double>(status->MotorVelocity(1)), static_cast<double>(status->MotorVelocity(2)),
     static_cast<double>(status->MotorVelocity(3))}};
  joint_temperature_states_ = {
    {static_cast<double>(status->MotorTemperature(1)),
     static_cast<double>(status->MotorTemperature(2)),
     static_cast<double>(status->MotorTemperature(3))}};
  joint_current_states_ = {
    {static_cast<double>(status->MotorCurrent(1)), static_cast<double>(status->MotorCurrent(2)),
     static_cast<double>(status->MotorCurrent(3))}};
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto & hw_params = info_.hardware_parameters;

  // Get optional parameters with defaults
  // default device_id_ = 0x01
  if (hw_params.find("device_id") != hw_params.end())
  {
    device_id_ = static_cast<uint8_t>(std::stoi(hw_params.at("device_id")));
  }

  // default finger_speed_deg_s_ = 10 deg/s
  if (hw_params.find("finger_speed_deg_s") != hw_params.end())
  {
    finger_speed_deg_s_ = static_cast<int16_t>(std::stoi(hw_params.at("finger_speed_deg_s")));
  }

  // default sampling_rate_ = 50 Hz
  if (hw_params.find("sampling_rate") != hw_params.end())
  {
    sampling_rate_ = static_cast<uint16_t>(std::stoi(hw_params.at("sampling_rate")));
  }

  // default status_timeout_s_ = 0.5 s without a status frame before read() reports an error
  if (hw_params.find("status_timeout") != hw_params.end())
  {
    status_timeout_s_ = std::stod(hw_params.at("status_timeout"));
  }

  if (info_.joints.size() != DEXHAND21S_JOINT_COUNT)
  {
    RCLCPP_FATAL(
      get_logger(),
      "DexHand21sHardwareInterface requires exactly 3 joints defined in the URDF/hardware config.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Validate we have one position command interface
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' command interface invalid. Expected exactly one position interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate we have three state interface
    if (joint.state_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' state interface invalid. Expected exactly four (position, velocity, "
        "temperature, current) state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate state interfaces
    bool has_position = false;
    bool has_velocity = false;
    bool has_temperature = false;
    bool has_current = false;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position = true;
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity = true;
      }
      else if (state_interface.name == "temperature")
      {
        has_temperature = true;
      }
      else if (state_interface.name == "current")
      {
        has_current = true;
      }
    }

    if (!has_position || !has_velocity || !has_current || !has_temperature)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' missing required state interfaces. Expected position, velocity, temperature, "
        "and current.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto finger_id_it = joint.parameters.find("finger_id");
    if (finger_id_it == joint.parameters.end())
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' is missing the 'finger_id' parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const int finger_id = std::atoi(finger_id_it->second.c_str());
    if (finger_id < 1 || finger_id > static_cast<int>(DEXHAND21S_JOINT_COUNT))
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has invalid finger_id '%s'. Expected 1, 2 or 3.",
        joint.name.c_str(), finger_id_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!joint_position_itfs_[finger_id - 1].empty())
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has finger_id %d, which is already used by another joint.",
        joint.name.c_str(), finger_id);
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_position_itfs_[finger_id - 1] = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    joint_velocity_itfs_[finger_id - 1] = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    joint_temperature_itfs_[finger_id - 1] = joint.name + "/temperature";
    joint_current_itfs_[finger_id - 1] = joint.name + "/current";

    RCLCPP_INFO(get_logger(), "Joint '%s' -> finger_id: %d", joint.name.c_str(), finger_id);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Probes the USB adapter, so it belongs here and not in on_init.
  hand_ = std::dynamic_pointer_cast<DexRobot::Dex021::DexHand_021S>(
    DexRobot::Dex021::DexHand::createInstance(
      DexRobot::Dex021::ProductType::DX021_S, DexRobot::Dex021::AdapterType::ZLG_MINI, 0));
  if (!hand_)
  {
    RCLCPP_FATAL(
      get_logger(), "Failed to create DexHand instance. Is the CANFD adapter connected?");
    return hardware_interface::CallbackReturn::ERROR;
  }

  DexRobot::Dex021::DH21StatusRxCallBack callback =
    std::bind(&DexHand21sHardwareInterface::stateCallbackFunc, this, std::placeholders::_1);
  hand_->setStatusRxCallback(callback);

  RCLCPP_INFO(get_logger(), "Connecting to Dex Hand...");
  try
  {
    if (!hand_->connect(true))
    {
      RCLCPP_FATAL(get_logger(), "Connection failure to DexHand.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_logger(), "Exception while connecting to Dex Hand: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hand_->setHandId(DexRobot::Dex021::AdapterChannel::CHN0, device_id_);
  hand_->setRealtimeResponse(device_id_, 0x00, sampling_rate_, 1);

  auto firmwareVersion = hand_->getFirmwareVersion(device_id_, 0x00);
  RCLCPP_INFO(get_logger(), "Firmware version = %d", firmwareVersion);

  last_status_ns_ = now_ns();
  RCLCPP_INFO(get_logger(), "Successfully connected and configured!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    hand_->clearFirmwareError(device_id_, 0x00);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Exception while clearing error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint positions and set them as the initial command values
  for (size_t i = 0; i < DEXHAND21S_JOINT_COUNT; ++i)
  {
    RCLCPP_INFO(
      get_logger(), "Initial value for interface %s: %f", joint_position_itfs_[i].c_str(),
      joint_position_states_[i]);
    set_command(joint_position_itfs_[i], joint_position_states_[i]);
  }

  failed_writes_ = 0;
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->clearFirmwareError(device_id_, 0x00);

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->clearFirmwareError(device_id_, 0x00);

  const bool disconnected = hand_->disconnect();
  hand_.reset();
  if (!disconnected)
  {
    RCLCPP_ERROR(get_logger(), "Failed to disconnect from Dex Hand.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DexHand21sHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double since_status_s = static_cast<double>(now_ns() - last_status_ns_) * 1e-9;
  if (since_status_s > status_timeout_s_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "No status from the hand for %.2f s (timeout %.2f s).",
      since_status_s, status_timeout_s_);
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < DEXHAND21S_JOINT_COUNT; ++i)
  {
    set_state(joint_position_itfs_[i], joint_position_states_[i]);
    set_state(joint_velocity_itfs_[i], joint_velocity_states_[i]);
    set_state(joint_temperature_itfs_[i], joint_temperature_states_[i]);
    set_state(joint_current_itfs_[i], joint_current_states_[i]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DexHand21sHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool all_sent = true;
  for (uint8_t i = 0; i < DEXHAND21S_JOINT_COUNT; ++i)
  {
    const double cmd = get_command(joint_position_itfs_[i]);
    if (!std::isfinite(cmd))
    {
      continue;
    }
    uint8_t finger_id = i + 1;
    int16_t hall_val = radToHall(finger_id, cmd);
    if (!hand_->moveFinger(
          device_id_, finger_id, 0x03, hall_val, static_cast<int16_t>(finger_speed_deg_s_ * 100),
          DexRobot::HALL_POSLIMIT_CONTROL_MODE, 10))
    {
      all_sent = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to send command to finger %d.", finger_id);
    }
  }

  failed_writes_ = all_sent ? 0 : failed_writes_ + 1;
  if (failed_writes_ >= MAX_FAILED_WRITES)
  {
    RCLCPP_ERROR(get_logger(), "%u consecutive write cycles failed.", failed_writes_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace dexhand21s_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dexhand21s_hardware_interface::DexHand21sHardwareInterface, hardware_interface::SystemInterface)
