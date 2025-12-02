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

#include "dexhand21s_hardware_interface/dexhand21s_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace DexRobot;
using namespace DexRobot::Dex021;

namespace dexhand21s_hardware_interface
{

void CallbackFunc(const DX21StatusRxData * status)
{
  printf(
    "Position1 = %d, Speed1 = %d, Current1=%d, MT Temp1=%d\n", status->MotorHallValue(1),
    status->MotorVelocity(1), status->MotorCurrent(1), status->MotorTemperature(1));

  printf(
    "Position2 = %d, Speed2 = %d, Current2=%d, MT Temp2=%d\n", status->MotorHallValue(2),
    status->MotorVelocity(2), status->MotorCurrent(2), status->MotorTemperature(2));

  printf(
    "Position3 = %d, Speed3 = %d, Current3=%d, MT Temp3=%d\n", status->MotorHallValue(3),
    status->MotorVelocity(3), status->MotorCurrent(3), status->MotorTemperature(3));

  printf(
    "Position4 = %d, Speed4 = %d, Current4=%d, MT Temp4=%d\n", status->MotorHallValue(4),
    status->MotorVelocity(4), status->MotorCurrent(4), status->MotorTemperature(4));

  printf("\n");
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

  // TODO(yara): read parameter for can type and initialize accordingly
  const AdapterType atype = AdapterType::ZLG_MINI;
  const auto device_ = DexHand::createInstance(ProductType::DX021_S, atype, 0);
  hand_ = std::dynamic_pointer_cast<DexHand_021S>(device_);

  if (!hand_)
  {
    RCLCPP_FATAL(get_logger(), "Failed to create DexHand instance.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  DH21StatusRxCallBack callback = std::bind(CallbackFunc, std::placeholders::_1);
  hand_->setStatusRxCallback(callback);

  if (!hand_->connect(true))
  {
    RCLCPP_FATAL(get_logger(), "Connection failure to DexHand.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DexHand 021S has exactly one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    uint8_t finger_id =
      static_cast<uint8_t>(std::stoi(joint.parameters.at("finger_id"), nullptr, 0));
    joint_finger_ids_[joint.name + "/position"] = finger_id;
    RCLCPP_INFO(get_logger(), "Joint '%s' -> finger_id: %d", joint.name.c_str(), finger_id);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  device_id_ = 0x01;
  hand_->setHandId(AdapterChannel::CHN0, device_id_);
  hand_->setRealtimeResponse(device_id_, 0x00, 50, 0);

  auto firmwareVersion = hand_->getFirmwareVersion(device_id_, 0x00);
  RCLCPP_INFO(get_logger(), "Firmware version = %d", firmwareVersion);

  auto maxCurrent = hand_->getSafeCurrent(device_id_, 0x01);
  RCLCPP_INFO(get_logger(), "Limited Max Current = %d", maxCurrent);

  auto maxTemperature = hand_->getSafeTemperature(device_id_, 0x01);
  RCLCPP_INFO(get_logger(), "Limited Max Temperature = %d", maxTemperature);

  hand_->clearFirmwareError(device_id_, 0x00);
  hand_->resetJoints(device_id_);

  hand_->clearFirmwareError(device_id_, 0x00);

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }
  hand_->clearFirmwareError(device_id_, 0x00);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DexHand21sHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hand_->clearFirmwareError(device_id_, 0x00);
  hand_->resetJoints(device_id_);

  hand_->clearFirmwareError(device_id_, 0x00);

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DexHand21sHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DexHand21sHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    hand_->moveFinger(
      device_id_, joint_finger_ids_[name], 0x03, static_cast<int16_t>(get_command(name)), 1500,
      HALL_POSLIMIT_CONTROL_MODE, 10);
  }
  // hand_->clearFirmwareError(device_id_, 0x00);

  return hardware_interface::return_type::OK;
}

}  // namespace dexhand21s_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dexhand21s_hardware_interface::DexHand21sHardwareInterface, hardware_interface::SystemInterface)
