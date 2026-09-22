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

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const char kPlugin[] = "dexhand21s_hardware_interface/DexHand21sHardwareInterface";

std::string joint(const std::string & name, int finger_id, bool all_states = true)
{
  std::string s = "<joint name=\"" + name + "\">\n";
  if (finger_id > 0)
  {
    s += "<param name=\"finger_id\">" + std::to_string(finger_id) + "</param>\n";
  }
  s +=
    "<command_interface name=\"position\"/>\n"
    "<state_interface name=\"position\"/>\n";
  if (all_states)
  {
    s +=
      "<state_interface name=\"velocity\"/>\n<state_interface name=\"temperature\"/>\n"
      "<state_interface name=\"current\"/>\n";
  }
  return s + "</joint>\n";
}

std::string description(const std::string & joints)
{
  return std::string(ros2_control_test_assets::urdf_head) +
         "<ros2_control name=\"DexHand21s\" type=\"system\"><hardware><plugin>" + kPlugin +
         "</plugin></hardware>\n" + joints + "</ros2_control>" +
         ros2_control_test_assets::urdf_tail;
}

hardware_interface::ResourceManager make_rm()
{
  return hardware_interface::ResourceManager(
    std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test_dexhand21s"));
}
}  // namespace

// Loads the shared library and resolves the vendor SDK libraries.
TEST(TestDexHand21sHardwareInterface, plugin_loads)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  ASSERT_NO_THROW(loader.createUnmanagedInstance(kPlugin));
}

// on_init must reject descriptions that don't match the hand, before touching the SDK.
TEST(TestDexHand21sHardwareInterface, rejects_wrong_joint_count)
{
  auto rm = make_rm();
  EXPECT_FALSE(
    rm.load_and_initialize_components(description(joint("joint1", 1) + joint("joint2", 2))));
}

TEST(TestDexHand21sHardwareInterface, rejects_missing_state_interfaces)
{
  auto rm = make_rm();
  EXPECT_FALSE(rm.load_and_initialize_components(
    description(joint("joint1", 1) + joint("joint2", 2) + joint("joint3", 3, false))));
}

TEST(TestDexHand21sHardwareInterface, rejects_missing_finger_id)
{
  auto rm = make_rm();
  EXPECT_FALSE(rm.load_and_initialize_components(
    description(joint("joint1", 1) + joint("joint2", 2) + joint("joint3", 0))));
}

TEST(TestDexHand21sHardwareInterface, rejects_duplicate_finger_id)
{
  auto rm = make_rm();
  EXPECT_FALSE(rm.load_and_initialize_components(
    description(joint("joint1", 1) + joint("joint2", 2) + joint("joint3", 2))));
}

TEST(TestDexHand21sHardwareInterface, rejects_out_of_range_finger_id)
{
  auto rm = make_rm();
  EXPECT_FALSE(rm.load_and_initialize_components(
    description(joint("joint1", 1) + joint("joint2", 2) + joint("joint3", 4))));
}

// Valid description without the CANFD adapter plugged in: init succeeds (no hardware access),
// configure fails cleanly instead of crashing.
TEST(TestDexHand21sHardwareInterface, valid_description_without_hardware)
{
  auto rm = make_rm();
  ASSERT_TRUE(rm.load_and_initialize_components(
    description(joint("joint1", 1) + joint("joint2", 2) + joint("joint3", 3))));
  rclcpp_lifecycle::State inactive(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
  EXPECT_EQ(rm.set_component_state("DexHand21s", inactive), hardware_interface::return_type::ERROR);
}
