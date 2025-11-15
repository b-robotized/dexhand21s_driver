// Copyright (c) 2025 b»robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential
//
// Adapted for <Insert_Company_Name> that received unlimited, worldwide
// use and change right, except distributing this library separately
// of their product.

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestDexHand21sHW : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    dexhand21s_hardware_2dof_ =
      R"(
        <ros2_control name="DexHand21sHW2dof" type="system">
          <hardware>
            <plugin>dexhand21s_hardware/DexHand21sHW</plugin>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">1.57</param>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">0.7854</param>
          </joint>
        </ros2_control>
    )";
  }

  std::string dexhand21s_hardware_2dof_;
};

TEST_F(TestDexHand21sHW, load_dexhand21s_hardware_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + dexhand21s_hardware_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
