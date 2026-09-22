# dexhand21s_hardware_interface

ros2 control hardware interface for DexRobot 21s hand.

```bash
Pluginlib-Library: dexhand21s_hardware_interface
Plugin: dexhand21s_hardware_interface/DexHand21sHardwareInterface (hardware_interface::SystemInterface)
```

## Installation

The SDK is wrapped in the `dexhand_vendor` package, which is not in rosdistro. Import it with the `.repos` file:

```bash
cd <ws>/src
git clone https://github.com/b-robotized/dexhand21s_driver.git
vcs import < dexhand21s_driver/dexhand21s_driver.jazzy.repos
cd <ws>
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

The ZLG USB-CANFD adapter needs a udev rule or root to be accessible.

## Usage

- View the robot with joint sliders: `ros2 launch dexhand21s_description view_dexhand21s.launch.xml`
- Bring up the real hand (ZLG USB-CANFD mini adapter plugged in, hand ID 1):
  `ros2 launch dexhand21s_bringup dexhand21s.launch.xml`
- Bring up with mock hardware, no adapter needed:
  `ros2 launch dexhand21s_bringup dexhand21s.launch.xml use_mock_hardware:=true`

Both bringup variants start the `joint_trajectory_controller` (active) and the `forward_position_controller` (inactive), plus RViz and rqt_joint_trajectory_controller. Test publishers for either controller are in `dexhand21s_bringup/launch/test_*.launch.xml`.

## Position limits for active joints

- Finger 1: from -1.33rad (= 0 hall = extended) to 0.0 (= 1000 hall = closed)

- Finger 2 & Finger 3: from 0.0rad (= 0 hall = extended) to 1.33rad (= 1000 hall = closed)

The interface automatically scales the internal hall readings to expose only rad in state and command interfaces.

## Limitations

- Only AdapterType::ZLG_MINI is supported.
- Commanding and reading the driving joints of the three fingers. Can't find the fourth joint.

## Resources

- Product Info: https://dexrobot.feishu.cn/file/YXD2bx4kmo8Zs4xLmPDcdD00nzd
- Manual: https://dexrobot.feishu.cn/docx/XZwrdnZuuo74tTx9cXMcKm7snNd
- Organization page: https://github.com/DexRobot
- All Dex Product info: https://dexrobot.feishu.cn/docx/XbM6dLH4counrgxNXHHcqSEOnve
- sdk and urdf (ROS1): https://github.com/DexRobot/dexhand_sdk_cpp/tree/main?tab=readme-ov-file
