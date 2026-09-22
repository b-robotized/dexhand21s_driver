# dexhand21s_hardware_interface

ros2 control hardware interface for DexRobot 21s hand.

Pluginlib-Library: dexhand21s_hardware_interface
Plugin: dexhand21s_hardware_interface/DexHand21sHardwareInterface (hardware_interface::SystemInterface)

## Testing

- view robot and rqt: `ros2 launch dexhand21s_description view_dexhand21s.launch.xml`
- bringup dexhand hardware with real hand: ``
- bringup dexhand with mockhw: upcoming!

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
