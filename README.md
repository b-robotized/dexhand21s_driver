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

## Startup checklist

Steps marked *(to confirm)* were verified with mock hardware only and still need one session with the real hand.

1. **Adapter access.** Plug in the ZLG USB-CANFD mini adapter and find it with `lsusb`. Either run as root or add a udev rule with the vendor and product IDs shown there, then replug:
   ```
   SUBSYSTEM=="usb", ATTR{idVendor}=="<vid>", ATTR{idProduct}=="<pid>", MODE="0666"
   ```
2. **Hand power.** Power the hand and connect it to channel 0 of the adapter. The driver assigns `device_id` (default 1) to whatever hand is on that channel.
3. **Launch.** `ros2 launch dexhand21s_bringup dexhand21s.launch.xml`. Expected log lines from the hardware interface:
   - `Joint '...' -> finger_id: N` for the three active joints
   - `Connecting to Dex Hand...` followed by `Firmware version = ...` *(to confirm)*
   - `Initial value for interface ...` for each joint, then `Successfully activated!`
4. **Check the controllers.** `ros2 control list_controllers` must show `joint_state_broadcaster` and `joint_trajectory_controller` active and `forward_position_controller` inactive. `ros2 control list_hardware_interfaces` lists 3 command and 12 state interfaces.
5. **Check the states.** `ros2 topic echo /joint_states --once` should show the current finger positions in radians, not zeros, and they should change when a finger is pushed by hand *(to confirm)*.
6. **First movement.** Use the rqt sliders that open with the launch, or run the test publisher, which cycles the fingers through three poses every 6 s:
   `ros2 launch dexhand21s_bringup test_joint_trajectory_controller.launch.xml`
   Keep clear of the fingers, the default speed is 10 deg/s so a full close takes about 7 s.
7. **Mock run.** The same sequence with `use_mock_hardware:=true` needs no adapter and is what steps 4 and 6 were validated against.

Common failures:

- `Failed to create DexHand instance` on configure: adapter not found or not accessible. Check `lsusb` and the udev rule.
- `Connection failure to DexHand`: adapter found but the hand does not answer. Check power and the cable on channel 0.
- `No status from the hand for X s`: the hand stopped streaming after a successful start. The component goes to the error state; check cable and power, then restart the launch. If this fires right after activation with a healthy hand, raise `status_timeout` *(to confirm the 0.5 s default)*.
- `N consecutive write cycles failed`: commands cannot be sent. Same checks as above.

## Hardware parameters

Set in the `<hardware>` block of the ros2_control xacro:

- `device_id` (default 1): hand ID assigned to the connected hand on configure.
- `finger_speed_deg_s` (default 10): motor speed used for every position command, in degrees per second. 75 degrees is the full range of a finger.
- `sampling_rate` (default 50): rate in Hz at which the hand streams its status over CANFD.
- `status_timeout` (default 0.5): seconds without a status frame after which `read()` returns an error and the component goes into the error state.

## State interfaces

Per active joint the interface exports:

- `position`: radians, converted from the hall sensor (see below).
- `velocity`, `temperature`, `current`: raw integer values from the SDK status frame, cast to double. The SDK does not document their units, so treat them as relative readings until verified against the manual. In particular `velocity` is **not** rad/s.

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
