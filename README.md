# OpenArm MuJoCo Hardware Interface

This package provides a ros2_control hardware interface for simulating OpenArm using the MuJoCo physics engine in place of physical hardware. It connects to a WebAssembly instance of MuJoCo through WebSockets.

# Prerequisites

- [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (tested with Humble)
- [openarm_description](https://github.com/enactic/openarm_description)
- [openarm_ros2](https://github.com/enactic/openarm_ros2)

## Usage

https://github.com/user-attachments/assets/a8e2d23b-b206-4966-91c6-e72fa95b18c1

To use MoveIt2 with simulated bimanual hardware, first run MuJoCo by visiting:

[thomasonzhou.github.io/mujoco_anywhere](https://thomasonzhou.github.io/mujoco_anywhere/)

Then run the original command with the `use_fake_hardware` and `use_sim_hardware` flags:
```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py hardware_type:=mujoco
```

Please note that running multiple instances of the website will cause conflicting signals. Future configurations will allow for multiple instances to run simultaneously.

## Configuration

### Hardware Plugin Config

The hardware plugin is specified in `openarm_description/openarm.ros2_control.xacro` as follows:

```xml
<ros2_control name="openarm_system" type="system">
  <hardware>
    <plugin>openarm_mujoco_hardware/MujocoHardware</plugin>
    <param name="prefix">left_</param>
    <param name="websocket_port">1337</param>
  </hardware>
  <!-- Joint configurations -->
</ros2_control>
```

When using OpenArm in a bimanual configuration, the WebSocket ports default to 1337 for right arm and 1338 for left arm commands. However, in practice commands can be sent and states can be received through any connected port.
