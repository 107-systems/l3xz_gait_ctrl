<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_gait_ctrl`
==============================
[![Build Status](https://github.com/107-systems/l3xz_gait_ctrl/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_gait_ctrl/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_gait_ctrl/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_gait_ctrl/actions/workflows/spell-check.yml)

Gait controller for the L3X-Z electric/hydraulic hexapod robot.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
##### Install Dependencies:
* [orocos-kdl](https://github.com/orocos/orocos_kinematics_dynamics):
  * How-to-build from [source](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md)
  * Install a [prebuilt](https://github.com/107-systems/orocos-kdl-debian) Debian package
##### Build via `colcon`
```bash
colcon_ws/src$ git clone https://github.com/107-systems/l3xz_gait_ctrl_gait_ctrl
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_gait_ctrl
```

#### How-to-run
```bash
colcon_ws$ . install/setup.bash
colcon_ws$ ros2 launch l3xz_gait_ctrl gait_ctrl.py
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/cmd_vel_robot` | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |
| `/l3xz/ctrl/leg/angle/actual` | [`l3xz_gait_ctrl/LegAngle`](msg/LegAngle.msg) |

##### Published Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/ctrl/leg/angle/target` | [`l3xz_gait_ctrl/LegAngle`](msg/LegAngle.msg) |
