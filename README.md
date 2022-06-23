<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_ctrl`
=========================

Gait controller for the L3XZ electric/hydraulic hexapod robot.

### How-to-build
#### Install Dependencies:
* [orocos-kdl](https://github.com/orocos/orocos_kinematics_dynamics):
  * How-to-build from [source](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/INSTALL.md)
  * Install a [prebuilt](https://github.com/107-systems/orocos-kdl-debian) Debian package
#### Build via `colcon`
```bash
# Clone this repository into colcon_ws/src.
git clone https://github.com/107-systems/l3xz_ctrl
# Invoke 'colcon build' from repository root.
source /opt/ros/galactic/setup.bash
colcon build --packages-select l3xz_ctrl
```

### How-to-run
```bash
. install/setup.bash
ros2 launch l3xz_ctrl ctrl.py
```

### Interface Documentation
#### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/ctrl/input` | [`l3xz_ctrl/Input`](msg/input/Input.msg) |
| `/l3xz/cmd_vel` | `l3xz_teleop/Teleop` |

#### Published Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/ctrl/output` | [`l3xz_ctrl/Output`](msg/output/Output.msg) |
