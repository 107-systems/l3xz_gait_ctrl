<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_ctrl`
=========================

Gait controller for the L3XZ electric/hydraulic hexapod robot.

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
