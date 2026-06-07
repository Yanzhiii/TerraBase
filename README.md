# TerraBase

TerraBase is a ROS 2 Humble overlay that connects a Unitree Go2 running the upstream `go2_ros2_sdk` driver to the upstream Autonomous Exploration Development Environment (AEDE) planner stack.

The external projects are kept under `third_party/` as git submodules and are treated as read-only dependencies. TerraBase-specific behavior lives in `terra_base_ws/src/go2_bridge`.

## Repository Layout

```text
.
├── third_party/
│   ├── go2_ros2_sdk/                              # submodule, upstream Go2 ROS 2 SDK
│   ├── autonomous_exploration_development_environment/ # submodule, upstream AEDE
│   ├── STEPP-Code/                                # submodule, STEPP ROS 2 Humble port
│   └── zed-ros2-wrapper/                          # submodule, official Stereolabs wrapper
├── terra_base_ws/
│   ├── src/
│   │   ├── go2_bridge/                            # TerraBase overlay package
│   │   ├── go2_interfaces -> ../../third_party/go2_ros2_sdk/go2_interfaces
│   │   ├── go2_robot_sdk -> ../../third_party/go2_ros2_sdk/go2_robot_sdk
│   │   ├── stepp_ros2_humble -> ../../third_party/STEPP-Code/stepp_ros2_humble
│   │   ├── zed_components -> ../../third_party/zed-ros2-wrapper/zed_components
│   │   ├── zed_wrapper -> ../../third_party/zed-ros2-wrapper/zed_wrapper
│   │   ├── local_planner -> ../../third_party/autonomous_exploration_development_environment/src/local_planner
│   │   ├── terrain_analysis -> ../../third_party/autonomous_exploration_development_environment/src/terrain_analysis
│   │   ├── terrain_analysis_ext -> ../../third_party/autonomous_exploration_development_environment/src/terrain_analysis_ext
│   │   ├── sensor_scan_generation -> ../../third_party/autonomous_exploration_development_environment/src/sensor_scan_generation
│   │   └── waypoint_rviz_plugin -> ../../third_party/autonomous_exploration_development_environment/src/waypoint_rviz_plugin
│   └── scripts/
└── docs/
```

## Reproduce From Scratch

Clone with submodules:

```bash
git clone --recurse-submodules <this-repo-url> TerraBase
cd TerraBase
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Install ROS dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip
```

Install Python dependencies used by the Go2 SDK:

```bash
python3 -m pip install --user \
  aiortc aioice cryptography pycryptodome wasmtime requests numpy pyyaml
```

Install the ZED SDK 5.3.1 with the official installer so the complete SDK lives at `/usr/local/zed`.
The full-stack ZED 2i path requires at least:

```text
/usr/local/zed/zed-config.cmake
/usr/local/zed/lib/libsl_zed.so
/usr/local/zed/lib/libsl_ai.so
```

Build the selected packages:

```bash
cd terra_base_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  go2_interfaces go2_robot_sdk go2_bridge \
  zed_components zed_wrapper \
  local_planner terrain_analysis terrain_analysis_ext \
  sensor_scan_generation waypoint_rviz_plugin
source install/setup.bash
```

For zsh, use `setup.zsh` instead of `setup.bash`.

## Run On A Real Go2

Set the robot connection:

```bash
export ROBOT_IP=192.168.123.161
# export ROBOT_TOKEN=<token-if-needed>
```

Launch the real-robot stack:

```bash
ros2 launch go2_bridge go2_aede_real.launch.py
```

Useful launch arguments:

```bash
ros2 launch go2_bridge go2_aede_real.launch.py \
  robot_ip:=192.168.123.161 \
  enable_video:=false \
  max_linear_speed:=0.3 \
  max_lateral_speed:=0.2 \
  max_yaw_rate:=0.5 \
  keyboard_teleop:=false \
  rviz:=true
```

Default command forwarding is speed-limited in `go2_bridge`. Keep limits low until `/cmd_vel_out` has been inspected on the target robot.

## Runtime Checks

Verify the driver and bridge:

```bash
ros2 node list | grep -E 'go2_driver_node|go2_aede_bridge|go2_joint_state_republisher'
ros2 topic hz /joint_states
ros2 topic hz /odom
ros2 topic hz /point_cloud2
```

Verify AEDE-facing topics:

```bash
ros2 topic echo /state_estimation --once | grep -E 'frame_id|child_frame_id'
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map
ros2 topic hz /path
```

Verify TF:

```bash
ros2 run tf2_tools view_frames
```

Expected key frames are `map -> odom -> base_link -> sensor`; AEDE also publishes/uses `sensor -> vehicle` and `sensor -> camera`.

By default `sensor` is colocated with `base_link`; physical Go2 radar/LiDAR TF comes from the URDF, and ZED mount extrinsics are managed in `terra_base_ws/src/go2_bridge/config/extrinsics.yaml`.

## Run With ZED 2i + STEPP

Build the STEPP-capable package set:

```bash
cd terra_base_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  go2_interfaces go2_robot_sdk go2_bridge \
  zed_components zed_wrapper \
  stepp_ros2_humble local_planner
source install/setup.bash
```

Launch the full stack:

```bash
export ROBOT_IP=192.168.123.161
export ZED_SERIAL_NUMBER=<ZED_SERIAL_NUMBER>
export STEPP_MODEL_PATH=/abs/path/to/checkpoint.pth
ros2 launch go2_bridge go2_zed_stepp_aede.launch.py
```

Before real autonomous runs, measure the ZED mount and update `terra_base_ws/src/go2_bridge/config/extrinsics.yaml`.

## Submodule Policy

TerraBase uses git submodules pointing to maintained forks under `github.com/Yanzhiii`. Each fork carries a small, documented patch set on top of the upstream release branch so that `git clone --recurse-submodules` produces a ready-to-build workspace without additional steps.

| Submodule | Upstream | Fork (TerraBase) | Patches |
|-----------|----------|-------------------|---------|
| `STEPP-Code` | [Yanzhiii/STEPP-Code](https://github.com/Yanzhiii/STEPP-Code) | (same) | add SLIC seg viz, fix zero-range div |
| `go2_ros2_sdk` | [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | [Yanzhiii/go2_ros2_sdk](https://github.com/Yanzhiii/go2_ros2_sdk) | `cv_bridge` lazy-import |
| `autonomous_exploration_development_environment` | [HongbiaoZ/autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | [Yanzhiii/autonomous_exploration_development_environment](https://github.com/Yanzhiii/autonomous_exploration_development_environment) | `static_transform_publisher` Humble syntax |
| `zed-ros2-wrapper` | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) | (upstream, no patches) | – |

## Notes

- TerraBase launches the official Stereolabs `zed-ros2-wrapper` directly; the ZED SDK itself must be installed under `/usr/local/zed`.
