# TerraBase Lite3 AEDE Docker

本分支用于在 DeepRobotics Lite3 感知主机上以 Docker 方式部署 TerraBase 的
AEDE 主干。目标是先复现
`HongbiaoZ/autonomous_exploration_development_environment` 的
`humble-matterport` 分支能力，再逐步接入 Lite3 的运动与传感器。

基本原则：

- 不修改 Jetson 宿主机 ROS Foxy、`/opt/ros`、systemd 服务或网卡长期配置。
- 传感器驱动仍由 Lite3 宿主机启动，Docker 只通过 host network 订阅 ROS 2 topic。
- 当前分支不保留 Go2 SDK、ZED wrapper 和 Go2 bridge。
- STEPP 暂时保留为后续 RealSense 阶段使用，但不进入 Lite3 v1 Docker 构建。

本地参考资料可放在 `docs/`，该目录默认不入库。

## 1. 临时联网方案

保持 Jetson `eth0` 的静态地址，不要改成 DHCP。需要下载或构建 Docker 镜像时，
在 PC 上临时开启 NAT。下面命令使用变量占位，按现场网络替换：

```bash
WAN=<PC_INTERNET_IFACE>
LAN=<PC_ROBOT_IFACE>
PC_LAN_IP=<PC_ROBOT_LAN_IP>
JETSON=<JETSON_IP>

sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -s ${JETSON}/32 -o "$WAN" -j MASQUERADE
sudo iptables -A FORWARD -i "$LAN" -o "$WAN" -s ${JETSON}/32 -j ACCEPT
sudo iptables -A FORWARD -i "$WAN" -o "$LAN" -d ${JETSON}/32 \
  -m state --state RELATED,ESTABLISHED -j ACCEPT
```

然后只在 Jetson 上临时添加默认路由：

```bash
sudo ip route add default via "$PC_LAN_IP" dev eth0
```

构建完成后删除 Jetson 默认路由：

```bash
sudo ip route del default via "$PC_LAN_IP" dev eth0
```

PC 侧 NAT 清理命令：

```bash
WAN=<PC_INTERNET_IFACE>
LAN=<PC_ROBOT_IFACE>
JETSON=<JETSON_IP>

sudo iptables -t nat -D POSTROUTING -s ${JETSON}/32 -o "$WAN" -j MASQUERADE 2>/dev/null || true
sudo iptables -D FORWARD -i "$LAN" -o "$WAN" -s ${JETSON}/32 -j ACCEPT 2>/dev/null || true
sudo iptables -D FORWARD -i "$WAN" -o "$LAN" -d ${JETSON}/32 \
  -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
```

## 2. 在 Jetson 上构建 Docker 镜像

推荐从 PC 同步当前分支到感知主机，避免依赖 Jetson 直接访问 GitHub：

```bash
JETSON_USER=<JETSON_USER>
JETSON_HOST=<JETSON_IP_OR_HOSTNAME>

rsync -az --delete \
  --exclude .git \
  --exclude terra_base_ws/build \
  --exclude terra_base_ws/install \
  --exclude terra_base_ws/log \
  --exclude log \
  ./ \
  "$JETSON_USER@$JETSON_HOST:~/TerraBase-lite3/"
```

在 Jetson 上构建：

```bash
cd ~/TerraBase-lite3
docker build -f docker/lite3-aede/Dockerfile -t terrabase-lite3:aede .
```

镜像内使用 ROS 2 Humble，并构建：

```text
lite3_bridge
local_planner
terrain_analysis
terrain_analysis_ext
sensor_scan_generation
waypoint_rviz_plugin
```

## 3. 启动 Lite3 原生传感器

在感知主机宿主环境中启动，不在 Docker 内启动：

```bash
sudo systemctl start transfer_ros2
cd ~/lite_cog_ros2/system/scripts/lidar
./start_livox.sh
```

RealSense D435i 当前只作为后续 STEPP 阶段接口保留：

```bash
sudo systemctl start realsense_ros2
```

不要在测试 AEDE 时同时启动 Lite3 原生 `start_nav.sh`，避免 Nav2 和 AEDE 同时争用
`/cmd_vel`。

## 4. 启动 Faster-LIO

实际 AEDE 输入推荐来自 Faster-LIO。不要直接用
`~/lite_cog_ros2/system/scripts/slam/start_slam.sh`，它会同时启动原生 RViz、gridmap
和保存地图流程。只给 AEDE 供输入时，单独启动 Faster-LIO：

```bash
LITE_COG_ROOT=~/lite_cog_ros2

source /opt/ros/foxy/setup.bash
source "$LITE_COG_ROOT/slam/install/setup.bash"

ROS_DOMAIN_ID=0 \
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
ros2 run faster_lio run_mapping_online \
  --ros-args \
  -r __node:=faster_lio_online \
  --params-file "$LITE_COG_ROOT/slam/src/faster_lio/config/c16.yaml"
```

启动后确认输出：

```bash
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic echo /Odometry --once --field header
ros2 topic echo /cloud_registered --once --field header
```

当前 Jetson Faster-LIO 代码中：

- `/Odometry` 是 `camera_init -> body` 语义。
- `/cloud_registered` 是 `camera_init` 下的注册点云。
- `lite3_bridge` 会把这套输入转成 AEDE 使用的 `map -> sensor -> vehicle`
  TF 语义。
- 如果 Faster-LIO 的 `body` 前向与机器狗真实前向不一致，使用
  `inputToVehicleYaw` 修正 `body -> vehicle`，不要用 `sensorYaw` 修正行走朝向。

如果 `/Odometry` 或 `/cloud_registered` 没有 publisher，不要启动
`input_profile:=faster_lio` 做导航验证，先排查 Faster-LIO 的雷达/IMU 输入。

## 5. 启动 AEDE 主干

安全 smoke test，默认不发布真实运动指令：

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  terrabase-lite3:aede \
  ros2 launch lite3_bridge lite3_aede.launch.py \
    input_profile:=raw_smoke \
    enable_motion:=false
```

`raw_smoke` 默认使用：

- `/leg_odom2` -> `/state_estimation`
- `/rslidar_points` -> `/registered_scan`

如果当前机器只发布 `/leg_odom`，直接覆盖参数：

```bash
ros2 launch lite3_bridge lite3_aede.launch.py \
  input_profile:=raw_smoke \
  odom_topic:=/leg_odom \
  enable_motion:=false
```

后续实际导航优先使用 Faster-LIO 输出：

```bash
docker run --rm -it --network host --ipc host \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  terrabase-lite3:aede \
  ros2 launch lite3_bridge lite3_aede.launch.py \
    input_profile:=faster_lio \
    enable_motion:=false
```

`faster_lio` 默认使用：

- `/Odometry` -> `/state_estimation`
- `/cloud_registered` -> `/registered_scan`

Faster-LIO `body` 到 AEDE `vehicle` 的修正参数默认全为 0。先保持
`enable_motion:=false`，按小角度试调：

```bash
ros2 launch lite3_bridge lite3_aede.launch.py \
  input_profile:=faster_lio \
  enable_motion:=false \
  inputToVehicleYaw:=0.174533
```

`inputToVehicleYaw` 单位是弧度，正值表示 `vehicle` 相对输入 odom child frame
绕 Z 轴正向旋转。实机朝向问题优先调这个参数；`sensorYaw` 只用于描述
`vehicle -> sensor` 的传感器安装旋转，AEDE 局部规划不适合用它修正机器狗行走前向。

TF 预期：

```text
map
└── sensor
    ├── vehicle
    └── camera
```

关键检查：

```bash
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 topic hz /terrain_map
ros2 run tf2_ros tf2_echo map vehicle
ros2 topic echo /cmd_vel_preview --once
```

默认 `enable_motion:=false` 时 bridge 只发布 `/cmd_vel_preview`。显式设置
`enable_motion:=true` 后才会发布 Lite3 原生 `/cmd_vel`，并限制：

- `linear.x <= 0.2 m/s`
- `linear.y = 0`
- `angular.z <= 0.3 rad/s`

Lite3 必须处于 Navigation mode 才会响应 `/cmd_vel`。

## 6. 在 NoMachine 中使用 RViz2

RViz2 已安装在 `terrabase-lite3:aede` 镜像内。建议在 NoMachine 远控桌面里的
Jetson 终端运行：

```bash
echo $DISPLAY
xhost +local:root
```

启动 RViz2：

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  terrabase-lite3:aede \
  ros2 launch lite3_bridge lite3_rviz.launch.py
```

如果 RViz 报 OpenGL/GLX 错误，可先用软件渲染重试：

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  terrabase-lite3:aede \
  ros2 launch lite3_bridge lite3_rviz.launch.py
```

用完收回 X11 权限：

```bash
xhost -local:root
```

Lite3 RViz 配置默认显示 `/registered_scan`、`/terrain_map`、`/aede/path`、
`/aede/free_paths` 和 `/way_point`。按 `w` 可使用 AEDE 的 Waypoint 工具，它会发布
`/way_point` 和 `/joy` 给 `localPlanner`。

## 7. 本地验证命令

```bash
python3 -m py_compile \
  terra_base_ws/src/lite3_bridge/lite3_bridge/bridge_node.py \
  terra_base_ws/src/lite3_bridge/launch/lite3_aede.launch.py \
  terra_base_ws/src/lite3_bridge/launch/lite3_rviz.launch.py

colcon list --base-paths terra_base_ws/src --packages-select \
  lite3_bridge local_planner terrain_analysis terrain_analysis_ext \
  sensor_scan_generation waypoint_rviz_plugin

docker run --rm terrabase-lite3:aede \
  python3 -m pytest -q /opt/terrabase/terra_base_ws/src/lite3_bridge/test
```
