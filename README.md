# GCOPTER-ros2 — ROS 2 Humble

[![ROS 2 Humble](https://github.com/hsc13576717115/GCOPTER-ros2/actions/workflows/humble.yml/badge.svg)](https://github.com/hsc13576717115/GCOPTER-ros2/actions/workflows/humble.yml)

将 [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) 的全局规划示例移植到 **Ubuntu 22.04 / ROS 2 Humble**，保留原有 MINCO、FIRI、微分平坦映射和 L-BFGS 主体算法。

这是**规划演示和参考轨迹回放**，不包含飞控、真实无人机动力学积分或闭环跟踪控制。RViz2 中的球体直接沿规划轨迹运动，状态曲线由轨迹及物理模型计算，不是传感器测量。

```text
mockamap 点云 → 体素占用/膨胀 → Informed RRT* 路径
  → FIRI 凸安全走廊 → GCOPTER + MINCO + L-BFGS
  → RViz2 地图/路径/走廊/轨迹/球体回放 + rqt_plot 状态曲线
```

## 安装与构建

先安装 ROS 2 Humble。建议使用系统 `/usr/bin/python3`，避免 Conda 覆盖 ROS Python 环境。

```bash
sudo apt update
sudo apt install libompl-dev libeigen3-dev libpcl-dev \
  ros-humble-pcl-conversions ros-humble-rviz2 ros-humble-rqt-plot \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  python3-colcon-common-extensions python3-rosdep python3-pytest

git clone https://github.com/hsc13576717115/GCOPTER-ros2.git
cd GCOPTER-ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths gcopter map_gen --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

两个 ROS 包仍叫 `gcopter` 和 `mockamap`。也可以把整个仓库放在现有 colcon 工作空间的 `src/` 中。

如果其他依赖已经安装、仅 OMPL 缺失且没有 sudo，可在 Ubuntu 22.04 x86_64 使用官方 Debian 软件包的用户目录安装方式：

```bash
bash scripts/install_ompl_local.sh
source scripts/env.sh
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source scripts/env.sh
```

这个备用脚本仅解包 OMPL，不安装其 Boost/ODE 等系统依赖；需要这些依赖预先存在。本机测试采用了该方式。依赖存放在 `~/.local/share/gcopter-ros2-deps/`，不进入仓库。

## 启动完整演示

```bash
source scripts/env.sh
ros2 launch gcopter global_planning.launch.py
```

1. 等待地图出现，终端输出 `MAP_READY`。
2. 在 RViz2 选择 **2D Goal Pose** 工具，点击并拖动箭头，选择起点。
3. 再次用该工具选择终点，触发路径搜索及轨迹优化。
4. 查看红色路径、蓝色平滑轨迹、透明走廊和移动球体；rqt_plot 显示速度、推力、倾角、机体角速度。
5. 输出 `COMPLETED` 后，重新选一组起终点可再次规划。原轨迹可视化会清理。

保留原版选高度方式：箭头的偏航四元数 `|q_z|` 映射到高度区间。

```text
z = z_min + DilateRadius + |q_z| × (z_max - z_min - 2 × DilateRadius)
```

默认高度范围为 0.5～4.5 m。箭头沿 +x 时高度最低，沿 -x 时最高。位置过于靠近障碍或位于地图之外会被拒绝，重新选择即可。所有输入使用 `odom` 坐标系；本示例不执行 TF 坐标变换。

```bash
# 无界面运行，适合自动测试
ros2 launch gcopter global_planning.launch.py rviz:=false plot:=false
# 只关闭曲线窗口
ros2 launch gcopter global_planning.launch.py plot:=false
# 接收外部静态 PointCloud2（发布端使用 reliable + transient_local）
ros2 launch gcopter global_planning.launch.py map:=false
# 使用自己的参数文件
ros2 launch gcopter global_planning.launch.py params_file:=/absolute/path/to/config.yaml
```

`mockamap` 原有 `perlin3d`、`post2d`、`maze2d`、`maze3d`、`mockamap` 启动文件均改为 `.launch.py`，例如：

```bash
ros2 launch mockamap maze3d.launch.py
```

地图只在规划节点初始化时接收一次。改变地图应重启规划节点。

## 配置与接口

规划参数在 `gcopter/config/global_planning.yaml` 中，使用 ROS 2 节点参数格式。地图生成参数位于主 launch。

| 参数 | 含义 |
|---|---|
| `MapBound` / `VoxelWidth` | xyz 边界及体素尺寸，跨度必须是体素尺寸的整数倍 |
| `DilateRadius` | 用户要求的障碍膨胀半径 |
| `TimeoutRRT` | 实际生效的前端搜索预算，默认 0.5 s |
| `PlanningSeed` | OMPL 随机种子，默认 42；搜索按时间终止，因此不同机器仍可能得到不同路径 |
| `MaxVelMag` / `MaxBdrMag` / `MaxTiltAngle` | 速度、机体角速度、倾角限制 |
| `MinThrust` / `MaxThrust` | 总推力范围 |
| `VehicleMass` / `GravAcc` / 阻力参数 | 平坦映射使用的物理模型 |
| `WeightT` / `ChiVec` | 时间权重及五类约束罚项权重 |
| `IntegralIntervs` / `RelCostTol` | 约束积分划分与优化停止阈值 |

FIRI 原算法处理体素表面中心点。为减少曲线擦入体素边缘，路径和走廊构造使用**额外一层体素膨胀**；优化后的曲线仍按原 `DilateRadius` 对应占用地图检查。代价是狭窄空间的可用范围变小。采样检查不能替代连续时间安全证明。

| 话题 | ROS 2 类型 / 行为 |
|---|---|
| `/voxel_map` | `sensor_msgs/msg/PointCloud2`，float32 x/y/z，支持字段重排、行填充和端序；空点云表示自由空间 |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped`，连续两个有效输入作为起终点 |
| `/visualizer/route`、`waypoints`、`mesh`、`edge`、`trajectory`、`spheres` | `visualization_msgs/msg/Marker`，可靠、transient local |
| `/visualizer/speed`、`total_thrust`、`tilt_angle`、`body_rate` | `std_msgs/msg/Float64`，100 Hz 参考状态回放 |
| `/planner/status` | `std_msgs/msg/String`，保留最近状态，便于自动验收 |

状态依次为 `WAITING_FOR_MAP → MAP_READY → START_ACCEPTED → PLANNING → VALIDATED → SUCCESS → COMPLETED`。异常使用 `INVALID_MAP`、`REJECTED`、`INVALID_TRAJECTORY` 或 `FAILED` 前缀，并附原因。`VALIDATED` 日志含轨迹总时间、终点误差、采样间隔和五类动态约束的最大超限值。

## 自动验收

```bash
source scripts/env.sh
colcon test --executor sequential --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose

# 单独运行真实节点端到端测试，保存日志与 JSON 到 ./artifacts
ros2 run gcopter verify_demo.py
```

验收覆盖带障碍、空地图、不可达后恢复、重复规划、非法点云、越界/障碍内/重复选点、晚订阅地图和轨迹、四种地图生成器，以及完整回放到终点。

每条被接受的轨迹会检查正分段时间、有限位置及动力学量、终点误差 ≤ 1 mm，并按不超过 0.01 s 的间隔检查膨胀地图占用；同时记录动态约束超限。约束采用软罚项，不声称连续时间严格满足所有限制。

测试默认使用 `ROS_DOMAIN_ID=83`（仅在未设置时生效），与其他机器人节点隔离。GitHub Actions 在 ROS 2 Humble 环境执行构建和无界面测试，上传测试日志。桌面验证及截图见 [验证记录](docs/validation.md)。

## 原版差异与来源

- ROS 1 `catkin` / `roscpp` 替换为 ROS 2 `ament_cmake` / `rclcpp`；适配参数、定时器、消息、QoS、launch、RViz2。
- 搜索时间从原入口硬编码 0.01 s 改为使用 `TimeoutRRT`，默认 0.5 s。
- 增加参数和点云校验，拒绝近似目标路径；修复空路径、空局部障碍、零膨胀表面点、负边界索引等问题。
- 修正 C++17 对排序比较器 `const` 的要求和反三角函数浮点范围，补齐回放终点与失败状态。
- 删除未在原地图节点调用的 ROS 1 `ces_randommap.cpp`；历史仍保留在 Git 中。
- 原始版本基于上游提交 `e0444f6`，保留 [原始 README 与论文引用](docs/README.upstream.md)。

GCOPTER 原作者为 Zhepei Wang、Fei Gao，根目录及核心头文件保留原 MIT 许可证；mockamap 的包声明保留 GPLv3。此仓库仅提供 ROS 2 移植和工程验证，不改变原作者归属。

### 从 Snap 版 VS Code 启动 GUI

若 RViz2/rqt_plot 报错涉及 `/snap/core20/.../libpthread.so.0`，这是编辑器注入的 GTK/GIO 环境与系统库冲突。先 `source scripts/env.sh` 再启动；该脚本在检测到 Snap GTK 路径时清理相关覆盖变量，不修改系统库。也可以使用独立系统终端。
