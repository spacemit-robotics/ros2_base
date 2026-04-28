# ESOS 底盘控制

## 项目简介

本组件是一个基于 ROS 2 的底盘控制节点，用于通过 RPMsg 与小核差速底盘驱动侧进行通信，实现移动机器人底盘的速度控制与里程计数据发布。

它主要解决以下问题：

- 将 ROS 2 标准速度指令 `/cmd_vel` 转换为底盘可执行的运动控制命令；
- 从底盘侧获取位姿与速度信息，并发布标准 `/odom` 里程计消息；
- 按需发布 `odom -> base_footprint` 的 TF 变换，便于上层导航、定位、建图等模块直接接入；
- 提供轮径、轮距、电机修正因子、RPMsg 设备参数等可配置能力，适配不同小车硬件参数。

该组件适合作为 ROS 2 移动机器人底盘接入层，方便与导航、遥控、语音控制、SLAM 等系统进行集成。

## 功能特性

### 已支持功能

- 订阅 `geometry_msgs/msg/Twist` 类型的 `/cmd_vel` 速度控制指令；
- 发布 `nav_msgs/msg/Odometry` 类型的 `/odom` 里程计消息；
- 发布 `odom` 到 `base_footprint` 的 TF 变换；
- 支持差速两轮底盘模型；
- 支持通过参数配置：
  - 底盘轮径 `wheel_diameter`
  - 轮距 `wheel_base`
  - 左右电机修正系数 `motor1_factor` / `motor2_factor`
  - 里程计/TF 的 frame 与 topic 名称
  - RPMsg 控制与数据通道参数
- 支持超时保护：当一段时间未收到 `/cmd_vel` 指令时，自动发送零速度，降低失控风险；
- 依赖底层 `components/control/base` 中的 `chassis` 能力与 `drv_rpmsg_esos` 驱动实现硬件通信。

### 当前不支持或需注意

- 当前实现面向差速两轮底盘，不适用于全向轮、麦克纳姆轮或阿克曼底盘；
- 轮径/轮距/电机系数需要结合实际设备自行调参；
- 默认依赖底层底盘驱动和 RPMsg 通道正常工作，若底层未正确构建或部署，节点无法启动；

## 快速开始

下面给出从零到可运行的最短路径，适合首次接入和快速验证。

### 环境准备

在开始前，请确认以下条件已经满足：

1. 已完成 ROS 2 环境安装与配置；
2. 已正确构建并安装底层底盘库 `components/control/base`；
3. 系统中可用的 RPMsg 设备节点或服务已经准备就绪；
4. 目标硬件为小核控制的差速底盘。

本组件依赖如下 ROS 2 软件包：

- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`

若在工作区源码构建，请确保 `components/control/base` 相关产物已安装到 `CMAKE_PREFIX_PATH` 可搜索路径中，否则会在编译阶段报错：

```text
chassis.h or libchassis not found. Build/install components/control/base first
```

### 构建编译

进入 ROS 2 工作区后，执行构建：

```bash
colcon build --packages-select base
```

如果当前工作区尚未加载环境，请在构建完成后执行：

```bash
source install/setup.bash
```

> 说明：
>
> - 包名为 `base`；
> - 可执行文件为 `esos_base_control_node`；
> - 启动文件为 `esos_base_control.launch.py`。

### 运行示例

#### 1. 直接运行节点

```bash
ros2 run base esos_base_control_node
```

#### 2. 使用 launch 文件启动

```bash
ros2 launch base esos_base_control.launch.py
```

#### 3. 带参数启动

```bash
ros2 launch base esos_base_control.launch.py \
    wheel_diameter:=0.067 \
    wheel_base:=0.28
```

#### 4. 发送测试速度指令

节点启动后，可以通过以下命令发送一条测试速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

#### 5. 查看里程计输出

```bash
ros2 topic echo /odom
```

如已开启 TF 发布，也可以结合 `rviz2`、`tf2_tools` 等工具检查坐标变换是否正常。

## 详细使用


当前可先关注以下关键参数：

| 参数名 | 默认值 | 说明 |
|---|---:|---|
| `send_hz` | `20.0` | 速度指令发送频率（Hz） |
| `odom_hz` | `50.0` | 里程计发布频率（Hz） |
| `cmd_vel_timeout` | `0.4` | 控制指令超时时间（秒） |
| `publish_tf` | `true` | 是否发布 TF |
| `odom_topic` | `odom` | 里程计话题名 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_footprint` | 机器人基座坐标系 |
| `wheel_diameter` | `0.067` | 轮径，单位米 |
| `wheel_base` | `0.28` | 轮距，单位米 |
| `motor1_factor` | `1.0` | 左轮速度修正因子 |
| `motor2_factor` | `1.0` | 右轮速度修正因子 |
| `rpmsg_ctrl_dev` | `""` | RPMsg 控制设备，空表示使用默认值 |
| `rpmsg_data_dev` | `""` | RPMsg 数据设备，空表示使用默认值 |
| `rpmsg_service_name` | `""` | RPMsg 服务名，空表示使用默认值 |
| `rpmsg_local_addr` | `0` | RPMsg 本地地址，0 表示默认值 |
| `rpmsg_remote_addr` | `0` | RPMsg 远端地址，0 表示默认值 |

## 常见问题

### 1. 编译时报找不到 `chassis.h` 或 `libchassis`

原因通常是底层 `components/control/base` 尚未构建安装，或安装路径未加入 `CMAKE_PREFIX_PATH`。

建议排查：

- 确认底层库已成功编译并安装；
- 确认当前 shell 已 source 对应环境脚本；
- 检查 `CMAKE_PREFIX_PATH` 是否包含底层库安装路径。

### 2. 节点可以启动，但底盘不动

建议检查：

- `/cmd_vel` 是否确实有消息输入；
- `cmd_vel_timeout` 是否过小，导致刚发送后又被超时清零；
- RPMsg 设备节点是否存在并可访问；
- ESOS 小核侧固件和驱动是否正常工作；
- 左右轮修正因子是否设置异常。

### 3. 有 `/odom`，但数据异常或漂移明显

建议重点核对：

- `wheel_diameter` 是否与实物一致；
- `wheel_base` 是否测量准确；
- 左右电机修正因子是否需要标定；
- 地面摩擦、打滑、编码器精度等是否影响结果。

### 4. TF 冲突或坐标系不一致

如果系统中已有其他模块发布 `odom -> base_footprint` 或相近 TF，可能产生冲突。

建议：

- 明确整机 TF 树唯一发布源；
- 必要时将 `publish_tf` 设为 `false`，由上层统一发布；
- 检查 `odom_frame`、`base_frame` 是否与整机约定一致。

### 5. 速度控制方向不正确或转向相反

这通常与底盘接线、电机方向定义、左右轮映射关系或底层驱动配置有关。

建议先用低速测试：

- 仅发送前进速度，确认小车是否直行；
- 仅发送角速度，确认旋转方向是否符合预期；
- 如有偏航或跑偏，再结合 `motor1_factor` 与 `motor2_factor` 做微调。


## 贡献方式

欢迎通过 Issue、文档修订、代码补丁、测试反馈等方式参与贡献。

建议贡献流程：

1. 阅读本仓库相关开发规范与目录说明；
2. 在修改前确认影响范围，避免破坏已有接口行为；
3. 提交前完成必要的编译与基础功能验证；
4. 对新增参数、接口或行为变化补充相应文档说明。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。