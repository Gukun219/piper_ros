# piper_mujoco — Admittance Controller 设计与测试指南

基于 ros2_control + AdmittanceController 的 Piper 机械臂导纳控制仿真包。

## 目录

1. [整体架构](#1-整体架构)
2. [硬件接口层](#2-硬件接口层)
3. [控制器栈](#3-控制器栈)
4. [关键节点说明](#4-关键节点说明)
5. [力注入机制](#5-力注入机制)
6. [Admittance 参数](#6-admittance-参数)
7. [构建与启动](#7-构建与启动)
8. [集成测试方案](#8-集成测试方案)
9. [手动验证](#9-手动验证)
10. [常见问题排查](#10-常见问题排查)

---

## 1. 整体架构

```
用户 / MoveIt
      │  FollowJointTrajectory action
      ▼
admittance_trajectory_bridge          [ROS2 节点, piper_mujoco]
      │  /admittance_controller/joint_references  (JointTrajectoryPoint topic)
      ▼
admittance_controller                 [ros2_control AdmittanceController]
      │  reads: ft_sensor state interfaces  (6-DOF wrench)
      │  writes: joint position + velocity command interfaces
      ▼
hardware interface
  ├─ mock_components/GenericSystem     [use_mock_hardware=true  — CI / 开发]
  └─ mujoco_ros2_control/MuJoCoSystem  [use_mock_hardware=false — 完整物理仿真]
      │
      ▼
/joint_states               (JointStateBroadcaster)
/ft_sensor_broadcaster/wrench  (ForceTorqueSensorBroadcaster)
```

**为何用 topic bridge 而非 JTC→Admittance 链式（chainable）模式？**

ROS2 Humble 的 `admittance_controller` 尚不完整支持 chainable 模式的 activate/deactivate
生命周期；topic 方式在 Humble 上最稳定，且不影响 Iron/Jazzy 升级（bridge 可直接替换为
chainable JTC）。

---

## 2. 硬件接口层

文件：`src/piper_description/urdf/piper_description_mujoco.xacro`

| xacro 参数 | 默认值 | 说明 |
|---|---|---|
| `use_mock_hardware` | `true` | `true` → `mock_components/GenericSystem`；`false` → `mujoco_ros2_control/MuJoCoSystem` |
| `lock_joints_4_6` | `false` | `true` → 从 ros2_control 块中移除 joint4/joint6，避免 4-DOF Jacobian 奇异 |

### 接口清单

| 接口 | 类型 | 说明 |
|---|---|---|
| `joint{1,2,3,5}/position` | command + state | 臂关节位置，范围见 xacro |
| `joint{1,2,3,5}/velocity` | command + state | 臂关节速度（AdmittanceController 必须同时写入） |
| `joint{7,8}/position` | command + state | 夹爪关节（仅 position，无 admittance） |
| `ft_sensor/force.{x,y,z}` | state (+ command in mock) | FT 传感器力 [N] |
| `ft_sensor/torque.{x,y,z}` | state (+ command in mock) | FT 传感器力矩 [N·m] |

`mock_sensor_commands=true` 时，mock 硬件额外暴露 FT 的 **command 接口**，使
`ForwardCommandController` 可向传感器写入伪力值——这是力注入机制的基础。

FT sensor link 固定于 link6 末端上方 5 cm（`ft_sensor_joint` fixed joint）。

---

## 3. 控制器栈

文件：`src/piper_sim/piper_mujoco/config/ros2_controllers.yaml`

### 控制器列表

| 控制器名 | 类型 | 状态 | 用途 |
|---|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | 始终 spawn | 发布 `/joint_states` |
| `force_torque_sensor_broadcaster` | ForceTorqueSensorBroadcaster | 始终 spawn | 发布 `/ft_sensor_broadcaster/wrench` |
| `admittance_controller` | AdmittanceController | 始终 spawn | 核心柔顺控制器 |
| `gripper_controller` | JointTrajectoryController | 始终 spawn | 夹爪独立控制 |
| `arm_controller` | JointTrajectoryController | **不 spawn** | 预留（未来 chainable 升级用） |
| `ft_{fx,fy,fz,tx,ty,tz}_injector` | ForwardCommandController | mock 模式 spawn | 伪力注入（各 1 分量） |

### 启动顺序（OnProcessExit 事件链）

```
spawn joint_state_broadcaster
  └─► spawn force_torque_sensor_broadcaster
        └─► spawn admittance_controller
            spawn gripper_controller
            spawn ft_{fx,fy,fz,tx,ty,tz}_injector  [仅 mock 模式]
```

顺序约束原因：`admittance_controller` 需要 FT sensor 状态接口已激活；
`joint_state_broadcaster` 必须先于 sensor broadcaster 运行以确保 `/joint_states` 就绪。

### controller_manager 参数

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz — 与 MuJoCo 仿真步长匹配
```

---

## 4. 关键节点说明

### 4.1 admittance_trajectory_bridge.py

提供与 MoveIt 兼容的 FollowJointTrajectory action server：
`/arm_controller/follow_joint_trajectory`

**内部逻辑：**

1. 接收 `FollowJointTrajectory` goal，将轨迹点重排为标准顺序 `[joint1, joint2, joint3, joint5]`
2. 在当前位置前插入 `t=0` 起始点，构成完整插值序列
3. 按 100 Hz 定时器线性插值，发布 `JointTrajectoryPoint` 到
   `/admittance_controller/joint_references`
4. 以 `goal_tolerance=0.02 rad` 判断目标到达；超出 `2×duration + 2s` 则 abort
5. 目标完成后将终止位置保存为**固定保持位置**（不追踪 joint_states）

**关键设计：保持位置与 joint_states 解耦**

admittance_controller 因外力对关节位置产生偏移。若 bridge 将偏移后的 joint_states
作为新参考点持续发布，将形成正反馈：偏移 → 参考更新 → 更大偏移。
bridge 仅在轨迹完成时更新保持位置，确保撤力后关节能回到参考点。

### 4.2 world_force_bridge.py

将世界坐标系下的 Wrench 命令（`/world_force_cmd`）经 TF 变换到 `ft_sensor_link`
坐标系，再分发到各 `ft_{fx,fy,fz,tx,ty,tz}_injector/commands`。

适用于需要在固定方向（如重力方向）施加力的场景，无需手动计算坐标变换。
仅在 `use_mock_hardware=true` 时启动。

---

## 5. 力注入机制

### 注入流程

```
/ft_fz_injector/commands  (Float64MultiArray)
         │  ForwardCommandController
         ▼
ft_sensor/force.z  [command interface]
         │  mock_components/GenericSystem  (mirror: command → state)
         ▼
ft_sensor/force.z  [state interface]
         │  ForceTorqueSensorBroadcaster
         ▼
/ft_sensor_broadcaster/wrench
         │  AdmittanceController 读取
         ▼
admittance law:  M·ẍ + D·ẋ + K·x = F_ext
         ▼
joint position offset → hardware command interface
```

### 话题与控制器对应

| 控制器 | 话题 | 接口 | 单位 |
|---|---|---|---|
| `ft_fx_injector` | `/ft_fx_injector/commands` | force.x | N |
| `ft_fy_injector` | `/ft_fy_injector/commands` | force.y | N |
| `ft_fz_injector` | `/ft_fz_injector/commands` | force.z | N |
| `ft_tx_injector` | `/ft_tx_injector/commands` | torque.x | N·m |
| `ft_ty_injector` | `/ft_ty_injector/commands` | torque.y | N·m |
| `ft_tz_injector` | `/ft_tz_injector/commands` | torque.z | N·m |

消息类型：`std_msgs/Float64MultiArray`，数组长度 1。

### 常用注入命令

```bash
# 向 ft_sensor_link Z 方向注入 10 N
ros2 topic pub /ft_fz_injector/commands std_msgs/Float64MultiArray "{data: [10.0]}"

# 取消力
ros2 topic pub /ft_fz_injector/commands std_msgs/Float64MultiArray "{data: [0.0]}"

# 世界坐标系施力（需 world_force_bridge 节点）
ros2 topic pub /world_force_cmd geometry_msgs/Wrench \
  "{force: {x: 0, y: 0, z: 20}, torque: {x: 0, y: 0, z: 0}}"
```

### 正弦力扫频（传感器坐标系）

```python
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('force_injector')
pub = node.create_publisher(Float64MultiArray, '/ft_fz_injector/commands', 10)
t0 = time.time()
while rclpy.ok():
    msg = Float64MultiArray()
    msg.data = [20.0 * math.sin(2 * math.pi * 0.5 * (time.time() - t0))]
    pub.publish(msg)
    time.sleep(0.01)
```

### 正弦力扫频（世界坐标系，经 TF 变换）

```python
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Wrench

rclpy.init()
node = Node('force_pub')
pub = node.create_publisher(Wrench, '/world_force_cmd', 10)
t0 = time.time()
amplitude = 20.0   # N
freq      = 0.1    # Hz
while rclpy.ok():
    F = amplitude * math.sin(2 * math.pi * freq * (time.time() - t0))
    msg = Wrench()
    msg.force.z = F   # 世界 Z = 竖直向上
    pub.publish(msg)
    time.sleep(0.01)
```

---

## 6. Admittance 参数

```yaml
admittance:
  selected_axes:  [true, true, true, false, false, false]  # 仅启用 XYZ 平动
  mass:           [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]          # 虚拟惯量 [kg]
  damping_ratio:  [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]          # 临界阻尼 = 1.0
  stiffness:      [200.0, 200.0, 200.0, 50.0, 50.0, 50.0]  # 虚拟刚度 [N/m]
```

**静态平衡偏移估算：**

```
Δx_cartesian ≈ F / K = 10 N / 200 N/m = 0.05 m
Δq_joint = J⁺(q) · Δx_cartesian   (依赖当前构型 Jacobian)
```

home 姿态（全零）下实测关节偏移通常在 0.01 ~ 0.1 rad 量级。

**旋转轴禁用原因：** `lock_joints_4_6=true` 时仅有 4 个 DOF（joint1/2/3/5），
启用 3 个平动轴恰好可解；启用旋转轴会导致 IK 欠约束，造成数值不稳定。

**运动学插件：**

```yaml
kinematics:
  plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
  base: base_link
  tip:  ft_sensor_link
```

---

## 7. 构建与启动

### 依赖安装

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-admittance-controller \
  ros-humble-force-torque-sensor-broadcaster \
  ros-humble-kinematics-interface \
  ros-humble-kinematics-interface-kdl \
  ros-humble-forward-command-controller
```

### 构建

```bash
cd ~/github_gk/piper_ros
colcon build --packages-select piper_description piper_mujoco
source install/setup.bash
```

只修改了 Python / YAML 文件时，可跳过 `colcon build` 直接 `source install/setup.bash`。

### 启动

```bash
# mock 硬件模式（默认，CI 友好，无 MuJoCo 窗口）
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py use_mock_hardware:=true

# 完整 MuJoCo 物理仿真（需 mujoco_ros2_control 包）
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py use_mock_hardware:=false
```

---

## 8. 集成测试方案

### 8.1 测试策略

采用 **`launch_testing`** 框架（ROS2 标准集成测试方案）。每个测试文件通过
`generate_test_description()` 自行拉起完整控制栈（mock 硬件，无 MuJoCo 依赖），
内嵌 `unittest.TestCase` 通过 rclpy 发送命令并订阅反馈完成断言。

**选择 mock hardware 的原因：**
- CI 环境无图形界面依赖，可直接运行
- mock 模式下位置命令立即镜像到状态，FT 注入器同样可用
- 能验证 admittance controller 核心逻辑：无外力时精确跟踪，有外力时产生位移

**可选 MuJoCo 可视化：** 设置 `PIPER_VISUAL_TEST=1` 后，测试框架会额外启动
`piper_mujoco_ctrl.py` viewer 节点，在 MuJoCo 窗口中实时渲染机器人运动；
为等待窗口就绪，`ReadyToTest` 会延迟 5 秒触发。CI 环境不设该变量，行为不变。

**已知行为：** launch_testing 发送 SIGINT 结束测试时，
`admittance_trajectory_bridge.py` 通过 `rclpy.ok()` 守护避免双重 shutdown（RCLError）。

### 8.2 测试文件一览

| 文件 | 测试类 | 测试方法 | 超时 |
|---|---|---|---|
| `test/test_trajectory_following.py` | `TestTrajectoryFollowing` | `test_trajectory_goal_success` | 60 s |
| `test/test_admittance_compliance.py` | `TestAdmittanceCompliance` | `test_force_induces_displacement` | 60 s |
| | | `test_displacement_recovers_on_force_removal` | 60 s |

---

### 8.3 Test 1 — 轨迹跟踪精度

**文件：** `test/test_trajectory_following.py`

**目标：** 验证完整链路（trajectory_bridge → admittance_controller → mock hw）正确跟踪轨迹。

#### 控制栈启动（generate_test_description）

```
robot_state_publisher   (mock hw URDF, lock_joints_4_6=true)
ros2_control_node       (mock hw)
piper_mujoco_ctrl.py    [仅 PIPER_VISUAL_TEST=1]
admittance_trajectory_bridge
spawner: joint_state_broadcaster
  └─► spawner: force_torque_sensor_broadcaster
        └─► spawner: admittance_controller
            spawner: gripper_controller
ReadyToTest()           [PIPER_VISUAL_TEST=1 时延迟 5 s 触发]
```

#### 测试执行流程

```
test_trajectory_goal_success()
  1. 等待 /arm_controller/follow_joint_trajectory action server 就绪   超时 25 s
  2. 发送 FollowJointTrajectory goal:
       joint1 =  0.3 rad
       joint2 =  0.2 rad
       joint3 = -0.1 rad
       joint5 = -0.2 rad
       duration = 3 s（单 waypoint，bridge 内部线性插值）
  3. 等待 action result                                                 超时 15 s
  4. 断言 result.error_code == 0  (SUCCESSFUL)
  5. 等待 0.5 s 让关节稳定，读取 /joint_states
  6. 断言每关节: |actual - target| < 0.05 rad
```

#### 断言阈值说明

mock 模式下位置命令立即反映到状态，预期误差接近 0（< 0.01 rad）；
0.05 rad 为保守裕量，兼顾 admittance controller 内部积分延迟。

---

### 8.4 Test 2 — 导纳柔顺性

**文件：** `test/test_admittance_compliance.py`

**目标：** 验证施加外力后 admittance controller 产生关节位移（柔顺效果），
撤力后关节向初始位置回归。

#### 控制栈启动（generate_test_description）

在 Test 1 基础上额外增加：

```
piper_mujoco_ctrl.py    [仅 PIPER_VISUAL_TEST=1]
  └─► spawner: ft_fx_injector
      spawner: ft_fy_injector
      spawner: ft_fz_injector
      spawner: ft_tx_injector
      spawner: ft_ty_injector
      spawner: ft_tz_injector
ReadyToTest()           [PIPER_VISUAL_TEST=1 时延迟 5 s 触发]
```

#### 测试执行流程

```
test_force_induces_displacement()
  1. 等待 /joint_states 出现                    超时 25 s
  2. 系统稳定等待 2 s
  3. 记录初始位置 pos_initial
  4. 以 10 Hz 持续发布 /ft_fz_injector/commands [10.0]    持续 2 s
  5. 记录稳定后位置 pos_final
  6. 断言: max_j |pos_final[j] - pos_initial[j]| > 0.005 rad  (有柔顺响应)
  7. 断言: max_j |pos_final[j] - pos_initial[j]| < 2.0 rad
          （mock hw 无关节惯性，积分量大于真实硬件；2.0 rad ≈ 关节物理限位，
           超出则表示控制器发散）

test_displacement_recovers_on_force_removal()
  1. 等待 /joint_states 出现，稳定 2 s，记录初始位置 pos_initial
  2. 以 10 Hz 持续发布 /ft_fz_injector/commands [10.0]    持续 2 s
  3. 记录受力偏移位置 pos_final
  4. 发布 /ft_fz_injector/commands [0.0]，等待 2 s
  5. 记录恢复位置 pos_recover
  6. 断言: max|pos_recover - pos_initial| < max|pos_final - pos_initial|
          （撤力后向初始位置回归）
```

#### Mock Hardware 阈值说明

mock_components/GenericSystem 直接将位置命令镜像到状态，没有关节惯性和阻尼。
admittance controller 在相同外力下积分出的位移比真实/MuJoCo 硬件大，
因此位移上限设为 2.0 rad（关节物理限位附近），而非按真实静态平衡推算的 0.1 rad。

若将测试移植到完整 MuJoCo 物理仿真，可将上限收紧至 0.3 rad 以增强敏感度。

#### 理论预期（参考，不作断言）

```
静态 Cartesian 偏移 ≈ F/K = 10 N / 200 N/m = 0.05 m
关节空间偏移 Δq = J⁺(q) · Δx_cartesian   (依赖 home 姿态 Jacobian)
```

---

### 8.5 运行测试

#### 方式一：`launch_test` CLI（推荐用于开发迭代，无需完整 colcon）

```bash
cd ~/github_gk/piper_ros
source install/setup.bash

# 轨迹跟踪精度测试
launch_test src/piper_sim/piper_mujoco/test/test_trajectory_following.py

# 导纳柔顺性测试
launch_test src/piper_sim/piper_mujoco/test/test_admittance_compliance.py

# 导出 JUnit XML 报告
launch_test src/piper_sim/piper_mujoco/test/test_trajectory_following.py \
  --junit-xml /tmp/traj_result.xml
```

#### 带 MuJoCo 可视化窗口运行

设置 `PIPER_VISUAL_TEST=1` 后，测试启动时额外弹出 MuJoCo 渲染窗口，
可实时观察机器人运动（测试逻辑与结果不变）：

```bash
# 轨迹测试：可观察到机器人从零位运动到目标姿态
PIPER_VISUAL_TEST=1 launch_test \
  src/piper_sim/piper_mujoco/test/test_trajectory_following.py

# 导纳测试：可观察到注入外力后关节偏移，撤力后回归
PIPER_VISUAL_TEST=1 launch_test \
  src/piper_sim/piper_mujoco/test/test_admittance_compliance.py
```

> **注意：** viewer 启动后需等待约 5 秒 MuJoCo 窗口初始化完成，测试才开始执行。
> CI 环境不设该环境变量，行为完全不变。

#### 方式二：`colcon test`（CI / 完整回归）

```bash
cd ~/github_gk/piper_ros

# 构建（add_launch_test 需要安装到 build/ 目录）
colcon build --packages-select piper_description piper_mujoco

# 运行测试，实时打印输出
colcon test --packages-select piper_mujoco \
  --event-handlers console_cohesion+

# 查看结果汇总
colcon test-result --verbose
```

测试报告存储：`build/piper_mujoco/test_results/piper_mujoco/*.xml`

---

### 8.6 构建系统集成

**CMakeLists.txt：**

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_trajectory_following.py  TIMEOUT 60)
  add_launch_test(test/test_admittance_compliance.py TIMEOUT 60)
endif()
```

**package.xml：**

```xml
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>
<test_depend>python3-pytest</test_depend>
```

---

## 9. 手动验证

### 9.1 验证轨迹跟踪

```bash
# 终端 1：启动 mock 仿真
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py use_mock_hardware:=true

# 终端 2：发送轨迹目标
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [joint1, joint2, joint3, joint5],
    points: [{
      positions: [0.3, 0.2, -0.1, -0.2],
      velocities: [0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }]
  }}"

# 终端 3：观察关节位置
ros2 topic echo /joint_states --field name,position
```

### 9.2 验证导纳柔顺性（传感器坐标系）

```bash
# 注入 10 N Z 方向力，观察关节偏移
ros2 topic pub /ft_fz_injector/commands std_msgs/Float64MultiArray "{data: [10.0]}"
ros2 topic echo /joint_states --field name,position

# 撤力，观察关节回归
ros2 topic pub /ft_fz_injector/commands std_msgs/Float64MultiArray "{data: [0.0]}"
```

### 9.3 验证导纳柔顺性（世界坐标系）

```bash
# 需要 world_force_bridge（use_mock_hardware=true 时自动启动）
ros2 topic pub /world_force_cmd geometry_msgs/Wrench \
  "{force: {x: 0.0, y: 0.0, z: 20.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 9.4 运行导纳演示脚本

```bash
ros2 run piper_mujoco admittance_demo.py
```

演示脚本依次执行：
1. 关节空间轨迹运动到预设姿态
2. 注入 Z 方向正弦外力，观察末端导纳响应

### 9.5 监测传感器与控制器状态

```bash
# FT 传感器读数
ros2 topic echo /ft_sensor_broadcaster/wrench

# 控制器状态
ros2 control list_controllers

# 硬件接口状态
ros2 control list_hardware_interfaces
```

---

## 10. 常见问题排查

### 控制器无法 activate

```bash
ros2 control list_controllers       # 查看各控制器状态
ros2 control list_hardware_interfaces  # 查看接口是否已声明
```

常见原因：

| 现象 | 原因 | 解决 |
|---|---|---|
| `admittance_controller` stuck in `inactive` | FT sensor 接口未就绪 | 确认 `force_torque_sensor_broadcaster` 先 spawn |
| `kinematics` 报错 | kdl 插件未安装 | `sudo apt install ros-humble-kinematics-interface-kdl` |
| 接口未找到 | `lock_joints_4_6` 不一致 | xacro 和 controllers yaml 中关节列表必须匹配 |

### 注入力无响应

1. 确认使用 `use_mock_hardware:=true`（MuJoCo 模式的 FT 读数来自物理引擎，injector 无法覆盖）
2. 确认 `ft_fz_injector` 处于 `active` 状态
3. 检查 xacro 中 `mock_sensor_commands: true` 已设置

### 轨迹目标被 abort

1. 检查 bridge 日志是否出现 `Goal timeout`——通常是 admittance controller 未激活
2. 检查参考话题有无数据：
   ```bash
   ros2 topic hz /admittance_controller/joint_references
   ```
3. `goal_tolerance` 默认 0.02 rad，若目标点接近关节限位可能难以到达

### 测试超时

`TIMEOUT 60` 覆盖：控制器 spawn（≈10~20 s）+ 测试内部等待（25 s）+ 执行时间（≈10 s）。
若 CI 机器较慢，在 `CMakeLists.txt` 中适当增大 `TIMEOUT` 值即可。
