# piper_mujoco — ROS 2 Admittance Control Simulation

基于 ros2_control + AdmittanceController 的 Piper 机械臂导纳控制仿真包。

## 1 依赖安装

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-admittance-controller \
  ros-humble-force-torque-sensor-broadcaster \
  ros-humble-kinematics-interface \
  ros-humble-kinematics-interface-kdl
```

## 2 编译

```bash
cd ~/piper_ros
colcon build --packages-select piper_description piper_mujoco
source install/setup.bash
```

若只修改了脚本/配置文件（Python/YAML），可跳过 `colcon build`，直接：

```bash
source install/setup.bash
```

## 3 启动仿真

### 3.1 Mock 硬件模式（默认，不需要 MuJoCo 插件）

```bash
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py
```

### 3.2 MuJoCo 物理引擎模式

```bash
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py use_mock_hardware:=false
```

python3 -c "
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Wrench

rclpy.init()
node = Node('force_pub')
pub = node.create_publisher(Wrench, '/world_force_cmd', 10)

amplitude = 20.0   # N
freq = 0.1         # Hz
t0 = time.time()
print(f'Injecting {amplitude}N sinusoidal force along world Z at {freq}Hz')
while rclpy.ok():
    F = amplitude + amplitude * math.sin(2 * math.pi * freq * (time.time() - t0))  # start at max force
    msg = Wrench()
    msg.force.z = F   # world Z = 竖直方向
    pub.publish(msg)
    time.sleep(0.01)
"


## 4 控制器架构

```
用户 / MoveIt
  │  FollowJointTrajectory action
  ▼
admittance_trajectory_bridge  →  /admittance_controller/joint_references
                                           │
                                  admittance_controller
                                           │ position command
                                           ▼
                               硬件接口（mock / MuJoCo）
```

## 5 外力注入（Mock 硬件模式）

仿真中通过 `forward_command_controller` 向力/力矩传感器写入外力，驱动导纳控制器响应。

外力作用于 `ft_sensor_link` 坐标系（位于末端 link6 上方 5 cm）。

### 5.1 话题与控制器对应关系

| 控制器          | 话题                        | 接口         |
|-----------------|-----------------------------|--------------|
| ft_fx_injector  | /ft_fx_injector/commands    | force.x      |
| ft_fy_injector  | /ft_fy_injector/commands    | force.y      |
| ft_fz_injector  | /ft_fz_injector/commands    | force.z      |
| ft_tx_injector  | /ft_tx_injector/commands    | torque.x     |
| ft_ty_injector  | /ft_ty_injector/commands    | torque.y     |
| ft_tz_injector  | /ft_tz_injector/commands    | torque.z     |

消息类型均为 `std_msgs/Float64MultiArray`，数组长度为 1，单位：力 N，力矩 N·m。

### 5.2 沿 Z 轴施加正弦力（单轴）

```python
python3 -c "
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('force_injector')
pub = node.create_publisher(Float64MultiArray, '/ft_fz_injector/commands', 10)
t0 = time.time()
msg = Float64MultiArray()
print('Injecting sinusoidal force on Z (20N, 0.5Hz)... Ctrl+C to stop')
while rclpy.ok():
    msg.data = [20.0 * math.sin(2*math.pi*0.5*(time.time()-t0))]
    pub.publish(msg)
    time.sleep(0.01)
"
```

### 5.3 沿任意方向施加外力（三轴合成）

通过同时向 ft_fx / ft_fy / ft_fz 三个话题发布，可在 `ft_sensor_link` 坐标系中施加任意方向的合力。

```python
python3 -c "
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()
node = Node('force_injector')
pub_fx = node.create_publisher(Float64MultiArray, '/ft_fx_injector/commands', 10)
pub_fy = node.create_publisher(Float64MultiArray, '/ft_fy_injector/commands', 10)
pub_fz = node.create_publisher(Float64MultiArray, '/ft_fz_injector/commands', 10)

# ── 参数配置 ──────────────────────────────────────────────────
# 力的方向（ft_sensor_link 坐标系下的单位向量）
nx, ny, nz = 0.577, 0.577, 0.577   # 示例：沿 (1,1,1) 方向
amplitude   = 20.0                  # 力幅值 [N]
frequency   = 0.5                   # 正弦频率 [Hz]
# ─────────────────────────────────────────────────────────────

# 归一化
norm = math.sqrt(nx**2 + ny**2 + nz**2)
nx, ny, nz = nx/norm, ny/norm, nz/norm

t0 = time.time()
print(f'Injecting {amplitude}N sinusoidal force along ({nx:.3f},{ny:.3f},{nz:.3f}), {frequency}Hz... Ctrl+C to stop')
while rclpy.ok():
    F = amplitude * math.sin(2 * math.pi * frequency * (time.time() - t0))
    for pub, n in [(pub_fx, nx), (pub_fy, ny), (pub_fz, nz)]:
        msg = Float64MultiArray()
        msg.data = [F * n]
        pub.publish(msg)
    time.sleep(0.01)
"
```

**说明**：力的方向在 `ft_sensor_link` 坐标系下指定。若需在世界坐标系下指定方向，需通过 TF 将 `base_link` → `ft_sensor_link` 的旋转矩阵对力向量进行坐标变换（可使用 `world_force_bridge.py` 节点完成此转换）。

### 5.4 施加恒定外力

```bash
ros2 topic pub --once /ft_fz_injector/commands std_msgs/msg/Float64MultiArray "data: [10.0]"
```

清除外力（归零）：

```bash
ros2 topic pub --once /ft_fx_injector/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
ros2 topic pub --once /ft_fy_injector/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
ros2 topic pub --once /ft_fz_injector/commands std_msgs/msg/Float64MultiArray "data: [0.0]"
```

## 6 监测传感器数据

```bash
ros2 topic echo /ft_sensor_broadcaster/wrench
```

## 7 运行导纳演示脚本

```bash
ros2 run piper_mujoco admittance_demo.py
```

演示脚本会依次执行：
1. 关节空间轨迹运动到预设姿态
2. 注入 Z 方向正弦外力，观察末端导纳响应
