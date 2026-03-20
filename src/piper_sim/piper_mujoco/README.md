# piper_mujoco — MuJoCo Viewer 节点

Piper 机械臂的 MuJoCo 可视化/仿真 viewer 节点包。

导纳控制逻辑（ros2_control、admittance controller、控制器配置、集成测试）已迁移至
[compliance_control](../../compliance_control/README.md) 包。

## 包内容

```
piper_mujoco/
└── scripts/
    ├── piper_mujoco_ctrl.py            # 有夹爪版 MuJoCo viewer（ros2_control 集成）
    ├── piper_mujoco_ctrl_debug.py      # 调试版 viewer
    └── piper_no_gripper_mujoco_ctrl.py # 无夹爪版 MuJoCo viewer
```

## 快速启动

### 仅运行 viewer（独立模式，无 ros2_control）

```bash
cd ~/github_gk/piper_ros
source install/setup.bash

# 有夹爪
ros2 run piper_mujoco piper_mujoco_ctrl.py

# 无夹爪
ros2 run piper_mujoco piper_no_gripper_mujoco_ctrl.py
```

### 完整导纳控制仿真（ros2_control + admittance）

由 `compliance_control` 包统一启动，viewer 作为其中一个节点运行：

```bash
# mock 硬件模式（无 MuJoCo 物理窗口）
ros2 launch compliance_control compliance_control.launch.py use_mock_hardware:=true

# 完整 MuJoCo 物理仿真
ros2 launch compliance_control compliance_control.launch.py use_mock_hardware:=false
```

详细文档见 [compliance_control README](../../compliance_control/README.md)。

## 通过 rviz_gui 控制（独立模式）

```bash
# 有夹爪
ros2 launch piper_description display_urdf.launch.py

# 无夹爪
ros2 launch piper_description display_no_gripper_urdf.launch.py
```

注：**若不能控制，请在运行 rviz_gui 后再运行 viewer。**

## 控制参数

MuJoCo 模型中的关节参数可在以下文件调整：

- 有夹爪：`piper_description/mujoco_model/piper_description.xml`
- 无夹爪：`piper_description/mujoco_model/piper_no_gripper_description.xml`

| 参数 | 说明 |
|---|---|
| `damping="100"` | 关节阻尼 |
| `kp="10000"` | 关节控制增益 |
| `forcerange="-100 100"` | 关节控制力矩范围 |
