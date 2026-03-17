# ROS2 断点调试指南

## 概述

| 调试对象 | 特点 | 推荐方法 |
|---|---|---|
| **Launch 文件** | 启动时一次性执行，不是长期进程 | `breakpoint()` + pdb |
| **普通节点脚本** | 长期运行的进程 | `debugpy` + VS Code attach |

---

## 一、调试 Launch 文件

### 适用场景
- 检查 xacro 解析结果
- 验证参数配置是否正确
- 排查节点启动参数问题

### 方法：`breakpoint()` + pdb

在 launch 文件中想暂停的位置插入 `breakpoint()`：

```python
def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')

    breakpoint()  # ← 暂停，检查变量

    doc = xacro.parse(open(xacro_file))

    breakpoint()  # ← 暂停，检查 xacro 解析结果
    ...
```

正常启动即可进入调试：

```bash
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py
```

### pdb 常用命令

| 命令 | 作用 |
|---|---|
| `n` | 执行下一行 |
| `s` | 进入函数内部 |
| `c` | 继续执行到下一个断点 |
| `p <变量名>` | 打印变量值 |
| `l` | 显示当前位置附近代码 |
| `q` | 退出调试 |

### 注意
- 调试完毕后记得移除 `breakpoint()`，否则每次启动都会暂停
- 节点启动顺序问题用以下命令观察：

```bash
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py --debug 2>&1 | grep -E "spawn|exit|event"
```

---

## 二、调试普通节点脚本

### 方法：`DEBUG_NODE` 环境变量 + VS Code attach

launch 文件已内置调试支持，通过 `DEBUG_NODE` 环境变量指定要调试的节点，**无需创建任何 wrapper 脚本，无需修改 launch 文件**。

#### 原理

launch 文件中每个 Python 节点都加了 `prefix`：

```python
def debug_prefix(executable_name):
    if os.environ.get('DEBUG_NODE') == executable_name:
        return 'python3 -m debugpy --listen 5678 --wait-for-client'
    return ''

mujoco_viewer = Node(
    executable='piper_mujoco_ctrl.py',
    prefix=debug_prefix('piper_mujoco_ctrl.py'),  # ← 匹配时自动注入 debugpy
    ...
)
```

#### 使用步骤

1. 安装 debugpy：`pip install debugpy`

2. 设置 `DEBUG_NODE` 启动：
```bash
# 调试 piper_mujoco_ctrl.py
DEBUG_NODE=piper_mujoco_ctrl.py ros2 launch piper_mujoco piper_mujoco_ros2.launch.py

# 调试 admittance_trajectory_bridge.py
DEBUG_NODE=admittance_trajectory_bridge.py ros2 launch piper_mujoco piper_mujoco_ros2.launch.py

# 正常运行（不调试）
ros2 launch piper_mujoco piper_mujoco_ros2.launch.py
```

3. 终端出现等待提示后，在 VS Code 中设置断点，按 `F5` 附加调试器。

#### VS Code launch.json

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Attach to ROS2 Node",
            "type": "debugpy",
            "request": "attach",
            "connect": { "host": "localhost", "port": 5678 },
            "pathMappings": [{
                "localRoot": "${workspaceFolder}",
                "remoteRoot": "${workspaceFolder}"
            }]
        }
    ]
}
```

#### 支持调试的节点

| `DEBUG_NODE` 值 | 对应节点 |
|---|---|
| `piper_mujoco_ctrl.py` | MuJoCo 仿真节点 |
| `admittance_trajectory_bridge.py` | 轨迹桥接节点 |
| `world_force_bridge.py` | 世界坐标力桥接节点 |

### 注意
- 实时控制循环（100Hz）命中断点会暂停物理仿真
- 对实时性要求高的逻辑，建议用日志代替断点：
```python
self.get_logger().info(f"joint_targets: {self.joint_targets}")
```

---

## 三、方法对比

| | Launch 文件 `breakpoint()` | 节点 `DEBUG_NODE` + debugpy |
|---|---|---|
| **侵入性** | 需在源码插入断点 | 源码零修改 |
| **IDE 支持** | 仅命令行 pdb | VS Code 图形化断点、变量面板 |
| **适用时机** | 进程启动阶段（一次性） | 进程运行阶段（长期） |
| **切换方式** | 移除 `breakpoint()` | 不设 `DEBUG_NODE` 即为正常运行 |
| **多节点支持** | 每处手动插入 | 一个 launch 文件统一支持所有节点 |
