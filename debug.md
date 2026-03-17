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

### 方法：独立 debug 包装脚本 + VS Code attach

#### 文件结构

```
scripts/
├── piper_mujoco_ctrl.py          ← 主脚本，不做任何修改
└── piper_mujoco_ctrl_debug.py    ← 调试用包装脚本
```

#### piper_mujoco_ctrl_debug.py

```python
#!/usr/bin/env python3
import debugpy
debugpy.listen(5678)
print("Waiting for debugger to attach on port 5678...")
debugpy.wait_for_client()

from piper_mujoco_ctrl import main
main()
```

#### VS Code launch.json

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Attach to piper_mujoco_ctrl",
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

#### 使用步骤

1. 安装 debugpy：`pip install debugpy`
2. 构建：`colcon build --symlink-install --packages-select piper_mujoco`
3. 启动调试版节点：
```bash
ros2 run piper_mujoco piper_mujoco_ctrl_debug.py
# 终端输出：Waiting for debugger to attach on port 5678...
```
4. 在 `piper_mujoco_ctrl.py` 中设置断点，VS Code 按 `F5` 附加。

#### 通过 launch 文件启动时调试

修改 launch 文件中对应节点的 executable：

```python
mujoco_viewer = Node(
    package='piper_mujoco',
    executable='piper_mujoco_ctrl_debug.py',  # ← 改为 debug 版本
    ...
)
```

### 注意
- 实时控制循环（100Hz）命中断点会暂停物理仿真
- 对实时性要求高的逻辑，建议用日志代替断点：
```python
self.get_logger().info(f"joint_targets: {self.joint_targets}")
```

---

## 三、方法对比

| | Launch 文件 `breakpoint()` | 节点 `debugpy` attach |
|---|---|---|
| **侵入性** | 需在源码插入断点 | 主脚本零修改 |
| **IDE 支持** | 仅命令行 pdb | VS Code 图形化断点、变量面板 |
| **适用时机** | 进程启动阶段（一次性） | 进程运行阶段（长期） |
| **调试完清理** | 移除 `breakpoint()` | 直接用主脚本运行，无需清理 |
