# 项目交接文档

> 本文档汇总了当前项目状态、已完成工作、关键技术决策以及剩余任务，方便切换到其他模型/开发者继续推进。

---

## 1. 项目概述

**项目名称**: simulation-based-adas-testing  
**目标**: 基于 Gazebo Harmonic 的 ADAS/自动驾驶仿真测试与验证框架  
**仓库路径**: `/Users/houjiakuan/Intelligent Driving Testing`  
**Python 版本**: 3.12（必须使用，因为 Gazebo Harmonic 的 Python 绑定只兼容 3.12）  
**Gazebo 版本**: Gazebo Harmonic (gz-harmonic)，通过 Homebrew 安装  
**操作系统**: macOS ARM64

---

## 2. 环境配置

### 2.1 虚拟环境

```bash
cd "/Users/houjiakuan/Intelligent Driving Testing"
python3.12 -m venv .venv312
source .venv312/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
```

### 2.2 Gazebo Python 绑定

Homebrew 的 `gz-harmonic` 会把 Python 绑定装到：

```
/opt/homebrew/lib/python3.12/site-packages
```

项目中已通过 `.pth` 文件让 venv 自动找到它们。如果导入 `gz.msgs10` 失败，检查：

```bash
ls .venv312/lib/python3.12/site-packages/ | grep homebrew
# 应有一个指向 /opt/homebrew/lib/python3.12/site-packages 的 .pth 文件
```

### 2.3 macOS 特殊说明

`gz sim` 在 macOS 上必须**分别启动 server 和 GUI**：

```bash
# 终端 1：启动仿真服务器
GZ_SIM_RESOURCE_PATH=/Users/houjiakuan/Intelligent\ Driving\ Testing/scenarios/models:$GZ_SIM_RESOURCE_PATH \
  gz sim -s scenarios/worlds/simple_road.sdf

# 终端 2：启动 GUI（可选）
gz sim -g
```

---

## 3. 已完成工作

### 3.1 项目骨架与依赖

- 从 CARLA 迁移到 Gazebo Harmonic
- `requirements.txt` 已清理 CARLA 依赖
- `pyproject.toml` 配置 Python 3.12、src layout、pytest
- `.venv312` 已创建并安装依赖

### 3.2 仿真后端 (`src/simulation/`)

| 文件 | 作用 |
|------|------|
| `src/simulation/simulator.py` | 抽象基类 `Simulator` |
| `src/simulation/gazebo_client.py` | `GazeboSimulator` 实现：连接、主题列表、发布、订阅、服务调用 |

关键 API：

```python
from simulation import GazeboSimulator

sim = GazeboSimulator()
sim.connect()

# 发布消息
sim.publish("/model/ego_vehicle/cmd_vel", "Twist", twist_msg)

# 订阅消息
sim.subscribe("/model/ego_vehicle/odometry", "Odometry", callback)

# 服务调用（例如解除暂停）
import gz.msgs10.world_control_pb2 as wc_pb
req = wc_pb.WorldControl()
req.pause = False
sim.request("/world/simple_road/control", req, "WorldControl", "Boolean", timeout_ms=10000)
```

**重要经验**: Gazebo headless server 启动后默认是**暂停状态**，必须通过 `/world/<world>/control` 服务发送 `WorldControl{pause: false}` 才能开始仿真。

### 3.3 车辆控制 (`src/control/`)

| 文件 | 作用 |
|------|------|
| `src/control/vehicle_controller.py` | 抽象接口 `VehicleController` |
| `src/control/gazebo_ackermann_controller.py` | Gazebo 阿克曼转向实现 `GazeboAckermannController` |
| `src/control/__init__.py` | 导出两个类 |

控制约定：

- 命令主题: `/model/{model_name}/cmd_vel`
- 消息类型: `gz.msgs.Twist`
- `linear.x` = 纵向速度 (m/s)
- `angular.z` = 转向角 (rad)
- 里程计主题: `/model/{model_name}/odometry`

用法示例：

```python
from control import GazeboAckermannController

with GazeboAckermannController(model_name="ego_vehicle") as controller:
    controller.send_command(speed=0.5, steering_angle=0.0)
```

### 3.4 场景与模型 (`scenarios/`)

| 文件 | 作用 |
|------|------|
| `scenarios/worlds/simple_road.sdf` | 主测试世界：带车道线的直路 |
| `scenarios/models/ego_vehicle/model.sdf` | 自定义阿克曼 ego 车辆 |
| `scenarios/models/ego_vehicle/model.config` | 模型元数据 |
| `scenarios/worlds/test_ackermann.sdf` | 调试用世界，复制了 Gazebo 官方 example_vehicle |
| `scenarios/models/example_vehicle/` | Gazebo 官方 `vehicle_blue` 的精确副本，用于对照 |

**关键修复**: `ego_vehicle` 之前因为放置高度错误导致车轮悬空无法移动。当前配置：

- `model.sdf` 中模型自身 `<pose>0 0 0 0 0 0</pose>`（原点在地面上）
- `simple_road.sdf` 中 `<include>` 引入高度 `<pose>0 0 0.03 0 0 0</pose>`（车轮落在路面）

### 3.5 手动验证脚本

| 文件 | 作用 |
|------|------|
| `scripts/control_vehicle.py` | 连接 ego_vehicle、解除暂停、前进、左转、停止并打印位姿 |
| `scripts/verify_gazebo_communication.py` | 验证 Python 与 Gazebo 的基础通信 |

运行方式：

```bash
# 终端 1
gz sim -s scenarios/worlds/simple_road.sdf

# 终端 2
source .venv312/bin/activate
python scripts/control_vehicle.py
```

### 3.6 测试

| 文件 | 覆盖内容 |
|------|---------|
| `tests/test_package_layout.py` | 包可导入性 |
| `tests/test_simulation_interface.py` | Simulator 抽象、GazeboSimulator 生命周期、发布/订阅/服务调用、消息解析 |
| `tests/test_vehicle_control.py` | VehicleController 抽象、GazeboAckermannController 主题/命令/stop/订阅/上下文管理 |

运行：

```bash
python -m pytest tests/ -v
# 当前结果：37 passed
```

---

## 4. 关键技术决策与踩坑记录

### 4.1 消息类型解析

`gz.msgs10` 中 protobuf 模块命名不规则：

- `Twist` → `twist_pb2`（简单小写）
- `StringMsg` → `stringmsg_pb2`（不规则）
- `WorldControl` → `world_control_pb2`（snake_case）
- `WorldStatistics` → `world_stats_pb2`（缩写）

`src/simulation/gazebo_client.py` 中的 `_resolve_msg_type()` 已实现：
1. 先尝试直接小写
2. 再尝试 camelCase → snake_case
3. 最后查硬编码例外表

如仍有未知类型，可直接传完整路径：`"gz.msgs10.xxx_pb2.Xxx"`。

### 4.2 服务调用返回值

`gz.transport13.Node.request()` 对 `Boolean` 响应返回的是 Python 原生 `bool`，不是 `Boolean` protobuf。判断成功时应写：

```python
response is True or (hasattr(response, "data") and response.data)
```

### 4.3 include pose 与模型 pose 的关系

在 Gazebo SDF 中，`<include><pose>` 会**覆盖**模型文件内部的 `<model><pose>`，而不是叠加。因此 world 里的 include pose 就是模型在世界中的实际原点。

### 4.4 Ackermann 插件参数

`ego_vehicle` 使用 `gz-sim-ackermann-steering-system` 插件，配置见 `scenarios/models/ego_vehicle/model.sdf:328`。关键参数：

- `wheel_base`: 0.7
- `wheel_separation`: 0.7
- `wheel_radius`: 0.15
- `kingpin_width`: 0.5
- `steering_limit`: 0.6

---

## 5. 剩余任务

按建议优先级排列：

### 高优先级

1. **感知模块 (`src/perception/`)**
   - 为 `ego_vehicle` 添加摄像头/激光雷达传感器
   - 实现图像订阅接口
   - 定义归一化感知输出数据结构

2. **测试/编排模块 (`src/testing/`)**
   - Scenario Runner：自动启动 Gazebo、加载世界、执行动作序列、断言结果
   - 测量框架：位置、速度、碰撞检测
   - 报告生成

3. **规划模块 (`src/planning/`)**
   - 路径/行为规划接口
   - 最简实现：车道保持控制器

### 中优先级

4. **更多场景**
   - 跟车、换道、行人横穿、交通标志识别等
   - 结构化场景定义格式（YAML + SDF）

5. **车辆与传感器完善**
   - 添加 IMU、激光雷达
   - 优化车辆动力学参数
   - 添加其他交通参与者

### 低优先级

6. **工程化**
   - 更新 `config/default.yaml` 的 `world` 字段
   - CI/CD 配置
   - 日志与报告目录自动化

---

## 6. 快速验证清单

接手后建议先跑一遍，确认环境没问题：

```bash
cd "/Users/houjiakuan/Intelligent Driving Testing"
source .venv312/bin/activate

# 1. 跑测试
python -m pytest tests/ -v

# 2. 终端 1 启动 server
gz sim -s scenarios/worlds/simple_road.sdf

# 3. 终端 2 运行手动控制
python scripts/control_vehicle.py
# 期望看到 Final pose: {'x': ~0.87, 'y': ~0.17, 'z': 0.0}
```

---

## 7. 关键文件速查

```text
/Users/houjiakuan/Intelligent Driving Testing/
├── src/
│   ├── simulation/
│   │   ├── simulator.py
│   │   └── gazebo_client.py
│   ├── control/
│   │   ├── vehicle_controller.py
│   │   └── gazebo_ackermann_controller.py
│   ├── perception/          # 空包，待实现
│   ├── planning/            # 空包，待实现
│   └── testing/             # 空包，待实现
├── scenarios/
│   ├── worlds/simple_road.sdf
│   └── models/ego_vehicle/
├── scripts/control_vehicle.py
├── tests/
│   ├── test_simulation_interface.py
│   └── test_vehicle_control.py
├── config/default.yaml
├── pyproject.toml
└── requirements.txt
```
