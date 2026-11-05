# 智能驾驶仿真测试平台（Intelligent Driving Testing）

基于 Python、Gazebo Harmonic、PyTest、OpenCV、YOLO 和 Pandas 搭建的 ADAS 仿真测试与验证框架。项目关注可重复执行的测试场景、可量化的安全判定、故障注入和回归报告。

## 验证内容

- 车道保持（LKA）：读取 Gazebo 摄像头画面，用 OpenCV 检测车道线
- 自动紧急制动（AEB）：使用 Gazebo 激光雷达测距
- LKA 与 AEB 联合控制，AEB 指令优先
- 采集车辆真实里程计、行驶距离、速度、横向偏差和碰撞数据
- 用 YAML 描述通过 / 失败条件，输出 CSV 和 JSON 报告
- 摄像头、激光雷达故障注入，可按时间段设置故障
- YOLO 检测的精确率、召回率、F1、误检、漏检和延迟统计
- 与纳入版本管理的指标阈值做回归对比

自动化测试集包含 5 个 Gazebo 场景：直道 LKA、前车 AEB、行人 AEB、ADAS 联合避障停车，以及摄像头掉帧时的降级 LKA。

## 架构

```text
Gazebo 仿真世界与车辆模型
        │
        ├── 摄像头 ── OpenCV / 可选 YOLO ─┐
        ├── 激光雷达 ── 障碍物距离 ───────┤
        └── 里程计 / 碰撞数据 ─────┐      │
                                   │      ▼
                              ADAS 处理流程
                                   │
                              控制指令仲裁
                           AEB > LKA > 定速巡航
                                   │
                                   ▼
                            Ackermann 控制器
                                   │
                                   ▼
场景执行器 ── 指标采集 ── 断言判定 ── 通过 / 失败
                                   │
                              CSV / JSON
                                   │
                               回归对比
```

模块边界和数据流见 [`docs/architecture.md`](docs/architecture.md)。

## 目录结构

```text
.
├── config/                 # 场景配置、回归阈值、CI 模板
├── scenarios/
│   ├── models/             # 本车、障碍物、行人 SDF 模型
│   └── worlds/             # 道路与危险场景世界
├── scripts/                # 命令行入口
├── src/
│   ├── control/            # 车辆控制接口与 Gazebo 控制器
│   ├── perception/         # 摄像头、激光雷达、车道线、YOLO、传感器融合
│   ├── planning/           # LKA、AEB 及联合 ADAS 流程
│   ├── simulation/         # Gazebo 通信与遥测适配
│   └── testing/            # 执行器、断言、故障注入、报告、回归
└── tests/                  # 可重复运行的 PyTest 测试集
```

`data/`、`logs/`、`reports/` 下的生成文件不提交到 Git，`*.pt` 等模型权重也不提交。

## 环境要求

- Python 3.12+
- Gazebo Harmonic
- Apple Silicon 的 macOS，或装有对应 Gazebo Python 绑定的 Linux

macOS 安装：

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

通过 Homebrew 安装的 Gazebo Python 绑定一般在 `/opt/homebrew/lib/python3.12/site-packages`。如果虚拟环境里 `import gz.transport13` 报找不到模块，需要把这个目录加入虚拟环境的搜索路径。

## 安装

```bash
python3.12 -m venv .venv312
source .venv312/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
```

检查环境：

```bash
python --version
gz sim --versions
python -c "import gz.transport13; print('Gazebo Python bindings ready')"
pytest -q
```

## 运行完整验证

批量执行器会为每个场景单独启动和关闭 Gazebo：

```bash
python scripts/run_scenarios.py \
  --config config/scenarios.yaml \
  --output reports/scenario_results.csv \
  --json reports/scenario_results.json
```

加 `--show-gui` 显示 Gazebo 界面，加 `--dry-run` 只校验 YAML 不运行，也可以只跑指定场景：

```bash
python scripts/run_scenarios.py \
  --scenario aeb_obstacle_ahead \
  --scenario aeb_pedestrian_crossing
```

任一场景或断言失败时命令返回状态码 `1`，可以直接接入自动化流程。

## 回归检查

跑完全部场景后执行：

```bash
python scripts/check_regression.py \
  --results reports/scenario_results.json \
  --baseline config/regression_baseline.yaml
```

回归阈值放在 [`config/regression_baseline.yaml`](config/regression_baseline.yaml)，随代码一起做版本管理。

## 故障注入

在场景的 `sut` 部分声明故障：

```yaml
sut:
  mode: lka
  duration: 8.0
  speed: 0.5
  faults:
    - target: camera
      type: dropout
      start_time: 3.0
      duration: 2.0
```

支持的故障类型：

- 摄像头：`dropout`（掉帧）、`frozen_frame`（画面冻结）、`blur`（模糊）、`noise`（噪声）
- 激光雷达：`dropout`（掉帧）、`frozen_scan`（扫描冻结）、`range_bias`（测距偏差）、`noise`（噪声）

## YOLO 检测验证

YOLO 权重在运行时传入，不提交到仓库。标注真值 CSV 的格式：

```text
image_path,class_name,x1,y1,x2,y2
frame_001.png,car,120,180,320,350
```

运行验证：

```bash
python scripts/validate_yolo.py \
  --manifest data/labels.csv \
  --image-root data/images \
  --model yolov8n.pt
```

在联合控制流程中启用 YOLO：

```bash
python scripts/run_adas.py --yolo-model yolov8n.pt --output reports/adas.json
```

## 测试与覆盖率

```bash
python -m pytest -q
python -m pytest --cov=src --cov-report=term-missing
```

最近一次本地运行结果：

```text
247 passed
88% total statement coverage
5/5 Gazebo validation scenarios passed
```

`config/github-actions-unit-tests.yml` 是 GitHub Actions 模板。给推送用的凭据开通 GitHub `workflow` 权限后，把它复制到 `.github/workflows/unit-tests.yml` 即可启用。Gazebo 端到端测试依赖仿真器、渲染和本地绑定，仍在本地执行。

## 说明

这是一个用于展示测试工程能力的个人项目，不是量产车辆软件。其中的 LKA 和 AEB 实现刻意保持简单易懂，用来演示仿真、验证、指标测量、故障注入和回归测试的方法。

## 作者

候家宽（Jia Kuan Hou）· 莫纳什大学人工智能硕士
