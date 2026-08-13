# 启动命令实际使用的文件清单

> 本文件供 AI 分析时快速定位，避免查看无关代码。
> 来源：`/root/ros2_ws/启动readme.md`

---

## 命令1: 启动底盘

```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/origincar/origincar_base/launch/origincar_bringup.launch.py` | 主入口，组装底盘子launch、TF发布、IMU滤波、EKF、关节发布、模型描述 |
| `src/origincar/origincar_base/launch/base_serial.launch.py` | 启动串口底盘节点 `origincar_base_node`，订阅 `/cmd_vel` 发送串口指令；若阿克曼模式则额外启动 `cmd_vel_to_ackermann_drive.py` |
| `src/origincar/origincar_base/launch/robot_mode_description.launch.py` | 启动 `robot_state_publisher`，加载 URDF 发布机器人模型 TF |
| `src/origincar/origincar_base/config/ekf.yaml` | 扩展卡尔曼滤波参数（里程计/IMU 融合） |
| `src/origincar/origincar_base/config/imu.yaml` | IMU madgwick 滤波参数 |
| `src/origincar/origincar_description/urdf/origincar.urdf` | 机器人 URDF 模型文件 |

---

## 命令2: 启动雷达

```bash
ros2 launch lslidar_driver lsn10_launch.py
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/LSLIDAR_X_ROS2-20240228/src/lslidar_driver/launch/lsn10_launch.py` | 启动 LSN10 激光雷达 LifecycleNode |
| `src/LSLIDAR_X_ROS2-20240228/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml` | 雷达参数（IP、端口、帧ID、角度裁剪、距离范围） |

---

## 命令3: 启动 Nav2 导航

```bash
ros2 launch racecar Run_nav.launch.py
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/racecar/launch/Run_nav.launch.py` | 导航启动入口，声明地图/参数路径，启动 waypoint_cycle、诊断监控、引入 bringup |
| `src/racecar/launch/bringup_launch.py` | 基于 nav2_bringup 的导航栈组装（localization + navigation，不含 slam） |
| `src/racecar/config/nav.yaml` | Nav2 核心参数（AMCL、MPPI控制器、costmap、规划器、BT、behavior等） |
| `src/racecar/config/navigate_through_poses_loop.xml` | 自定义行为树（路点循环导航 + 恢复行为） |
| `src/racecar/scripts/trigger_monitor.py` | 导航诊断监控脚本，检测异常并写日志 |
| `map/ai_map.yaml` | 地图描述文件（引用 pgm 图片） |
| `map/ai_map.pgm` | 栅格地图图片 |

---

## 命令4: 启动深度相机

```bash
ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py
```

**本工作区无相关文件**（纯系统包）。

---

## 命令5: 打开 Rviz2

```bash
rviz2
```

**系统命令，本工作区无相关文件。**

---

## 命令6: 启动合并版节点（双摄像头扫码 + 图生文 + image发布）

```bash
cd /root/ros2_ws/src/racecar/scripts/main/
python3 saoma.py
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/racecar/scripts/main/saoma.py` | 合并版 ROS2 节点：双摄像头扫码 + 图生文（调用大模型）+ CompressedImage 发布 |

---

## 命令7: 启动语音播报

```bash
ros2 launch voice_driver voice_tts.launch.py mode:=tts_only
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/racing/voice_driver/launch/voice_tts.launch.py` | 启动 TTS 语音播报节点（仅播报模式），订阅 `/tts_text` 话题 |
| `src/racing/voice_driver/voice_api/voice_broadcast_node.py` | TTS 播报节点实现（读取环境变量、调用语音 API/串口模块） |
| `src/racing/voice_driver/voice_api/voice_broadcast.py` | 语音播报核心逻辑 |
| `src/racing/voice_driver/voice_api/tts_client.py` | TTS 客户端（调用云端或本地 TTS） |
| `src/racing/voice_driver/voice_api/cn_tts_player.py` | CN-TTS 串口模块播报驱动 |
| `src/racing/voice_driver/voice_api/env_config.py` | 环境变量配置读取 |
| `src/racing/voice_driver/voice_api/audio_player.py` | 音频播放器 |
| `src/racing/voice_driver/voice_api/module_player.py` | 模块播报控制 |
| `src/racing/voice_driver/voice_api/voice_ids.py` | 语音 ID 配置 |
| `src/racing/voice_driver/voice_api/dashscope_tts_ws.py` | 阿里云 dashscope TTS WebSocket 客户端 |
| `src/racing/voice_driver/voice_api/dashscope_qwen_tts_realtime.py` | 阿里云 Qwen 实时 TTS |
| `src/racing/voice_driver/voice_api/vision_analyzer.py` | 视觉分析（图生文描述） |
| `src/racing/voice_driver/voice_api/voice_speak_text.py` | 文本播报工具 |
| `src/racing/voice_driver/voice_api/voice_speak_now.py` | 即时播报工具 |
| `src/racing/voice_driver/voice_api/voice_broadcast_cli.py` | 语音播报 CLI 工具 |
| `src/racing/voice_driver/.env` | 环境变量（API Key 等配置） |

---

## 命令8: 启动导航文件（路点导航）

```bash
cd /root/ros2_ws/src/racecar/scripts/src
python3 h1.py
```

| 文件路径 | 功能说明 |
|---------|---------|
| `src/racecar/scripts/src/h1.py` | 路点导航客户端：先执行 dating.csv 第一阶段，收到二维码后切换至 shunshizhen1.csv 或 test_1.csv |
| `src/racecar/scripts/point/dating.csv` | 第一阶段路点文件（通用路线） |
| `src/racecar/scripts/point/shunshizhen1.csv` | 二维码奇数路线（顺时针） |
| `src/racecar/scripts/point/test_1.csv` | 二维码偶数路线（逆时针） |

---

## 汇总：本工作区实际使用的文件

共 **31 个文件**：

| # | 文件路径 | 所属命令 |
|---|---------|---------|
| 1 | `src/origincar/origincar_base/launch/origincar_bringup.launch.py` | 1 |
| 2 | `src/origincar/origincar_base/launch/base_serial.launch.py` | 1 |
| 3 | `src/origincar/origincar_base/launch/robot_mode_description.launch.py` | 1 |
| 4 | `src/origincar/origincar_base/config/ekf.yaml` | 1 |
| 5 | `src/origincar/origincar_base/config/imu.yaml` | 1 |
| 6 | `src/origincar/origincar_description/urdf/origincar.urdf` | 1 |
| 7 | `src/LSLIDAR_X_ROS2-20240228/src/lslidar_driver/launch/lsn10_launch.py` | 2 |
| 8 | `src/LSLIDAR_X_ROS2-20240228/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml` | 2 |
| 9 | `src/racecar/launch/Run_nav.launch.py` | 3 |
| 10 | `src/racecar/launch/bringup_launch.py` | 3 |
| 11 | `src/racecar/config/nav.yaml` | 3 |
| 12 | `src/racecar/config/navigate_through_poses_loop.xml` | 3 |
| 13 | `src/racecar/scripts/trigger_monitor.py` | 3 |
| 14 | `map/ai_map.yaml` | 3 |
| 15 | `map/ai_map.pgm` | 3 |
| 16 | `src/racecar/scripts/main/saoma.py` | 6 |
| 17 | `src/racing/voice_driver/launch/voice_tts.launch.py` | 7 |
| 18 | `src/racing/voice_driver/voice_api/voice_broadcast_node.py` | 7 |
| 19 | `src/racing/voice_driver/voice_api/voice_broadcast.py` | 7 |
| 20 | `src/racing/voice_driver/voice_api/tts_client.py` | 7 |
| 21 | `src/racing/voice_driver/voice_api/cn_tts_player.py` | 7 |
| 22 | `src/racing/voice_driver/voice_api/env_config.py` | 7 |
| 23 | `src/racing/voice_driver/voice_api/audio_player.py` | 7 |
| 24 | `src/racing/voice_driver/voice_api/module_player.py` | 7 |
| 25 | `src/racing/voice_driver/voice_api/voice_ids.py` | 7 |
| 26 | `src/racing/voice_driver/voice_api/dashscope_tts_ws.py` | 7 |
| 27 | `src/racing/voice_driver/voice_api/dashscope_qwen_tts_realtime.py` | 7 |
| 28 | `src/racing/voice_driver/voice_api/vision_analyzer.py` | 7 |
| 29 | `src/racing/voice_driver/voice_api/voice_speak_text.py` | 7 |
| 30 | `src/racing/voice_driver/voice_api/voice_speak_now.py` | 7 |
| 31 | `src/racing/voice_driver/voice_api/voice_broadcast_cli.py` | 7 |
| 32 | `src/racing/voice_driver/.env` | 7 |
| 33 | `src/racecar/scripts/src/h1.py` | 8 |
| 34 | `src/racecar/scripts/point/dating.csv` | 8 |
| 35 | `src/racecar/scripts/point/shunshizhen1.csv` | 8 |
| 36 | `src/racecar/scripts/point/test_1.csv` | 8 |