# 语音播报配置

赛事生产链只启动 `voice_broadcast_node`，由 `voice_tts.launch.py` 以 `tts_only` 模式订阅：

```text
competition_qr_task  ─┐
ai_description       ─┼─> voice_broadcast_node ─> 串口语音模块 / 云端 TTS + ALSA
                      ┘
```

二维码确认和视觉识别结果都通过后台队列播报，不阻塞导航。总启动入口是：

```bash
ros2 launch racing_bringup competition_total.launch.py
```

## 环境变量

复制 `.env.example` 为 `.env`，至少配置语音模块串口：

```env
AUDIO_OUTPUT=both
VOICE_SERIAL_PORT=/dev/ttyS1
VOICE_SERIAL_BAUD=9600
```

`AUDIO_OUTPUT` 可取：

- `mae01`：仅串口语音模块；
- `alsa`：仅板载/USB 音箱，需配置 TTS API；
- `both`：先尝试串口模块，失败后使用云端 TTS。

云端 TTS 使用百炼时配置 `DASHSCOPE_API_KEY`、`DASHSCOPE_TTS_MODEL` 和
`DASHSCOPE_TTS_VOICE`。串口模块文本使用 GBK 编码发送，实际播报能力由模块固件决定。

## 隔离检查

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

# 预设短句
ros2 run voice_driver voice_speak forward

# 直接播报文本
ros2 run voice_driver voice_speak_text -- "语音链路测试"

# 检查云端 TTS 和串口模块
ros2 run voice_driver voice_hardware_test --preset-only
```

不要向 `ttyACM0` 或 `ttyACM1` 发送语音数据，它们属于底盘电机串口。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select voice_driver
```
