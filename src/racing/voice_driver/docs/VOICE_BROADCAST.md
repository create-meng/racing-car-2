# Voice Broadcast

**请先读：[VOICE_SETUP.md](VOICE_SETUP.md)** — 生产语音链路和串口配置。

## 快速命令

```bash
source install/setup.bash

# 串口模块预设
ros2 run voice_driver voice_speak forward

# API 文字（.env 里 AUDIO_OUTPUT=mae01 或 alsa）
ros2 run voice_driver voice_speak_text -- "测试"

# 图片 → 大模型 → 播报
ros2 run voice_driver voice_broadcast -- --image /path/to.jpg

# ROS 节点
ros2 launch voice_driver voice_broadcast.launch.py mode:=tts_only
```

## AUDIO_OUTPUT

| 值 | 出声位置 | 能否播 API 任意长文本 |
|----|----------|----------------------|
| `mae01` | 串口语音模块 | 由模块固件决定 |
| `alsa` | RDK/USB 音箱 | ✅ 百炼 TTS |
| `both` | 两者都试 | 视硬件而定 |
