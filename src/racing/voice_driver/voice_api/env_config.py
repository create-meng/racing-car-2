"""Load API keys and runtime settings from .env and environment variables."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - optional until pip install
    load_dotenv = None

# 单一固定来源：.env 由 CMake 安装到本包的 ROS share 目录，
# 运行时通过 ament index 定位，不依赖 __file__/cwd 猜测，任何构建方式都稳定。
ENV_FILENAME = '.env'


def env_file_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    share_dir = Path(get_package_share_directory('voice_driver'))
    return share_dir / ENV_FILENAME


def _load_env_file(path: Path) -> None:
    for raw_line in path.read_text(encoding='utf-8').splitlines():
        line = raw_line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, _, value = line.partition('=')
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def load_env() -> None:
    """Load the package .env from the ament share directory."""
    try:
        path = env_file_path()
    except Exception as exc:  # noqa: BLE001 - ament_index not sourced
        raise RuntimeError(
            '无法定位 voice_driver 的 .env：先 source install/setup.bash '
            '并确认已执行 colcon build --packages-select voice_driver'
        ) from exc
    if not path.is_file():
        raise FileNotFoundError(
            f'找不到语音配置 {path}，请执行: '
            'colcon build --packages-select voice_driver'
        )
    if load_dotenv is not None:
        load_dotenv(path, override=False)
    else:
        _load_env_file(path)


def _get(name: str, default: str = '') -> str:
    value = os.environ.get(name, default)
    return value.strip() if isinstance(value, str) else default


@dataclass(frozen=True)
class VoiceEnvConfig:
    """Runtime configuration for vision + TTS voice broadcast."""

    vision_provider: str
    dashscope_api_key: str
    dashscope_model_id: str
    ark_api_key: str
    ark_model_id: str
    vision_prompt: str
    volc_tts_app_id: str
    volc_tts_access_token: str
    volc_tts_cluster: str
    volc_tts_voice_type: str
    dashscope_tts_model: str
    dashscope_tts_voice: str
    dashscope_tts_ws_url: str
    tts_provider: str
    edge_tts_voice: str
    audio_player: str
    audio_device: str
    audio_output: str
    voice_serial_port: str
    voice_serial_baud: int
    target_sign: int
    sign_topic: str
    image_topic: str
    ai_description_topic: str
    broadcast_mode: str

    @classmethod
    def from_env(cls) -> VoiceEnvConfig:
        load_env()
        dashscope_key = _get('DASHSCOPE_API_KEY')
        ark_key = _get('ARK_API_KEY')
        dashscope_model = _get('DASHSCOPE_MODEL_ID')
        ark_model = _get('ARK_MODEL_ID', 'doubao-seed-1-6-250615')

        return cls(
            vision_provider=_get('VISION_PROVIDER', 'auto'),
            dashscope_api_key=dashscope_key or ark_key,
            dashscope_model_id=dashscope_model or ark_model,
            ark_api_key=ark_key,
            ark_model_id=ark_model,
            vision_prompt=_get(
                'VISION_PROMPT',
                '请用一小段话简单描述这张图片中的内容，包括物体、场景、颜色等信息。',
            ),
            volc_tts_app_id=_get('VOLC_TTS_APP_ID'),
            volc_tts_access_token=_get('VOLC_TTS_ACCESS_TOKEN'),
            volc_tts_cluster=_get('VOLC_TTS_CLUSTER', 'volcano_tts'),
            volc_tts_voice_type=_get('VOLC_TTS_VOICE_TYPE', 'BV700_streaming'),
            dashscope_tts_model=_get(
                'DASHSCOPE_TTS_MODEL', 'qwen-tts-realtime-2025-07-15'
            ),
            dashscope_tts_voice=_get('DASHSCOPE_TTS_VOICE', 'Cherry'),
            dashscope_tts_ws_url=_get(
                'DASHSCOPE_TTS_WS_URL',
                'wss://dashscope.aliyuncs.com/api-ws/v1/realtime',
            ),
            tts_provider=_get('TTS_PROVIDER', 'auto'),
            edge_tts_voice=_get('EDGE_TTS_VOICE', 'zh-CN-XiaoxiaoNeural'),
            audio_player=_get('AUDIO_PLAYER', 'auto'),
            audio_device=_get('AUDIO_DEVICE', 'plughw:0,0'),
            audio_output=_get('AUDIO_OUTPUT', 'mae01'),
            voice_serial_port=_get('VOICE_SERIAL_PORT', '/dev/ttyS1'),
            voice_serial_baud=int(_get('VOICE_SERIAL_BAUD', '9600') or '9600'),
            target_sign=int(_get('TARGET_SIGN', '9') or '9'),
            sign_topic=_get('SIGN_TOPIC', 'sign4return'),
            image_topic=_get('IMAGE_TOPIC', '/image'),
            ai_description_topic=_get('AI_DESCRIPTION_TOPIC', 'ai_description'),
            broadcast_mode=_get('BROADCAST_MODE', 'full'),
        )

    def resolved_vision_provider(self) -> str:
        provider = self.vision_provider.lower()
        if provider in {'dashscope', 'bailian', '百炼'}:
            return 'dashscope'
        if provider == 'ark':
            return 'ark'
        model = (self.dashscope_model_id or self.ark_model_id).lower()
        if self.dashscope_api_key and (
            model.startswith('qwen') or model.startswith('qvq') or 'dashscope' in model
        ):
            return 'dashscope'
        if self.ark_api_key and not self.dashscope_api_key:
            return 'ark'
        if self.dashscope_api_key:
            return 'dashscope'
        return 'ark'

    def vision_api_key(self) -> str:
        if self.resolved_vision_provider() == 'dashscope':
            return self.dashscope_api_key
        return self.ark_api_key

    def vision_model_id(self) -> str:
        if self.resolved_vision_provider() == 'dashscope':
            return self.dashscope_model_id
        return self.ark_model_id

    @property
    def vision_ready(self) -> bool:
        return bool(self.vision_api_key() and self.vision_model_id())

    @property
    def volc_tts_ready(self) -> bool:
        return bool(self.volc_tts_app_id and self.volc_tts_access_token)

    @property
    def dashscope_tts_ready(self) -> bool:
        return bool(self.dashscope_api_key)

    def resolved_tts_provider(self) -> str:
        provider = self.tts_provider.lower()
        if provider == 'auto':
            if self.dashscope_tts_ready:
                return 'dashscope'
            if self.volc_tts_ready:
                return 'volcengine'
            return 'edge_tts'
        if provider in {'dashscope', 'bailian', '百炼'}:
            return 'dashscope'
        return provider

    def resolved_audio_output(self) -> str:
        """mae01 = voice module only; alsa = board/USB speaker; both = try both."""
        mode = self.audio_output.lower()
        if mode in {'module', 'yahboom', 'mae01_serial'}:
            return 'mae01'
        return mode

    def uses_mae01(self) -> bool:
        return self.resolved_audio_output() in {'mae01', 'both'}

    def uses_alsa(self) -> bool:
        return self.resolved_audio_output() in {'alsa', 'both', 'auto'}
