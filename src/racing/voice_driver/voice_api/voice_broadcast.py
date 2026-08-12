"""High-level image -> LLM -> TTS -> playback pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .audio_player import AudioPlayer
from .env_config import VoiceEnvConfig
from .module_player import ModuleVoicePlayer
from .tts_client import TtsClient
from .vision_analyzer import VisionAnalyzer
from .voice_ids import VoiceId


class VoiceBroadcastService:
    """Orchestrate vision analysis and spoken output."""

    def __init__(self, config: VoiceEnvConfig | None = None, logger: Any | None = None) -> None:
        self._config = config or VoiceEnvConfig.from_env()
        self._logger = logger
        self._vision = VisionAnalyzer(
            provider=self._config.resolved_vision_provider(),
            api_key=self._config.vision_api_key(),
            model_id=self._config.vision_model_id(),
            prompt=self._config.vision_prompt,
            logger=logger,
        )
        self._tts = TtsClient(
            provider=self._config.resolved_tts_provider(),
            app_id=self._config.volc_tts_app_id,
            access_token=self._config.volc_tts_access_token,
            cluster=self._config.volc_tts_cluster,
            voice_type=self._config.volc_tts_voice_type,
            edge_voice=self._config.edge_tts_voice,
            dashscope_api_key=self._config.dashscope_api_key,
            dashscope_model=self._config.dashscope_tts_model,
            dashscope_voice=self._config.dashscope_tts_voice,
            dashscope_tts_ws_url=self._config.dashscope_tts_ws_url,
            logger=logger,
        )
        self._player = AudioPlayer(
            player=self._config.audio_player,
            alsa_device=self._config.audio_device,
            logger=logger,
        )
        self._module = ModuleVoicePlayer.from_config(
            self._config, logger=logger
        )

    @property
    def config(self) -> VoiceEnvConfig:
        return self._config

    def speak_text(self, text: str) -> bool:
        """Speak API/LLM text through the configured local/cloud outputs."""
        cleaned = text.strip()
        if not cleaned:
            self._log_error('[VOICE] Empty text')
            return False

        mode = self._config.resolved_audio_output()
        self._log_info(f'[VOICE] AUDIO_OUTPUT={mode}  text_len={len(cleaned)}')
        self._log_info(f'[VOICE] 播报内容: "{cleaned[:100]}"{"..." if len(cleaned) > 100 else ""}')

        # 1) 优先：本地 CN-TTS 模块（支持任意中文/英文文本，无需音箱）
        if self._config.uses_mae01():
            self._log_info('[VOICE] 尝试使用 CN-TTS 模块播报')
            ok_module = self._module.speak_text(cleaned)
            if ok_module:
                self._log_info('[VOICE] CN-TTS 播报成功')
                return True
            else:
                self._log_error('[VOICE] CN-TTS 播报失败，尝试备用方案')

        # 2) 备用：云端 TTS（需要板载/USB 音箱）
        if self._config.uses_alsa():
            self._log_info('[VOICE] 尝试使用云端 TTS 播报')
            ok_alsa = self._speak_via_cloud_tts(cleaned)
            if ok_alsa:
                self._log_info('[VOICE] 云端 TTS 播报成功')
                return True

        if mode == 'mae01':
            self._log_error(
                '本地语音模块播报失败；请检查串口、波特率和模块供电，'
                '或外接 USB 音箱并改 .env AUDIO_OUTPUT=alsa（或 both）'
            )
        else:
            self._log_error(
                '云端 TTS 播报失败：检查 DASHSCOPE_API_KEY、网络、AUDIO_DEVICE、音箱'
            )
        return False

    def _speak_via_cloud_tts(self, cleaned: str) -> bool:
        self._log_info(
            f'TTS synthesize ({self._config.resolved_tts_provider()}, '
            f'{len(cleaned)} chars)...'
        )
        audio_path = self._tts.synthesize_to_file(cleaned)
        if audio_path is None:
            self._log_error('Cloud TTS failed; check DASHSCOPE_API_KEY in .env')
            return False

        self._log_info(f'TTS audio: {audio_path} ({audio_path.stat().st_size} bytes)')
        self._player.prepare_alsa()
        if self._player.play_file(audio_path):
            self._log_info(
                f'Playback on {self._config.audio_device} ({len(cleaned)} chars)'
            )
            print(
                f'>>> TTS 已从 {self._config.audio_device} 播放 '
                f'（需板载/USB 音箱）'
            )
            return True

        self._log_error(
            f'ALSA playback failed on {self._config.audio_device}; '
            'connect USB speaker or set AUDIO_OUTPUT=mae01'
        )
        return False

    def play_hardware_preset(self, name: str = 'forward') -> bool:
        preset_id = VoiceId.CAR_FORWARD if name == 'forward' else VoiceId.GO_AHEAD
        if name in {'welcome', 'stop', 'back', 'go_ahead'}:
            preset_id = {
                'welcome': 0x00,
                'go_ahead': VoiceId.GO_AHEAD,
                'stop': VoiceId.CAR_STOPPED,
                'back': VoiceId.BACK,
            }.get(name, preset_id)
        return self._module.play_preset(int(preset_id))

    def describe_image_path(self, image_path: str | Path) -> str | None:
        if not self._vision.ready:
            self._log_error('Vision API not configured')
            return None
        text = self._vision.analyze_path(image_path)
        if text:
            self._log_info(f'Vision result: {text}')
        return text

    def describe_image_bgr(self, bgr_image: np.ndarray) -> str | None:
        if not self._vision.ready:
            self._log_error('Vision API not configured')
            return None
        text = self._vision.analyze_bgr(bgr_image)
        if text:
            self._log_info(f'Vision result: {text}')
        return text

    def describe_and_speak_path(self, image_path: str | Path) -> str | None:
        text = self.describe_image_path(image_path)
        if not text:
            return None
        if not self.speak_text(text):
            return None
        return text

    def describe_and_speak_bgr(self, bgr_image: np.ndarray) -> str | None:
        text = self.describe_image_bgr(bgr_image)
        if not text:
            return None
        if not self.speak_text(text):
            return None
        return text

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[VoiceBroadcast] {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[VoiceBroadcast] ERROR: {message}')
