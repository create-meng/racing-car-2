"""Drive the serial voice module used by the competition voice path."""

from __future__ import annotations

from typing import Any

from .cn_tts_player import CnTtsPlayer
from .env_config import VoiceEnvConfig


class ModuleVoicePlayer:
    """Serial voice-module player with bounded port fallback."""

    FALLBACK_PORTS = ('/dev/ttyS1', '/dev/ttyS5')
    FALLBACK_BAUDS = (9600, 115200)

    def __init__(
        self,
        *,
        cn_tts: CnTtsPlayer,
        logger: Any | None = None,
    ) -> None:
        """
        初始化语音模块播放器
        
        Args:
            cn_tts: CN-TTS UART 播放器
            logger: ROS 2 logger
        """
        self._cn_tts = cn_tts
        self._logger = logger

    @classmethod
    def from_config(
        cls,
        config: VoiceEnvConfig | None = None,
        *,
        logger: Any | None = None,
    ) -> ModuleVoicePlayer:
        """从环境配置创建播放器。"""
        cfg = config or VoiceEnvConfig.from_env()
        cn_tts = CnTtsPlayer(
            port=cfg.voice_serial_port or '/dev/ttyS1',
            baudrate=cfg.voice_serial_baud,
            logger=logger,
        )
        return cls(cn_tts=cn_tts, logger=logger)

    def speak_text(self, text: str) -> bool:
        """
        播报自定义文本（CN-TTS 支持任意中文/英文/数字）
        
        Args:
            text: 要播报的文本
            
        Returns:
            bool: 播报是否成功
        """
        cleaned = text.strip()
        if not cleaned:
            return False

        self._log_info(f'CN-TTS 播报文本 (长度={len(cleaned)}): {cleaned[:100]}...')

        if self._cn_tts.speak_text(cleaned):
            return True

        for port in self.FALLBACK_PORTS:
            if port == self._cn_tts._port:  # noqa: SLF001
                continue
            for baud in (self._cn_tts._baudrate, *self.FALLBACK_BAUDS):  # noqa: SLF001
                backup_player = CnTtsPlayer(port=port, baudrate=baud, logger=self._logger)
                if backup_player.speak_text(cleaned):
                    self._log_info(f'备用端口成功: {port}@{baud}')
                    return True
        
        self._log_error('CN-TTS 播报失败，请检查串口连接和权限')
        return False

    def play_preset(self, voice_id: int) -> bool:
        """
        播放预设短语（兼容旧接口）
        新 CN-TTS 模块通过音效 ID 播放，或直接文本播报
        """
        preset_map = {
            0x00: '欢迎',
            0x03: '前进',
            0x05: '后退',
            0x0F: '左转',
            0x10: '右转',
            0x11: '停止',
        }
        
        text = preset_map.get(voice_id)
        if text:
            return self.speak_text(text)
        
        self._log_error(f'未知预设 ID: 0x{voice_id:02X}')
        return False

    def play_named(self, name: str) -> bool:
        """
        播放命名预设（兼容旧接口）
        CN-TTS 支持直接文本播报
        """
        key = name.strip().lower()
        
        # 关键词映射
        keyword_map = {
            'welcome': '欢迎',
            'forward': '前进',
            'back': '后退',
            'left': '左转',
            'right': '右转',
            'stop': '停止',
        }
        
        text = keyword_map.get(key, name)
        return self.speak_text(text)

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(f'[ModuleVoicePlayer] {message}')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[ModuleVoicePlayer] ERROR: {message}')
