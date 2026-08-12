"""Text-to-speech clients for voice broadcast."""

from __future__ import annotations

import asyncio
import base64
import tempfile
import uuid
from pathlib import Path
from typing import Any

try:
    import requests
except ImportError:  # pragma: no cover
    requests = None

from .dashscope_tts_ws import DashScopeTtsWebSocket
from .dashscope_qwen_tts_realtime import QwenTtsRealtimeWebSocket


class TtsClient:
    """Convert text to an audio file using DashScope, Volcengine, or edge-tts."""

    VOLCENGINE_TTS_URL = 'https://openspeech.bytedance.com/api/v1/tts'

    def __init__(
        self,
        *,
        provider: str,
        app_id: str = '',
        access_token: str = '',
        cluster: str = 'volcano_tts',
        voice_type: str = 'BV700_streaming',
        edge_voice: str = 'zh-CN-XiaoxiaoNeural',
        dashscope_api_key: str = '',
        dashscope_model: str = 'qwen-tts-realtime-2025-07-15',
        dashscope_voice: str = 'Cherry',
        dashscope_tts_ws_url: str = '',
        logger: Any | None = None,
    ) -> None:
        self._provider = provider.lower()
        self._app_id = app_id
        self._access_token = access_token
        self._cluster = cluster
        self._voice_type = voice_type
        self._edge_voice = edge_voice
        self._dashscope_api_key = dashscope_api_key
        self._dashscope_model = dashscope_model
        self._dashscope_voice = dashscope_voice
        self._dashscope_tts_ws_url = dashscope_tts_ws_url
        self._logger = logger

    def synthesize_to_file(self, text: str, output_path: Path | None = None) -> Path | None:
        cleaned = text.strip()
        if not cleaned:
            self._log_error('TTS skipped: empty text')
            return None

        if self._provider == 'dashscope':
            if QwenTtsRealtimeWebSocket.is_realtime_model(self._dashscope_model):
                ws_base = (
                    self._dashscope_tts_ws_url
                    or QwenTtsRealtimeWebSocket.DEFAULT_WS_BASE
                )
                return QwenTtsRealtimeWebSocket(
                    api_key=self._dashscope_api_key,
                    model=self._dashscope_model,
                    voice=self._dashscope_voice,
                    ws_base_url=ws_base,
                    logger=self._logger,
                ).synthesize_to_file(cleaned, output_path)
            return DashScopeTtsWebSocket(
                api_key=self._dashscope_api_key,
                model=self._dashscope_model,
                voice=self._dashscope_voice,
                logger=self._logger,
            ).synthesize_to_file(cleaned, output_path)
        if self._provider == 'volcengine':
            return self._synthesize_volcengine(cleaned, output_path)
        if self._provider == 'edge_tts':
            return self._synthesize_edge_tts(cleaned, output_path)
        self._log_error(f'Unsupported TTS provider: {self._provider}')
        return None

    def _synthesize_volcengine(self, text: str, output_path: Path | None) -> Path | None:
        if requests is None:
            self._log_error('requests not installed; pip install requests')
            return None
        if not self._app_id or not self._access_token:
            self._log_error('VOLC_TTS_APP_ID / VOLC_TTS_ACCESS_TOKEN not configured')
            return None

        payload = {
            'app': {
                'appid': self._app_id,
                'token': self._access_token,
                'cluster': self._cluster,
            },
            'user': {'uid': 'racing_robot'},
            'audio': {
                'voice_type': self._voice_type,
                'encoding': 'mp3',
                'speed_ratio': 1.0,
                'volume_ratio': 1.0,
                'pitch_ratio': 1.0,
            },
            'request': {
                'reqid': str(uuid.uuid4()),
                'text': text,
                'text_type': 'plain',
                'operation': 'query',
            },
        }
        headers = {
            'Authorization': f'Bearer;{self._access_token}',
            'Content-Type': 'application/json',
        }

        try:
            response = requests.post(
                self.VOLCENGINE_TTS_URL,
                json=payload,
                headers=headers,
                timeout=30,
            )
            response.raise_for_status()
            body = response.json()
            if body.get('code') != 3000:
                self._log_error(f'Volcengine TTS error: {body}')
                return None
            audio_bytes = base64.b64decode(body['data'])
            target = output_path or Path(tempfile.gettempdir()) / f'voice_broadcast_{uuid.uuid4().hex}.mp3'
            target.write_bytes(audio_bytes)
            return target
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Volcengine TTS failed: {exc}')
            return None

    def _synthesize_edge_tts(self, text: str, output_path: Path | None) -> Path | None:
        try:
            import edge_tts
        except ImportError:
            self._log_error('edge-tts not installed; pip install edge-tts')
            return None

        target = output_path or Path(tempfile.gettempdir()) / f'voice_broadcast_{uuid.uuid4().hex}.mp3'

        async def _run() -> None:
            communicate = edge_tts.Communicate(text, self._edge_voice)
            await communicate.save(str(target))

        try:
            asyncio.run(_run())
            return target
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'edge-tts failed: {exc}')
            return None

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[TtsClient] ERROR: {message}')
