"""DashScope Qwen-TTS Realtime over WebSocket (qwen-tts-realtime-2025-07-15)."""

from __future__ import annotations

import base64
import json
import tempfile
import time
import uuid
import wave
from pathlib import Path
from typing import Any


class QwenTtsRealtimeWebSocket:
    """Synthesize speech via wss://dashscope.aliyuncs.com/api-ws/v1/realtime."""

    DEFAULT_WS_BASE = 'wss://dashscope.aliyuncs.com/api-ws/v1/realtime'

    def __init__(
        self,
        *,
        api_key: str,
        model: str = 'qwen-tts-realtime-2025-07-15',
        voice: str = 'Cherry',
        ws_base_url: str = DEFAULT_WS_BASE,
        response_format: str = 'pcm',
        sample_rate: int = 24000,
        language_type: str = 'Chinese',
        mode: str = 'server_commit',
        logger: Any | None = None,
    ) -> None:
        self._api_key = api_key
        self._model = model
        self._voice = voice
        self._ws_base_url = ws_base_url.rstrip('/').split('?')[0]
        self._response_format = response_format
        self._sample_rate = sample_rate
        self._language_type = language_type
        self._mode = mode
        self._logger = logger

    @staticmethod
    def is_realtime_model(model: str) -> bool:
        name = model.lower()
        return 'realtime' in name or name.startswith('qwen-tts') or name.startswith('qwen3-tts')

    def synthesize_to_file(self, text: str, output_path: Path | None = None) -> Path | None:
        cleaned = text.strip()
        if not cleaned:
            self._log_error('TTS skipped: empty text')
            return None
        if not self._api_key:
            self._log_error('DASHSCOPE_API_KEY not configured')
            return None

        try:
            from websockets.sync.client import connect
        except ImportError:
            self._log_error('websockets not installed')
            return None

        suffix = '.wav' if self._response_format == 'pcm' else '.mp3'
        target = output_path or Path(tempfile.gettempdir()) / f'voice_broadcast_{uuid.uuid4().hex}{suffix}'
        audio_chunks: list[bytes] = []
        session_ready = False
        text_sent = False
        deadline = time.time() + 90.0

        ws_url = f'{self._ws_base_url}?model={self._model}'

        try:
            with connect(
                ws_url,
                additional_headers={'Authorization': f'Bearer {self._api_key}'},
                open_timeout=90,
                close_timeout=5,
            ) as ws:
                while time.time() < deadline:
                    raw = ws.recv()
                    if isinstance(raw, bytes):
                        audio_chunks.append(raw)
                        continue

                    event = json.loads(raw)
                    event_type = event.get('type', '')

                    if event_type == 'error':
                        detail = event.get('error', event)
                        self._log_error(f'Qwen-TTS Realtime error: {detail}')
                        return None

                    if event_type == 'session.created' and not session_ready:
                        ws.send(json.dumps(self._session_update_event()))
                        continue

                    if event_type == 'session.updated' and not session_ready:
                        session_ready = True
                        ws.send(json.dumps(self._append_text_event(cleaned)))
                        ws.send(json.dumps(self._finish_session_event()))
                        text_sent = True
                        continue

                    if event_type == 'response.audio.delta':
                        delta = event.get('delta', '')
                        if delta:
                            audio_chunks.append(base64.b64decode(delta))
                        continue

                    if text_sent and event_type == 'response.done':
                        break

                    if text_sent and event_type == 'session.finished':
                        break
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Qwen-TTS Realtime websocket failed: {exc}')
            return None

        if not session_ready:
            self._log_error('Qwen-TTS Realtime did not reach session.updated')
            return None
        if not audio_chunks:
            self._log_error('Qwen-TTS Realtime returned no audio data')
            return None

        pcm_data = b''.join(audio_chunks)
        if self._response_format == 'pcm':
            with wave.open(str(target), 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self._sample_rate)
                wav_file.writeframes(pcm_data)
        else:
            target.write_bytes(pcm_data)
        return target

    def _event_id(self) -> str:
        return f'event_{uuid.uuid4().hex}'

    def _session_update_event(self) -> dict[str, Any]:
        return {
            'event_id': self._event_id(),
            'type': 'session.update',
            'session': {
                'mode': self._mode,
                'voice': self._voice,
                'language_type': self._language_type,
                'response_format': self._response_format,
                'sample_rate': self._sample_rate,
            },
        }

    def _append_text_event(self, text: str) -> dict[str, Any]:
        return {
            'event_id': self._event_id(),
            'type': 'input_text_buffer.append',
            'text': text,
        }

    def _finish_session_event(self) -> dict[str, Any]:
        return {
            'event_id': self._event_id(),
            'type': 'session.finish',
        }

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[QwenTtsRealtime] ERROR: {message}')
