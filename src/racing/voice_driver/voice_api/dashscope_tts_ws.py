"""DashScope CosyVoice TTS over WebSocket (no dashscope SDK required)."""

from __future__ import annotations

import json
import tempfile
import uuid
from pathlib import Path
from typing import Any


class DashScopeTtsWebSocket:
    WS_URL = 'wss://dashscope.aliyuncs.com/api-ws/v1/inference'

    def __init__(
        self,
        *,
        api_key: str,
        model: str = 'cosyvoice-v1',
        voice: str = 'longxiaochun',
        logger: Any | None = None,
    ) -> None:
        self._api_key = api_key
        self._model = model
        self._voice = voice
        self._logger = logger

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

        task_id = uuid.uuid4().hex
        target = output_path or Path(tempfile.gettempdir()) / f'voice_broadcast_{task_id}.mp3'
        audio_chunks: list[bytes] = []
        task_started = False
        text_sent = False

        try:
            with connect(
                self.WS_URL,
                additional_headers={'Authorization': f'bearer {self._api_key}'},
                open_timeout=90,
            ) as ws:
                ws.send(json.dumps(self._run_task_message(task_id)))

                while True:
                    frame = ws.recv()
                    if isinstance(frame, bytes):
                        audio_chunks.append(frame)
                        continue

                    message = json.loads(frame)
                    event = message.get('header', {}).get('event', '')
                    if event == 'task-started' and not task_started:
                        task_started = True
                        ws.send(json.dumps(self._continue_task_message(task_id, cleaned)))
                        ws.send(json.dumps(self._finish_task_message(task_id)))
                        text_sent = True
                    elif event == 'task-finished':
                        break
                    elif event == 'task-failed':
                        detail = message.get('payload', {}).get('output', message)
                        self._log_error(f'DashScope TTS task failed: {detail}')
                        return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'DashScope TTS websocket failed: {exc}')
            return None

        if not text_sent:
            self._log_error('DashScope TTS did not reach task-started')
            return None
        if not audio_chunks:
            self._log_error('DashScope TTS returned no audio data')
            return None

        target.write_bytes(b''.join(audio_chunks))
        return target

    def _run_task_message(self, task_id: str) -> dict[str, Any]:
        return {
            'header': {
                'action': 'run-task',
                'task_id': task_id,
                'streaming': 'duplex',
            },
            'payload': {
                'task_group': 'audio',
                'task': 'tts',
                'function': 'SpeechSynthesizer',
                'model': self._model,
                'parameters': {
                    'text_type': 'PlainText',
                    'voice': self._voice,
                    'format': 'mp3',
                    'sample_rate': 22050,
                },
                'input': {},
            },
        }

    @staticmethod
    def _continue_task_message(task_id: str, text: str) -> dict[str, Any]:
        return {
            'header': {
                'action': 'continue-task',
                'task_id': task_id,
                'streaming': 'duplex',
            },
            'payload': {
                'input': {
                    'text': text,
                },
            },
        }

    @staticmethod
    def _finish_task_message(task_id: str) -> dict[str, Any]:
        return {
            'header': {
                'action': 'finish-task',
                'task_id': task_id,
                'streaming': 'duplex',
            },
            'payload': {
                'input': {},
            },
        }

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[DashScopeTtsWebSocket] ERROR: {message}')
