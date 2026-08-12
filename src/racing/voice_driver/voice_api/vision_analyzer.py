"""Vision analysis via Alibaba Bailian (DashScope) or Volcengine Ark."""

from __future__ import annotations

import base64
import io
import json
from pathlib import Path
from collections.abc import Callable
from typing import Any
import urllib.error
import urllib.request

import cv2
import numpy as np
from PIL import Image

try:
    from volcenginesdkarkruntime import Ark
except ImportError:  # pragma: no cover - optional until pip install
    Ark = None


class VisionAnalyzer:
    """Send one image to a multimodal LLM and return descriptive text."""

    def __init__(
        self,
        *,
        provider: str = 'dashscope',
        api_key: str,
        model_id: str,
        prompt: str,
        jpeg_quality: int = 85,
        image_max_edge_px: int = 0,
        base_url: str = '',
        request_timeout_sec: float | None = 120.0,
        max_tokens: int | None = None,
        thinking_enabled: bool | None = None,
        logger: Any | None = None,
    ) -> None:
        self._provider = provider.lower()
        self._api_key = api_key
        self._model_id = model_id
        self._prompt = prompt
        self._jpeg_quality = jpeg_quality
        self._image_max_edge_px = max(0, int(image_max_edge_px))
        self._base_url = base_url.rstrip('/')
        self._request_timeout_sec = (
            None if request_timeout_sec is None or float(request_timeout_sec) <= 0.0
            else max(1.0, float(request_timeout_sec))
        )
        self._max_tokens = max_tokens if max_tokens and max_tokens > 0 else None
        self._thinking_enabled = thinking_enabled
        self._logger = logger
        self._ark_client = None
        if self._provider == 'ark' and Ark is not None and api_key:
            self._ark_client = Ark(
                api_key=api_key,
                base_url=self._ark_sdk_base_url(self._base_url),
            )

    @property
    def ready(self) -> bool:
        if not self._model_id:
            return False
        if self._provider == 'openai_compatible':
            return bool(self._base_url)
        if not self._api_key:
            return False
        if self._provider == 'ark':
            return True
        return True

    @property
    def provider(self) -> str:
        return self._provider

    def _thinking_request_options(self) -> dict[str, Any]:
        """Return the provider-specific payload that disables hidden reasoning."""
        if self._thinking_enabled is not False:
            return {}
        if self._provider == 'ark':
            return {'thinking': {'type': 'disabled'}}
        if self._provider == 'dashscope':
            return {'enable_thinking': False}
        return {}

    def analyze_path(self, image_path: str | Path) -> str | None:
        path = Path(image_path)
        image = cv2.imread(str(path))
        if image is None:
            self._log_error(f'Failed to read image: {path}')
            return None
        return self.analyze_bgr(image)

    def analyze_bgr(self, bgr_image: np.ndarray) -> str | None:
        encoded = self._encode_bgr(bgr_image)
        if encoded is None:
            return None
        if self._provider == 'dashscope':
            return self._call_dashscope(encoded)
        if self._provider == 'ark':
            return self._call_ark(encoded)
        if self._provider == 'openai_compatible':
            return self._call_openai_compatible(encoded)
        self._log_error(f'Unsupported vision provider: {self._provider}')
        return None

    def analyze_bgr_stream(
        self,
        bgr_image: np.ndarray,
        on_delta: Callable[[str], None],
    ) -> str | None:
        """Stream provider text deltas while returning the complete response."""
        encoded = self._encode_bgr(bgr_image)
        if encoded is None:
            return None
        if self._provider == 'ark':
            return self._call_ark_stream(encoded, on_delta)
        if self._provider == 'dashscope':
            return self._call_openai_stream(
                self._base_url or 'https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions',
                encoded,
                on_delta,
                {'Authorization': f'Bearer {self._api_key}'},
            )
        if self._provider == 'openai_compatible':
            return self._call_openai_stream(self._base_url, encoded, on_delta, {})
        self._log_error(f'Unsupported streaming vision provider: {self._provider}')
        return None

    def _encode_bgr(self, bgr_image: np.ndarray) -> str | None:
        try:
            height, width = bgr_image.shape[:2]
            longest_edge = max(height, width)
            if self._image_max_edge_px and longest_edge > self._image_max_edge_px:
                scale = self._image_max_edge_px / longest_edge
                bgr_image = cv2.resize(
                    bgr_image,
                    (round(width * scale), round(height * scale)),
                    interpolation=cv2.INTER_AREA,
                )
            rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(rgb)
            buffer = io.BytesIO()
            pil_image.save(buffer, format='JPEG', quality=self._jpeg_quality)
            return base64.b64encode(buffer.getvalue()).decode('utf-8')
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Image encode failed: {exc}')
            return None

    def _call_dashscope(self, base64_image: str) -> str | None:
        if self._base_url:
            return self._call_dashscope_http(base64_image)
        try:
            import dashscope
            from dashscope import MultiModalConversation
        except ImportError:
            return self._call_dashscope_http(base64_image)

        dashscope.api_key = self._api_key
        try:
            response = MultiModalConversation.call(
                model=self._model_id,
                messages=[
                    {
                        'role': 'user',
                        'content': [
                            {'image': f'data:image/jpeg;base64,{base64_image}'},
                            {'text': self._prompt},
                        ],
                    }
                ],
            )
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'DashScope request failed: {exc}')
            return None

        if getattr(response, 'status_code', None) != 200:
            code = getattr(response, 'code', 'unknown')
            message = getattr(response, 'message', response)
            self._log_error(f'DashScope API error: {code} {message}')
            return None

        text = self._extract_dashscope_text(response)
        if text:
            return text
        self._log_error('DashScope returned empty content')
        return None

    def _call_dashscope_http(self, base64_image: str) -> str | None:
        import json
        import urllib.error
        import urllib.request

        url = self._base_url or 'https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions'
        payload = {
            'model': self._model_id,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {
                            'type': 'image_url',
                            'image_url': {
                                'url': f'data:image/jpeg;base64,{base64_image}',
                            },
                        },
                        {
                            'type': 'text',
                            'text': self._prompt,
                        },
                    ],
                }
            ],
        }
        payload.update(self._thinking_request_options())
        if self._max_tokens is not None:
            payload['max_tokens'] = self._max_tokens
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self._api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self._request_timeout_sec) as response:
                body = json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            self._log_error(f'DashScope HTTP error {exc.code}: {detail[:300]}')
            return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'DashScope HTTP request failed: {exc}')
            return None

        try:
            content = body['choices'][0]['message']['content']
            if isinstance(content, str) and content.strip():
                return content.strip()
        except (KeyError, IndexError, TypeError):
            pass
        self._log_error('DashScope HTTP returned empty content')
        return None

    def _call_openai_compatible(self, base64_image: str) -> str | None:
        """Call a local OpenAI-compatible VLM endpoint, such as llama-server."""
        payload = {
            'model': self._model_id,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': self._prompt},
                        {
                            'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'},
                        },
                    ],
                }
            ],
        }
        payload.update(self._thinking_request_options())
        if self._max_tokens is not None:
            payload['max_tokens'] = self._max_tokens
        request = urllib.request.Request(
            self._base_url,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self._request_timeout_sec) as response:
                body = json.loads(response.read().decode('utf-8'))
            content = body['choices'][0]['message']['content']
            if isinstance(content, str) and content.strip():
                return content.strip()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            self._log_error(f'Local VLM HTTP error {exc.code}: {detail[:300]}')
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Local VLM request failed: {exc}')
        return None

    def _call_openai_stream(
        self,
        url: str,
        base64_image: str,
        on_delta: Callable[[str], None],
        extra_headers: dict[str, str],
    ) -> str | None:
        """Read an OpenAI-compatible SSE chat stream used by Qwen and llama.cpp."""
        payload: dict[str, Any] = {
            'model': self._model_id,
            'stream': True,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': self._prompt},
                        {
                            'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'},
                        },
                    ],
                }
            ],
        }
        payload.update(self._thinking_request_options())
        if self._max_tokens is not None:
            payload['max_tokens'] = self._max_tokens
        headers = {'Content-Type': 'application/json', **extra_headers}
        request = urllib.request.Request(
            url,
            data=json.dumps(payload).encode('utf-8'),
            headers=headers,
            method='POST',
        )
        parts: list[str] = []
        try:
            with urllib.request.urlopen(request, timeout=self._request_timeout_sec) as response:
                for raw_line in response:
                    line = raw_line.decode('utf-8', errors='replace').strip()
                    if not line.startswith('data:'):
                        continue
                    data = line[5:].strip()
                    if data == '[DONE]':
                        break
                    try:
                        event = json.loads(data)
                        content = event['choices'][0]['delta'].get('content')
                    except (IndexError, KeyError, TypeError, json.JSONDecodeError):
                        continue
                    if isinstance(content, str) and content:
                        parts.append(content)
                        on_delta(content)
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            self._log_error(f'Streaming vision HTTP error {exc.code}: {detail[:300]}')
            return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Streaming vision request failed: {exc}')
            return None
        result = ''.join(parts).strip()
        if result:
            return result
        self._log_error('Streaming vision response was empty')
        return None

    @staticmethod
    def _extract_dashscope_text(response: Any) -> str | None:
        try:
            message = response.output.choices[0].message
            content = getattr(message, 'content', None)
            if isinstance(content, str) and content.strip():
                return content.strip()
            if isinstance(content, list):
                parts: list[str] = []
                for item in content:
                    if isinstance(item, dict) and item.get('text'):
                        parts.append(str(item['text']))
                    elif isinstance(item, str):
                        parts.append(item)
                merged = ''.join(parts).strip()
                if merged:
                    return merged
        except Exception:
            return None
        return None

    def _call_ark(self, base64_image: str) -> str | None:
        if self._ark_client is None:
            return self._call_ark_http(base64_image)
        try:
            request_kwargs: dict[str, Any] = {
                'model': self._model_id,
                'input': [
                    {
                        'role': 'user',
                        'content': [
                            {
                                'type': 'input_image',
                                'image_url': f'data:image/jpeg;base64,{base64_image}',
                            },
                            {
                                'type': 'input_text',
                                'text': self._prompt,
                            },
                        ],
                    }
                ],
            }
            request_kwargs.update(self._thinking_request_options())
            if self._request_timeout_sec is not None:
                request_kwargs['timeout'] = self._request_timeout_sec
            if self._max_tokens is not None:
                request_kwargs['max_output_tokens'] = self._max_tokens
            response = self._ark_client.responses.create(**request_kwargs)
            content = self._extract_ark_response_text(response)
            if isinstance(content, str) and content.strip():
                return content.strip()
            self._log_error('Ark model returned empty content')
            return None
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Ark API call failed: {exc}')
            return None

    def _call_ark_http(self, base64_image: str) -> str | None:
        """Call Ark through its OpenAI-compatible HTTP endpoint."""
        payload = {
            'model': self._model_id,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': self._prompt},
                        {
                            'type': 'image_url',
                            'image_url': {'url': f'data:image/jpeg;base64,{base64_image}'},
                        },
                    ],
                }
            ],
        }
        payload.update(self._thinking_request_options())
        if self._max_tokens is not None:
            payload['max_tokens'] = self._max_tokens
        request = urllib.request.Request(
            self._base_url or 'https://ark.cn-beijing.volces.com/api/v3/chat/completions',
            data=json.dumps(payload).encode('utf-8'),
            headers={
                'Authorization': f'Bearer {self._api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=self._request_timeout_sec) as response:
                body = json.loads(response.read().decode('utf-8'))
            content = body['choices'][0]['message']['content']
            if isinstance(content, str) and content.strip():
                return content.strip()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            self._log_error(f'Ark HTTP error {exc.code}: {detail[:300]}')
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Ark HTTP request failed: {exc}')
        return None

    def _call_ark_stream(
        self, base64_image: str, on_delta: Callable[[str], None]
    ) -> str | None:
        if self._ark_client is None:
            return self._call_openai_stream(
                self._base_url or 'https://ark.cn-beijing.volces.com/api/v3/chat/completions',
                base64_image,
                on_delta,
                {'Authorization': f'Bearer {self._api_key}'},
            )
        try:
            request_kwargs: dict[str, Any] = {
                'model': self._model_id,
                'input': [
                    {
                        'role': 'user',
                        'content': [
                            {
                                'type': 'input_image',
                                'image_url': f'data:image/jpeg;base64,{base64_image}',
                            },
                            {'type': 'input_text', 'text': self._prompt},
                        ],
                    }
                ],
                'stream': True,
            }
            request_kwargs.update(self._thinking_request_options())
            if self._request_timeout_sec is not None:
                request_kwargs['timeout'] = self._request_timeout_sec
            if self._max_tokens is not None:
                request_kwargs['max_output_tokens'] = self._max_tokens
            stream = self._ark_client.responses.create(**request_kwargs)
            parts: list[str] = []
            for event in stream:
                # The Responses API also streams reasoning-summary deltas. Only
                # publish final output text so internal reasoning is never spoken.
                if getattr(event, 'type', None) != 'response.output_text.delta':
                    continue
                content = getattr(event, 'delta', None)
                if not isinstance(content, str) or not content:
                    continue
                parts.append(content)
                on_delta(content)
            result = ''.join(parts).strip()
            if result:
                return result
            self._log_error('Ark stream returned empty content')
        except Exception as exc:  # noqa: BLE001
            self._log_error(f'Ark streaming API call failed: {exc}')
        return None

    @staticmethod
    def _extract_ark_response_text(response: Any) -> str:
        """Extract final text from Ark Responses API output items."""
        output_text = getattr(response, 'output_text', None)
        if isinstance(output_text, str) and output_text.strip():
            return output_text.strip()

        parts: list[str] = []
        for output in getattr(response, 'output', None) or []:
            if getattr(output, 'type', None) != 'message':
                continue
            for content in getattr(output, 'content', None) or []:
                text = getattr(content, 'text', None)
                if isinstance(text, str) and text.strip():
                    parts.append(text.strip())
        return ''.join(parts).strip()

    @staticmethod
    def _ark_sdk_base_url(base_url: str) -> str:
        """Convert a chat-completions URL into the Ark SDK API root."""
        if not base_url:
            return 'https://ark.cn-beijing.volces.com/api/v3'
        return base_url.removesuffix('/chat/completions')

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[VisionAnalyzer] ERROR: {message}')
