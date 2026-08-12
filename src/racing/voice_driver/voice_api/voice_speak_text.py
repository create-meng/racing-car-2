#!/usr/bin/env python3
"""Speak text through the serial voice module or cloud TTS fallback."""

from __future__ import annotations

import argparse
import sys

from voice_api.env_config import VoiceEnvConfig
from voice_api.voice_broadcast import VoiceBroadcastService


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Speak API/LLM text (see .env AUDIO_OUTPUT)',
    )
    parser.add_argument('text', nargs='?', help='Text to speak')
    parser.add_argument('-f', '--file', help='Read text from UTF-8 file')
    args = parser.parse_args(argv)

    text = (args.text or '').strip()
    if args.file:
        text = open(args.file, encoding='utf-8').read().strip()
    if not text:
        print('Usage: ros2 run voice_driver voice_speak_text -- "文字"', file=sys.stderr)
        return 1

    config = VoiceEnvConfig.from_env()
    mode = config.resolved_audio_output()
    print(f'AUDIO_OUTPUT={mode}  TTS={config.resolved_tts_provider()}')
    print(f'内容: {text[:200]}{"..." if len(text) > 200 else ""}')
    print('说明: mae01=串口语音模块 | alsa=百炼TTS需外接音箱')
    print('文档: src/racing/voice_driver/docs/VOICE_SETUP.md')
    print()

    service = VoiceBroadcastService(config)
    if service.speak_text(text):
        return 0

    return 2


if __name__ == '__main__':
    raise SystemExit(main())
