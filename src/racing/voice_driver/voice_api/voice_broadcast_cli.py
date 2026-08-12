#!/usr/bin/env python3
"""CLI: test image->LLM->TTS without ROS graph wiring."""

from __future__ import annotations

import argparse
import sys

from voice_api.env_config import VoiceEnvConfig
from voice_api.voice_broadcast import VoiceBroadcastService


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Image/text voice broadcast test CLI')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--image', help='Path to a local image file')
    group.add_argument('--text', help='Speak text directly without vision model')
    parser.add_argument(
        '--describe-only',
        action='store_true',
        help='Only run vision model and print text, do not speak',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = VoiceEnvConfig.from_env()
    service = VoiceBroadcastService(config)

    if args.text:
        if args.describe_only:
            print(args.text)
            return 0
        ok = service.speak_text(args.text)
        return 0 if ok else 2

    text = service.describe_image_path(args.image)
    if not text:
        print('Vision analysis failed', file=sys.stderr)
        return 2
    print(text)
    if args.describe_only:
        return 0
    return 0 if service.speak_text(text) else 3


if __name__ == '__main__':
    raise SystemExit(main())
