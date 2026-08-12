"""Play synthesized speech on the robot host."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any


class AudioPlayer:
    """Play mp3/wav files using common Linux CLI players."""

    def __init__(
        self,
        *,
        player: str = 'auto',
        alsa_device: str = 'plughw:0,0',
        logger: Any | None = None,
    ) -> None:
        self._player = player.lower()
        self._alsa_device = alsa_device
        self._logger = logger

    def prepare_alsa(self) -> None:
        """Raise ES8326 DAC playback volume before TTS."""
        if not shutil.which('amixer'):
            return
        card = '0'
        if self._alsa_device.startswith('plughw:'):
            card = self._alsa_device.split(':', 1)[1].split(',')[0]
        commands = [
            ['amixer', '-c', card, 'cset', 'numid=1', '100%'],
            ['amixer', '-c', card, 'sset', 'DAC', '100% unmute'],
            ['amixer', '-c', card, 'sset', 'Headphone', '100% unmute'],
            ['amixer', '-c', card, 'sset', 'Speaker', '100% unmute'],
        ]
        for command in commands:
            try:
                subprocess.run(
                    command,
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                return

    def play_file(self, audio_path: Path, *, delete_after: bool = True) -> bool:
        path = Path(audio_path)
        if not path.is_file():
            self._log_error(f'Audio file not found: {path}')
            return False

        commands = self._player_commands(path)
        if not commands:
            self._log_error('No audio player found. Install mpg123, ffplay, or aplay.')
            return False

        env = os.environ.copy()
        env['AUDIODEV'] = self._alsa_device

        for command in commands:
            try:
                subprocess.run(command, check=True, env=env)
                self._log_info(
                    f'Played audio with {" ".join(command[:3])} device={self._alsa_device}'
                )
                return True
            except (subprocess.CalledProcessError, FileNotFoundError) as exc:
                self._log_debug(f'Player failed ({command[0]}): {exc}')
                continue

        self._log_error('All audio players failed')
        return False

    def _player_commands(self, path: Path) -> list[list[str]]:
        suffix = path.suffix.lower()
        device = self._alsa_device

        if self._player == 'mpg123':
            return [['mpg123', '-q', '-a', device, str(path)]]
        if self._player == 'ffplay':
            return [self._ffplay_command(path)]
        if self._player == 'aplay':
            if suffix != '.wav':
                wav_path = self._convert_to_wav(path)
                if wav_path is None:
                    return []
                path = wav_path
            return [['aplay', '-q', '-D', device, str(path)]]

        commands: list[list[str]] = []
        if suffix == '.wav' and shutil.which('aplay'):
            commands.append(['aplay', '-q', '-D', device, str(path)])
        if suffix in {'.mp3', '.mpeg'}:
            if shutil.which('mpg123'):
                commands.append(['mpg123', '-q', '-a', device, str(path)])
            if shutil.which('ffplay'):
                commands.append(self._ffplay_command(path))
        if suffix == '.wav' and shutil.which('aplay'):
            pass  # already first
        elif shutil.which('aplay') and suffix in {'.mp3', '.mpeg'}:
            wav_path = self._convert_to_wav(path)
            if wav_path is not None:
                commands.append(['aplay', '-q', '-D', device, str(wav_path)])
        return commands

    def _ffplay_command(self, path: Path) -> list[str]:
        """Play via ffplay; AUDIODEV env selects ALSA device on this platform."""
        return [
            'ffplay',
            '-nodisp',
            '-autoexit',
            '-loglevel',
            'quiet',
            '-af',
            'volume=3',
            '-i',
            str(path),
        ]

    def _convert_to_wav(self, path: Path) -> Path | None:
        if not shutil.which('ffmpeg'):
            self._log_error('Need ffmpeg to convert mp3 to wav for aplay')
            return None
        wav_path = Path(tempfile.gettempdir()) / f'{path.stem}_{path.stat().st_mtime_ns}.wav'
        try:
            subprocess.run(
                ['ffmpeg', '-y', '-i', str(path), str(wav_path)],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return wav_path
        except subprocess.CalledProcessError as exc:
            self._log_error(f'ffmpeg convert failed: {exc}')
            return None

    def _log_info(self, message: str) -> None:
        if self._logger is not None:
            self._logger.info(message)

    def _log_debug(self, message: str) -> None:
        if self._logger is not None:
            self._logger.debug(message)

    def _log_error(self, message: str) -> None:
        if self._logger is not None:
            self._logger.error(message)
        else:
            print(f'[AudioPlayer] ERROR: {message}')
