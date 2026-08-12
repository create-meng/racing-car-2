"""Unified logging and terminal output module for racing robot.

Usage::

    from racing_common.racing_logger import RacingLogger

    # In ROS node __init__:
    self.logger = RacingLogger(self, 'direct_inertial_test')

    # Later:
    self.logger.config('方向=顺时针 field_track=...')
    self.logger.segment('#3 type=move desc=rect_top L=2.59m')
    self.logger.feedback('开始执行，方向: 顺时针')
    self.logger.telemetry('wheel_odom', 't=123.45 mission=1 ...')
```

Output format: ``[TAG] message`` — written to both ROS logger and file.
"""

import os
import time
import sys

from .session_file_log import SessionFileLog


REAL_POSE_MIN_DELTA_M = 0.10
REAL_POSE_MAX_INTERVAL_SEC = 1.0


def should_report_real_pose(previous, previous_at, position, now, force=False,
                            min_delta_m=REAL_POSE_MIN_DELTA_M,
                            max_interval_sec=REAL_POSE_MAX_INTERVAL_SEC):
    """Keep position output readable while guaranteeing a one-second heartbeat."""
    if force or previous is None:
        return True
    moved = ((position[0] - previous[0]) ** 2 + (position[1] - previous[1]) ** 2) ** 0.5
    return moved >= min_delta_m or now - previous_at >= max_interval_sec


def terminal_write(message):
    """Write one approved operator message directly to the controlling TTY."""
    terminal_path = os.environ.get('RACING_OPERATOR_TTY', '/dev/tty')
    try:
        with open(terminal_path, 'w', encoding='utf-8', buffering=1) as terminal:
            terminal.write(str(message).rstrip() + '\n')
    except (OSError, IOError):
        print(message, file=sys.stdout, flush=True)


class RacingLogger:
    """Unified logger — writes [TAG] message to ROS logger + file."""

    def __init__(self, node, log_subdir='default', log_filename='latest.log',
                 session_title='session', defer_file=False,
                 telemetry_period_sec=0.50):
        self._node = node
        self._log_subdir = log_subdir
        self._log_filename = log_filename
        self._session_title = session_title
        self._file_log = None
        self._telemetry_period_sec = max(0.0, float(telemetry_period_sec))
        self._last_telemetry_sec = {}
        if not defer_file:
            self.start_session()

    def start_session(self):
        """Open a new file session, replacing the previous latest log."""
        if self._file_log is None:
            self._file_log = SessionFileLog(
                self._log_subdir, self._log_filename,
                session_title=self._session_title,
            )

    # ── path property for callers that need the log file path ──

    @property
    def path(self):
        return None if self._file_log is None else self._file_log.path

    # ── low-level write ──

    def _write(self, tag, message, level='info', file_only=False, terminal=False):
        formatted = f'[{tag}] {message}'
        if terminal and not file_only:
            self._terminal_write(formatted)
        if self._file_log is not None:
            self._file_log.write(formatted)

    @staticmethod
    def _terminal_write(message):
        terminal_write(message)

    # ── public: generic tag ──

    def info(self, tag, message, file_only=False):
        self._write(tag, message, 'info', file_only=file_only)

    def warn(self, tag, message):
        self._write(tag, message, 'warn')

    def error(self, tag, message):
        self._write(tag, message, 'error')

    # ── shortcut: Stage2 tags ──
    # 默认 file_only=True 的：配置、启动、里程锚点、计划、段完成（仅文件）
    # 默认终端显示的：段、进度、反馈、任务、避障、超时

    def config(self, message):
        self._write('CONFIG', message, 'info', file_only=True)

    def startup(self, message):
        self._write('STARTUP', message, 'info', file_only=True)

    def mission(self, message):
        self._write('MISSION', message, 'info', file_only=True)

    def segment(self, message):
        self._write('SEGMENT', message, 'info')

    def progress(self, message):
        self._write('PROGRESS', message, 'info', file_only=True)

    def feedback(self, message):
        self._write('FEEDBACK', message, 'info', file_only=True)

    def telemetry(self, reason, message):
        now = time.monotonic()
        last = self._last_telemetry_sec.get(reason, float('-inf'))
        if now - last < self._telemetry_period_sec:
            return
        self._last_telemetry_sec[reason] = now
        self._write('TELEM', f'{reason} | {message}', 'info', file_only=True)

    def odom_wheel(self, message):
        self._write('ODOM_WHEEL', message, 'info', file_only=True)

    def odom_anchor(self, message):
        self._write('ODOM_ANCHOR', message, 'info', file_only=True)

    def plan(self, message):
        self._write('PLAN', message, 'info', file_only=True)

    def segment_done(self, message):
        self._write('SEGMENT_DONE', message, 'info', file_only=True)

    def corner_avoid(self, message):
        self._write('CORNER_AVOID', message, 'info', file_only=True)

    def timeout(self, message):
        self._write('TIMEOUT', message, 'warn', file_only=True)

    def task(self, message):
        """Write a task event and show the same concise event in the terminal."""
        self._write('TASK', message, 'info', terminal=True)

    def real_pose(self, x, y, source='map_tf', force=False,
                  min_delta_m=REAL_POSE_MIN_DELTA_M,
                  max_interval_sec=REAL_POSE_MAX_INTERVAL_SEC):
        """Report the robot's measured map position, never a controller target."""
        position = (float(x), float(y))
        previous = getattr(self, '_last_real_pose', None)
        now = time.monotonic()
        previous_at = getattr(self, '_last_real_pose_at', float('-inf'))
        if not should_report_real_pose(
            previous, previous_at, position, now, force,
            min_delta_m=min_delta_m,
            max_interval_sec=max_interval_sec,
        ):
            return
        self._last_real_pose = position
        self._last_real_pose_at = now
        self._write(
            'POSE_REAL',
            f'real_map=({position[0]:.3f},{position[1]:.3f}) source={source}',
            'info',
            terminal=True,
        )

    def controller_pose(self, x, y, source):
        """Record an internal navigation pose without exposing it in terminal."""
        self._write(
            'POSE_CTRL',
            f'controller_map=({float(x):.3f},{float(y):.3f}) source={source}',
            'info',
            file_only=True,
        )

    def target_pose(self, x, y, name='target'):
        self._write(
            'POSE_TARGET',
            f'target_map=({float(x):.3f},{float(y):.3f}) name={name}',
            'info',
            file_only=True,
        )

    # ── shortcuts: Stage3 tags ──

    def stage3_ready(self, message):
        self._write('STAGE3', message, 'info')

    def stage3_plan(self, message):
        self._write('STAGE3_PLAN', message, 'info')

    def stage3_waypoint(self, message):
        self._write('STAGE3_WP', message, 'info')

    def stage3_param(self, message):
        self._write('STAGE3_PARAM', message, 'error')

    # ── lifecycle ──

    def close(self):
        if self._file_log is not None:
            self._file_log.close()
            self._file_log = None
