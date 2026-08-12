"""Session file logger with one production directory per competition run."""

import os
import fcntl
from datetime import datetime


def resolve_workspace_root():
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, 'src', 'racing')):
        return cwd
    dev_ws = os.environ.get('DEV_WS', '').strip()
    if dev_ws and os.path.isdir(dev_ws):
        return dev_ws
    return cwd


def resolve_runtime_log_dir(subdir, workspace_root=None):
    """Return a run-scoped directory when production provides one."""
    root = workspace_root or resolve_workspace_root()
    session_root = os.environ.get('RACING_SESSION_ROOT', '').strip()
    return os.path.join(session_root, subdir) if session_root else os.path.join(root, 'log', subdir)


class SessionFileLog:
    """Append-only session log with a stable ``latest.log`` convenience link."""

    def __init__(
        self,
        subdir,
        filename='latest.log',
        workspace_root=None,
        session_title='session',
    ):
        root = workspace_root or resolve_workspace_root()
        self.session_root = os.environ.get('RACING_SESSION_ROOT', '').strip()
        self.log_dir = resolve_runtime_log_dir(subdir, root)
        os.makedirs(self.log_dir, exist_ok=True)
        session_id = os.environ.get('COMPETITION_SESSION_ID', '').strip()
        if not session_id:
            session_id = os.path.basename(self.session_root.rstrip(os.sep)) or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.session_id = session_id
        self.session_dir = self.log_dir if self.session_root else os.path.join(self.log_dir, session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        self.path = os.path.join(self.session_dir, filename)
        latest_path = os.path.join(self.log_dir, filename)
        relative_target = filename if self.session_root else os.path.join(session_id, filename)
        if self.session_root:
            latest_session = os.path.join(os.path.dirname(self.session_root), 'latest')
            try:
                if os.path.lexists(latest_session):
                    os.unlink(latest_session)
                os.symlink(os.path.basename(self.session_root), latest_session)
            except OSError:
                pass
        if latest_path != self.path:
            try:
                if os.path.lexists(latest_path) and not os.path.islink(latest_path):
                    legacy_path = os.path.join(
                        self.log_dir,
                        f'legacy_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{filename}',
                    )
                    os.replace(latest_path, legacy_path)
                if os.path.lexists(latest_path):
                    os.unlink(latest_path)
                os.symlink(relative_target, latest_path)
            except OSError:
                # Logging must never prevent a control node from starting on a
                # read-only or unusual filesystem.  The session file remains the
                # authoritative copy even if the convenience link cannot update.
                pass
        self._file = open(self.path, 'w', encoding='utf-8')
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.write(f'=== {session_title} {stamp} ===')
        self.write(f'log_path: {self.path}')

    def write(self, line):
        if self._file is None:
            return
        # Other Stage1 nodes may append diagnostics to this file.  Re-seek under
        # an advisory lock so this persistent handle never overwrites them.
        fcntl.flock(self._file.fileno(), fcntl.LOCK_EX)
        try:
            self._file.seek(0, os.SEEK_END)
            self._file.write(str(line).rstrip() + '\n')
            self._file.flush()
        finally:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)

    def close(self):
        if self._file is None:
            return
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.write(f'=== session closed {stamp} ===')
        self._file.close()
        self._file = None
