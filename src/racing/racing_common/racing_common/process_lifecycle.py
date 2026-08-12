"""Process lifetime helpers for nodes launched by the competition supervisor."""

import ctypes
import os
import signal
import threading
import time


_LIBC = ctypes.CDLL(None)


def _process_start_ticks(pid):
    """Return Linux /proc start ticks, or None when the process is absent."""
    try:
        with open(f'/proc/{pid}/stat', 'r', encoding='utf-8') as stream:
            # The command name can contain spaces, so field counting begins
            # only after the final closing parenthesis.
            fields = stream.read().rsplit(')', 1)[1].split()
        return fields[19]
    except (IndexError, OSError):
        return None


def _watch_supervisor(pid, start_ticks, signal_number):
    """End this node if the Supervisor from this session no longer exists."""
    while True:
        if _process_start_ticks(pid) != start_ticks:
            try:
                os.kill(os.getpid(), signal_number)
            except OSError:
                pass
            # A custom SIGTERM handler may only publish the final stop and
            # fail to unwind a blocked ROS spin. Do not let that leave an
            # orphan phase node behind after the Supervisor is gone.
            time.sleep(0.5)
            os._exit(0)
        time.sleep(0.2)


def install_parent_death_signal(signal_number=signal.SIGTERM):
    """Terminate this node when its launch parent or Supervisor disappears."""
    parent_pid = os.getppid()
    installed = False
    try:
        # Linux PR_SET_PDEATHSIG = 1.
        if _LIBC.prctl(1, int(signal_number)) != 0:
            installed = False
        else:
            installed = True
            # Cover the fork/exec race where the parent died before prctl().
            if os.getppid() != parent_pid:
                os.kill(os.getpid(), signal_number)
    except (AttributeError, OSError):
        pass

    # ``ros2 launch`` is an intermediate parent.  PR_SET_PDEATHSIG observes
    # that wrapper, but its child nodes can survive after it exits.  The
    # Supervisor identity is injected for production stage processes so the
    # final node also exits when the owning competition session ends.
    raw_pid = os.environ.get('COMPETITION_SUPERVISOR_PID')
    expected_start_ticks = os.environ.get('COMPETITION_SUPERVISOR_START_TICKS')
    try:
        supervisor_pid = int(raw_pid) if raw_pid else 0
    except ValueError:
        supervisor_pid = 0
    if supervisor_pid > 0 and expected_start_ticks:
        threading.Thread(
            target=_watch_supervisor,
            args=(supervisor_pid, expected_start_ticks, signal_number),
            name='CompetitionSupervisorWatch',
            daemon=True,
        ).start()
    return installed
