import shlex

from launch.actions import ExecuteProcess


def startup_status(label, node_name):
    """Print success once direct DDS discovery confirms the node exists.

    There is deliberately no wall-clock deadline: slow hardware and DDS
    discovery are not startup failures while the target process is alive.
    """
    ready_message = f'[STARTUP] {label} 启动成功'
    quoted_node = shlex.quote(node_name)
    quoted_ready = shlex.quote(ready_message)
    command = f'''
while true; do
  if ros2 node list --no-daemon --spin-time 1 2>/dev/null | grep -Fqx -- {quoted_node}; then
    terminal_path="${{RACING_OPERATOR_TTY:-/dev/tty}}"
    printf '%s\\n' {quoted_ready} > "$terminal_path" 2>/dev/null || true
    exit 0
  fi
  sleep 0.25
done
'''
    return ExecuteProcess(
        cmd=['bash', '-c', command],
        output='own_log',
    )
