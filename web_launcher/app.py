#!/usr/bin/env python3
import argparse
import os
import re
import signal
import shlex
import shutil
import subprocess
import time
from pathlib import Path
from threading import Lock

from flask import Flask, jsonify, render_template, request
import yaml


APP_DIR = Path(__file__).resolve().parent
LOG_DIR = APP_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)

ROS_WS = os.environ.get("ROS_WS", "/root/ros2_ws")
SETUP_FILE = os.environ.get("ROS_SETUP_FILE", f"{ROS_WS}/install/setup.bash")
NAV_CONFIG_PATH = os.environ.get("NAV_CONFIG_FILE", f"{ROS_WS}/src/racecar/config/nav.yaml")
NAV_CONFIG_FILE = Path(NAV_CONFIG_PATH)
DEFAULT_NAV_READY_PATTERNS = [
    "Server velocity_smoother connected with bond.",
]
NAV_READY_PATTERNS = [
    pattern.strip()
    for pattern in os.environ.get(
        "NAV_READY_PATTERNS", "|".join(DEFAULT_NAV_READY_PATTERNS)
    ).split("|")
    if pattern.strip()
]


def env_int(name, default, minimum):
    try:
        value = int(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, value)


def env_float(name, default, minimum):
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, value)


STOP_TOPIC = os.environ.get("STOP_TOPIC", "/cmd_vel")
STOP_REPEAT = env_int("STOP_REPEAT", 5, 1)
STOP_INTERVAL = env_float("STOP_INTERVAL", 0.1, 0.05)
STOP_MESSAGE = (
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, "
    "angular: {x: 0.0, y: 0.0, z: 0.0}}"
)


COMMANDS = {
    "base": {
        "name": "启动底盘",
        "group": "run",
        "cmd": f"source {SETUP_FILE} && ros2 launch origincar_base origincar_bringup.launch.py",
    },
    "lidar": {
        "name": "启动雷达",
        "group": "run",
        "cmd": f"cd {ROS_WS} && source {SETUP_FILE} && ros2 launch lslidar_driver lsn10_launch.py",
    },
    "nav_script": {
        "name": "启动导航脚本",
        "group": "optional",
        "cmd": f"cd {ROS_WS}/src/racecar/scripts/src && source {SETUP_FILE} && python3 h.py",
    },
    "amcl": {
        "name": "启动 AMCL",
        "group": "optional",
        "cmd": f"cd {ROS_WS} && source {SETUP_FILE} && ros2 run nav2_amcl amcl --ros-args --params-file {shlex.quote(NAV_CONFIG_PATH)}",
    },
    "qrcode": {
        "name": "启动二维码识别",
        "group": "run",
        "cmd": f"cd {ROS_WS}/src/racecar/scripts && source {SETUP_FILE} && python3 saoma.py",
    },
    "nav_launch": {
        "name": "启动 Navigation2",
        "group": "run",
        "cmd": f"cd {ROS_WS} && source {SETUP_FILE} && ros2 launch racecar Run_nav.launch.py",
        "ready_patterns": NAV_READY_PATTERNS,
        "ready_message": "Navigation2 已加载完成，可以开始导航",
    },
    "gmapping": {
        "name": "启动建图",
        "group": "map",
        "cmd": f"cd {ROS_WS} && source {SETUP_FILE} && ros2 launch slam_gmapping slam_gmapping.launch.py",
    },
    "rviz": {
        "name": "打开 RViz2",
        "group": "map",
        "cmd": "rviz2",
    },
    "save_map": {
        "name": "保存地图",
        "group": "map",
        "cmd": f"cd {ROS_WS} && bash save.sh",
        "oneshot": True,
    },
}

START_ALL = ["base", "lidar", "nav_launch", "qrcode"]

app = Flask(__name__)
lock = Lock()
processes = {}
ready_states = {}
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def build_shell_command(command):
    return ["bash", "-lc", command]


def build_stop_command():
    topic = shlex.quote(STOP_TOPIC)
    message = shlex.quote(STOP_MESSAGE)
    setup_file = shlex.quote(SETUP_FILE)
    interval = shlex.quote(str(STOP_INTERVAL))
    return (
        f"source {setup_file} && command -v ros2 >/dev/null || exit $?; "
        f"for i in $(seq 1 {STOP_REPEAT}); do "
        f"rc=0; "
        f"timeout 2s ros2 topic pub --once {topic} geometry_msgs/msg/Twist {message} || rc=$?; "
        f'if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ]; then exit "$rc"; fi; '
        f"sleep {interval}; "
        f"done"
    )


def is_running(key):
    proc = processes.get(key)
    return bool(proc and proc.poll() is None)


def find_ready_pattern(text, patterns):
    normalized = ANSI_ESCAPE_RE.sub("", text).lower()
    for pattern in patterns:
        if pattern.lower() in normalized:
            return pattern
    return None


def read_log_chunk(log_path, offset, limit=200000):
    if not log_path.exists():
        return "", offset
    size = log_path.stat().st_size
    if size <= offset:
        return "", offset
    with open(log_path, "rb") as log_file:
        log_file.seek(offset)
        data = log_file.read(limit)
        return data.decode("utf-8", errors="replace"), offset + len(data)


def refresh_ready_state(key):
    meta = COMMANDS[key]
    patterns = meta.get("ready_patterns")
    if not patterns:
        return None

    state = ready_states.setdefault(
        key,
        {
            "ready": False,
            "ready_at": None,
            "started_at": time.time(),
            "log_offset": 0,
            "scan_offset": 0,
            "matched_pattern": None,
        },
    )
    if state["ready"]:
        return state

    log_path = LOG_DIR / f"{key}.log"
    text, next_offset = read_log_chunk(
        log_path, state.get("scan_offset", state.get("log_offset", 0))
    )
    matched_pattern = find_ready_pattern(text, patterns)
    if matched_pattern:
        state["ready"] = True
        state["ready_at"] = time.time()
        state["matched_pattern"] = matched_pattern
    else:
        state["scan_offset"] = next_offset
    return state


def status_for(key):
    proc = processes.get(key)
    meta = COMMANDS[key]
    log_path = LOG_DIR / f"{key}.log"
    if proc is None:
        state = "stopped"
        pid = None
        returncode = None
    elif proc.poll() is None:
        state = "running"
        pid = proc.pid
        returncode = None
    else:
        state = "exited"
        pid = proc.pid
        returncode = proc.returncode

    info = {
        "key": key,
        "name": meta["name"],
        "group": meta.get("group", "run"),
        "state": state,
        "pid": pid,
        "returncode": returncode,
        "oneshot": meta.get("oneshot", False),
        "log": str(log_path),
    }
    if meta.get("ready_patterns"):
        info["ready_check"] = True
        if state == "running":
            ready_state = refresh_ready_state(key)
            info["ready"] = bool(ready_state and ready_state["ready"])
            info["readiness"] = "ready" if info["ready"] else "starting"
            info["ready_at"] = ready_state.get("ready_at") if ready_state else None
            info["started_at"] = ready_state.get("started_at") if ready_state else None
            info["running_seconds"] = (
                round(time.time() - ready_state["started_at"], 1)
                if ready_state and ready_state.get("started_at")
                else None
            )
            info["ready_message"] = (
                meta.get("ready_message")
                if info["ready"]
                else f"{meta['name']} 正在加载，请稍等"
            )
            info["matched_pattern"] = (
                ready_state.get("matched_pattern") if ready_state else None
            )
        else:
            info["ready"] = False
            info["readiness"] = None
            info["ready_at"] = None
            info["started_at"] = None
            info["running_seconds"] = None
            info["ready_message"] = None
            info["matched_pattern"] = None
    return info


def start_command(key):
    if key not in COMMANDS:
        raise KeyError(f"unknown command: {key}")

    with lock:
        if is_running(key):
            return status_for(key)

        log_path = LOG_DIR / f"{key}.log"
        log_offset = log_path.stat().st_size if log_path.exists() else 0
        log_file = open(log_path, "ab", buffering=0)
        header = f"\n\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} START {key}: {COMMANDS[key]['cmd']} =====\n"
        log_file.write(header.encode("utf-8"))

        proc = subprocess.Popen(
            build_shell_command(COMMANDS[key]["cmd"]),
            cwd=str(APP_DIR),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env=os.environ.copy(),
        )
        processes[key] = proc
        if COMMANDS[key].get("ready_patterns"):
            ready_states[key] = {
                "ready": False,
                "ready_at": None,
                "started_at": time.time(),
                "log_offset": log_offset,
                "scan_offset": log_offset,
                "matched_pattern": None,
            }
        else:
            ready_states.pop(key, None)
        return status_for(key)


def stop_command(key):
    with lock:
        proc = processes.get(key)
        if proc is None or proc.poll() is not None:
            return status_for(key)

        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)
        ready_states.pop(key, None)
        return status_for(key)


def read_tail(key, limit=6000):
    log_path = LOG_DIR / f"{key}.log"
    if not log_path.exists():
        return ""
    data = log_path.read_bytes()
    return data[-limit:].decode("utf-8", errors="replace")


def publish_stop():
    command = build_stop_command()
    result = subprocess.run(
        build_shell_command(command),
        cwd=str(APP_DIR),
        capture_output=True,
        text=True,
        timeout=max(5, STOP_REPEAT * 3),
        env=os.environ.copy(),
    )
    output = (result.stdout or "") + (result.stderr or "")
    if result.returncode != 0:
        raise RuntimeError(output.strip() or f"停车命令失败，返回码 {result.returncode}")
    return {
        "topic": STOP_TOPIC,
        "repeat": STOP_REPEAT,
        "interval": STOP_INTERVAL,
        "message": "已发布 0 速度停车指令",
        "output": output[-3000:],
    }


@app.route("/")
def index():
    grouped = {
        "run": [],
        "optional": [],
        "map": [],
    }
    for key, meta in COMMANDS.items():
        grouped.setdefault(meta.get("group", "run"), []).append({"key": key, **meta})
    return render_template(
        "index.html",
        grouped=grouped,
        start_all=START_ALL,
        ros_ws=ROS_WS,
        nav_config_path=str(NAV_CONFIG_FILE),
        stop_topic=STOP_TOPIC,
    )


@app.get("/api/status")
def api_status():
    return jsonify({key: status_for(key) for key in COMMANDS})


@app.post("/api/start/<key>")
def api_start(key):
    try:
        return jsonify(start_command(key))
    except KeyError as exc:
        return jsonify({"error": str(exc)}), 404
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@app.post("/api/stop/<key>")
def api_stop(key):
    try:
        return jsonify(stop_command(key))
    except KeyError as exc:
        return jsonify({"error": str(exc)}), 404
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@app.post("/api/start_all")
def api_start_all():
    results = []
    for key in START_ALL:
        results.append(start_command(key))
        time.sleep(1.0)
    return jsonify(results)


@app.post("/api/stop_all")
def api_stop_all():
    results = []
    for key in reversed(list(COMMANDS.keys())):
        if is_running(key):
            results.append(stop_command(key))
    return jsonify(results)


@app.post("/api/emergency_stop")
def api_emergency_stop():
    try:
        return jsonify(publish_stop())
    except subprocess.TimeoutExpired:
        return jsonify({"error": "停车命令超时，请检查 ROS2 环境和速度话题"}), 500
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@app.get("/api/log/<key>")
def api_log(key):
    if key not in COMMANDS:
        return jsonify({"error": f"unknown command: {key}"}), 404
    return jsonify({"key": key, "text": read_tail(key)})


@app.get("/api/nav_config")
def api_get_nav_config():
    if not NAV_CONFIG_FILE.exists():
        return jsonify({
            "error": f"导航参数文件不存在: {NAV_CONFIG_FILE}",
            "path": str(NAV_CONFIG_FILE),
        }), 404
    try:
        return jsonify({
            "path": str(NAV_CONFIG_FILE),
            "text": NAV_CONFIG_FILE.read_text(encoding="utf-8"),
        })
    except Exception as exc:
        return jsonify({"error": str(exc), "path": str(NAV_CONFIG_FILE)}), 500


@app.post("/api/nav_config")
def api_save_nav_config():
    data = request.get_json(silent=True) or {}
    text = data.get("text")
    if not isinstance(text, str):
        return jsonify({"error": "缺少 text 字段"}), 400
    if not NAV_CONFIG_FILE.exists():
        return jsonify({
            "error": f"导航参数文件不存在: {NAV_CONFIG_FILE}",
            "path": str(NAV_CONFIG_FILE),
        }), 404

    try:
        yaml.safe_load(text)
    except yaml.YAMLError as exc:
        return jsonify({"error": f"YAML 格式错误: {exc}"}), 400

    try:
        stamp = time.strftime("%Y%m%d-%H%M%S")
        backup_path = NAV_CONFIG_FILE.with_name(f"{NAV_CONFIG_FILE.name}.{stamp}.bak")
        shutil.copy2(NAV_CONFIG_FILE, backup_path)

        tmp_path = NAV_CONFIG_FILE.with_suffix(f"{NAV_CONFIG_FILE.suffix}.tmp")
        tmp_path.write_text(text, encoding="utf-8")
        tmp_path.replace(NAV_CONFIG_FILE)
        return jsonify({
            "path": str(NAV_CONFIG_FILE),
            "backup": str(backup_path),
            "message": "导航参数已保存",
        })
    except Exception as exc:
        return jsonify({"error": str(exc), "path": str(NAV_CONFIG_FILE)}), 500


def parse_args():
    parser = argparse.ArgumentParser(description="ROS2 web launcher for OriginCar")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    app.run(host=args.host, port=args.port, debug=args.debug)
