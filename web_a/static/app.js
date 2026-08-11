let selectedLogKey = null;
const previousReadiness = {};

async function requestJson(url, options = {}) {
  const res = await fetch(url, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.error || `请求失败: ${res.status}`);
  }
  return data;
}

function setBusy(button, busy) {
  if (!button) return;
  button.disabled = busy;
  button.dataset.oldText = button.dataset.oldText || button.textContent;
  button.textContent = busy ? "处理中" : button.dataset.oldText;
}

function showToast(text) {
  let toast = document.querySelector("#toast");
  if (!toast) {
    toast = document.createElement("div");
    toast.id = "toast";
    toast.className = "toast";
    document.body.appendChild(toast);
  }
  toast.textContent = text;
  toast.classList.add("show");
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => {
    toast.classList.remove("show");
  }, 5000);
}

function updateService(key, info) {
  const card = document.querySelector(`.service[data-key="${key}"]`);
  if (!card) return;
  const status = card.querySelector(".status");
  const stateText = {
    running: `运行中 PID ${info.pid}`,
    stopped: "未启动",
    exited: `已退出 ${info.returncode ?? ""}`,
  };
  if (info.state === "running" && info.ready_check) {
    const seconds = Number.isFinite(info.running_seconds)
      ? ` ${Math.floor(info.running_seconds)}s`
      : "";
    stateText.running =
      info.readiness === "ready"
        ? `已加载完成 PID ${info.pid}`
        : `启动中${seconds} PID ${info.pid}`;
  }
  status.textContent = stateText[info.state] || info.state;
  status.className = `status ${info.state} ${info.readiness || ""}`.trim();
  card.dataset.readiness = info.readiness || info.state;

  const previous = previousReadiness[key];
  if (info.ready_check && info.readiness === "ready" && previous === "starting") {
    showToast(info.ready_message || `${info.name} 已加载完成`);
  }
  previousReadiness[key] = info.readiness || info.state;
}

async function refreshStatus() {
  const data = await requestJson("/api/status");
  Object.entries(data).forEach(([key, info]) => updateService(key, info));
}

async function refreshLog() {
  if (!selectedLogKey) return;
  const data = await requestJson(`/api/log/${selectedLogKey}`);
  document.querySelector("#log").textContent = data.text || "暂无日志";
}

function setConfigMessage(text, kind = "") {
  const message = document.querySelector("#config-file-message");
  if (!message) return;
  message.textContent = text;
  message.className = `message ${kind}`.trim();
}

function currentConfigFileKey() {
  return document.querySelector("#config-file-select")?.value || "nav_config";
}

async function loadConfigFile(key = currentConfigFileKey()) {
  const editor = document.querySelector("#config-file-editor");
  setConfigMessage("正在读取文件...");
  const data = await requestJson(`/api/file/${encodeURIComponent(key)}`);
  document.querySelector("#config-file-path").textContent = data.path;
  editor.value = data.text || "";
  setConfigMessage(`已加载 ${data.name}`, "success");
}

async function saveConfigFile(key = currentConfigFileKey()) {
  const editor = document.querySelector("#config-file-editor");
  setConfigMessage("正在保存文件...");
  const data = await requestJson(`/api/file/${encodeURIComponent(key)}`, {
    method: "POST",
    body: JSON.stringify({ text: editor.value }),
  });
  const backupText = data.backup ? `，备份：${data.backup}` : "";
  setConfigMessage(`${data.message}${backupText}`, "success");
}

async function handleClick(event) {
  const button = event.target.closest("button");
  if (!button) return;

  const action = button.dataset.action;
  const key = button.dataset.key;

  try {
    if (action === "start" && key) {
      const card = button.closest(".service");
      const confirmText = card?.dataset.confirmStart;
      if (confirmText && !window.confirm(confirmText)) {
        return;
      }
    }

    setBusy(button, true);
    if (action === "start") {
      await requestJson(`/api/start/${key}`, { method: "POST" });
      selectedLogKey = key;
      document.querySelector("#log-title").textContent = key;
      await refreshLog();
    }
    if (action === "stop") {
      await requestJson(`/api/stop/${key}`, { method: "POST" });
    }
    if (action === "log") {
      selectedLogKey = key;
      document.querySelector("#log-title").textContent = key;
      await refreshLog();
    }
    if (action === "start-all") {
      await requestJson("/api/start_all", { method: "POST" });
    }
    if (action === "stop-all") {
      await requestJson("/api/stop_all", { method: "POST" });
    }
    if (action === "emergency-stop") {
      const data = await requestJson("/api/emergency_stop", { method: "POST" });
      showToast(`${data.message}：${data.topic}，连续 ${data.repeat} 次`);
    }
    if (action === "load-config-file") {
      await loadConfigFile();
    }
    if (action === "save-config-file") {
      await saveConfigFile();
    }
    await refreshStatus();
  } catch (error) {
    if (action === "load-config-file" || action === "save-config-file") {
      setConfigMessage(error.message, "error");
    }
    alert(error.message);
  } finally {
    setBusy(button, false);
  }
}

document.addEventListener("click", handleClick);
document.querySelector("#config-file-select")?.addEventListener("change", () => {
  loadConfigFile().catch((error) => setConfigMessage(error.message, "error"));
});
refreshStatus();
loadConfigFile().catch((error) => setConfigMessage(error.message, "error"));
setInterval(refreshStatus, 2000);
setInterval(refreshLog, 2000);
