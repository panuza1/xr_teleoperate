const DEFAULTS = {
  arm: "G1_29", ee: "", input_mode: "hand", motion: true,
  display_mode: "immersive", img_server_ip: "192.168.123.164", image_transport: "zmq",
  frequency: 30, network_interface: "", headless: false, sim: false, ipc: false,
  affinity: false, record: false, task_dir: "./utils/data/", task_name: "pick cube",
  task_goal: "pick up cube.", task_desc: "task description",
  task_steps: "step1: do this; step2: do that;", extra_args: ""
};
const CHECKBOXES = new Set(["motion", "headless", "sim", "ipc", "affinity", "record"]);
const $ = id => document.getElementById(id);
let currentCommand = "";
let terminalText = "";
let terminalRunning = false;
let terminalConnected = false;
let previewGeneration = 0;
let terminalSocket = null;
let terminalDecoder = new TextDecoder();
const terminal = new Terminal({
  cursorBlink: true,
  convertEol: false,
  fontFamily: "'JetBrains Mono', ui-monospace, Menlo, monospace",
  fontSize: 12,
  lineHeight: 1.2,
  scrollback: 10000,
  theme: terminalTheme("light"),
});
const fitAddon = new FitAddon.FitAddon();
terminal.loadAddon(fitAddon);
terminal.open($("terminalOutput"));

function terminalTheme(theme) {
  if (theme === "mono") return {background: "#0b0b0b", foreground: "#f1f1f1", cursor: "#f1f1f1", selectionBackground: "#555555"};
  if (theme === "dark") return {background: "#07090b", foreground: "#a9d8d3", cursor: "#56d6c8", selectionBackground: "#235e58"};
  return {background: "#07090b", foreground: "#a9d8d3", cursor: "#0e9488", selectionBackground: "#235e58"};
}

function setTheme(theme) {
  if (!["light", "mono", "dark"].includes(theme)) theme = "light";
  document.documentElement.dataset.theme = theme;
  localStorage.setItem("xrTeleopTheme", theme);
  document.querySelectorAll(".theme-btn").forEach(button => button.classList.toggle("active", button.dataset.theme === theme));
  terminal.options.theme = terminalTheme(theme);
}

function getConfig() {
  return Object.fromEntries(Object.keys(DEFAULTS).map(name => [name,
    name === "frequency" ? Number($(name).value) : CHECKBOXES.has(name) ? $(name).checked : $(name).value
  ]));
}

function setConfig(config) {
  for (const [name, fallback] of Object.entries(DEFAULTS)) {
    const value = config[name] ?? fallback;
    if (CHECKBOXES.has(name)) $(name).checked = value === true;
    else $(name).value = value;
  }
}

function saveConfig() {
  localStorage.setItem("xrTeleopConfig", JSON.stringify(getConfig()));
}

function restoreConfig() {
  try {
    const saved = JSON.parse(localStorage.getItem("xrTeleopConfig") || "{}");
    setConfig({...DEFAULTS, ...saved});
  } catch (_) {
    setConfig(DEFAULTS);
  }
}

function message(text, error = false) {
  $("message").textContent = text;
  $("message").classList.toggle("error", error);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: {"Content-Type": "application/json", ...(options.headers || {})},
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `request failed (${response.status})`);
  return data;
}

async function updateCommand(changedId = null) {
  const generation = ++previewGeneration;
  saveConfig();
  try {
    const data = await api("/api/preview", {method: "POST", body: JSON.stringify(getConfig())});
    if (generation !== previewGeneration) return;
    currentCommand = data.command;
    $("cmd").textContent = currentCommand;
    $("cmdHint").textContent = changedId ? "updated" : "ready";
    message("Only start launches teleoperation. Status checks are read-only.");
  } catch (error) {
    if (generation !== previewGeneration) return;
    $("cmdHint").textContent = "invalid";
    message(error.message, true);
  }
}

function setStatus(id, state) {
  const element = $(id);
  element.textContent = state;
  element.className = `v ${["connected", "running"].includes(state) ? "good" : ["disconnected", "error"].includes(state) ? "bad" : "idle"}`;
}

function setTerminalState(data) {
  const wasRunning = terminalRunning;
  terminalRunning = data.running;
  const state = terminalConnected ? (data.running ? "running" : data.state) : "disconnected";
  $("terminalState").textContent = state;
  $("terminalPlaceholder").hidden = terminalConnected && data.running;
  $("terminalPlaceholder").textContent = terminalConnected ? `${data.state} · start a process, then click here to type` : "disconnected · terminal input disabled";
  $("exitStatus").textContent = data.exit_code ?? "—";
  if (data.running && !wasRunning) setTimeout(resizePty);
}

async function pollStatus() {
  const config = getConfig();
  const query = new URLSearchParams({img_server_ip: config.img_server_ip, image_transport: config.image_transport, ee: config.ee});
  try {
    const data = await api(`/api/status?${query}`);
    setStatus("s-xr", data.components.xr);
    setStatus("s-image", data.components.image);
    setStatus("s-g1", data.components.g1_dds);
    setStatus("s-inspire", data.components.inspire_dfx);
    setStatus("s-process", data.components.process);
    $("statusSummary").textContent = data.running ? "active" : data.state;
    $("statusPulse").classList.toggle("on", data.running);
    $("railState").textContent = data.state;
    $("btnStart").disabled = data.running;
    $("btnStop").disabled = !data.running;
    setTerminalState(data);
  } catch (error) {
    $("statusSummary").textContent = "error";
    terminalRunning = false;
    $("terminalState").textContent = "disconnected";
    $("terminalPlaceholder").hidden = false;
    $("terminalPlaceholder").textContent = "disconnected · terminal input disabled";
    message(error.message, true);
  }
}

function resizePty() {
  fitAddon.fit();
  if (terminalRunning && terminalSocket?.readyState === WebSocket.OPEN) {
    terminalSocket.send(JSON.stringify({type: "resize", columns: terminal.cols, rows: terminal.rows}));
  }
}

function connectTerminal() {
  if (terminalSocket && [WebSocket.OPEN, WebSocket.CONNECTING].includes(terminalSocket.readyState)) return;
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  terminalSocket = new WebSocket(`${scheme}://${location.host}/ws/terminal`);
  terminalSocket.binaryType = "arraybuffer";
  terminalSocket.addEventListener("open", () => {
    terminalConnected = true;
    terminal.reset();
    terminalText = "";
    terminalDecoder = new TextDecoder();
    resizePty();
    pollStatus();
  });
  terminalSocket.addEventListener("message", event => {
    if (typeof event.data === "string") {
      try {
        const control = JSON.parse(event.data);
        if (control.type === "error") message(control.message, true);
      } catch (_) {}
      return;
    }
    const data = new Uint8Array(event.data);
    terminalText = (terminalText + terminalDecoder.decode(data, {stream: true})).slice(-1000000);
    const viewport = terminal.buffer.active.viewportY;
    terminal.write(data, () => $("autoScroll").checked ? terminal.scrollToBottom() : terminal.scrollToLine(viewport));
  });
  terminalSocket.addEventListener("close", () => {
    terminalConnected = false;
    terminalRunning = false;
    $("terminalState").textContent = "disconnected";
    $("terminalPlaceholder").hidden = false;
    $("terminalPlaceholder").textContent = "disconnected · terminal input disabled";
    setTimeout(connectTerminal, 500);
  });
}

terminal.onData(data => {
  if (terminalRunning && terminal.element.contains(document.activeElement) && terminalSocket?.readyState === WebSocket.OPEN) {
    terminalSocket.send(JSON.stringify({type: "input", data}));
  }
});

document.querySelectorAll(".theme-btn").forEach(button => button.addEventListener("click", () => setTheme(button.dataset.theme)));
setTheme(localStorage.getItem("xrTeleopTheme") || "light");
restoreConfig();

for (const name of Object.keys(DEFAULTS)) {
  $(name).addEventListener("input", () => updateCommand(name));
  $(name).addEventListener("change", () => updateCommand(name));
}

$("advRow").addEventListener("click", () => {
  const open = $("advRow").getAttribute("aria-expanded") !== "true";
  $("advRow").setAttribute("aria-expanded", String(open));
  $("advSwitch").classList.toggle("on", open);
  $("advBody").classList.toggle("open", open);
});

document.querySelectorAll(".chip[data-preset]").forEach(button => button.addEventListener("click", () => {
  const preset = button.dataset.preset;
  if (preset === "baseline") setConfig(DEFAULTS);
  if (preset === "inspire") setConfig({...getConfig(), arm: "G1_29", ee: "inspire_dfx", input_mode: "hand"});
  if (preset === "hands") setConfig({...getConfig(), input_mode: "hand", motion: false});
  if (preset === "zmq") setConfig({...getConfig(), image_transport: "zmq"});
  if (preset === "webrtc") setConfig({...getConfig(), image_transport: "webrtc"});
  updateCommand();
}));

document.querySelectorAll(".nav a").forEach(link => link.addEventListener("click", () => {
  document.querySelectorAll(".nav a").forEach(item => item.classList.toggle("active", item === link));
}));

$("btnCopy").addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText(currentCommand);
    $("cmdHint").textContent = "copied";
    setTimeout(() => $("cmdHint").textContent = "ready", 1200);
  } catch (_) {
    message("Clipboard access was denied.", true);
  }
});

$("btnStart").addEventListener("click", async () => {
  const config = getConfig();
  if (config.motion && !window.confirm("Start teleoperation with motion control enabled?")) return;
  try {
    const data = await api("/api/start", {method: "POST", body: JSON.stringify(config)});
    currentCommand = data.command;
    $("cmd").textContent = currentCommand;
    message(`Teleoperation started (PID ${data.pid}).`);
    await pollStatus();
  } catch (error) {
    message(error.message, true);
  }
});

$("btnStop").addEventListener("click", async () => {
  try {
    const data = await api("/api/stop", {method: "POST", body: "{}"});
    message(data.exit_code === 0 ? "Teleoperation stopped cleanly." : `Teleoperation stopped (exit ${data.exit_code ?? "unknown"}).`, data.exit_code !== 0);
    await pollStatus();
  } catch (error) {
    message(error.message, true);
  }
});

$("btnClear").addEventListener("click", async () => {
  await api("/api/terminal/clear", {method: "POST", body: "{}"});
  terminalText = "";
  terminal.clear();
});

$("btnTerminalCopy").addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText(terminal.hasSelection() ? terminal.getSelection() : terminalText);
  } catch (_) {
    message("Clipboard access was denied.", true);
  }
});

$("terminalSurface").addEventListener("click", () => terminal.focus());

function setTerminalHeight(delta) {
  const surface = $("terminalSurface");
  surface.style.height = `${Math.max(220, Math.min(window.innerHeight * .75, surface.offsetHeight + delta))}px`;
  resizePty();
}
$("terminalLarger").addEventListener("click", () => setTerminalHeight(80));
$("terminalSmaller").addEventListener("click", () => setTerminalHeight(-80));
$("terminalMaximize").addEventListener("click", () => {
  const maximized = $("diagnostics").classList.toggle("maximized");
  document.body.classList.toggle("terminal-maximized", maximized);
  $("terminalMaximize").textContent = maximized ? "restore" : "max";
  $("terminalMaximize").setAttribute("aria-label", maximized ? "Restore terminal" : "Maximize terminal");
  setTimeout(resizePty);
});

$("terminalResize").addEventListener("pointerdown", event => {
  event.preventDefault();
  const startY = event.clientY;
  const startHeight = $("terminalSurface").offsetHeight;
  const move = moveEvent => {
    $("terminalSurface").style.height = `${Math.max(220, Math.min(window.innerHeight * .75, startHeight + moveEvent.clientY - startY))}px`;
  };
  const up = () => {
    window.removeEventListener("pointermove", move);
    window.removeEventListener("pointerup", up);
    resizePty();
  };
  window.addEventListener("pointermove", move);
  window.addEventListener("pointerup", up);
});
new ResizeObserver(resizePty).observe($("terminalSurface"));

connectTerminal();
updateCommand();
pollStatus();
setInterval(pollStatus, 2000);
