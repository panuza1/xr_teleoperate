const $ = id => document.getElementById(id);
const GROUP_IDS = {"Robot": "robot", "Vision / Streaming": "camera", "Recording": "recordings"};
let parameters = [], defaults = {}, presets = {}, values = {};
let included = new Set(), extraArgs = "", parameterMode = "basic", schemaReady = false;
let currentCommand = "", terminalText = "", terminalRunning = false, terminalConnected = false;
let previewGeneration = 0, terminalSocket = null, terminalDecoder = new TextDecoder();

const terminal = new Terminal({
  cursorBlink: true, convertEol: false, fontFamily: "'JetBrains Mono', ui-monospace, Menlo, monospace",
  fontSize: 12, lineHeight: 1.2, scrollback: 10000, theme: terminalTheme("light"),
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

function sameValue(value, fallback, type) {
  if (fallback === null && value === "") return true;
  if (type === "float") return value !== "" && Number(value) === Number(fallback);
  return value === fallback;
}

function getConfig() { return {values: {...values}, included: [...included], extra_args: extraArgs}; }
function saveConfig() { localStorage.setItem("xrTeleopConfig", JSON.stringify(getConfig())); }

function applyConfiguration(config) {
  values = {...defaults, ...(config.values || {})};
  included = new Set((config.included || []).filter(name => parameters.some(spec => spec.dest === name)));
  extraArgs = typeof config.extra_args === "string" ? config.extra_args : "";
  renderParameters();
  updateCommand();
}

function restoreConfiguration(baseline) {
  try {
    const saved = JSON.parse(localStorage.getItem("xrTeleopConfig") || "null");
    applyConfiguration(saved && saved.values ? saved : baseline);
  } catch (_) { applyConfiguration(baseline); }
}

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function modified(spec) { return !sameValue(values[spec.dest] ?? "", spec.default, spec.type); }

function updateFieldState(spec) {
  const field = $(`field-${spec.dest}`);
  if (!field) return;
  const changed = modified(spec);
  field.classList.toggle("modified", changed);
  field.querySelector(".modified-badge").hidden = !changed;
  field.querySelector(".reset-field").hidden = !changed && !included.has(spec.dest);
}

function createField(spec) {
  const field = element("div", `field parameter-field${spec.action === "store_true" ? " boolean-field" : ""}`);
  field.id = `field-${spec.dest}`;
  field.dataset.parameter = spec.dest;
  const heading = element("div", "field-heading");
  const label = element("label", null, spec.label);
  label.htmlFor = spec.dest;
  const meta = element("span", "field-meta");
  const badge = element("span", "modified-badge", "modified");
  badge.hidden = true;
  const reset = element("button", "reset-field", "reset");
  reset.type = "button";
  reset.dataset.reset = spec.dest;
  reset.hidden = true;
  meta.append(badge, reset);
  heading.append(label, meta);
  let control;
  if (spec.action === "store_true") {
    const toggle = element("label", "toggle-control");
    control = document.createElement("input");
    control.type = "checkbox";
    control.className = "toggle-input";
    control.checked = values[spec.dest] === true;
    const switchNode = element("span", "switch");
    switchNode.setAttribute("aria-hidden", "true");
    toggle.append(control, switchNode);
    field.append(heading, toggle);
  } else {
    control = document.createElement("input");
    control.type = spec.type === "float" ? "number" : "text";
    if (spec.type === "float") control.step = "any";
    control.value = values[spec.dest] ?? "";
    if (spec.choices) {
      const list = document.createElement("datalist");
      list.id = `choices-${spec.dest}`;
      spec.choices.forEach(choice => {
        const option = document.createElement("option");
        option.value = choice;
        list.append(option);
      });
      control.setAttribute("list", list.id);
      field.append(heading, control, list);
    } else field.append(heading, control);
  }
  control.id = spec.dest;
  control.dataset.input = spec.dest;
  control.setAttribute("aria-describedby", `help-${spec.dest} error-${spec.dest}`);
  const details = element("div", "field-details");
  const flag = element("code", "cli-flag", spec.flag);
  const help = element("span", "field-help", spec.help || "No CLI help text.");
  help.id = `help-${spec.dest}`;
  details.append(flag, help);
  const error = element("div", "field-error");
  error.id = `error-${spec.dest}`;
  error.setAttribute("role", "alert");
  field.append(details, error);
  const changed = modified(spec);
  field.classList.toggle("modified", changed);
  badge.hidden = !changed;
  reset.hidden = !changed && !included.has(spec.dest);
  return field;
}

function createExtraField() {
  const field = element("div", "field parameter-field");
  field.id = "field-extra_args";
  const heading = element("div", "field-heading");
  const label = element("label", null, "Extra CLI arguments");
  label.htmlFor = "extra_args";
  const reset = element("button", "reset-field", "reset");
  reset.type = "button";
  reset.dataset.reset = "extra_args";
  reset.hidden = !extraArgs;
  heading.append(label, reset);
  const input = document.createElement("input");
  input.id = "extra_args";
  input.dataset.input = "extra_args";
  input.value = extraArgs;
  input.placeholder = "--future-option value";
  input.setAttribute("aria-describedby", "help-extra_args error-extra_args");
  const details = element("div", "field-details");
  const flag = element("code", "cli-flag", "pass-through");
  const help = element("span", "field-help", "Additional arguments are tokenized and appended without shell execution.");
  help.id = "help-extra_args";
  details.append(flag, help);
  const error = element("div", "field-error");
  error.id = "error-extra_args";
  error.setAttribute("role", "alert");
  field.append(heading, input, details, error);
  return field;
}

function renderParameters() {
  const container = $("parameterSections");
  container.replaceChildren();
  const visible = parameters.filter(spec => parameterMode === "all" || spec.level === parameterMode);
  const groups = new Map();
  visible.forEach(spec => {
    if (!groups.has(spec.group)) groups.set(spec.group, []);
    groups.get(spec.group).push(spec);
  });
  if (parameterMode !== "basic") groups.set("Extra arguments", []);
  for (const [name, specs] of groups) {
    const section = element("section", "block parameter-section");
    section.id = GROUP_IDS[name] || `group-${name.toLowerCase().replace(/[^a-z0-9]+/g, "-")}`;
    const grid = element("div", "parameter-grid");
    specs.forEach(spec => grid.append(createField(spec)));
    if (name === "Extra arguments") grid.append(createExtraField());
    section.append(element("div", "block-label", name.toLowerCase()), grid);
    container.append(section);
  }
}

function clearFieldErrors() {
  document.querySelectorAll(".parameter-field.invalid").forEach(field => field.classList.remove("invalid"));
  document.querySelectorAll(".field-error").forEach(error => error.textContent = "");
}

function showFieldErrors(errors = {}) {
  clearFieldErrors();
  for (const [name, text] of Object.entries(errors)) {
    const field = $(`field-${name}`), output = $(`error-${name}`);
    if (field) field.classList.add("invalid");
    if (output) output.textContent = text;
  }
}

function message(text, error = false) {
  $("message").textContent = text;
  $("message").classList.toggle("error", error);
}

async function api(path, options = {}) {
  const response = await fetch(path, {...options, headers: {"Content-Type": "application/json", ...(options.headers || {})}});
  const data = await response.json();
  if (!response.ok) {
    const error = new Error(data.error || `request failed (${response.status})`);
    error.fieldErrors = data.field_errors || {};
    throw error;
  }
  return data;
}

async function updateCommand(changedId = null) {
  if (!schemaReady) return;
  const generation = ++previewGeneration;
  saveConfig();
  try {
    const data = await api("/api/preview", {method: "POST", body: JSON.stringify(getConfig())});
    if (generation !== previewGeneration) return;
    clearFieldErrors();
    currentCommand = data.command;
    $("cmd").textContent = currentCommand;
    $("cmdHint").textContent = changedId ? "updated" : "ready";
    message("Only start launches teleoperation. Status checks are read-only.");
  } catch (error) {
    if (generation !== previewGeneration) return;
    showFieldErrors(error.fieldErrors);
    const details = Object.entries(error.fieldErrors || {}).map(([name, text]) => `${name}: ${text}`).join("\n");
    $("cmd").textContent = `Cannot generate command:\n${details || error.message}`;
    $("cmdHint").textContent = "invalid";
    message(error.message, true);
  }
}

function setStatus(id, state) {
  const node = $(id);
  node.textContent = state;
  node.className = `v ${["connected", "running"].includes(state) ? "good" : ["disconnected", "error"].includes(state) ? "bad" : "idle"}`;
}

function setTerminalState(data) {
  const wasRunning = terminalRunning;
  terminalRunning = data.running;
  $("terminalState").textContent = terminalConnected ? (data.running ? "running" : data.state) : "disconnected";
  $("terminalPlaceholder").hidden = terminalConnected && data.running;
  $("terminalPlaceholder").textContent = terminalConnected ? `${data.state} · start a process, then click here to type` : "disconnected · terminal input disabled";
  $("exitStatus").textContent = data.exit_code ?? "—";
  if (data.running && !wasRunning) setTimeout(resizePty);
}

async function pollStatus() {
  if (!schemaReady) return;
  const query = new URLSearchParams({img_server_ip: values.img_server_ip ?? "", image_transport: values.image_transport ?? "zmq", ee: values.ee ?? ""});
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

$("parameterSections").addEventListener("input", event => {
  const name = event.target.dataset.input;
  if (!name) return;
  if (name === "extra_args") {
    extraArgs = event.target.value;
    event.target.closest(".parameter-field").querySelector(".reset-field").hidden = !extraArgs;
  } else {
    const spec = parameters.find(item => item.dest === name);
    values[name] = spec.action === "store_true" ? event.target.checked : event.target.value;
    if (sameValue(values[name], spec.default, spec.type)) included.delete(name); else included.add(name);
    updateFieldState(spec);
  }
  showFieldErrors({});
  updateCommand(name);
});

$("parameterSections").addEventListener("click", event => {
  const name = event.target.dataset.reset;
  if (!name) return;
  if (name === "extra_args") extraArgs = "";
  else { values[name] = defaults[name]; included.delete(name); }
  renderParameters();
  updateCommand(name);
});

document.querySelectorAll(".mode-btn").forEach(button => button.addEventListener("click", () => {
  parameterMode = button.dataset.mode;
  document.querySelectorAll(".mode-btn").forEach(item => {
    const active = item === button;
    item.classList.toggle("active", active);
    item.setAttribute("aria-selected", String(active));
  });
  renderParameters();
}));

$("resetAll").addEventListener("click", () => applyConfiguration({values: defaults, included: [], extra_args: ""}));
$("presetButtons").addEventListener("click", event => {
  const preset = presets[event.target.dataset.preset];
  if (preset) applyConfiguration(preset);
});

document.querySelectorAll(".nav a").forEach(link => link.addEventListener("click", event => {
  document.querySelectorAll(".nav a").forEach(item => item.classList.toggle("active", item === link));
  if (!["config", "diagnostics"].includes(link.dataset.target) && !$(link.dataset.target)) {
    event.preventDefault();
    parameterMode = "all";
    document.querySelectorAll(".mode-btn").forEach(item => {
      const active = item.dataset.mode === "all";
      item.classList.toggle("active", active);
      item.setAttribute("aria-selected", String(active));
    });
    renderParameters();
    $(link.dataset.target)?.scrollIntoView();
  }
}));

$("btnCopy").addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText(currentCommand);
    $("cmdHint").textContent = "copied";
    setTimeout(() => $("cmdHint").textContent = "ready", 1200);
  } catch (_) { message("Clipboard access was denied.", true); }
});

$("btnStart").addEventListener("click", async () => {
  if (values.motion === true && !window.confirm("Start teleoperation with motion control enabled?")) return;
  try {
    const data = await api("/api/start", {method: "POST", body: JSON.stringify(getConfig())});
    clearFieldErrors();
    currentCommand = data.command;
    $("cmd").textContent = currentCommand;
    message(`Teleoperation started (PID ${data.pid}).`);
    await pollStatus();
  } catch (error) { showFieldErrors(error.fieldErrors); message(error.message, true); }
});

$("btnStop").addEventListener("click", async () => {
  try {
    const data = await api("/api/stop", {method: "POST", body: "{}"});
    message(data.exit_code === 0 ? "Teleoperation stopped cleanly." : `Teleoperation stopped (exit ${data.exit_code ?? "unknown"}).`, data.exit_code !== 0);
    await pollStatus();
  } catch (error) { message(error.message, true); }
});

$("btnClear").addEventListener("click", async () => {
  await api("/api/terminal/clear", {method: "POST", body: "{}"});
  terminalText = "";
  terminal.clear();
});

$("btnTerminalCopy").addEventListener("click", async () => {
  try { await navigator.clipboard.writeText(terminal.hasSelection() ? terminal.getSelection() : terminalText); }
  catch (_) { message("Clipboard access was denied.", true); }
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
  const startY = event.clientY, startHeight = $("terminalSurface").offsetHeight;
  const move = moveEvent => { $("terminalSurface").style.height = `${Math.max(220, Math.min(window.innerHeight * .75, startHeight + moveEvent.clientY - startY))}px`; };
  const up = () => {
    window.removeEventListener("pointermove", move);
    window.removeEventListener("pointerup", up);
    resizePty();
  };
  window.addEventListener("pointermove", move);
  window.addEventListener("pointerup", up);
});
new ResizeObserver(resizePty).observe($("terminalSurface"));

async function initialize() {
  try {
    const schema = await api("/api/schema");
    parameters = schema.parameters;
    defaults = schema.defaults;
    presets = schema.presets;
    for (const [name, preset] of Object.entries(presets)) {
      const button = element("button", "chip", preset.label || name);
      button.type = "button";
      button.dataset.preset = name;
      $("presetButtons").append(button);
    }
    schemaReady = true;
    restoreConfiguration(schema.baseline);
    await pollStatus();
    setInterval(pollStatus, 2000);
  } catch (error) { message(`Unable to load CLI parameter schema: ${error.message}`, true); }
}

connectTerminal();
initialize();
