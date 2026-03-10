const app = document.getElementById("app");
const fieldNodes = new Map();
const statusNodes = new Map();

function valueText(value) {
  return typeof value === "string" ? value : JSON.stringify(value, null, 2);
}

function setFieldValue(node, value, renderer) {
  if (!node) return;
  if (renderer === "text" || renderer === "value") node.textContent = valueText(value);
  else if (renderer === "list" || renderer === "logger" || renderer === "json") node.textContent = valueText(value);
  else node.textContent = valueText(value);
}

function renderMonitor(monitorId, monitor) {
  const wrap = document.createElement("section");
  wrap.className = "monitor";
  wrap.innerHTML = `<h2>${monitor.name}</h2><div class="status" id="status-${monitorId}"></div>`;
  const groups = new Map();

  for (const group of monitor.groups) {
    const section = document.createElement("div");
    section.className = "group";
    section.innerHTML = `<h3>${group.label}</h3>`;
    wrap.appendChild(section);
    groups.set(group.id, section);
  }

  for (const field of monitor.fields) {
    const section = groups.get(field.group) || wrap;
    const row = document.createElement("div");
    row.className = "item";
    const valueId = `${monitorId}:${field.id}`;
    let control = `<div id="${valueId}"></div>`;
    let action = "";
    if (field.mode === "rw") {
      control = `<input id="${valueId}-input" value='${String(field.value ?? "")}'>`;
      action = `<button data-set="${valueId}">Set</button>`;
    } else if (field.renderer === "logger" || field.renderer === "list" || field.renderer === "json") {
      control = `<pre id="${valueId}"></pre>`;
    } else {
      control = `<div id="${valueId}"></div>`;
    }
    row.innerHTML = `<div class="label">${field.label}</div>${control}<div>${action}</div>`;
    section.appendChild(row);
    fieldNodes.set(valueId, { renderer: field.renderer, mode: field.mode });
    setFieldValue(document.getElementById(valueId), field.value, field.renderer);
  }

  const actionsByGroup = new Map();
  for (const action of monitor.actions) {
    if (!actionsByGroup.has(action.group)) actionsByGroup.set(action.group, []);
    actionsByGroup.get(action.group).push(action);
  }

  for (const [groupId, actions] of actionsByGroup.entries()) {
    const section = groups.get(groupId) || wrap;
    const actionsWrap = document.createElement("div");
    actionsWrap.className = "actions";
    for (const action of actions) {
      const form = document.createElement("form");
      form.dataset.action = `${monitorId}:${action.id}`;
      let html = `<strong>${action.label}</strong>`;
      for (const argument of action.arguments) {
        const defaultValue = argument.default ?? "";
        html += `<input name="${argument.name}" placeholder="${argument.name} (${argument.type})" value='${String(defaultValue)}'>`;
      }
      html += `<button type="submit">Run</button>`;
      form.innerHTML = html;
      actionsWrap.appendChild(form);
    }
    section.appendChild(actionsWrap);
  }

  app.appendChild(wrap);
  statusNodes.set(monitorId, document.getElementById(`status-${monitorId}`));
}

async function loadStructure() {
  app.innerHTML = "";
  const response = await fetch("/api/structure");
  const payload = await response.json();
  for (const [monitorId, monitor] of Object.entries(payload.monitors)) renderMonitor(monitorId, monitor);
}

document.addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-set]");
  if (!button) return;
  const key = button.dataset.set;
  const [monitor, field] = key.split(":");
  const input = document.getElementById(`${key}-input`);
  const response = await fetch("/api/set", { method:"POST", headers:{ "Content-Type":"application/json" }, body: JSON.stringify({ monitor, field, value: input.value }) });
  const payload = await response.json();
  statusNodes.get(monitor).textContent = payload.ok ? "Field updated" : payload.error;
});

document.addEventListener("submit", async (event) => {
  const form = event.target.closest("form[data-action]");
  if (!form) return;
  event.preventDefault();
  const [monitor, action] = form.dataset.action.split(":");
  const args = Object.fromEntries(new FormData(form).entries());
  const response = await fetch("/api/action", { method:"POST", headers:{ "Content-Type":"application/json" }, body: JSON.stringify({ monitor, action, arguments: args }) });
  const payload = await response.json();
  statusNodes.get(monitor).textContent = payload.ok ? `Action result: ${valueText(payload.result)}` : payload.error;
});

const events = new EventSource("/events");
events.addEventListener("message", (event) => {
  const payload = JSON.parse(event.data);
  if (payload.type === "update") {
    for (const [fieldId, value] of Object.entries(payload.fields)) {
      const key = `${payload.monitor}:${fieldId}`;
      const node = document.getElementById(key);
      const meta = fieldNodes.get(key);
      if (meta?.mode === "rw") {
        const input = document.getElementById(`${key}-input`);
        if (input && document.activeElement !== input) input.value = value ?? "";
      } else {
        setFieldValue(node, value, meta?.renderer);
      }
    }
  } else if (payload.type === "structure") {
    loadStructure();
  } else if (payload.type === "action_result" || payload.type === "ack") {
    statusNodes.get(payload.monitor).textContent = valueText(payload.result);
  }
});

loadStructure();
