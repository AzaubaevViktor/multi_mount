const versionNode = document.getElementById("version");
const streamNode = document.getElementById("stream");
const cardsNode = document.getElementById("cards");

function renderSnapshot(snapshot) {
  versionNode.textContent = String(snapshot.version);
  cardsNode.innerHTML = "";

  for (const [name, objectSnapshot] of Object.entries(snapshot.objects)) {
    const card = document.createElement("article");
    card.className = "card";

    const title = document.createElement("h2");
    title.textContent = name;
    card.appendChild(title);

    for (const group of objectSnapshot.groups) {
      const groupNode = document.createElement("section");
      groupNode.className = "group";

      const groupTitle = document.createElement("h3");
      groupTitle.textContent = group.name;
      groupNode.appendChild(groupTitle);

      for (const field of group.fields) {
        const row = document.createElement("div");
        row.className = "row";
        row.innerHTML = `<span>${field.name}</span><strong>${field.value}${field.unit || ""}</strong>`;
        groupNode.appendChild(row);
      }

      card.appendChild(groupNode);
    }

    cardsNode.appendChild(card);
  }
}

async function bootstrap() {
  const response = await fetch("/api/snapshot");
  renderSnapshot(await response.json());
}

function connectEvents() {
  const source = new EventSource("/events");
  source.onopen = () => {
    streamNode.textContent = "live";
  };
  source.onerror = () => {
    streamNode.textContent = "reconnecting";
  };
  source.onmessage = (event) => {
    renderSnapshot(JSON.parse(event.data));
  };
}

bootstrap().then(connectEvents).catch(() => {
  streamNode.textContent = "error";
});
