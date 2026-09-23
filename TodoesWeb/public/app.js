"use strict";

const TODO_MAX_ITEMS = 80;
const TODO_TITLE_MAX_LEN = 80;
const TODO_DISPLAY_ACTIVE = 0;
const TODO_DISPLAY_DONE = 1;
const STORAGE_KEY = "todoes-web-state-v1";
const FIREBASE_COLLECTION = "todoLists";
const FIREBASE_DOC_ID = "main";

const state = {
  items: [],
  nextId: 1,
  editorParentId: 0,
  editorEditId: 0,
};

const sync = {
  mode: "local",
  status: "local",
  docRef: null,
  saveTimer: 0,
  applyingRemote: false,
};

const elements = {
  statusLabel: document.getElementById("statusLabel"),
  addTodoButton: document.getElementById("addTodoButton"),
  todoList: document.getElementById("todoList"),
  editorOverlay: document.getElementById("editorOverlay"),
  editorPanel: document.getElementById("editorPanel"),
  editorTitle: document.getElementById("editorTitle"),
  editorTextarea: document.getElementById("editorTextarea"),
  cancelButton: document.getElementById("cancelButton"),
};

function cleanTitle(title) {
  return String(title || "").trim().slice(0, TODO_TITLE_MAX_LEN);
}

function findItem(id) {
  return state.items.find((item) => item.id === id) || null;
}

function hasChildren(parentId) {
  return state.items.some((item) => item.parentId === parentId);
}

function isDescendantOf(itemId, ancestorId) {
  let item = findItem(itemId);
  let depthGuard = 0;

  while (item && item.parentId !== 0 && depthGuard < TODO_MAX_ITEMS) {
    if (item.parentId === ancestorId) {
      return true;
    }

    item = findItem(item.parentId);
    depthGuard += 1;
  }

  return false;
}

function normalizeItem(raw, maxId) {
  const id = Number(raw.id);
  if (!Number.isSafeInteger(id) || id <= 0) {
    return null;
  }

  const title = cleanTitle(raw.title);
  if (!title) {
    return null;
  }

  const display = raw.display === TODO_DISPLAY_DONE ? TODO_DISPLAY_DONE : TODO_DISPLAY_ACTIVE;

  return {
    id,
    parentId: Number.isSafeInteger(Number(raw.parentId)) ? Number(raw.parentId) : 0,
    title,
    done: Boolean(raw.done),
    expanded: raw.expanded !== false,
    display,
    maxId: Math.max(maxId, id),
  };
}

function importState(data) {
  if (!data || !Array.isArray(data.items)) {
    return false;
  }

  let maxId = 0;
  const imported = [];

  for (const raw of data.items.slice(0, TODO_MAX_ITEMS)) {
    const item = normalizeItem(raw, maxId);
    if (item) {
      maxId = item.maxId;
      delete item.maxId;
      imported.push(item);
    }
  }

  const ids = new Set(imported.map((item) => item.id));
  for (const item of imported) {
    if (item.parentId !== 0 && !ids.has(item.parentId)) {
      item.parentId = 0;
    }
  }

  state.items = imported;
  state.nextId = Number.isSafeInteger(Number(data.nextId)) ? Number(data.nextId) : maxId + 1;
  if (state.nextId <= maxId) {
    state.nextId = maxId + 1;
  }

  return true;
}

function exportState() {
  return {
    nextId: state.nextId,
    items: state.items.map((item) => ({
      id: item.id,
      parentId: item.parentId,
      title: item.title,
      done: item.done,
      expanded: item.expanded,
      display: item.display,
    })),
  };
}

function loadLocalState() {
  try {
    importState(JSON.parse(localStorage.getItem(STORAGE_KEY)));
  } catch {
    state.items = [];
    state.nextId = 1;
  }
}

function saveLocalState() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(exportState()));
}

function saveState() {
  saveLocalState();
  queueRemoteSave();
}

function setStatus(text) {
  elements.statusLabel.textContent = text || "";
}

function renderStatus() {
  setStatus(`${state.items.length}/${TODO_MAX_ITEMS} ${sync.status}`);
}

function loadScript(src) {
  return new Promise((resolve, reject) => {
    const existing = document.querySelector(`script[src="${src}"]`);
    if (existing && existing.dataset.loaded === "true") {
      resolve();
      return;
    }

    const script = existing || document.createElement("script");
    script.src = src;
    script.async = false;

    script.addEventListener("load", () => {
      script.dataset.loaded = "true";
      resolve();
    }, { once: true });
    script.addEventListener("error", () => reject(new Error(`Could not load ${src}`)), { once: true });

    if (!existing) {
      document.head.appendChild(script);
    }
  });
}

function queueRemoteSave() {
  if (sync.applyingRemote || !sync.docRef) {
    return;
  }

  sync.status = "saving";
  renderStatus();

  const data = exportState();
  window.clearTimeout(sync.saveTimer);
  sync.saveTimer = window.setTimeout(() => {
    saveRemoteState(data);
  }, 250);
}

async function saveRemoteState(data) {
  if (!sync.docRef || !window.firebase) {
    return;
  }

  try {
    await sync.docRef.set({
      ...data,
      updatedAt: window.firebase.firestore.FieldValue.serverTimestamp(),
    });
    sync.status = "saved";
    renderStatus();
  } catch (error) {
    console.warn("Firebase save failed", error);
    sync.status = "local";
    renderStatus();
  }
}

function handleRemoteSnapshot(snapshot) {
  let shouldSeedRemote = false;
  sync.applyingRemote = true;

  try {
    if (!snapshot.exists) {
      sync.status = "saved";
      if (state.items.length > 0) {
        shouldSeedRemote = true;
      }
    } else {
      if (importState(snapshot.data())) {
        saveLocalState();
        renderList();
      }

      sync.status = snapshot.metadata && snapshot.metadata.hasPendingWrites ? "saving" : "saved";
    }

    renderStatus();
  } finally {
    sync.applyingRemote = false;
  }

  if (shouldSeedRemote) {
    queueRemoteSave();
  }
}

function handleRemoteError(error) {
  console.warn("Firebase sync failed", error);
  sync.mode = "local";
  sync.status = "local";
  renderStatus();
}

async function initializeRemoteStorage() {
  try {
    const configResponse = await fetch("/__/firebase/init.json", { cache: "no-store" });
    if (!configResponse.ok) {
      renderStatus();
      return;
    }

    const config = await configResponse.json();
    await loadScript("/__/firebase/8.10.1/firebase-app.js");
    await loadScript("/__/firebase/8.10.1/firebase-firestore.js");

    if (!window.firebase) {
      throw new Error("Firebase SDK did not initialize");
    }

    if (window.firebase.apps.length === 0) {
      window.firebase.initializeApp(config);
    }

    sync.mode = "firebase";
    sync.status = "syncing";
    sync.docRef = window.firebase.firestore().collection(FIREBASE_COLLECTION).doc(FIREBASE_DOC_ID);
    renderStatus();
    sync.docRef.onSnapshot(handleRemoteSnapshot, handleRemoteError);
  } catch (error) {
    console.warn("Firebase is not available; using local storage", error);
    sync.mode = "local";
    sync.status = "local";
    renderStatus();
  }
}

function addTodo(parentId, title) {
  if (state.items.length >= TODO_MAX_ITEMS) {
    setStatus("Todo list is full");
    return 0;
  }

  const parent = parentId !== 0 ? findItem(parentId) : null;
  if (parentId !== 0 && !parent) {
    return 0;
  }

  const clean = cleanTitle(title);
  if (!clean) {
    setStatus("Enter a title first");
    return 0;
  }

  const item = {
    id: state.nextId,
    parentId,
    title: clean,
    done: false,
    expanded: true,
    display: parent ? parent.display : TODO_DISPLAY_ACTIVE,
  };

  state.nextId += 1;
  state.items.push(item);
  return item.id;
}

function updateTodoTitle(id, title) {
  const item = findItem(id);
  const clean = cleanTitle(title);

  if (!item || !clean) {
    setStatus("Enter a title first");
    return false;
  }

  item.title = clean;
  return true;
}

function removeTodo(id) {
  state.items = state.items.filter((item) => item.id !== id && !isDescendantOf(item.id, id));
}

function setExpanded(id, expanded) {
  const item = findItem(id);
  if (item) {
    item.expanded = expanded;
  }
}

function setDone(id, done) {
  for (const item of state.items) {
    if (item.id === id || isDescendantOf(item.id, id)) {
      item.done = done;
    }
  }
}

function createButton(label, classNames, onClick, attributes = {}) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = classNames;
  button.textContent = label;

  for (const [name, value] of Object.entries(attributes)) {
    button.setAttribute(name, value);
  }

  if (onClick) {
    button.addEventListener("click", onClick);
  }

  return button;
}

function renderTodoRow(item, depth) {
  if (!item || item.display !== TODO_DISPLAY_ACTIVE) {
    return;
  }

  const itemHasChildren = hasChildren(item.id);
  const row = document.createElement("article");
  row.className = "todo-row";
  row.style.paddingLeft = `${10 + depth * 30}px`;

  const expand = itemHasChildren
    ? createButton(
        item.expanded ? "v" : ">",
        "button row-button expand-button",
        () => {
          setExpanded(item.id, !item.expanded);
          saveState();
          renderList();
        },
        {
          "aria-label": item.expanded ? "Collapse todo" : "Expand todo",
          title: item.expanded ? "Collapse todo" : "Expand todo",
        },
      )
    : document.createElement("div");

  if (!itemHasChildren) {
    expand.className = "button row-button expand-button is-placeholder";
    expand.setAttribute("aria-hidden", "true");
  }

  row.appendChild(expand);

  const done = document.createElement("input");
  done.type = "checkbox";
  done.className = "done-checkbox";
  done.checked = item.done;
  done.setAttribute("aria-label", item.done ? `Mark ${item.title} active` : `Mark ${item.title} done`);
  done.title = item.done ? "Mark active" : "Mark done";
  done.addEventListener("change", () => {
    setDone(item.id, done.checked);
    saveState();
    renderList();
  });
  row.appendChild(done);

  const title = document.createElement("div");
  title.className = `todo-title${item.done ? " is-done" : ""}`;
  title.textContent = item.title;
  row.appendChild(title);

  row.appendChild(
    createButton(
      "+",
      "button row-button child-button",
      () => openEditor(item.id, 0),
      {
        "aria-label": "New sub-todo",
        title: "New sub-todo",
      },
    ),
  );

  row.appendChild(
    createButton(
      "Edit",
      "button row-button edit-button",
      () => openEditor(0, item.id),
      {
        "aria-label": `Edit ${item.title}`,
        title: "Edit todo",
      },
    ),
  );

  row.appendChild(
    createButton(
      "Del",
      "button button-danger row-button delete-button",
      () => {
        removeTodo(item.id);
        saveState();
        renderList();
      },
      {
        "aria-label": `Delete ${item.title}`,
        title: "Delete todo",
      },
    ),
  );

  elements.todoList.appendChild(row);

  if (itemHasChildren && item.expanded) {
    for (const child of state.items) {
      if (child.parentId === item.id) {
        renderTodoRow(child, depth + 1);
      }
    }
  }
}

function renderList() {
  elements.todoList.replaceChildren();

  let visibleRoots = 0;
  for (const item of state.items) {
    if (item.parentId === 0 && item.display === TODO_DISPLAY_ACTIVE) {
      renderTodoRow(item, 0);
      visibleRoots += 1;
    }
  }

  if (visibleRoots === 0) {
    const empty = document.createElement("p");
    empty.className = "empty-state";
    empty.textContent = "No todoes yet";
    elements.todoList.appendChild(empty);
  }

  renderStatus();
}

function closeEditor() {
  elements.editorOverlay.hidden = true;
  elements.editorTextarea.value = "";
  state.editorParentId = 0;
  state.editorEditId = 0;
}

function openEditor(parentId, editId) {
  const editing = findItem(editId);
  state.editorParentId = parentId;
  state.editorEditId = editId;

  if (editing) {
    elements.editorTitle.textContent = "Edit todo";
    elements.editorTextarea.value = editing.title;
  } else if (parentId !== 0) {
    elements.editorTitle.textContent = "New sub-todo";
    elements.editorTextarea.value = "";
  } else {
    elements.editorTitle.textContent = "New todo";
    elements.editorTextarea.value = "";
  }

  elements.editorOverlay.hidden = false;
  requestAnimationFrame(() => {
    elements.editorTextarea.focus();
    elements.editorTextarea.select();
  });
}

function saveEditor() {
  const changed =
    state.editorEditId !== 0
      ? updateTodoTitle(state.editorEditId, elements.editorTextarea.value)
      : addTodo(state.editorParentId, elements.editorTextarea.value) !== 0;

  if (!changed) {
    return;
  }

  saveState();
  closeEditor();
  renderList();
}

elements.addTodoButton.addEventListener("click", () => {
  if (state.items.length >= TODO_MAX_ITEMS) {
    setStatus("Todo list is full");
    return;
  }

  openEditor(0, 0);
});

elements.cancelButton.addEventListener("click", closeEditor);
elements.editorPanel.addEventListener("submit", (event) => {
  event.preventDefault();
  saveEditor();
});

elements.editorOverlay.addEventListener("click", (event) => {
  if (event.target === elements.editorOverlay) {
    closeEditor();
  }
});

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !elements.editorOverlay.hidden) {
    closeEditor();
  }
});

loadLocalState();
renderList();
initializeRemoteStorage();
