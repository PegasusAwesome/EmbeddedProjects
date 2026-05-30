#include "todo_storage.h"

#include <Preferences.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "todo_model.h"

static const char *TODO_STORAGE_NAMESPACE = "todoes";
static const char *TODO_STORAGE_META_KEY = "meta";
static const char *TODO_STORAGE_LEGACY_KEY = "state";
static const uint32_t TODO_STORAGE_MAGIC = 0x54444f45;
static const uint16_t TODO_STORAGE_VERSION = 2;
static const uint16_t TODO_STORAGE_LEGACY_VERSION = 1;

struct TodoStoredMeta {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t next_id;
};

struct TodoStoredItem {
    uint32_t id;
    uint32_t parent_id;
    uint8_t done;
    uint8_t expanded;
    uint8_t display;
    char title[TODO_TITLE_MAX_LEN + 1];
};

struct TodoLegacyStoredState {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint32_t next_id;
    TodoItem items[TODO_MAX_ITEMS];
};

static TodoItem g_storage_items[TODO_MAX_ITEMS];

static void make_item_key(uint16_t index, char *key, size_t key_size)
{
    snprintf(key, key_size, "item%02u", index);
}

static TodoStoredItem to_stored_item(const TodoItem &item)
{
    TodoStoredItem stored = {};
    stored.id = item.id;
    stored.parent_id = item.parent_id;
    stored.done = item.done ? 1 : 0;
    stored.expanded = item.expanded ? 1 : 0;
    stored.display = item.display;
    strncpy(stored.title, item.title, TODO_TITLE_MAX_LEN);
    stored.title[TODO_TITLE_MAX_LEN] = '\0';
    return stored;
}

static TodoItem from_stored_item(const TodoStoredItem &stored)
{
    TodoItem item = {};
    item.id = stored.id;
    item.parent_id = stored.parent_id;
    item.done = stored.done != 0;
    item.expanded = stored.expanded != 0;
    item.display = stored.display;
    strncpy(item.title, stored.title, TODO_TITLE_MAX_LEN);
    item.title[TODO_TITLE_MAX_LEN] = '\0';
    return item;
}

static bool load_compact()
{
    Preferences preferences;
    if(!preferences.begin(TODO_STORAGE_NAMESPACE, true)) {
        return false;
    }

    const size_t meta_size = preferences.getBytesLength(TODO_STORAGE_META_KEY);
    if(meta_size != sizeof(TodoStoredMeta)) {
        preferences.end();
        return false;
    }

    TodoStoredMeta meta = {};
    const size_t meta_read_size = preferences.getBytes(TODO_STORAGE_META_KEY, &meta, sizeof(meta));
    if(meta_read_size != sizeof(meta) || meta.magic != TODO_STORAGE_MAGIC || meta.version != TODO_STORAGE_VERSION ||
       meta.count > TODO_MAX_ITEMS) {
        preferences.end();
        return false;
    }

    memset(g_storage_items, 0, sizeof(g_storage_items));

    for(uint16_t i = 0; i < meta.count; ++i) {
        char key[8];
        make_item_key(i, key, sizeof(key));

        if(preferences.getBytesLength(key) != sizeof(TodoStoredItem)) {
            preferences.end();
            return false;
        }

        TodoStoredItem stored = {};
        const size_t item_read_size = preferences.getBytes(key, &stored, sizeof(stored));
        if(item_read_size != sizeof(stored)) {
            preferences.end();
            return false;
        }

        g_storage_items[i] = from_stored_item(stored);
    }

    preferences.end();

    return todo_model_import(g_storage_items, meta.count, meta.next_id);
}

static bool load_legacy()
{
    Preferences preferences;
    if(!preferences.begin(TODO_STORAGE_NAMESPACE, true)) {
        return false;
    }

    const size_t stored_size = preferences.getBytesLength(TODO_STORAGE_LEGACY_KEY);
    if(stored_size != sizeof(TodoLegacyStoredState)) {
        preferences.end();
        return false;
    }

    TodoLegacyStoredState *state = static_cast<TodoLegacyStoredState *>(malloc(sizeof(TodoLegacyStoredState)));
    if(!state) {
        preferences.end();
        return false;
    }
    memset(state, 0, sizeof(TodoLegacyStoredState));

    const size_t read_size = preferences.getBytes(TODO_STORAGE_LEGACY_KEY, state, sizeof(TodoLegacyStoredState));
    preferences.end();

    bool ok = false;
    if(read_size == sizeof(TodoLegacyStoredState) && state->magic == TODO_STORAGE_MAGIC &&
       state->version == TODO_STORAGE_LEGACY_VERSION && state->count <= TODO_MAX_ITEMS) {
        ok = todo_model_import(state->items, state->count, state->next_id);
    }

    free(state);
    return ok;
}

bool todo_storage_load()
{
    if(load_compact()) {
        return true;
    }

    if(load_legacy()) {
        todo_storage_save();
        return true;
    }

    return false;
}

bool todo_storage_save()
{
    memset(g_storage_items, 0, sizeof(g_storage_items));

    TodoStoredMeta meta = {};
    meta.magic = TODO_STORAGE_MAGIC;
    meta.version = TODO_STORAGE_VERSION;
    meta.count = todo_model_export(g_storage_items, TODO_MAX_ITEMS, &meta.next_id);

    Preferences preferences;
    if(!preferences.begin(TODO_STORAGE_NAMESPACE, false)) {
        return false;
    }

    bool ok = true;
    for(uint16_t i = 0; i < meta.count; ++i) {
        char key[8];
        make_item_key(i, key, sizeof(key));

        const TodoStoredItem stored = to_stored_item(g_storage_items[i]);
        if(preferences.putBytes(key, &stored, sizeof(stored)) != sizeof(stored)) {
            ok = false;
            break;
        }
    }

    if(ok) {
        ok = preferences.putBytes(TODO_STORAGE_META_KEY, &meta, sizeof(meta)) == sizeof(meta);
    }

    if(ok) {
        for(uint16_t i = meta.count; i < TODO_MAX_ITEMS; ++i) {
            char key[8];
            make_item_key(i, key, sizeof(key));
            preferences.remove(key);
        }
        preferences.remove(TODO_STORAGE_LEGACY_KEY);
    }

    preferences.end();

    return ok;
}

void todo_storage_clear()
{
    Preferences preferences;
    if(preferences.begin(TODO_STORAGE_NAMESPACE, false)) {
        preferences.clear();
        preferences.end();
    }
    todo_model_clear();
}
