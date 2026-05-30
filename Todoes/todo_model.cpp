#include "todo_model.h"

#include <string.h>

static TodoItem g_items[TODO_MAX_ITEMS];
static uint16_t g_count = 0;
static uint32_t g_next_id = 1;

static TodoItem *find_mutable(uint32_t id)
{
    for(uint16_t i = 0; i < g_count; ++i) {
        if(g_items[i].id == id) return &g_items[i];
    }
    return nullptr;
}

static bool copy_clean_title(char *dest, const char *src)
{
    if(!dest || !src) return false;

    while(*src == ' ' || *src == '\t' || *src == '\r' || *src == '\n') {
        ++src;
    }

    uint8_t len = 0;
    while(src[len] != '\0' && len < TODO_TITLE_MAX_LEN) {
        dest[len] = src[len];
        ++len;
    }

    while(len > 0) {
        const char c = dest[len - 1];
        if(c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
        --len;
    }

    dest[len] = '\0';
    return len > 0;
}

static bool id_exists_in_buffer(const TodoItem *items, uint16_t count, uint32_t id)
{
    if(id == 0) return true;
    for(uint16_t i = 0; i < count; ++i) {
        if(items[i].id == id) return true;
    }
    return false;
}

static bool is_descendant_of(uint32_t item_id, uint32_t ancestor_id)
{
    const TodoItem *item = todo_model_find(item_id);
    uint16_t depth_guard = 0;
    while(item && item->parent_id != 0 && depth_guard++ < TODO_MAX_ITEMS) {
        if(item->parent_id == ancestor_id) return true;
        item = todo_model_find(item->parent_id);
    }
    return false;
}

void todo_model_clear()
{
    memset(g_items, 0, sizeof(g_items));
    g_count = 0;
    g_next_id = 1;
}

uint16_t todo_model_count()
{
    return g_count;
}

bool todo_model_can_add()
{
    return g_count < TODO_MAX_ITEMS;
}

const TodoItem *todo_model_at(uint16_t index)
{
    if(index >= g_count) return nullptr;
    return &g_items[index];
}

const TodoItem *todo_model_find(uint32_t id)
{
    for(uint16_t i = 0; i < g_count; ++i) {
        if(g_items[i].id == id) return &g_items[i];
    }
    return nullptr;
}

bool todo_model_has_children(uint32_t parent_id)
{
    for(uint16_t i = 0; i < g_count; ++i) {
        if(g_items[i].parent_id == parent_id) return true;
    }
    return false;
}

uint32_t todo_model_add(uint32_t parent_id, const char *title)
{
    if(g_count >= TODO_MAX_ITEMS) return 0;
    if(parent_id != 0 && todo_model_find(parent_id) == nullptr) return 0;

    TodoItem item = {};
    if(!copy_clean_title(item.title, title)) return 0;

    item.id = g_next_id++;
    item.parent_id = parent_id;
    item.done = false;
    item.expanded = true;
    item.display = TODO_DISPLAY_ACTIVE;

    const TodoItem *parent = todo_model_find(parent_id);
    if(parent) {
        item.display = parent->display;
    }

    g_items[g_count++] = item;
    return item.id;
}

bool todo_model_update_title(uint32_t id, const char *title)
{
    TodoItem *item = find_mutable(id);
    if(!item) return false;
    return copy_clean_title(item->title, title);
}

bool todo_model_remove(uint32_t id)
{
    if(todo_model_find(id) == nullptr) return false;

    uint16_t write_index = 0;
    for(uint16_t read_index = 0; read_index < g_count; ++read_index) {
        const bool remove_item = g_items[read_index].id == id || is_descendant_of(g_items[read_index].id, id);
        if(!remove_item) {
            if(write_index != read_index) g_items[write_index] = g_items[read_index];
            ++write_index;
        }
    }

    for(uint16_t i = write_index; i < g_count; ++i) {
        memset(&g_items[i], 0, sizeof(g_items[i]));
    }

    g_count = write_index;
    return true;
}

bool todo_model_set_done(uint32_t id, bool done)
{
    TodoItem *item = find_mutable(id);
    if(!item) return false;
    item->done = done;
    return true;
}

bool todo_model_set_expanded(uint32_t id, bool expanded)
{
    TodoItem *item = find_mutable(id);
    if(!item) return false;
    item->expanded = expanded;
    return true;
}

bool todo_model_move_to_display(uint32_t id, uint8_t display, bool done)
{
    if(display != TODO_DISPLAY_ACTIVE && display != TODO_DISPLAY_DONE) return false;
    if(todo_model_find(id) == nullptr) return false;

    for(uint16_t i = 0; i < g_count; ++i) {
        if(g_items[i].id == id || is_descendant_of(g_items[i].id, id)) {
            g_items[i].display = display;
            g_items[i].done = done;
        }
    }

    return true;
}

uint16_t todo_model_export(TodoItem *items, uint16_t capacity, uint32_t *next_id)
{
    const uint16_t copy_count = g_count < capacity ? g_count : capacity;
    if(items && copy_count > 0) {
        memcpy(items, g_items, copy_count * sizeof(TodoItem));
    }
    if(next_id) *next_id = g_next_id;
    return copy_count;
}

bool todo_model_import(const TodoItem *items, uint16_t count, uint32_t next_id)
{
    if(!items || count > TODO_MAX_ITEMS) return false;

    todo_model_clear();

    uint32_t max_id = 0;
    for(uint16_t i = 0; i < count; ++i) {
        if(items[i].id == 0) continue;

        TodoItem clean = items[i];
        clean.title[TODO_TITLE_MAX_LEN] = '\0';
        if(clean.title[0] == '\0') continue;
        if(clean.display != TODO_DISPLAY_ACTIVE && clean.display != TODO_DISPLAY_DONE) {
            clean.display = TODO_DISPLAY_ACTIVE;
        }

        g_items[g_count++] = clean;
        if(clean.id > max_id) max_id = clean.id;
    }

    for(uint16_t i = 0; i < g_count; ++i) {
        if(!id_exists_in_buffer(g_items, g_count, g_items[i].parent_id)) {
            g_items[i].parent_id = 0;
        }
    }

    g_next_id = next_id;
    if(g_next_id <= max_id) g_next_id = max_id + 1;
    if(g_next_id == 0) g_next_id = 1;

    return true;
}
