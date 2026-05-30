#pragma once

#include <stdbool.h>
#include <stdint.h>

static const uint16_t TODO_MAX_ITEMS = 80;
static const uint8_t TODO_TITLE_MAX_LEN = 80;

enum TodoDisplay : uint8_t {
    TODO_DISPLAY_ACTIVE = 0,
    TODO_DISPLAY_DONE = 1,
};

struct TodoItem {
    uint32_t id;
    uint32_t parent_id;
    char title[TODO_TITLE_MAX_LEN + 1];
    bool done;
    bool expanded;
    uint8_t display;
};

void todo_model_clear();
uint16_t todo_model_count();
bool todo_model_can_add();

const TodoItem *todo_model_at(uint16_t index);
const TodoItem *todo_model_find(uint32_t id);
bool todo_model_has_children(uint32_t parent_id);

uint32_t todo_model_add(uint32_t parent_id, const char *title);
bool todo_model_update_title(uint32_t id, const char *title);
bool todo_model_remove(uint32_t id);
bool todo_model_set_done(uint32_t id, bool done);
bool todo_model_set_expanded(uint32_t id, bool expanded);
bool todo_model_move_to_display(uint32_t id, uint8_t display, bool done);

uint16_t todo_model_export(TodoItem *items, uint16_t capacity, uint32_t *next_id);
bool todo_model_import(const TodoItem *items, uint16_t count, uint32_t next_id);
