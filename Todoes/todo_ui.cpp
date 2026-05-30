#include "todo_ui.h"

#include <stdint.h>

#include "lvgl.h"
#include "todo_model.h"
#include "todo_storage.h"

static lv_obj_t *g_screen = nullptr;
static lv_obj_t *g_list = nullptr;
static lv_obj_t *g_status_label = nullptr;
static lv_obj_t *g_editor_overlay = nullptr;
static lv_obj_t *g_editor_title = nullptr;
static lv_obj_t *g_editor_textarea = nullptr;
static lv_obj_t *g_editor_keyboard = nullptr;

static uint32_t g_editor_parent_id = 0;
static uint32_t g_editor_edit_id = 0;

static uint32_t event_id(lv_event_t *event)
{
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
}

static void set_status(const char *text);
static void refresh_list();

static void save_and_refresh()
{
    const bool saved = todo_storage_save();
    refresh_list();
    if(!saved) {
        set_status("Save failed");
    }
}

static void set_status(const char *text)
{
    if(g_status_label) {
        lv_label_set_text(g_status_label, text ? text : "");
    }
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text, lv_coord_t width)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, width, 38);
    lv_obj_set_style_radius(button, 6, LV_PART_MAIN);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);

    return button;
}

static void close_editor()
{
    if(!g_editor_overlay) return;
    lv_obj_add_flag(g_editor_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(g_editor_keyboard, nullptr);
    g_editor_parent_id = 0;
    g_editor_edit_id = 0;
}

static void open_editor(uint32_t parent_id, uint32_t edit_id)
{
    if(!g_editor_overlay) return;

    g_editor_parent_id = parent_id;
    g_editor_edit_id = edit_id;

    const TodoItem *editing = todo_model_find(edit_id);
    if(editing) {
        lv_label_set_text(g_editor_title, "Edit todo");
        lv_textarea_set_text(g_editor_textarea, editing->title);
    } else if(parent_id != 0) {
        lv_label_set_text(g_editor_title, "New sub-todo");
        lv_textarea_set_text(g_editor_textarea, "");
    } else {
        lv_label_set_text(g_editor_title, "New todo");
        lv_textarea_set_text(g_editor_textarea, "");
    }

    lv_obj_clear_flag(g_editor_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(g_editor_keyboard, g_editor_textarea);
    lv_obj_add_state(g_editor_textarea, LV_STATE_FOCUSED);
}

static void add_root_event(lv_event_t *event)
{
    LV_UNUSED(event);
    if(!todo_model_can_add()) {
        set_status("Todo list is full");
        return;
    }
    open_editor(0, 0);
}

static void add_child_event(lv_event_t *event)
{
    if(!todo_model_can_add()) {
        set_status("Todo list is full");
        return;
    }
    open_editor(event_id(event), 0);
}

static void edit_event(lv_event_t *event)
{
    open_editor(0, event_id(event));
}

static void delete_event(lv_event_t *event)
{
    todo_model_remove(event_id(event));
    save_and_refresh();
}

static void expand_event(lv_event_t *event)
{
    const uint32_t id = event_id(event);
    const TodoItem *item = todo_model_find(id);
    if(!item) return;

    todo_model_set_expanded(id, !item->expanded);
    save_and_refresh();
}

static void editor_cancel_event(lv_event_t *event)
{
    LV_UNUSED(event);
    close_editor();
}

static void editor_save_event(lv_event_t *event)
{
    LV_UNUSED(event);

    const char *title = lv_textarea_get_text(g_editor_textarea);
    bool changed = false;

    if(g_editor_edit_id != 0) {
        changed = todo_model_update_title(g_editor_edit_id, title);
    } else {
        changed = todo_model_add(g_editor_parent_id, title) != 0;
    }

    if(changed) {
        const bool saved = todo_storage_save();
        close_editor();
        refresh_list();
        if(!saved) {
            set_status("Save failed");
        }
    } else {
        set_status("Enter a title first");
    }
}

static void keyboard_event(lv_event_t *event)
{
    const lv_event_code_t code = lv_event_get_code(event);
    if(code == LV_EVENT_CANCEL) {
        close_editor();
    } else if(code == LV_EVENT_READY) {
        editor_save_event(event);
    }
}

static void create_editor()
{
    g_editor_overlay = lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_editor_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(g_editor_overlay, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(g_editor_overlay, 160, LV_PART_MAIN);
    lv_obj_clear_flag(g_editor_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_editor_overlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *panel = lv_obj_create(g_editor_overlay);
    lv_obj_set_size(panel, 820, 250);
    lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 24);
    lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 18, LV_PART_MAIN);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    g_editor_title = lv_label_create(panel);
    lv_label_set_text(g_editor_title, "New todo");
    lv_obj_set_style_text_font(g_editor_title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_width(g_editor_title, LV_PCT(100));

    g_editor_textarea = lv_textarea_create(panel);
    lv_obj_set_width(g_editor_textarea, LV_PCT(100));
    lv_obj_set_height(g_editor_textarea, 86);
    lv_textarea_set_max_length(g_editor_textarea, TODO_TITLE_MAX_LEN);
    lv_textarea_set_one_line(g_editor_textarea, false);
    lv_textarea_set_placeholder_text(g_editor_textarea, "Todo title");

    lv_obj_t *actions = lv_obj_create(panel);
    lv_obj_remove_style_all(actions);
    lv_obj_set_width(actions, LV_PCT(100));
    lv_obj_set_height(actions, 46);
    lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(actions, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *cancel = make_button(actions, "Cancel", 112);
    lv_obj_add_event_cb(cancel, editor_cancel_event, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *save = make_button(actions, "Save", 112);
    lv_obj_set_style_bg_color(save, lv_color_hex(0x2563eb), LV_PART_MAIN);
    lv_obj_add_event_cb(save, editor_save_event, LV_EVENT_CLICKED, nullptr);

    g_editor_keyboard = lv_keyboard_create(g_editor_overlay);
    lv_obj_set_size(g_editor_keyboard, LV_PCT(100), 285);
    lv_obj_align(g_editor_keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(g_editor_keyboard, keyboard_event, LV_EVENT_ALL, nullptr);
}

static lv_obj_t *create_row_container(lv_obj_t *parent, uint8_t depth)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_min_height(row, 58, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_border_color(row, lv_color_hex(0xe5e7eb), LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(row, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_top(row, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(row, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_left(row, 10 + (depth * 30), LV_PART_MAIN);
    lv_obj_set_style_pad_right(row, 10, LV_PART_MAIN);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    return row;
}

static void render_todo_row(const TodoItem *item, uint8_t depth)
{
    if(!item || item->display != TODO_DISPLAY_ACTIVE) return;

    const bool has_children = todo_model_has_children(item->id);
    lv_obj_t *row = create_row_container(g_list, depth);

    lv_obj_t *expand = make_button(row, has_children ? (item->expanded ? "v" : ">") : " ", 38);
    if(has_children) {
        lv_obj_add_event_cb(expand, expand_event, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<uintptr_t>(item->id)));
    } else {
        lv_obj_clear_flag(expand, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_opa(expand, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(expand, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(expand, 0, LV_PART_MAIN);
    }

    lv_obj_t *title = lv_label_create(row);
    lv_label_set_text(title, item->title);
    lv_obj_set_flex_grow(title, 1);
    lv_obj_set_width(title, 1);
    lv_label_set_long_mode(title, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(title, item->done ? lv_color_hex(0x6b7280) : lv_color_hex(0x111827), LV_PART_MAIN);
    if(item->done) {
        lv_obj_set_style_text_decor(title, LV_TEXT_DECOR_STRIKETHROUGH, LV_PART_MAIN);
    }

    lv_obj_t *child = make_button(row, "+", 42);
    lv_obj_add_event_cb(child, add_child_event, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<uintptr_t>(item->id)));

    lv_obj_t *edit = make_button(row, "Edit", 64);
    lv_obj_add_event_cb(edit, edit_event, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<uintptr_t>(item->id)));

    lv_obj_t *del = make_button(row, "Del", 58);
    lv_obj_set_style_bg_color(del, lv_color_hex(0xdc2626), LV_PART_MAIN);
    lv_obj_add_event_cb(del, delete_event, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<uintptr_t>(item->id)));

    if(has_children && item->expanded) {
        for(uint16_t i = 0; i < todo_model_count(); ++i) {
            const TodoItem *child_item = todo_model_at(i);
            if(child_item && child_item->parent_id == item->id) {
                render_todo_row(child_item, depth + 1);
            }
        }
    }
}

static void refresh_list()
{
    if(!g_list) return;

    lv_obj_clean(g_list);

    uint16_t visible_roots = 0;
    for(uint16_t i = 0; i < todo_model_count(); ++i) {
        const TodoItem *item = todo_model_at(i);
        if(item && item->parent_id == 0 && item->display == TODO_DISPLAY_ACTIVE) {
            render_todo_row(item, 0);
            ++visible_roots;
        }
    }

    if(visible_roots == 0) {
        lv_obj_t *empty = lv_label_create(g_list);
        lv_label_set_text(empty, "No todoes yet");
        lv_obj_set_width(empty, LV_PCT(100));
        lv_obj_set_style_text_align(empty, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_style_text_color(empty, lv_color_hex(0x6b7280), LV_PART_MAIN);
        lv_obj_set_style_pad_top(empty, 80, LV_PART_MAIN);
    }

    char status[48];
    lv_snprintf(status, sizeof(status), "%u/%u items", todo_model_count(), TODO_MAX_ITEMS);
    set_status(status);
}

void todo_ui_init()
{
    todo_model_clear();
    todo_storage_load();

    lv_disp_t *display = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(display, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(display, theme);

    g_screen = lv_obj_create(nullptr);
    lv_obj_set_size(g_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(g_screen, lv_color_hex(0xf3f4f6), LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_screen, 0, LV_PART_MAIN);
    lv_obj_clear_flag(g_screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(g_screen, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *header = lv_obj_create(g_screen);
    lv_obj_set_width(header, LV_PCT(100));
    lv_obj_set_height(header, 64);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(header, 18, LV_PART_MAIN);
    lv_obj_set_style_pad_right(header, 18, LV_PART_MAIN);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *title = lv_label_create(header);
    lv_label_set_text(title, "Todoes");
    lv_obj_set_style_text_color(title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_flex_grow(title, 1);

    g_status_label = lv_label_create(header);
    lv_label_set_text(g_status_label, "");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xd1d5db), LV_PART_MAIN);
    lv_obj_set_width(g_status_label, 120);
    lv_obj_set_style_text_align(g_status_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t *add = make_button(header, "+", 52);
    lv_obj_set_style_bg_color(add, lv_color_hex(0x2563eb), LV_PART_MAIN);
    lv_obj_add_event_cb(add, add_root_event, LV_EVENT_CLICKED, nullptr);

    g_list = lv_obj_create(g_screen);
    lv_obj_set_width(g_list, LV_PCT(100));
    lv_obj_set_flex_grow(g_list, 1);
    lv_obj_set_style_bg_opa(g_list, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(g_list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(g_list, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_row(g_list, 8, LV_PART_MAIN);
    lv_obj_set_flex_flow(g_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(g_list, LV_DIR_VER);

    create_editor();
    refresh_list();

    lv_disp_load_scr(g_screen);
}
