/*
* Ported LVGL 8.4 and display the official demo interface.
*/
#include "Arcanet.h"
#include "lvgl_port.h"       // LVGL porting functions for integration
#include "ui/ui.h"
#include "lvgl.h"

#include "Quotes.h"

LV_IMG_DECLARE(ui_img_wifi_disabled_png);    
LV_IMG_DECLARE(ui_img_battery_disabled_png);   
LV_IMG_DECLARE(ui_img_lantern_disabled_png);

LV_IMG_DECLARE(ui_img_lantern_on_png);    
LV_IMG_DECLARE(ui_img_lantern_off_png);   

LV_IMG_DECLARE(ui_img_gem_disabled_png);
LV_IMG_DECLARE(ui_img_gem_transparent_png);
LV_IMG_DECLARE(ui_img_gem_off_png);
LV_IMG_DECLARE(ui_img_gem_white_png);
LV_IMG_DECLARE(ui_img_gem_red_png);
LV_IMG_DECLARE(ui_img_gem_green_png);
LV_IMG_DECLARE(ui_img_gem_blue_png);
LV_IMG_DECLARE(ui_img_gem_cyan_png);
LV_IMG_DECLARE(ui_img_gem_magenta_png);
LV_IMG_DECLARE(ui_img_gem_yellow_png);

LV_IMG_DECLARE(ui_img_battery_empty_png); 
LV_IMG_DECLARE(ui_img_battery_0_25_png);  
LV_IMG_DECLARE(ui_img_battery_25_50_png); 
LV_IMG_DECLARE(ui_img_battery_50_75_png); 
LV_IMG_DECLARE(ui_img_battery_75_100_png);

LV_IMG_DECLARE(ui_img_wifi_0_20_png);    
LV_IMG_DECLARE(ui_img_wifi_20_40_png);   
LV_IMG_DECLARE(ui_img_wifi_40_60_png);   
LV_IMG_DECLARE(ui_img_wifi_60_80_png);   
LV_IMG_DECLARE(ui_img_wifi_80_100_png);  


// Your device's unique ID
const String MY_ID = "HEPTAGLYPH1";//aka HG1 or HaGeen

uint32_t tCheckStalenessPeriod  = 60000;
uint32_t tLastStalenessCheck    = 0;
uint32_t tMinStalenessTime      = 200000;



struct Relic {
    const char* name;
    const char* netID;
    uint32_t lastUpdate;
    lv_obj_t* mainButton;
    lv_obj_t* nameLabel;
    lv_obj_t* networkIcon;
    lv_obj_t* batteryIcon;
    lv_obj_t* relicIcon;
};

struct Lantern : Relic {
    int on;
};

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
    double w;       // a fraction between 0 and 1
} rgbw;

struct LuxArcana : Relic {
    rgbw _rgbw;
    lv_obj_t* wheelIcon;
    lv_obj_t* slider;
};

struct RelicConfig {
    const char* netID;
    const char* label;
};

static const RelicConfig LANTERN_CONFIGS[] = {
    { "LANTERN11", "Lantaarn 11" },
    { "LANTERN12", "Lantaarn 12" },
    { "LANTERN13", "Lantaarn 13" },
    { "LANTERN14", "Lantaarn 14" },
    { "LANTERN15", "Lantaarn 15" },
    { "LANTERN16", "Lantaarn 16" },
    { "LANTERN19", "Lantaarn 19" },
    { "LANTERN20", "Lantaarn 20" },
    { "LANTERN21", "Lantaarn 21" },
    { "LANTERN22", "Lantaarn 22" },
    { "LANTERN23", "Lantaarn 23" },
    { "LANTERN24", "Lantaarn 24" },
};

static const RelicConfig LUX_CONFIGS[] = {
    { "LUX1", "Straler 1" },
    { "LUX2", "Straler 2" },
    { "LUX3", "Straler 3" },
    { "LUX4", "Straler 4" },
    { "LUX5", "Straler 5" },
    { "LUX6", "Straler 6" },
};

static const size_t LANTERN_COUNT = sizeof(LANTERN_CONFIGS) / sizeof(LANTERN_CONFIGS[0]);
static const size_t LUX_COUNT = sizeof(LUX_CONFIGS) / sizeof(LUX_CONFIGS[0]);

static Lantern lanterns[LANTERN_COUNT];
static LuxArcana luxes[LUX_COUNT];

static inline lv_style_selector_t lv_selector(uint32_t part, uint32_t state) {
    return static_cast<lv_style_selector_t>(part | state);
}

#define CONSOLE_LABEL uic_Console
static const int CONSOLE_MAX_LINES = 30;      // tune to label height
static const int CONSOLE_MAX_CHARS = 2048;    // total buffer size

static char consoleBuffer[CONSOLE_MAX_CHARS] = "";
static int consoleLineCount = 0;

#ifndef DEBUG_MIRROR_SERIAL0
#define DEBUG_MIRROR_SERIAL0 0
#endif

static void debug_begin(unsigned long baud) {
    Serial.begin(baud);
#if DEBUG_MIRROR_SERIAL0
    Serial0.begin(baud);
#endif
    delay(200);
}

static void debug_println(const char* message) {
    Serial.println(message);
#if DEBUG_MIRROR_SERIAL0
    Serial0.println(message);
#endif
}

static void debug_println(const String& message) {
    Serial.println(message);
#if DEBUG_MIRROR_SERIAL0
    Serial0.println(message);
#endif
}


// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
    // Show everything that comes in
    console_append("[" + id + "] " + command);

    if (id == MY_ID) {

    } else if (id == "CONTROLLER") {
        ui_apply_status_string(command.c_str());
    } else if (id == "ALL") {
        ui_apply_status_string(command.c_str());
    }
}
// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);

int eeCount = 0;

void setEasterEgg(lv_event_t * e) {
    eeCount++;
    if (eeCount>=5 && eeCount<15) {
        lv_label_set_text(uic_EasterEggResult, "...");
    } else if (eeCount>=15 && eeCount<20) {
        lv_label_set_text(uic_EasterEggResult, "Noeoeoes!");
    } else if (eeCount>=20 && eeCount<30) {
        lv_label_set_text(uic_EasterEggResult, "Why you pokin' me again?");

    } else if (eeCount>=30 && eeCount<40) {
        lv_label_set_text(uic_EasterEggResult, "Me busy! Leave me alone!");

    } else if (eeCount>=40 && eeCount<140) {
        String text = String(140-eeCount)+" little clicks left on the screen. . .";
        lv_label_set_text(uic_EasterEggResult, text.c_str());

    } else if (eeCount>=140) {
        const char* q = FunQuotes::get_random_quote();
        lv_label_set_text(uic_EasterEggResult, q);
    }
}

// 3) One event handler, works for every row
static void on_lantern_button_event(lv_event_t* e) {
    Lantern* lantern = static_cast<Lantern*>(lv_event_get_user_data(e));
    if (!lantern) return;

    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_CLICKED) return;

    String lanternID = String(lantern->netID);
    if (lantern->on==1) {
        arcanet.sendCommand(lanternID, "LANTERN_OFF");
    } else {
        arcanet.sendCommand(lanternID, "LANTERN_ON");
    }
}

static void on_lux_button_event(lv_event_t* e) {
    LuxArcana* lux = static_cast<LuxArcana*>(lv_event_get_user_data(e));
    if (!lux) return;

    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_CLICKED) return;
}

static LuxArcana* find_lux_by_widget(lv_obj_t* widget) {
    for (auto &lux : luxes) {
        if (lux.wheelIcon == widget || lux.slider == widget) {
            return &lux;
        }
    }
    return nullptr;
}

static void on_lux_color_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;

    LuxArcana* lux = static_cast<LuxArcana*>(lv_event_get_user_data(e));
    if (!lux) lux = find_lux_by_widget(lv_event_get_target(e));
    if (!lux) return;

    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(lux->wheelIcon);
    arcanet.sendCommand(String(lux->netID), "SET_HUE_" + String(hsv.h));
}

static void on_lux_brightness_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;

    LuxArcana* lux = static_cast<LuxArcana*>(lv_event_get_user_data(e));
    if (!lux) lux = find_lux_by_widget(lv_event_get_target(e));
    if (!lux) return;

    int val = lv_slider_get_value(lux->slider);

    arcanet.sendCommand(String(lux->netID), "SET_BRIGHTNESS_" + String(val));
}

static void on_lux_black_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    LuxArcana* lux = static_cast<LuxArcana*>(lv_event_get_user_data(e));
    if (!lux) lux = find_lux_by_widget(lv_event_get_target(e));
    if (!lux) return;

    arcanet.sendCommand(String(lux->netID), "BLACK_ON");
}

void setColorLux1(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux2(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux3(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux4(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux5(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux6(lv_event_t * e) {
    on_lux_color_event(e);
}
void setColorLux7(lv_event_t * e) {
    on_lux_color_event(e);
}




static void style_relic_button(lv_obj_t* button, int height, int widthPct) {
    lv_obj_set_height(button, height);
    lv_obj_set_width(button, lv_pct(widthPct));
    lv_obj_set_align(button, LV_ALIGN_CENTER);
    lv_obj_add_flag(button, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x9CA3AF), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(button, 255, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_border_color(button, lv_color_hex(0x1E293B), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_border_opa(button, 255, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_border_width(button, 3, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
}

static lv_obj_t* create_relic_label(lv_obj_t* parent, const char* labelText) {
    lv_obj_t* label = lv_label_create(parent);
    lv_obj_set_width(label, LV_SIZE_CONTENT);
    lv_obj_set_height(label, LV_SIZE_CONTENT);
    lv_obj_set_align(label, LV_ALIGN_BOTTOM_MID);
    lv_label_set_text(label, labelText);
    lv_obj_set_style_text_color(label, lv_color_hex(0x0D0D0D), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_text_opa(label, 255, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    return label;
}

static lv_obj_t* create_top_icon(lv_obj_t* parent, const lv_img_dsc_t* src, int xOffset) {
    lv_obj_t* icon = lv_img_create(parent);
    lv_img_set_src(icon, src);
    lv_obj_set_width(icon, LV_SIZE_CONTENT);
    lv_obj_set_height(icon, LV_SIZE_CONTENT);
    lv_obj_set_x(icon, xOffset);
    lv_obj_set_y(icon, 0);
    lv_obj_set_align(icon, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(icon, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_SCROLLABLE);
    return icon;
}

static void create_lantern_card(lv_obj_t* parent, Lantern* lantern, const RelicConfig* config) {
    lantern->name = config->label;
    lantern->netID = config->netID;
    lantern->lastUpdate = 0;
    lantern->on = 0;

    lantern->mainButton = lv_btn_create(parent);
    style_relic_button(lantern->mainButton, 148, 20);
    lv_obj_add_event_cb(lantern->mainButton, on_lantern_button_event, LV_EVENT_CLICKED, lantern);

    lantern->nameLabel = create_relic_label(lantern->mainButton, config->label);
    lantern->networkIcon = create_top_icon(lantern->mainButton, &ui_img_wifi_disabled_png, -30);
    lantern->batteryIcon = create_top_icon(lantern->mainButton, &ui_img_battery_disabled_png, 0);

    lantern->relicIcon = lv_img_create(lantern->mainButton);
    lv_img_set_src(lantern->relicIcon, &ui_img_lantern_disabled_png);
    lv_obj_set_width(lantern->relicIcon, LV_SIZE_CONTENT);
    lv_obj_set_height(lantern->relicIcon, LV_SIZE_CONTENT);
    lv_obj_set_align(lantern->relicIcon, LV_ALIGN_CENTER);
    lv_obj_add_flag(lantern->relicIcon, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_clear_flag(lantern->relicIcon, LV_OBJ_FLAG_SCROLLABLE);
    lv_img_set_zoom(lantern->relicIcon, 255);
}

static void create_lux_card(lv_obj_t* parent, LuxArcana* lux, const RelicConfig* config) {
    lux->name = config->label;
    lux->netID = config->netID;
    lux->lastUpdate = 0;
    lux->_rgbw = {0, 0, 0, 0};

    lux->mainButton = lv_btn_create(parent);
    style_relic_button(lux->mainButton, 297, 33);
    lv_obj_add_event_cb(lux->mainButton, on_lux_button_event, LV_EVENT_CLICKED, lux);

    lux->nameLabel = create_relic_label(lux->mainButton, config->label);
    lv_obj_set_x(lux->nameLabel, -13);
    lv_obj_set_y(lux->nameLabel, -252);

    lux->networkIcon = create_top_icon(lux->mainButton, &ui_img_wifi_disabled_png, -190);
    lux->batteryIcon = create_top_icon(lux->mainButton, &ui_img_battery_disabled_png, 0);

    lux->relicIcon = nullptr;

    lux->wheelIcon = lv_colorwheel_create(lux->mainButton, true);
    lv_colorwheel_set_mode_fixed(lux->wheelIcon, true);
    lv_obj_set_width(lux->wheelIcon, 180);
    lv_obj_set_height(lux->wheelIcon, 180);
    lv_obj_set_x(lux->wheelIcon, 15);
    lv_obj_set_y(lux->wheelIcon, -20);
    lv_obj_set_align(lux->wheelIcon, LV_ALIGN_LEFT_MID);
    lv_obj_set_style_blend_mode(lux->wheelIcon, LV_BLEND_MODE_NORMAL, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_arc_width(lux->wheelIcon, 15, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_add_flag(lux->wheelIcon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(lux->wheelIcon, on_lux_color_event, LV_EVENT_RELEASED, lux);

    lux->slider = lv_slider_create(lux->mainButton);
    lv_slider_set_value(lux->slider, 50, LV_ANIM_OFF);
    lv_obj_set_height(lux->slider, 20);
    lv_obj_set_width(lux->slider, lv_pct(96));
    lv_obj_set_x(lux->slider, 0);
    lv_obj_set_y(lux->slider, 120);
    lv_obj_set_align(lux->slider, LV_ALIGN_RIGHT_MID);
    lv_obj_set_style_radius(lux->slider, 8, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_color(lux->slider, lv_color_hex(0x000000), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(lux->slider, 255, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_grad_color(lux->slider, lv_color_hex(0xFFFFFF), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_grad_dir(lux->slider, LV_GRAD_DIR_HOR, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_border_opa(lux->slider, 0, lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(lux->slider, 0, lv_selector(LV_PART_INDICATOR, LV_STATE_DEFAULT));
    lv_obj_set_style_border_opa(lux->slider, 0, lv_selector(LV_PART_INDICATOR, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_color(lux->slider, lv_color_hex(0xFFFFFF), lv_selector(LV_PART_KNOB, LV_STATE_DEFAULT));
    lv_obj_set_style_bg_opa(lux->slider, 255, lv_selector(LV_PART_KNOB, LV_STATE_DEFAULT));
    lv_obj_set_style_border_color(lux->slider, lv_color_hex(0x000000), lv_selector(LV_PART_KNOB, LV_STATE_DEFAULT));
    lv_obj_set_style_border_opa(lux->slider, 255, lv_selector(LV_PART_KNOB, LV_STATE_DEFAULT));
    lv_obj_set_style_border_width(lux->slider, 4, lv_selector(LV_PART_KNOB, LV_STATE_DEFAULT));
    lv_obj_add_flag(lux->slider, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(lux->slider, on_lux_brightness_event, LV_EVENT_RELEASED, lux);
}

static void create_dynamic_relic_ui() {
    if (ui_Container5) {
        lv_obj_clean(ui_Container5);
        lv_obj_set_scroll_dir(ui_Container5, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(ui_Container5, LV_SCROLLBAR_MODE_AUTO);
        for (size_t i = 0; i < LUX_COUNT; ++i) {
            create_lux_card(ui_Container5, &luxes[i], &LUX_CONFIGS[i]);
        }
    }

    if (uic_Container2) {
        lv_obj_clean(uic_Container2);
        lv_obj_set_scroll_dir(uic_Container2, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(uic_Container2, LV_SCROLLBAR_MODE_AUTO);
        for (size_t i = 0; i < LANTERN_COUNT; ++i) {
            create_lantern_card(uic_Container2, &lanterns[i], &LANTERN_CONFIGS[i]);
        }
    }
}




void setup() {

    delay(500);
    debug_begin(115200);

    static esp_lcd_panel_handle_t panel_handle = NULL; // Declare a handle for the LCD panel
    static esp_lcd_touch_handle_t tp_handle = NULL;    // Declare a handle for the touch panel

    // Initialize the GT911 touch screen controller
    tp_handle = touch_gt911_init();  

    // Initialize the Waveshare ESP32-S3 RGB LCD hardware
    panel_handle = waveshare_esp32_s3_rgb_lcd_init(); 

    // Turn on the LCD backlight
    wavesahre_rgb_lcd_bl_on();   

    // Initialize LVGL with the panel and touch handles
    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle));

    // Lock the mutex because LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        ui_init();

        lv_label_set_long_mode(CONSOLE_LABEL, LV_LABEL_LONG_WRAP);
        create_dynamic_relic_ui();

        
        lvgl_port_unlock();
    }

    // Initialize the Arcanet network
    arcanet.init();
    delay(500);

    debug_println("#################################");
    debug_println("### HeptaGlyph setup complete ###");
    debug_println("#################################");
    debug_println("My ID is: " + String(MY_ID));
}

void loop() {
    arcanet.loop();
    checkStaleness();
    delay(1);

}

void checkStaleness() {
    uint32_t now = millis();
    if ( now>tMinStalenessTime && now>tCheckStalenessPeriod && (now - tCheckStalenessPeriod) > tLastStalenessCheck) {
        if (lvgl_port_lock(-1)) {
            for (auto &r : lanterns) {
                if (!r.mainButton || !r.networkIcon || !r.batteryIcon || !r.relicIcon) continue;
                if (r.lastUpdate < (now-tMinStalenessTime)) {
                    lv_img_set_src(r.networkIcon, &ui_img_wifi_disabled_png);
                    lv_img_set_src(r.batteryIcon, &ui_img_battery_disabled_png);
                    lv_img_set_src(r.relicIcon, &ui_img_lantern_disabled_png);

                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x9CA3AF), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
                } else {
                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x8F8376), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
                }
            }
            for (auto &r : luxes) {
                if (!r.mainButton || !r.networkIcon || !r.batteryIcon || !r.wheelIcon || !r.slider) continue;
                if (r.lastUpdate < (now-tMinStalenessTime)) {
                    lv_img_set_src(r.networkIcon, &ui_img_wifi_disabled_png);
                    lv_img_set_src(r.batteryIcon, &ui_img_battery_disabled_png);

                    lv_obj_add_flag(r.wheelIcon, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(r.slider, LV_OBJ_FLAG_HIDDEN);

                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x9CA3AF), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
                } else {
                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x8F8376), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
                }
            }
            lvgl_port_unlock();
        }
        tLastStalenessCheck = now;
    }
}


// Generic lookup by string
Lantern* findLanternByNetID(const char* id) {
    for (auto &r : lanterns) {
        if (strcmp(r.netID, id) == 0) {
            return &r;
        }
    }
    return nullptr;
}

// Generic lookup by string
LuxArcana* findLuxByNetID(const char* id) {
    for (auto &r : luxes) {
        if (strcmp(r.netID, id) == 0) {
            return &r;
        }
    }
    return nullptr;
}


static const lv_img_dsc_t *pick_lantern_battery_icon(int mv) {
    if(mv <= 3100) return &ui_img_battery_empty_png;
    if(mv <= 3600) return &ui_img_battery_0_25_png;
    if(mv <= 3840) return &ui_img_battery_25_50_png;
    if(mv <= 3980) return &ui_img_battery_50_75_png;
    return &ui_img_battery_75_100_png;
}
static const lv_img_dsc_t *pick_lux_battery_icon(int mv) {
    if(mv <= 12500) return &ui_img_battery_empty_png;
    if(mv <= 12950) return &ui_img_battery_0_25_png;
    if(mv <= 13100) return &ui_img_battery_25_50_png;
    if(mv <= 13250) return &ui_img_battery_50_75_png;
    return &ui_img_battery_75_100_png;
}

static const lv_img_dsc_t *pick_wifi_icon(int rssi_dbm) {
    if(rssi_dbm >= -40) return &ui_img_wifi_80_100_png;
    if(rssi_dbm >= -55) return &ui_img_wifi_60_80_png;
    if(rssi_dbm >= -70) return &ui_img_wifi_40_60_png;
    if(rssi_dbm >= -84) return &ui_img_wifi_20_40_png;
    return &ui_img_wifi_0_20_png;
}

static const lv_img_dsc_t *pick_lantern_icon(int on) {
    return on ? &ui_img_lantern_on_png : &ui_img_lantern_off_png;
}  

void ui_update_lantern_status(Lantern* lantern, int batt_mv, int rssi_dbm, char* status) {
    if (lantern && lantern->mainButton && lantern->networkIcon && lantern->batteryIcon && lantern->relicIcon && lvgl_port_lock(-1)) {
        lv_img_set_src(lantern->networkIcon, pick_wifi_icon(rssi_dbm));
        lv_img_set_src(lantern->batteryIcon, pick_lantern_battery_icon(batt_mv));

        int on = (strcasecmp(status, "ON") == 0);
        lv_img_set_src(lantern->relicIcon, pick_lantern_icon(on));
        lantern->on = on;

        lv_obj_set_style_bg_color(lantern->mainButton, lv_color_hex(0x8F8376), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));

        lvgl_port_unlock();
    }
}  

int ui_apply_status_string(const char *msg) {

    if(!msg) return -1;
    char buf[128];
    size_t n = strlen(msg);
    if(n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, msg, n);
    buf[n] = '\0';

    char *id = nullptr;
    int mv = 0;
    int rssi = -127;
    // int on = 0;
    char *status1 = nullptr;
    char *status2 = nullptr;
    char *status3 = nullptr;
    char *status4 = nullptr;
    char *status5 = nullptr;
    char *status6 = nullptr;

    char *saveptr = NULL;
    char *tok = strtok_r(buf, "_", &saveptr);
    int field = 0;

    while(tok) {

        if (!id) {
            id = tok;
        } else if (strcmp(tok, "BLVL") == 0) {
            char *v = strtok_r(NULL, "_", &saveptr); 
            if(!v) break; 
            mv = atoi(v);
        } else if (strcmp(tok, "SGNL") == 0) {
            char *v = strtok_r(NULL, "_", &saveptr); 
            if(!v) break; 
            rssi = atoi(v);
        } else if (strcmp(tok, "STATE") == 0) {
            status1 = strtok_r(NULL, "_", &saveptr); 
            if(!status1) break;
            status2 = strtok_r(NULL, "_", &saveptr); 
            status3 = strtok_r(NULL, "_", &saveptr); 
            status4 = strtok_r(NULL, "_", &saveptr); 
            status5 = strtok_r(NULL, "_", &saveptr); 
            status6 = strtok_r(NULL, "_", &saveptr); 
            // on = (strcasecmp(v, "ON") == 0);
        }
//        char def0[] = "0";

        tok = strtok_r(NULL, "_", &saveptr);
        field++;
    }
    if (!status1) return -2;
    if(!id) return -2;

    String sStatus1 = String(status1);
    Lantern* lantern = findLanternByNetID(id);
    if (lantern) {
        ui_update_lantern_status(lantern, mv, rssi, status1);
        lantern->lastUpdate = millis();
    } else {
        if (!status2 ||!status3 || !status4) return -2;

        LuxArcana* lux = findLuxByNetID(id);
        if (lux) {
            ui_update_lux_status(lux, mv, rssi, status1, status2, status3, status4, status5);
            lux->lastUpdate = millis();
        }
    }

    return 0;
} 

void ui_update_lux_status(LuxArcana* lux, int batt_mv, int rssi_dbm, char* status1, char* status2, char* status3, char* status4, char* status5) {
    float fStatR = atof(status1);
    float fStatG = atof(status2);
    float fStatB = atof(status3);
    float fStatW = atof(status4);
    float fStatBright = atof(status5);

    if (lux && lux->mainButton && lux->networkIcon && lux->batteryIcon && lux->wheelIcon && lux->slider && lvgl_port_lock(-1)) {

        lux->_rgbw.r = fStatR;
        lux->_rgbw.g = fStatG;
        lux->_rgbw.b = fStatB;
        lux->_rgbw.w = fStatW;
        uint8_t r = fStatBright>0.0 ? fStatR*255 : 0;
        uint8_t g = fStatBright>0.0 ? fStatG*255 : 0;
        uint8_t b = fStatBright>0.0 ? fStatB*255 : 0;
        uint8_t w = fStatBright>0.0 ? fStatW*255 : 0;

        lv_obj_set_style_bg_color(lux->mainButton, lv_color_hex(0x8F8376), lv_selector(LV_PART_MAIN, LV_STATE_DEFAULT));
        lv_img_set_src(lux->networkIcon, pick_wifi_icon(rssi_dbm));
        lv_img_set_src(lux->batteryIcon, pick_lux_battery_icon(batt_mv));

        lv_color_t colorRgb = lv_color_make(r, g, b);
        lv_colorwheel_set_rgb(lux->wheelIcon, colorRgb);
        lv_obj_clear_flag(lux->wheelIcon, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lux->slider, LV_OBJ_FLAG_HIDDEN);
        lv_slider_set_value(lux->slider, fStatBright*100, LV_ANIM_OFF);

        lvgl_port_unlock();
    }
}

void console_clear() {
    consoleBuffer[0] = '\0';
    consoleLineCount = 0;

    if (lvgl_port_lock(-1)) {
        lv_label_set_text(CONSOLE_LABEL, "");
        lvgl_port_unlock();
    }
}

void console_append(const String& msg) {
    // Make one line from the incoming command
    String line = "[" + formatTimestamp() + "] " + msg;
    line.replace("\r", "");
    line.replace("\n", " ");   // keep each command on one visual log line
    line += "\n";

    // If one line is too large, truncate it
    if (line.length() > 100) {
        line = line.substring(0, 97) + "...\n";
    }

    // If buffer would overflow, drop oldest lines until it fits
    while ((strlen(consoleBuffer) + line.length() >= CONSOLE_MAX_CHARS) || 
           (consoleLineCount >= CONSOLE_MAX_LINES)) {

        char* firstNewline = strchr(consoleBuffer, '\n');
        if (firstNewline) {
            size_t remainingLen = strlen(firstNewline + 1);
            memmove(consoleBuffer, firstNewline + 1, remainingLen + 1);
            consoleLineCount--;
        } else {
            consoleBuffer[0] = '\0';
            consoleLineCount = 0;
            break;
        }
    }

    // Append new line
    strncat(consoleBuffer, line.c_str(), CONSOLE_MAX_CHARS - strlen(consoleBuffer) - 1);
    consoleLineCount++;

    // Update LVGL label
    if (lvgl_port_lock(-1)) {
//        lv_obj_set_width(CONSOLE_LABEL, lv_obj_get_width(CONSOLE_LABEL)); // ensures wrapping uses current width
        lv_label_set_text(CONSOLE_LABEL, consoleBuffer);
        lvgl_port_unlock();
    }
}

String formatTimestamp() {
    uint32_t totalSeconds = millis() / 1000;
    uint32_t hours   = totalSeconds / 3600;
    uint32_t minutes = (totalSeconds % 3600) / 60;
    uint32_t seconds = totalSeconds % 60;

    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds);

    return String(buf);
}
