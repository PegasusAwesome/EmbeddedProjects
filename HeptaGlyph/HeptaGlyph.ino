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
};

struct RelicRef {
    const char* name;
    const char* netID;
    lv_obj_t** mainButton;
    lv_obj_t** nameLabel;
    lv_obj_t** networkIcon;
    lv_obj_t** batteryIcon;
    lv_obj_t** relicIcon;
    lv_obj_t** wheelIcon;
};


static const RelicRef RELIC_REFS[] = {
    { "lantern_11", "LANTERN11", &uic_Lantern11Button, &uic_Lantern11Label, &uic_Network11Icon, &uic_Battery11Icon, &uic_Lantern11Icon, nullptr},
    { "lantern_12", "LANTERN12", &uic_Lantern12Button, &uic_Lantern12Label, &uic_Network12Icon, &uic_Battery12Icon, &uic_Lantern12Icon, nullptr},
    { "lantern_13", "LANTERN13", &uic_Lantern13Button, &uic_Lantern13Label, &uic_Network13Icon, &uic_Battery13Icon, &uic_Lantern13Icon, nullptr},
    { "lantern_14", "LANTERN14", &uic_Lantern14Button, &uic_Lantern14Label, &uic_Network14Icon, &uic_Battery14Icon, &uic_Lantern14Icon, nullptr},
    { "lantern_15", "LANTERN15", &uic_Lantern15Button, &uic_Lantern15Label, &uic_Network15Icon, &uic_Battery15Icon, &uic_Lantern15Icon, nullptr},
    { "lantern_16", "LANTERN16", &uic_Lantern16Button, &uic_Lantern16Label, &uic_Network16Icon, &uic_Battery16Icon, &uic_Lantern16Icon, nullptr},
    // { "lantern_17", "LANTERN17", &uic_Lantern17Button, &uic_Lantern17Label, &uic_Network17Icon, &uic_Battery17Icon, &uic_Lantern17Icon},
    // { "lantern_18", "LANTERN18", &uic_Lantern18Button, &uic_Lantern18Label, &uic_Network18Icon, &uic_Battery18Icon, &uic_Lantern18Icon},
    { "lantern_19", "LANTERN19", &uic_Lantern19Button, &uic_Lantern19Label, &uic_Network19Icon, &uic_Battery19Icon, &uic_Lantern19Icon, nullptr},
    { "lantern_20", "LANTERN20", &uic_Lantern20Button, &uic_Lantern20Label, &uic_Network20Icon, &uic_Battery20Icon, &uic_Lantern20Icon, nullptr},
    { "lantern_21", "LANTERN21", &uic_Lantern21Button, &uic_Lantern21Label, &uic_Network21Icon, &uic_Battery21Icon, &uic_Lantern21Icon, nullptr},
    { "lantern_22", "LANTERN22", &uic_Lantern22Button, &uic_Lantern22Label, &uic_Network22Icon, &uic_Battery22Icon, &uic_Lantern22Icon, nullptr},
    { "lantern_23", "LANTERN23", &uic_Lantern23Button, &uic_Lantern23Label, &uic_Network23Icon, &uic_Battery23Icon, &uic_Lantern23Icon, nullptr},
    { "lantern_24", "LANTERN24", &uic_Lantern24Button, &uic_Lantern24Label, &uic_Network24Icon, &uic_Battery24Icon, &uic_Lantern24Icon, nullptr},
    { "lux_1", "LUX1", &uic_Lux1Button, &uic_Lux1Label, &uic_Network1Icon, &uic_Battery1Icon, &uic_Lux1Icon, &uic_Colorwheel1},
    { "lux_2", "LUX2", &uic_Lux2Button, &uic_Lux2Label, &uic_Network2Icon, &uic_Battery2Icon, &uic_Lux2Icon, &uic_Colorwheel2},
    { "lux_3", "LUX3", &uic_Lux3Button, &uic_Lux3Label, &uic_Network3Icon, &uic_Battery3Icon, &uic_Lux3Icon, &uic_Colorwheel3},
    { "lux_4", "LUX4", &uic_Lux4Button, &uic_Lux4Label, &uic_Network4Icon, &uic_Battery4Icon, &uic_Lux4Icon, &uic_Colorwheel4},
    { "lux_5", "LUX5", &uic_Lux5Button, &uic_Lux5Label, &uic_Network5Icon, &uic_Battery5Icon, &uic_Lux5Icon, &uic_Colorwheel5},
    { "lux_6", "LUX6", &uic_Lux6Button, &uic_Lux6Label, &uic_Network6Icon, &uic_Battery6Icon, &uic_Lux6Icon, &uic_Colorwheel6},
    { "lux_7", "LUX7", &uic_Lux7Button, &uic_Lux7Label, &uic_Network7Icon, &uic_Battery7Icon, &uic_Lux7Icon, &uic_Colorwheel7},
};

static Lantern lanterns[] = {
    { "lantern_11", "LANTERN11", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_12", "LANTERN12", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_13", "LANTERN13", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_14", "LANTERN14", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_15", "LANTERN15", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_16", "LANTERN16", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    // { "lantern_17", "LANTERN17", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    // { "lantern_18", "LANTERN18", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_19", "LANTERN19", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_20", "LANTERN20", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_21", "LANTERN21", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_22", "LANTERN22", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_23", "LANTERN23", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lantern_24", "LANTERN24", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
};

static LuxArcana luxes[] = {
    { "lux_1", "LUX1", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_2", "LUX2", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_3", "LUX3", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_4", "LUX4", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_5", "LUX5", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_6", "LUX6", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_7", "LUX7", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
};

#define CONSOLE_LABEL uic_Console
static const int CONSOLE_MAX_LINES = 30;      // tune to label height
static const int CONSOLE_MAX_CHARS = 2048;    // total buffer size

static char consoleBuffer[CONSOLE_MAX_CHARS] = "";
static int consoleLineCount = 0;

static void debug_begin(unsigned long baud) {
    Serial.begin(baud);
    Serial0.begin(baud);
    delay(200);
}

static void debug_println(const char* message) {
    Serial.println(message);
    Serial0.println(message);
}

static void debug_println(const String& message) {
    Serial.println(message);
    Serial0.println(message);
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

    String luxID = String(lux->netID);
}



void setColorLux1(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX1", "SET_HUE_"+String(hsv.h));
}
void setColorLux2(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX2", "SET_HUE_"+String(hsv.h));
}
void setColorLux3(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX3", "SET_HUE_"+String(hsv.h));
}
void setColorLux4(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX4", "SET_HUE_"+String(hsv.h));
}
void setColorLux5(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX5", "SET_HUE_"+String(hsv.h));
}
void setColorLux6(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX6", "SET_HUE_"+String(hsv.h));
}
void setColorLux7(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX7", "SET_HUE_"+String(hsv.h));
}


void setBrightnessLux1(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX1", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux2(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX2", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux3(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX3", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux4(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX4", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux5(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX5", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux6(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX6", "SET_BRIGHTNESS_"+String(val));
}
void setBrightnessLux7(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    arcanet.sendCommand("LUX7", "SET_BRIGHTNESS_"+String(val));
}


void setToBlack1(lv_event_t * e) {
    arcanet.sendCommand("LUX1", "BLACK_ON");
}
void setToBlack2(lv_event_t * e) {
    arcanet.sendCommand("LUX2", "BLACK_ON");
}
void setToBlack3(lv_event_t * e) {
    arcanet.sendCommand("LUX3", "BLACK_ON");
}
void setToBlack4(lv_event_t * e) {
    arcanet.sendCommand("LUX4", "BLACK_ON");
}
void setToBlack5(lv_event_t * e) {
    arcanet.sendCommand("LUX5", "BLACK_ON");
}
void setToBlack6(lv_event_t * e) {
    arcanet.sendCommand("LUX6", "BLACK_ON");
}
void setToBlack7(lv_event_t * e) {
    arcanet.sendCommand("LUX7", "BLACK_ON");
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

        size_t refs_count = sizeof(RELIC_REFS) / sizeof(RELIC_REFS[0]);
        size_t lanterns_count = sizeof(lanterns) / sizeof(lanterns[0]);
        size_t luxes_count    = sizeof(luxes) / sizeof(luxes[0]);

        size_t n = (refs_count < lanterns_count) ? refs_count : lanterns_count;
        for (size_t i = 0; i < n; ++i) {
            lanterns[i].name        = RELIC_REFS[i].name;
            lanterns[i].netID       = RELIC_REFS[i].netID;
            lanterns[i].mainButton  = *RELIC_REFS[i].mainButton;
            lanterns[i].nameLabel   = *RELIC_REFS[i].nameLabel;
            lanterns[i].networkIcon = *RELIC_REFS[i].networkIcon;
            lanterns[i].batteryIcon = *RELIC_REFS[i].batteryIcon;
            lanterns[i].relicIcon   = *RELIC_REFS[i].relicIcon;
            lanterns[i].on          = 0;
            lanterns[i].lastUpdate  = 0;

            if (lanterns[i].mainButton) {
                lv_obj_add_event_cb(lanterns[i].mainButton, on_lantern_button_event, LV_EVENT_CLICKED, &lanterns[i]);
                lv_obj_set_style_bg_color(lanterns[i].mainButton, lv_color_hex(0x9CA3AF), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);

                lv_img_set_src(lanterns[i].networkIcon, &ui_img_wifi_disabled_png);
                lv_img_set_src(lanterns[i].batteryIcon, &ui_img_battery_disabled_png);
                lv_img_set_src(lanterns[i].relicIcon, &ui_img_lantern_disabled_png);

            }
        }
        for (size_t i = n; i < n+luxes_count; ++i) {
            luxes[i-n].name        = RELIC_REFS[i].name;
            luxes[i-n].netID       = RELIC_REFS[i].netID;
            luxes[i-n].mainButton  = *RELIC_REFS[i].mainButton;
            luxes[i-n].nameLabel   = *RELIC_REFS[i].nameLabel;
            luxes[i-n].networkIcon = *RELIC_REFS[i].networkIcon;
            luxes[i-n].batteryIcon = *RELIC_REFS[i].batteryIcon;
            luxes[i-n].relicIcon   = *RELIC_REFS[i].relicIcon;
            luxes[i-n].wheelIcon   = *RELIC_REFS[i].wheelIcon;
            luxes[i-n]._rgbw       = {0, 0, 0, 0};
            luxes[i-n].lastUpdate  = 0;

            if (luxes[i-n].mainButton) {
                lv_obj_add_event_cb(luxes[i-n].mainButton, on_lux_button_event, LV_EVENT_CLICKED, &luxes[i-n]);
                lv_obj_set_style_bg_color(luxes[i-n].mainButton, lv_color_hex(0x9CA3AF), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);

                lv_img_set_src(luxes[i-n].networkIcon, &ui_img_wifi_disabled_png);
                lv_img_set_src(luxes[i-n].batteryIcon, &ui_img_battery_disabled_png);
                lv_img_set_src(luxes[i-n].relicIcon, &ui_img_gem_disabled_png);
                lv_obj_add_flag(luxes[i-n].wheelIcon, LV_OBJ_FLAG_HIDDEN);
            }
        }

        
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
                if (r.lastUpdate < (now-tMinStalenessTime)) {
                    lv_img_set_src(r.networkIcon, &ui_img_wifi_disabled_png);
                    lv_img_set_src(r.batteryIcon, &ui_img_battery_disabled_png);
                    lv_img_set_src(r.relicIcon, &ui_img_lantern_disabled_png);

                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x9CA3AF), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);
                } else {
                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0xDDDDDD), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);
                }
            }
            for (auto &r : luxes) {
                if (r.lastUpdate < (now-tMinStalenessTime)) {
                    lv_img_set_src(r.networkIcon, &ui_img_wifi_disabled_png);
                    lv_img_set_src(r.batteryIcon, &ui_img_battery_disabled_png);
                    lv_img_set_src(r.relicIcon, &ui_img_gem_disabled_png);

                    lv_obj_set_style_bg_color(r.relicIcon, lv_color_hex(0x9CA3AF), LV_PART_MAIN);//div by 0.45 because that  happens at the lux side, we should actually fix the value that is send on the lux side
                    lv_obj_set_style_bg_opa(r.relicIcon, LV_OPA_COVER, LV_PART_MAIN);

                    lv_obj_add_flag(r.wheelIcon, LV_OBJ_FLAG_HIDDEN);

                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0x9CA3AF), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);
                } else {
                    lv_obj_set_style_bg_color(r.mainButton, lv_color_hex(0xDDDDDD), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);
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
    if (lantern && lvgl_port_lock(-1)) {
        lv_img_set_src(lantern->networkIcon, pick_wifi_icon(rssi_dbm));
        lv_img_set_src(lantern->batteryIcon, pick_lantern_battery_icon(batt_mv));

        int on = (strcasecmp(status, "ON") == 0);
        lv_img_set_src(lantern->relicIcon, pick_lantern_icon(on));
        lantern->on = on;

        lv_obj_set_style_bg_color(lantern->mainButton, lv_color_hex(0xDDDDDD), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);

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
            ui_update_lux_status(lux, mv, rssi, status1, status2, status3, status4);
            lux->lastUpdate = millis();
        }
    }

    return 0;
} 

void ui_update_lux_status(LuxArcana* lux, int batt_mv, int rssi_dbm, char* status1, char* status2, char* status3, char* status4) {
    float fStatR = atof(status1);
    float fStatG = atof(status2);
    float fStatB = atof(status3);
    float fStatW = atof(status4);

    if (lux && lvgl_port_lock(-1)) {

        lux->_rgbw.r = fStatR;
        lux->_rgbw.g = fStatG;
        lux->_rgbw.b = fStatB;
        lux->_rgbw.w = fStatW;
        uint8_t r = fStatR*255/0.45;
        uint8_t g = fStatG*255/0.45;
        uint8_t b = fStatB*255/0.45;
        uint8_t w = fStatW*255/0.45;

        lv_obj_set_style_bg_color(lux->mainButton, lv_color_hex(0xDDDDDD), (lv_style_selector_t)LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_img_set_src(lux->relicIcon, &ui_img_gem_transparent_png);
        lv_img_set_src(lux->networkIcon, pick_wifi_icon(rssi_dbm));
        lv_img_set_src(lux->batteryIcon, pick_lux_battery_icon(batt_mv));

        lv_color_t colorRgb = lv_color_make(r, g, b);
        lv_colorwheel_set_rgb(lux->wheelIcon, colorRgb);
        lv_obj_clear_flag(lux->wheelIcon, LV_OBJ_FLAG_HIDDEN);

        lv_obj_set_style_bg_color(lux->relicIcon, colorRgb, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(lux->relicIcon, LV_OPA_COVER, LV_PART_MAIN);

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
