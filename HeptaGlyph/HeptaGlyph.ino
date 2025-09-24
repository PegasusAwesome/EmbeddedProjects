/*
* Ported LVGL 8.4 and display the official demo interface.
*/
#include "Arcanet.h"
#include "lvgl_port.h"       // LVGL porting functions for integration
#include "ui/ui.h"
#include "lvgl.h"

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
    { "lux_2", "LUX2", &uic_Lux2Button, &uic_Lux2Label, &uic_Network2Icon, &uic_Battery2Icon, &uic_Lux2Icon, &uic_Colorwheel2},
    { "lux_3", "LUX3", &uic_Lux3Button, &uic_Lux3Label, &uic_Network3Icon, &uic_Battery3Icon, &uic_Lux3Icon, &uic_Colorwheel3},
    { "lux_4", "LUX4", &uic_Lux4Button, &uic_Lux4Label, &uic_Network4Icon, &uic_Battery4Icon, &uic_Lux4Icon, &uic_Colorwheel4},
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
    { "lux_2", "LUX2", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_3", "LUX3", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
    { "lux_4", "LUX4", 0, nullptr, nullptr, nullptr, nullptr, nullptr, 0},
};



// Callback function to handle received commands
void onCommandReceived(const String& id, const String& command) {
    if (id == MY_ID) {

    } else if (id == "CONTROLLER") {
        ui_apply_status_string(command.c_str());
    } else if (id == "ALL") {
        ui_apply_status_string(command.c_str());
    }
}
// Create an instance of the Arcanet library
Arcanet arcanet(MY_ID, onCommandReceived);


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



void setColorFromScreen2(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX2", "SET_HUE_"+String(hsv.h));

}
void setColorFromScreen3(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX3", "SET_HUE_"+String(hsv.h));

}
void setColorFromScreen4(lv_event_t * e) {
    lv_obj_t * cw = lv_event_get_target(e);
    lv_color_hsv_t hsv = lv_colorwheel_get_hsv(cw);
    arcanet.sendCommand("LUX4", "SET_HUE_"+String(hsv.h));
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
void activateGolemInSequence(lv_event_t * e) {
    arcanet.sendCommand("CONSOLE71", "PRESSEDGOLEMIN");
}

void activateGolemOutSequence(lv_event_t * e) {
    arcanet.sendCommand("CONSOLE71", "PRESSEDGOLEMOUT");
}


void setup() {
    delay(3000);
    Serial.begin(115200);
    delay(3000);

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
                lv_obj_set_style_bg_color(lanterns[i].mainButton, lv_color_hex(0x9CA3AF), LV_PART_MAIN | LV_STATE_DEFAULT);

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
                lv_obj_set_style_bg_color(luxes[i-n].mainButton, lv_color_hex(0x9CA3AF), LV_PART_MAIN | LV_STATE_DEFAULT);

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
    delay(1000);

    Serial.println("##################################");
    Serial.println("### HeptaGlyph setup complete ###");
    Serial.println("##################################");
    Serial.println("My ID is: "+String(MY_ID));
}

void loop() {
    delay(1);

}


