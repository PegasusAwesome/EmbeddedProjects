/*
* Ported LVGL 8.4 and display the official demo interface.
*/
#include "lvgl_port.h"       // LVGL porting functions for integration
#include "ui/ui.h"
#include "lvgl.h"



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



        lvgl_port_unlock();
    }

    delay(1000);

    Serial.println("######################################");
    Serial.println("### WaveShare7BDemo setup complete ###");
    Serial.println("######################################");

}

void loop() {
    delay(1);

}


