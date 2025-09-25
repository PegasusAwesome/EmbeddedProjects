SET(SOURCES screens/ui_Screen2.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_wifi_disabled_png.c
    images/ui_img_battery_disabled_png.c
    images/ui_img_lantern_disabled_png.c
    images/ui_img_wifi_0_20_png.c
    images/ui_img_battery_empty_png.c
    images/ui_img_lantern_off_png.c
    images/ui_img_wifi_20_40_png.c
    images/ui_img_battery_0_25_png.c
    images/ui_img_lantern_on_png.c
    images/ui_img_wifi_40_60_png.c
    images/ui_img_battery_25_50_png.c
    images/ui_img_wifi_60_80_png.c
    images/ui_img_battery_50_75_png.c
    images/ui_img_wifi_80_100_png.c
    images/ui_img_battery_75_100_png.c
    images/ui_img_golem_base_png.c
    images/ui_img_golem_on_png.c
    images/ui_img_gem_transparent_png.c
    images/ui_img_gem_off_png.c
    images/ui_img_gem_disabled_png.c
    images/ui_img_golem_off_png.c)

add_library(ui ${SOURCES})
