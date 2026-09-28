#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void lvgl_port_init(void);

/* ---- page control ---- */
#define PAGE_HOME   0
#define PAGE_CONFIG 1
void ui_show_page(int page);      /* switch page */
int  ui_get_page(void);

/* config selection: 0=CH1 UNIT, 1=CH1 MAX, 2=CH2 UNIT, 3=CH2 MAX */
void ui_config_select(int idx);
int  ui_config_get_select(void);

/* ---- home page live values (thread-safe) ---- */
void ui_set_channels(const char *ch1_live, const char *ch1_peak,
                     const char *ch2_live, const char *ch2_peak);
void ui_set_battery(int percent);

/* pause/resume LVGL rendering (wrap flash writes to avoid PSRAM cache crash) */
void ui_lvgl_lock(void);
void ui_lvgl_unlock(void);

/* wrap NVS/flash writes: stops panel+LVGL during write to avoid cache crash */
void ui_flash_begin(void);
void ui_flash_end(void);

/* stop display + DMA before writing flash then rebooting */
void ui_display_teardown(void);

/* ---- config page values (thread-safe) ---- */
void ui_set_config(const char *ch1_unit, const char *ch1_max,
                   const char *ch2_unit, const char *ch2_max);

/* ZERO/PEAK overlay */
void ui_overlay_show(const char *title);
void ui_overlay_highlight(int idx);
int  ui_overlay_get_highlight(void);
void ui_overlay_hide(void);

#ifdef __cplusplus
}
#endif

#endif