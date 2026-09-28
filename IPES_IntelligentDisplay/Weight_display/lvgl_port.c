#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "user_config.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "src/st7701_bsp/esp_lcd_st7701.h"
#include "src/io_additions/esp_lcd_panel_io_additions.h"

static SemaphoreHandle_t lvgl_mux = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * BYTES_PER_PIXEL)
#define EXAMPLE_LCD_BIT_PER_PIXEL 16

uint8_t *lvgl_dest = NULL;
static esp_lcd_panel_handle_t g_panel = NULL;  /* saved for safe flash writes */
static TaskHandle_t g_lvgl_task = NULL;         /* LVGL task, for suspend during flash */

static esp_lcd_panel_handle_t rgb_port_init(void);
static void example_increase_lvgl_tick(void *arg);
static void example_lvgl_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);
static void lvgl_flush_wait_cb(lv_display_t * disp);
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);
static void example_lvgl_port_task(void *arg);
static void build_home_page(void);
static void build_config_page(void);
static void apply_config_selection(void);

static const st7701_lcd_init_cmd_t lcd_init_cmds[] =
{
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xEF,(uint8_t []){0x08},1,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x10},5,0},
  {0xC0,(uint8_t []){0xE5,0x02},2,0},
  {0xC1,(uint8_t []){0x15,0x0A},2,0},
  {0xC2,(uint8_t []){0x07,0x02},2,0},
  {0xCC,(uint8_t []){0x10},1,0},
  {0xB0,(uint8_t []){0x00,0x08,0x51,0x0D,0xCE,0x06,0x00,0x08,0x08,0x24,0x05,0xD0,0x0F,0x6F,0x36,0x1F},16,0},
  {0xB1,(uint8_t []){0x00,0x10,0x4F,0x0C,0x11,0x05,0x00,0x07,0x07,0x18,0x02,0xD3,0x11,0x6E,0x34,0x1F},16,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x11},5,0},
  {0xB0,(uint8_t []){0x4D},1,0},
  {0xB1,(uint8_t []){0x37},1,0},
  {0xB2,(uint8_t []){0x87},1,0},
  {0xB3,(uint8_t []){0x80},1,0},
  {0xB5,(uint8_t []){0x4A},1,0},
  {0xB7,(uint8_t []){0x85},1,0},
  {0xB8,(uint8_t []){0x21},1,0},
  {0xB9,(uint8_t []){0x00,0x13},2,0},
  {0xC0,(uint8_t []){0x09},1,0},
  {0xC1,(uint8_t []){0x78},1,0},
  {0xC2,(uint8_t []){0x78},1,0},
  {0xD0,(uint8_t []){0x88},1,0},
  {0xE0,(uint8_t []){0x80,0x00,0x02},3,100},
  {0xE1,(uint8_t []){0x0F,0xA0,0x00,0x00,0x10,0xA0,0x00,0x00,0x00,0x60,0x60},11,0},
  {0xE2,(uint8_t []){0x30,0x30,0x60,0x60,0x45,0xA0,0x00,0x00,0x46,0xA0,0x00,0x00,0x00},13,0},
  {0xE3,(uint8_t []){0x00,0x00,0x33,0x33},4,0},
  {0xE4,(uint8_t []){0x44,0x44},2,0},
  {0xE5,(uint8_t []){0x0F,0x4A,0xA0,0xA0,0x11,0x4A,0xA0,0xA0,0x13,0x4A,0xA0,0xA0,0x15,0x4A,0xA0,0xA0},16,0},
  {0xE6,(uint8_t []){0x00,0x00,0x33,0x33},4,0},
  {0xE7,(uint8_t []){0x44,0x44},2,0},
  {0xE8,(uint8_t []){0x10,0x4A,0xA0,0xA0,0x12,0x4A,0xA0,0xA0,0x14,0x4A,0xA0,0xA0,0x16,0x4A,0xA0,0xA0},16,0},
  {0xEB,(uint8_t []){0x02,0x00,0x4E,0x4E,0xEE,0x44,0x00},7,0},
  {0xED,(uint8_t []){0xFF,0xFF,0x04,0x56,0x72,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x27,0x65,0x40,0xFF,0xFF},16,0},
  {0xEF,(uint8_t []){0x08,0x08,0x08,0x40,0x3F,0x64},6,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xE8,(uint8_t []){0x00,0x0E},2,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x00},5,0},
  {0x11,(uint8_t []){0x00},0,120},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xE8,(uint8_t []){0x00,0x0C},2,10},
  {0xE8,(uint8_t []){0x00,0x00},2,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x00},5,0},
  {0x3A,(uint8_t []){0x55},1,0},
  {0x36,(uint8_t []){0x00},1,0},
  {0x35,(uint8_t []){0x00},1,0},
  {0x29,(uint8_t []){0x00},0,20},
};

IRAM_ATTR static bool example_on_bounce_frame_finish_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
  BaseType_t hi = pdFALSE;
  xSemaphoreGiveFromISR(flush_done_semaphore, &hi);
  return hi == pdTRUE;
}

static esp_lcd_panel_handle_t rgb_port_init(void)
{
  spi_line_config_t line_config = {
    .cs_io_type = IO_TYPE_GPIO, .cs_gpio_num = EXAMPLE_LCD_IO_SPI_CS,
    .scl_io_type = IO_TYPE_GPIO, .scl_gpio_num = EXAMPLE_LCD_IO_SPI_SCK,
    .sda_io_type = IO_TYPE_GPIO, .sda_gpio_num = EXAMPLE_LCD_IO_SPI_SDO,
    .io_expander = NULL,
  };
  esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
  esp_lcd_panel_io_handle_t io_handle = NULL;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

  esp_lcd_rgb_panel_config_t rgb_config = {};
  rgb_config.clk_src = LCD_CLK_SRC_DEFAULT;
  rgb_config.psram_trans_align = 64;
  rgb_config.bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES;
  rgb_config.num_fbs = 2;
  rgb_config.data_width = 16;
  rgb_config.bits_per_pixel = 16;
  rgb_config.de_gpio_num = EXAMPLE_LCD_IO_RGB_DE;
  rgb_config.pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK;
  rgb_config.vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC;
  rgb_config.hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC;
  rgb_config.flags.fb_in_psram = true;
  rgb_config.disp_gpio_num = -1;
  rgb_config.data_gpio_nums[0]=EXAMPLE_LCD_IO_RGB_B0; rgb_config.data_gpio_nums[1]=EXAMPLE_LCD_IO_RGB_B1;
  rgb_config.data_gpio_nums[2]=EXAMPLE_LCD_IO_RGB_B2; rgb_config.data_gpio_nums[3]=EXAMPLE_LCD_IO_RGB_B3;
  rgb_config.data_gpio_nums[4]=EXAMPLE_LCD_IO_RGB_B4; rgb_config.data_gpio_nums[5]=EXAMPLE_LCD_IO_RGB_G0;
  rgb_config.data_gpio_nums[6]=EXAMPLE_LCD_IO_RGB_G1; rgb_config.data_gpio_nums[7]=EXAMPLE_LCD_IO_RGB_G2;
  rgb_config.data_gpio_nums[8]=EXAMPLE_LCD_IO_RGB_G3; rgb_config.data_gpio_nums[9]=EXAMPLE_LCD_IO_RGB_G4;
  rgb_config.data_gpio_nums[10]=EXAMPLE_LCD_IO_RGB_G5; rgb_config.data_gpio_nums[11]=EXAMPLE_LCD_IO_RGB_R0;
  rgb_config.data_gpio_nums[12]=EXAMPLE_LCD_IO_RGB_R1; rgb_config.data_gpio_nums[13]=EXAMPLE_LCD_IO_RGB_R2;
  rgb_config.data_gpio_nums[14]=EXAMPLE_LCD_IO_RGB_R3; rgb_config.data_gpio_nums[15]=EXAMPLE_LCD_IO_RGB_R4;

  rgb_config.timings.pclk_hz = 18 * 1000 * 1000;
  rgb_config.timings.h_res = EXAMPLE_LCD_H_RES;
  rgb_config.timings.v_res = EXAMPLE_LCD_V_RES;
  rgb_config.timings.hsync_back_porch = 30;
  rgb_config.timings.hsync_front_porch = 30;
  rgb_config.timings.hsync_pulse_width = 6;
  rgb_config.timings.vsync_back_porch = 20;
  rgb_config.timings.vsync_front_porch = 20;
  rgb_config.timings.vsync_pulse_width = 40;

  st7701_vendor_config_t vendor_config = {};
  vendor_config.rgb_config = &rgb_config;
  vendor_config.init_cmds = lcd_init_cmds;
  vendor_config.init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t);
  vendor_config.flags.mirror_by_cmd = 1;
  vendor_config.flags.enable_io_multiplex = 0;

  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = EXAMPLE_LCD_IO_RGB_RESET,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,
    .vendor_config = &vendor_config,
  };
  esp_lcd_panel_handle_t panel_handle = NULL;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle));
  esp_lcd_rgb_panel_event_callbacks_t cbs = { .on_bounce_frame_finish = example_on_bounce_frame_finish_event };
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  return panel_handle;
}

/* ================= IPES UI: HOME + CONFIG pages ================= */

/* If mint/blue looks wrong, set to 1 and reflash to swap R<->B */
#define RGB_SWAPPED 0
static inline lv_color_t C(uint32_t rgb){
  if(RGB_SWAPPED){uint8_t r=(rgb>>16)&0xFF,g=(rgb>>8)&0xFF,b=rgb&0xFF;rgb=((uint32_t)b<<16)|((uint32_t)g<<8)|r;}
  return lv_color_hex(rgb);
}

/* logo image (generated files ipes_logo.c / ipes_logo_swapped.c) */
extern const lv_image_dsc_t ipes_logo;
extern const lv_image_dsc_t ipes_logo_swapped;
#define IPES_LOGO_SRC ipes_logo          /* switch to ipes_logo_swapped if orange */

#define COL_BG        0xE9E2CE   /* beige page bg          */
#define COL_BORDER    0x0B0B0B   /* dark panel border      */
#define COL_LIVE      0xBFE3F2   /* light blue live box    */
#define COL_LIVE_BDR  0x2C9BD6   /* blue border on sel     */
#define COL_PEAK      0xDDE1E6   /* grey peak box          */
#define COL_RED       0xF06A6A   /* selected (config) red  */
#define COL_NUM       0x14352B   /* dark green number      */
#define COL_LABEL     0x1A2A24   /* caption text           */
#define COL_SUB       0x2A3A34   /* subtitle grey          */
#define COL_WHITE     0xFFFFFF
#define COL_BLACK     0x000000
#define COL_BATT_GRN  0x2ECC40   /* vivid green */
#define COL_BATT_OFF  0xC8D2C4   /* light grey (empty seg) */

#define NBATT 4

/* screens */
static lv_obj_t *scr_home;
static lv_obj_t *scr_config;
static int cur_page = PAGE_HOME;
static int cfg_sel  = 0;   /* 0=CH1 UNIT,1=CH1 MAX,2=CH2 UNIT,3=CH2 MAX */

/* home widgets */
static lv_obj_t *h_ch1_live,*h_ch1_peak,*h_ch2_live,*h_ch2_peak;
static lv_obj_t *h_batt_pct,*h_batt_seg[NBATT];
/* config widgets */
static lv_obj_t *c_boxes[4];          /* the 4 config boxes (for red highlight) */
static lv_obj_t *c_ch1_unit,*c_ch1_max,*c_ch2_unit,*c_ch2_max;
static lv_obj_t *c_arrows[4];         /* up/down arrows on each config box     */
static lv_obj_t *c_batt_pct,*c_batt_seg[NBATT];

/* ---- ZERO/PEAK overlay ---- */
static lv_obj_t *ov_root=NULL;     /* dimmed backdrop + menu container */
static lv_obj_t *ov_title=NULL;
static lv_obj_t *ov_opt[3];        /* [CH1] [CH2] [CANCEL] */
static int ov_hl=0;                /* current highlight 0..2 */


/* ---- shared header (logo + subtitle + battery), returns the panel ---- */
static lv_obj_t* make_panel(lv_obj_t *parent)
{
    lv_obj_set_style_bg_color(parent, C(COL_BORDER), LV_PART_MAIN);
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, 806, 306);
    lv_obj_center(panel);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(panel, C(COL_BG), 0);
    lv_obj_set_style_border_width(panel, 5, 0);
    lv_obj_set_style_border_color(panel, C(COL_BORDER), 0);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_set_style_pad_all(panel, 8, 0);
    return panel;
}

/* battery: % text + segments + nub pointing LEFT (built into 'parent' panel) */
static void make_battery(lv_obj_t *panel, lv_obj_t **pct_out, lv_obj_t *seg_out[])
{
    lv_obj_t *pct = lv_label_create(panel);
    lv_label_set_text(pct, "90%");
    lv_obj_set_style_text_color(pct, C(COL_LABEL), 0);
    lv_obj_set_style_text_font(pct, &lv_font_montserrat_14, 0);
    lv_obj_align(pct, LV_ALIGN_TOP_RIGHT, -78, 8);
    *pct_out = pct;

    /* black tip pointing LEFT: nub sits on the LEFT side of the body */
    lv_obj_t *body = lv_obj_create(panel);
    lv_obj_set_size(body, 56, 26);
    lv_obj_align(body, LV_ALIGN_TOP_RIGHT, -12, 4);
    lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(body, C(COL_WHITE), 0);
    lv_obj_set_style_border_width(body, 2, 0);
    lv_obj_set_style_border_color(body, C(COL_BLACK), 0);
    lv_obj_set_style_radius(body, 3, 0);
    lv_obj_set_style_pad_all(body, 2, 0);

    lv_obj_t *nub = lv_obj_create(panel);
    lv_obj_set_size(nub, 4, 12);
    lv_obj_align_to(nub, body, LV_ALIGN_OUT_LEFT_MID, -1, 0);  /* LEFT tip */
    lv_obj_set_style_bg_color(nub, C(COL_BLACK), 0);
    lv_obj_set_style_border_width(nub, 0, 0);
    lv_obj_set_style_radius(nub, 1, 0);

    for (int i=0;i<NBATT;i++){
        seg_out[i]=lv_obj_create(body);
        lv_obj_set_size(seg_out[i],10,18);
        lv_obj_align(seg_out[i],LV_ALIGN_LEFT_MID,i*12,0);
        lv_obj_clear_flag(seg_out[i],LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(seg_out[i],0,0);
        lv_obj_set_style_radius(seg_out[i],1,0);
        lv_obj_set_style_bg_color(seg_out[i],C(COL_BATT_GRN),0);
    }
}

/* header logo + subtitle */
static void make_header(lv_obj_t *panel, const char *subtitle)
{
    lv_obj_t *logo = lv_image_create(panel);
    lv_image_set_src(logo, &IPES_LOGO_SRC);
    lv_image_set_scale(logo, 256);              /* 256 = 1.0x, NO scaling -> crisp */
    lv_image_set_antialias(logo, false);        /* avoid softening on rotation     */
    lv_obj_align(logo, LV_ALIGN_TOP_LEFT, 4, 2);

    lv_obj_t *sub = lv_label_create(panel);
    lv_label_set_text(sub, subtitle);
    lv_obj_set_style_text_color(sub, C(COL_SUB), 0);
    lv_obj_set_style_text_font(sub, &lv_font_montserrat_14, 0);
    lv_obj_align(sub, LV_ALIGN_TOP_LEFT, 4, 46);  /* clearly below the 40px logo */
}

/* one value/config box */
static lv_obj_t* make_box(lv_obj_t *panel,int x,int y,int w,int h,uint32_t bg,
                          const char *caption, lv_obj_t **num_out)
{
    lv_obj_t *box=lv_obj_create(panel);
    lv_obj_set_size(box,w,h);
    lv_obj_align(box,LV_ALIGN_TOP_LEFT,x,y);
    lv_obj_clear_flag(box,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(box,C(bg),0);
    lv_obj_set_style_border_width(box,2,0);
    lv_obj_set_style_border_color(box,C(COL_BORDER),0);
    lv_obj_set_style_radius(box,6,0);
    lv_obj_set_style_pad_all(box,6,0);

    lv_obj_t *cap=lv_label_create(box);
    lv_label_set_text(cap,caption);
    lv_obj_set_style_text_color(cap,C(COL_LABEL),0);
    lv_obj_set_style_text_font(cap,&lv_font_montserrat_14,0);
    lv_obj_align(cap,LV_ALIGN_TOP_LEFT,2,0);

    lv_obj_t *num=lv_label_create(box);
    lv_label_set_text(num,"0.000  Kg");
    lv_obj_set_style_text_color(num,C(COL_NUM),0);
    lv_obj_set_style_text_font(num,&lv_font_montserrat_40,0);
    lv_obj_align(num,LV_ALIGN_CENTER,0,8);
    *num_out=num;
    return box;
}

/* ---- HOME PAGE ---- */
static void build_home_page(void)
{
    scr_home = lv_obj_create(NULL);
    lv_obj_t *panel = make_panel(scr_home);
    make_header(panel, "INTELLIGENT DIGITAL DISPLAY");
    make_battery(panel, &h_batt_pct, h_batt_seg);

    int gx=2, gy=66, bw=378, bh=104, gapx=14, gapy=10;
    lv_obj_t *b;
    b=make_box(panel,gx,             gy,           bw,bh,COL_LIVE,"CHANNEL 1",     &h_ch1_live);
    lv_obj_set_style_border_color(b,C(COL_LIVE_BDR),0);
    make_box(panel,gx+bw+gapx,       gy,           bw,bh,COL_PEAK,"CH1 PEAK VALUE",&h_ch1_peak);
    b=make_box(panel,gx,             gy+bh+gapy,   bw,bh,COL_LIVE,"CHANNEL 2",     &h_ch2_live);
    lv_obj_set_style_border_color(b,C(COL_LIVE_BDR),0);
    make_box(panel,gx+bw+gapx,       gy+bh+gapy,   bw,bh,COL_PEAK,"CH2 PEAK VALUE",&h_ch2_peak);
}

/* ---- CONFIG PAGE ---- */
static void build_config_page(void)
{
    scr_config = lv_obj_create(NULL);
    lv_obj_t *panel = make_panel(scr_config);
    make_header(panel, "INTELLIGENT DIGITAL DISPLAY : CONFIG PAGE");
    make_battery(panel, &c_batt_pct, c_batt_seg);

    int gx=2, gy=66, bw=378, bh=104, gapx=14, gapy=10;
    /* box0 = CH1 UNIT, box1 = CH1 MAX, box2 = CH2 UNIT, box3 = CH2 MAX */
    c_boxes[0]=make_box(panel,gx,           gy,           bw,bh,COL_LIVE,"CHANNEL 1 UNIT",&c_ch1_unit);
    c_boxes[1]=make_box(panel,gx+bw+gapx,   gy,           bw,bh,COL_PEAK,"CH1 MAX VALUE", &c_ch1_max);
    c_boxes[2]=make_box(panel,gx,           gy+bh+gapy,   bw,bh,COL_LIVE,"CHANNEL 2 UNIT",&c_ch2_unit);
    c_boxes[3]=make_box(panel,gx+bw+gapx,   gy+bh+gapy,   bw,bh,COL_PEAK,"CH2 MAX VALUE", &c_ch2_max);

    lv_label_set_text(c_ch1_unit,"Kg");
    lv_label_set_text(c_ch1_max, "100");
    lv_label_set_text(c_ch2_unit,"Kg");
    lv_label_set_text(c_ch2_max, "0.5");

    /* up/down arrows on each config box (shown only when that box is selected) */
    for (int i=0;i<4;i++){
        c_arrows[i]=lv_label_create(c_boxes[i]);
        lv_label_set_text(c_arrows[i], LV_SYMBOL_UP "\n" LV_SYMBOL_DOWN);
        lv_obj_set_style_text_color(c_arrows[i], C(COL_BLACK), 0);
        lv_obj_set_style_text_font(c_arrows[i], &lv_font_montserrat_20, 0);
        lv_obj_align(c_arrows[i], LV_ALIGN_RIGHT_MID, -8, 6);
        lv_obj_add_flag(c_arrows[i], LV_OBJ_FLAG_HIDDEN);
    }

    apply_config_selection();
}

/* paint selected config box red, others to their base color; place arrows */
static void apply_config_selection(void)
{
    uint32_t base[4] = {COL_LIVE, COL_PEAK, COL_LIVE, COL_PEAK};
    for (int i=0;i<4;i++){
        lv_obj_set_style_bg_color(c_boxes[i], C(i==cfg_sel?COL_RED:base[i]), 0);
        if (i==cfg_sel) lv_obj_clear_flag(c_arrows[i], LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(c_arrows[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/* ---- ZERO/PEAK overlay UI ---- */
static void build_overlay(void)
{
    /* full-screen dim backdrop on the top layer so it floats over any page */
    ov_root = lv_obj_create(lv_layer_top());
    lv_obj_set_size(ov_root, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(ov_root, C(COL_BLACK), 0);
    lv_obj_set_style_bg_opa(ov_root, LV_OPA_50, 0);
    lv_obj_set_style_border_width(ov_root, 0, 0);
    lv_obj_set_style_radius(ov_root, 0, 0);
    lv_obj_clear_flag(ov_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ov_root, LV_OBJ_FLAG_HIDDEN);

    /* centered menu card */
    lv_obj_t *card = lv_obj_create(ov_root);
    lv_obj_set_size(card, 520, 180);
    lv_obj_center(card);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(card, C(COL_BG), 0);
    lv_obj_set_style_border_width(card, 3, 0);
    lv_obj_set_style_border_color(card, C(COL_BORDER), 0);
    lv_obj_set_style_radius(card, 8, 0);
    lv_obj_set_style_pad_all(card, 10, 0);

    ov_title = lv_label_create(card);
    lv_label_set_text(ov_title, "SELECT CHANNEL");
    lv_obj_set_style_text_color(ov_title, C(COL_LABEL), 0);
    lv_obj_set_style_text_font(ov_title, &lv_font_montserrat_20, 0);
    lv_obj_align(ov_title, LV_ALIGN_TOP_MID, 0, 0);

    const char *names[3] = {"CH1","CH2","CANCEL"};
    int w=150, h=70, gap=10;
    int total = 3*w + 2*gap;
    int x0 = -(total/2) + w/2;
    for (int i=0;i<3;i++){
        ov_opt[i]=lv_obj_create(card);
        lv_obj_set_size(ov_opt[i], w, h);
        lv_obj_align(ov_opt[i], LV_ALIGN_BOTTOM_MID, x0 + i*(w+gap), -4);
        lv_obj_clear_flag(ov_opt[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(ov_opt[i], 2, 0);
        lv_obj_set_style_border_color(ov_opt[i], C(COL_BORDER), 0);
        lv_obj_set_style_radius(ov_opt[i], 6, 0);
        lv_obj_t *l=lv_label_create(ov_opt[i]);
        lv_label_set_text(l, names[i]);
        lv_obj_set_style_text_color(l, C(COL_NUM), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_20, 0);
        lv_obj_center(l);
    }
}

static void overlay_paint(void)
{
    for (int i=0;i<3;i++)
        lv_obj_set_style_bg_color(ov_opt[i], C(i==ov_hl?COL_RED:COL_PEAK), 0);
}

void ui_overlay_show(const char *title)
{
    if (example_lvgl_lock(100)){
        if (title) lv_label_set_text(ov_title, title);
        ov_hl=0; overlay_paint();
        lv_obj_clear_flag(ov_root, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(ov_root);
        example_lvgl_unlock();
    }
}
void ui_overlay_highlight(int idx)
{
    if (idx<0)idx=0; if(idx>2)idx=2;
    if (example_lvgl_lock(100)){ ov_hl=idx; overlay_paint(); example_lvgl_unlock(); }
}
int ui_overlay_get_highlight(void){ return ov_hl; }
void ui_overlay_hide(void)
{
    if (example_lvgl_lock(100)){ lv_obj_add_flag(ov_root, LV_OBJ_FLAG_HIDDEN); example_lvgl_unlock(); }
}

/* ---- public page/config API ---- */
void ui_show_page(int page)
{
    if (example_lvgl_lock(100)) {
        cur_page = page;
        lv_screen_load(page==PAGE_CONFIG ? scr_config : scr_home);
        example_lvgl_unlock();
    }
}
int ui_get_page(void){ return cur_page; }

void ui_config_select(int idx)
{
    if (idx<0) idx=0; if (idx>3) idx=3;
    if (example_lvgl_lock(100)) {
        cfg_sel = idx;
        apply_config_selection();
        example_lvgl_unlock();
    }
}
int ui_config_get_select(void){ return cfg_sel; }

/* ---- home live values ---- */
void ui_set_channels(const char *a,const char *b,const char *c2,const char *d)
{
    if (example_lvgl_lock(50)) {
        if (h_ch1_live&&a) lv_label_set_text(h_ch1_live,a);
        if (h_ch1_peak&&b) lv_label_set_text(h_ch1_peak,b);
        if (h_ch2_live&&c2)lv_label_set_text(h_ch2_live,c2);
        if (h_ch2_peak&&d) lv_label_set_text(h_ch2_peak,d);
        example_lvgl_unlock();
    }
}

static void set_batt(lv_obj_t *seg[], lv_obj_t *pct, int percent)
{
    if(percent<0)percent=0; if(percent>100)percent=100;
    int on=(percent*NBATT+50)/100;
    for(int i=0;i<NBATT;i++)
        lv_obj_set_style_bg_color(seg[i], C(i<on?COL_BATT_GRN:COL_BATT_OFF),0);
    if(pct){char b[8];snprintf(b,sizeof(b),"%d%%",percent);lv_label_set_text(pct,b);}
}
void ui_set_battery(int percent)
{
    if (example_lvgl_lock(50)) {
        set_batt(h_batt_seg,h_batt_pct,percent);
        set_batt(c_batt_seg,c_batt_pct,percent);
        example_lvgl_unlock();
    }
}

/* ---- config values ---- */
void ui_set_config(const char *u1,const char *m1,const char *u2,const char *m2)
{
    if (example_lvgl_lock(50)) {
        if (c_ch1_unit&&u1) lv_label_set_text(c_ch1_unit,u1);
        if (c_ch1_max &&m1) lv_label_set_text(c_ch1_max,m1);
        if (c_ch2_unit&&u2) lv_label_set_text(c_ch2_unit,u2);
        if (c_ch2_max &&m2) lv_label_set_text(c_ch2_max,m2);
        example_lvgl_unlock();
    }
}

/* ================= LVGL PORT / DISPLAY DRIVER ================= */
/* public LVGL lock wrappers so the .ino can pause rendering during flash writes
   (prevents 'Cache disabled but cached memory region accessed' crash) */
void ui_lvgl_lock(void){ example_lvgl_lock(-1); }
void ui_lvgl_unlock(void){ example_lvgl_unlock(); }

/* ---- safe flash-write guard ----
   Writing NVS disables the PSRAM cache; the RGB panel DMA continuously reads
   the framebuffer from PSRAM, so a write mid-refresh panics the S3 with
   "Cache disabled but cached memory region accessed". We therefore take the
   LVGL lock (stop the render task) AND disable the RGB panel output for the
   brief write, then re-enable. */
void ui_flash_begin(void){
  example_lvgl_lock(-1);
  if (g_lvgl_task) vTaskSuspend(g_lvgl_task);
  vTaskDelay(pdMS_TO_TICKS(10));
}
void ui_flash_end(void){
  if (g_lvgl_task) vTaskResume(g_lvgl_task);
  example_lvgl_unlock();
}

/* Tear down the display so the RGB DMA STOPS reading PSRAM. After this, NVS
   writes are safe (nothing accesses the PSRAM framebuffer). Intended to be
   called right before writing flash + rebooting; the display is not restored
   (the reboot re-inits it). This is the only reliable way to write flash on
   an RGB-panel S3 with the framebuffer in PSRAM. */
void ui_display_teardown(void){
  /* 1) suspend + delete the LVGL task so it stops calling lv_timer_handler */
  if (g_lvgl_task){ vTaskSuspend(g_lvgl_task); vTaskDelay(pdMS_TO_TICKS(20)); vTaskDelete(g_lvgl_task); g_lvgl_task=NULL; }
  vTaskDelay(pdMS_TO_TICKS(20));
  /* 2) delete the RGB panel -> stops the continuous DMA from PSRAM */
  if (g_panel){ esp_lcd_panel_del(g_panel); g_panel=NULL; }
  vTaskDelay(pdMS_TO_TICKS(20));
}

void lvgl_port_init(void)
{
  lvgl_mux = xSemaphoreCreateMutex();
  assert(lvgl_mux);
  flush_done_semaphore = xSemaphoreCreateBinary();
  assert(flush_done_semaphore);
  esp_lcd_panel_handle_t panel_handle = rgb_port_init();
  g_panel = panel_handle;
  lv_init();
  lv_display_t * disp = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
  lv_display_set_flush_cb(disp, example_lvgl_flush_cb);
  lv_display_set_flush_wait_cb(disp, lvgl_flush_wait_cb);
  uint8_t *buf_1 = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
  uint8_t *buf_2 = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
  lv_display_set_buffers(disp, buf_1, buf_2, BUFF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_user_data(disp, panel_handle);
#ifdef EXAMPLE_Rotate_90
  lvgl_dest = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);
#endif
  const esp_timer_create_args_t targ = { .callback=&example_increase_lvgl_tick, .name="lvgl_tick" };
  esp_timer_handle_t th=NULL;
  ESP_ERROR_CHECK(esp_timer_create(&targ,&th));
  ESP_ERROR_CHECK(esp_timer_start_periodic(th, EXAMPLE_LVGL_TICK_PERIOD_MS*1000));
  xTaskCreatePinnedToCore(example_lvgl_port_task,"LVGL",EXAMPLE_LVGL_TASK_STACK_SIZE,NULL,EXAMPLE_LVGL_TASK_PRIORITY,&g_lvgl_task,1);

  if (example_lvgl_lock(-1)) {
    build_home_page();
    build_config_page();
    build_overlay();
    lv_screen_load(scr_home);     /* boot to home */
    cur_page = PAGE_HOME;
    example_lvgl_unlock();
  }
}

static void example_lvgl_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p)
{
  esp_lcd_panel_handle_t ph = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
#ifdef EXAMPLE_Rotate_90
  lv_display_rotation_t rot = lv_display_get_rotation(disp);
  lv_area_t ra;
  if(rot != LV_DISPLAY_ROTATION_0){
    lv_color_format_t cf = lv_display_get_color_format(disp);
    ra=*area; lv_display_rotate_area(disp,&ra);
    uint32_t ss=lv_draw_buf_width_to_stride(lv_area_get_width(area),cf);
    uint32_t ds=lv_draw_buf_width_to_stride(lv_area_get_width(&ra),cf);
    int32_t w=lv_area_get_width(area),h=lv_area_get_height(area);
    lv_draw_sw_rotate(color_p,lvgl_dest,w,h,ss,ds,rot,cf);
    area=&ra;
  }
  esp_lcd_panel_draw_bitmap(ph,area->x1,area->y1,area->x2+1,area->y2+1,lvgl_dest);
#else
  esp_lcd_panel_draw_bitmap(ph,area->x1,area->y1,area->x2+1,area->y2+1,color_p);
#endif
}
static void lvgl_flush_wait_cb(lv_display_t * disp){ xSemaphoreTake(flush_done_semaphore,portMAX_DELAY); }
static bool example_lvgl_lock(int ms){
  assert(lvgl_mux);
  const TickType_t t=(ms==-1)?portMAX_DELAY:pdMS_TO_TICKS(ms);
  return xSemaphoreTake(lvgl_mux,t)==pdTRUE;
}
static void example_lvgl_unlock(void){ assert(lvgl_mux); xSemaphoreGive(lvgl_mux); }
static void example_lvgl_port_task(void *arg){
  uint32_t d=EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
  for(;;){
    if(example_lvgl_lock(-1)){ d=lv_timer_handler(); example_lvgl_unlock(); }
    if(d>EXAMPLE_LVGL_TASK_MAX_DELAY_MS)d=EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    else if(d<EXAMPLE_LVGL_TASK_MIN_DELAY_MS)d=EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
    vTaskDelay(pdMS_TO_TICKS(d));
  }
}
static void example_increase_lvgl_tick(void *arg){ lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS); }