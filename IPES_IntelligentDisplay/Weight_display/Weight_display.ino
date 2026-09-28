/* Weight_display.ino  --  IPES dual-channel  (5-button model)
 *
 * BUTTONS:  POWER  SETTINGS  ZERO  PEAK  NAV
 *
 * HOME page:
 *   SETTINGS  -> instant switch to CONFIG page
 *   ZERO      -> open ZERO menu [CH1][CH2][CANCEL]; press ZERO again to close
 *   PEAK      -> open PEAK menu [CH1][CH2][CANCEL]; press PEAK again to close
 *                (only one menu open at a time; other button ignored meanwhile)
 *   NAV       -> (no menu open) does nothing
 *   POWER     -> toggle backlight
 *
 *   Inside an open ZERO/PEAK menu:
 *     NAV      -> move highlight CH1 -> CH2 -> CANCEL -> loop
 *     SETTINGS -> select highlighted option:
 *                   ZERO menu: CH1/CH2 = tare that channel, then close
 *                   PEAK menu: CH1/CH2 = calibrate 5V ref, then close
 *                   CANCEL   = close menu, no change
 *   (Home ZERO/PEAK actions are NOT saved to flash.)
 *
 * CONFIG page:
 *   SETTINGS  -> instant switch back to HOME page
 *   NAV       -> move selected box CH1U -> CH1MAX -> CH2U -> CH2MAX -> loop
 *   PEAK = UP, ZERO = DOWN on the selected box:
 *     UNIT boxes: cycle through 12 units (one step per press)
 *     MAX boxes : quick press = +/-0.01 ; hold >2s = +/-50 every 0.8s
 *   (Config settings ARE saved to flash and restored on restart.)
 *
 * ---- KEYBOARD TEST COMMANDS (type in Serial Monitor, 115200) ----
 *   s        = SETTINGS  (page switch / select in menu)
 *   z        = ZERO      (open/close zero menu ; or DOWN in config)
 *   p        = PEAK      (open/close peak menu ; or UP in config)
 *   n        = NAV       (navigate: menu highlight / config box)
 *   x        = SELECT    (same as SETTINGS-select inside a menu)
 *   u        = UP        (increase value in config)
 *   d        = DOWN      (decrease value in config)
 *   b        = POWER     (toggle backlight)
 *   Arrow keys also work where possible:
 *     Right/Down arrow = NAV ; Up arrow = UP ; Left arrow = DOWN
 */
#include "user_config.h"
#include "lvgl.h"
#include "lvgl_port.h"
#include "lcd_bl_pwm_bsp.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#define BTN_RX_PIN 44
#define BTN_TX_PIN 43
#define BTN_BAUD   115200
HardwareSerial BtnSerial(1);


Adafruit_ADS1115 ads;
bool ads_ok=false;

const char *UNIT_NAMES[]={"Kg","MM","CM","M","In","Ft","G","Mg","MT","lbm","L","mL"};
#define NUM_UNITS (int)(sizeof(UNIT_NAMES)/sizeof(UNIT_NAMES[0]))
int ch1_unit_idx=0, ch2_unit_idx=0;
#define ch1_unit (UNIT_NAMES[ch1_unit_idx])
#define ch2_unit (UNIT_NAMES[ch2_unit_idx])

float ch1_max=100.0f, ch2_max=0.5f;
float ch1_off=0.0f,  ch2_off=0.0f;
float ch1_vref=5.0f, ch2_vref=5.0f;
float peak1=0, peak2=0;

Preferences prefs;

/* Save config to NVS ONLY when leaving the config page (few, safe writes).
   The write is wrapped in ui_flash_begin/end which stops the RGB panel + LVGL
   during the write, preventing the S3 "cache disabled" PSRAM crash. */
bool cfg_changed=false;                 /* set when any config value edited */
void markConfigDirty(){ cfg_changed=true; }

void saveConfigSafe(){
  if (!cfg_changed){ ui_show_page(PAGE_HOME); return; }
  cfg_changed=false;

  Serial.println("saving... (display will blank, then reboot)");
  delay(30);

  /* Tear the display down completely so the RGB DMA stops reading PSRAM.
     Only then is an NVS write safe on this board. Screen goes blank here;
     the reboot below brings it back with the saved values. */
  ui_display_teardown();

  prefs.begin("ipescfg", false);
  prefs.putInt("u1", ch1_unit_idx);
  prefs.putInt("u2", ch2_unit_idx);
  prefs.putFloat("m1", ch1_max);
  prefs.putFloat("m2", ch2_max);
  prefs.end();
  Serial.println("cfg saved to NVS");

  delay(80);
  esp_restart();
}
void loadConfig(){
  /* No flash-guard here: called from setup() BEFORE the LVGL task/panel are
     actively rendering, so the read is safe. Using the guard here caused a
     boot loop (suspending a not-yet-ready task). */
  prefs.begin("ipescfg", true);   // read-only
  ch1_unit_idx = prefs.getInt("u1", 0);
  ch2_unit_idx = prefs.getInt("u2", 0);
  ch1_max      = prefs.getFloat("m1", 100.0f);
  ch2_max      = prefs.getFloat("m2", 0.5f);
  prefs.end();
  if (ch1_unit_idx<0||ch1_unit_idx>=NUM_UNITS) ch1_unit_idx=0;
  if (ch2_unit_idx<0||ch2_unit_idx>=NUM_UNITS) ch2_unit_idx=0;
}


float v0_now=0, v1_now=0;
float w1_now=0, w2_now=0;   /* last computed weights */

/* ---- battery monitor on ADS1115 A3 ---- */
#define BATT_DIV_RATIO 11.0f   /* (10k+1k)/1k = 11 ; divider MUST be on RAW battery, not 5V rail */
#define BATT_FULL_V    12.6f   /* 3S Li-ion full  -> 100% */
#define BATT_EMPTY_V   9.0f    /* 3S Li-ion empty -> 0%   */
float vbat_now=0;              /* last measured battery voltage */
int   batt_pct=0;             /* last computed percentage */

/* ---- menu state (HOME ZERO/PEAK overlay) ---- */
int  ov_mode=0;          /* 0=none, 1=ZERO menu, 2=PEAK menu */
int  ov_sel=0;           /* highlighted option 0=CH1 1=CH2 2=CANCEL */

/* config MAX fast-repeat state (hold PEAK/ZERO > 2s -> +/-50 every 0.8s) */
bool cfg_hold=false; int cfg_hold_dir=0; unsigned long cfg_hold_start=0, cfg_last_fast=0;
#define CFG_HOLD_MS   2000   /* hold this long to enter fast mode */
#define CFG_FAST_MS   800    /* then apply +/-50 every this interval */

void tryInitADS(){
  if (ads.begin(0x48,&Wire)){ ads_ok=true; ads.setGain(GAIN_TWOTHIRDS); Serial.println("ADS ok"); }
  else { ads_ok=false; Serial.println("ADS not found"); }
}
void fmt_val(float v,const char*u,char*o,size_t n){ if(v<0)v=0; snprintf(o,n,"%.3f  %s",v,u); }
void pushConfig(){
  char m1[16],m2[16];
  snprintf(m1,sizeof(m1),"%.2f",ch1_max);
  snprintf(m2,sizeof(m2),"%.2f",ch2_max);
  ui_set_config(ch1_unit,m1,ch2_unit,m2);
}
float weight(float v,float off,float vref,float maxv){
  float span=vref-off; if(span<0.01f)span=5.0f;
  float w=((v-off)/span)*maxv; return (w<0)?0:w;
}

/* ---- config edit helpers (PEAK=up dir+1, ZERO=down dir-1) ---- */
/* quick action: units cycle by 1; MAX changes by 0.01 */
void adjustQuick(int dir){
  int sel=ui_config_get_select();  /* 0=CH1U 1=CH1MAX 2=CH2U 3=CH2MAX */
  if (sel==0){ ch1_unit_idx=(ch1_unit_idx+dir+NUM_UNITS)%NUM_UNITS; }
  else if (sel==2){ ch2_unit_idx=(ch2_unit_idx+dir+NUM_UNITS)%NUM_UNITS; }
  else if (sel==1){ ch1_max+=dir*0.01f; if(ch1_max<0)ch1_max=0; }
  else if (sel==3){ ch2_max+=dir*0.01f; if(ch2_max<0)ch2_max=0; }
  pushConfig();
  markConfigDirty();
  Serial.printf("cfg quick sel=%d dir=%d\n",sel,dir);
}
/* fast action (hold): MAX changes by 50; ignored on unit boxes */
void adjustFast(int dir){
  int sel=ui_config_get_select();
  if (sel==1){ ch1_max+=dir*50.0f; if(ch1_max<0)ch1_max=0; pushConfig(); markConfigDirty(); }
  else if (sel==3){ ch2_max+=dir*50.0f; if(ch2_max<0)ch2_max=0; pushConfig(); markConfigDirty(); }
}

/* =============== ACTIONS =============== */
/* execute a ZERO-menu selection */
void doZero(int hl){
  if (hl==0){ ch1_off=v0_now; peak1=0; Serial.println("TARE CH1"); }
  else if (hl==1){ ch2_off=v1_now; peak2=0; Serial.println("TARE CH2"); }
  else Serial.println("ZERO cancel");
}
/* execute a PEAK-menu selection */
void doPeak(int hl){
  if (hl==0){ ch1_vref=v0_now; Serial.printf("CAL CH1 ref=%.3fV\n",ch1_vref); }
  else if (hl==1){ ch2_vref=v1_now; Serial.printf("CAL CH2 ref=%.3fV\n",ch2_vref); }
  else Serial.println("PEAK cancel");
}

/* open / close the home ZERO or PEAK menu (mode: 1=zero, 2=peak) */
void openMenu(int mode){
  ov_mode=mode; ov_sel=0;
  ui_overlay_show(mode==1 ? "ZERO - select channel" : "PEAK - select channel");
  ui_overlay_highlight(0);
}
void closeMenu(){
  ov_mode=0;
  ui_overlay_hide();
}

/* NAV press: move highlight/selection depending on context */
void navPress(){
  if (ov_mode!=0){                       /* a home menu is open */
    ov_sel=(ov_sel+1)%3;                  /* CH1->CH2->CANCEL->loop */
    ui_overlay_highlight(ov_sel);
  } else if (ui_get_page()==PAGE_CONFIG){ /* config: move box */
    int s=(ui_config_get_select()+1)%4;   /* CH1U->CH1MAX->CH2U->CH2MAX */
    ui_config_select(s);
  }
  /* Home with no menu: NAV does nothing */
}

/* SELECT (SETTINGS inside a menu, or 'x'): confirm menu choice */
void selectPress(){
  if (ov_mode==1){ doZero(ov_sel); closeMenu(); }
  else if (ov_mode==2){ doPeak(ov_sel); closeMenu(); }
  /* if no menu open, SELECT does nothing (SETTINGS page-switch handled separately) */
}

/* SETTINGS press: if a menu is open -> it acts as SELECT;
   otherwise -> instant page switch Home<->Config */
void settingsPress(){
  if (ov_mode!=0){ selectPress(); return; }
  int page=ui_get_page();
  if (page==PAGE_CONFIG){
    /* leaving config: if changed -> stage+reboot to save; else just go home */
    saveConfigSafe();          /* reboots if changed; else shows home */
  } else {
    ui_show_page(PAGE_CONFIG);
  }
}

/* ZERO button press (context-aware) */
void zeroPress(){
  int page=ui_get_page();
  if (page==PAGE_HOME){
    if (ov_mode==0) openMenu(1);          /* open zero menu */
    else if (ov_mode==1) closeMenu();     /* same button closes */
    /* if PEAK menu open (ov_mode==2): ignore */
  } else { /* CONFIG: ZERO = DOWN */
    adjustQuick(-1);
    cfg_hold=true; cfg_hold_dir=-1; cfg_hold_start=millis(); cfg_last_fast=0;
  }
}
void zeroRelease(){
  if (ui_get_page()==PAGE_CONFIG) cfg_hold=false;
}

/* PEAK button press (context-aware) */
void peakPress(){
  int page=ui_get_page();
  if (page==PAGE_HOME){
    if (ov_mode==0) openMenu(2);          /* open peak menu */
    else if (ov_mode==2) closeMenu();     /* same button closes */
    /* if ZERO menu open (ov_mode==1): ignore */
  } else { /* CONFIG: PEAK = UP */
    adjustQuick(+1);
    cfg_hold=true; cfg_hold_dir=+1; cfg_hold_start=millis(); cfg_last_fast=0;
  }
}
void peakRelease(){
  if (ui_get_page()==PAGE_CONFIG) cfg_hold=false;
}

/* =============== UART BUTTON EVENTS =============== */
void handleButton(const char *button,const char *event){
  bool pressed=(strcmp(event,"PRESSED")==0);
  Serial.printf("BTN %s %s (page %d, menu %d)\n",button,event,ui_get_page(),ov_mode);

  if (strcmp(button,"SETTINGS")==0){ if (pressed) settingsPress(); return; }
  if (strcmp(button,"NAV")==0){      if (pressed) navPress();      return; }

  if (strcmp(button,"ZERO")==0){
    if (pressed) zeroPress(); else zeroRelease();
    return;
  }
  if (strcmp(button,"PEAK")==0){
    if (pressed) peakPress(); else peakRelease();
    return;
  }

  if (strcmp(button,"POWER")==0 && pressed){
    static bool bl_on=true;
    bl_on=!bl_on;
    setUpduty(bl_on ? LCD_PWM_MODE_255 : LCD_PWM_MODE_0);
    Serial.printf("Backlight %s\n", bl_on ? "ON":"OFF");
  }
}

/* MAX fast repeat: after holding PEAK/ZERO > 2s, apply +/-50 every 0.8s */
void updateConfigHold(){
  if (!cfg_hold || ui_get_page()!=PAGE_CONFIG) return;
  unsigned long held = millis()-cfg_hold_start;
  if (held < CFG_HOLD_MS) return;               /* not yet in fast mode */
  if (millis()-cfg_last_fast >= CFG_FAST_MS){
    cfg_last_fast = millis();
    adjustFast(cfg_hold_dir);
  }
}

void processButtonUart(){
  static char buf[160]; static size_t idx=0;
  while (BtnSerial.available()){
    char c=BtnSerial.read();
    if (c=='\n'){
      buf[idx]=0; idx=0;
      if (strlen(buf)>0){
        StaticJsonDocument<160> doc;
        if (!deserializeJson(doc,buf)){
          const char*b=doc["button"]; const char*e=doc["event"];
          if (b&&e) handleButton(b,e);
        }
      }
    } else if (idx<sizeof(buf)-1) buf[idx++]=c;
  }
}


/* ---- KEYBOARD TEST INPUT (Serial Monitor) ----
   s=SETTINGS  z=ZERO  p=PEAK  n=NAV  x=SELECT  u=UP  d=DOWN  b=POWER
   Arrows: Right/Down=NAV, Up=UP, Left=DOWN  */
void handleKeyboard(){
  while (Serial.available()){
    int c=Serial.read();

    /* arrow keys arrive as ESC '[' 'A'/'B'/'C'/'D' */
    if (c==27){                       /* ESC */
      if (Serial.available() && Serial.peek()=='['){
        Serial.read();                /* consume '[' */
        int a=Serial.read();          /* A=up B=down C=right D=left */
        if (a=='A'){ if(ui_get_page()==PAGE_CONFIG) peakPress(); }   /* Up = UP */
        else if (a=='B'){ if(ui_get_page()==PAGE_CONFIG) zeroPress(); else navPress(); } /* Down = NAV/DOWN */
        else if (a=='C'){ navPress(); }                              /* Right = NAV */
        else if (a=='D'){ if(ui_get_page()==PAGE_CONFIG) zeroPress(); } /* Left = DOWN */
      }
      continue;
    }

    switch (c){
      case 's': case 'S': settingsPress(); break;
      case 'n': case 'N': navPress();      break;
      case 'x': case 'X': selectPress();   break;
      case 'z': case 'Z':                  /* ZERO: menu toggle or config-down */
        zeroPress();  zeroRelease();  break;
      case 'p': case 'P':                  /* PEAK: menu toggle or config-up */
        peakPress();  peakRelease();  break;
      case 'u': case 'U':                  /* UP in config */
        if (ui_get_page()==PAGE_CONFIG){ peakPress(); peakRelease(); } break;
      case 'd': case 'D':                  /* DOWN in config */
        if (ui_get_page()==PAGE_CONFIG){ zeroPress(); zeroRelease(); } break;
      case 'b': case 'B':{                 /* POWER toggle */
        static bool klbl=true; klbl=!klbl;
        setUpduty(klbl?LCD_PWM_MODE_255:LCD_PWM_MODE_0);
        Serial.printf("Backlight %s\n", klbl?"ON":"OFF");
      } break;
      default: break;                      /* ignore \r \n and others */
    }
  }
}

/* 1s periodic status line (interleaves with event logs, nothing removed) */
void printStatus(){
  static uint32_t t=0;
  if (millis()-t < 1000) return;
  t=millis();
  Serial.printf(
    "STATUS | CH1 cur=%.3f%s peak=%.3f%s max=%g ref=%.3fV | CH2 cur=%.3f%s peak=%.3f%s max=%g ref=%.3fV | BATT %.2fV %d%%\n",
    w1_now, ch1_unit, peak1, ch1_unit, ch1_max, ch1_vref,
    w2_now, ch2_unit, peak2, ch2_unit, ch2_max, ch2_vref,
    vbat_now, batt_pct);
}

int readBatteryPercent(){
  int16_t r3 = ads.readADC_SingleEnded(3);     /* A3 = divider midpoint */
  float vadc = ads.computeVolts(r3);
  vbat_now = vadc * BATT_DIV_RATIO;            /* real battery voltage */
  float pct = (vbat_now - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)(pct + 0.5f);
}

void setup(){
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== IPES page-aware buttons ===");
  loadConfig();          /* read saved settings FIRST, before display starts */
  lvgl_port_init();
  lcd_bl_pwm_bsp_init(LCD_PWM_MODE_255);
  Wire.begin(ESP32_SDA_NUM, ESP32_SCL_NUM);
  Wire.setClock(400000);
  tryInitADS();
  pushConfig();
  BtnSerial.begin(BTN_BAUD, SERIAL_8N1, BTN_RX_PIN, BTN_TX_PIN);
  Serial.println("Button UART ready");
}

void loop(){
  processButtonUart();
  handleKeyboard();
  updateConfigHold();
  printStatus();

  static uint32_t last=0;
  if (millis()-last<120) return;
  last=millis();

  char b1[24],p1[24],b2[24],p2[24];
  if (ads_ok){
    int16_t r0=ads.readADC_SingleEnded(0);
    int16_t r1=ads.readADC_SingleEnded(1);
    v0_now=ads.computeVolts(r0);
    v1_now=ads.computeVolts(r1);
    float w1=weight(v0_now,ch1_off,ch1_vref,ch1_max);
    float w2=weight(v1_now,ch2_off,ch2_vref,ch2_max);
    w1_now=w1; w2_now=w2;
    if(w1>peak1)peak1=w1;
    if(w2>peak2)peak2=w2;

    fmt_val(w1,ch1_unit,b1,sizeof(b1));
    fmt_val(w2,ch2_unit,b2,sizeof(b2));

    /* PEAK VALUE boxes: always live session peak */
    fmt_val(peak1,ch1_unit,p1,sizeof(p1));
    fmt_val(peak2,ch2_unit,p2,sizeof(p2));
  } else {
    strcpy(b1,"NO ADS");strcpy(p1,"----");
    strcpy(b2,"NO ADS");strcpy(p2,"----");
    tryInitADS();
  }
  ui_set_channels(b1,p1,b2,p2);
  batt_pct = readBatteryPercent();
  ui_set_battery(batt_pct);
}
