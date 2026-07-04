/*
 * watcher_os — extensible app framework for the SenseCAP Watcher.
 *
 * Design goals (after the earlier freeze):
 *  - The LVGL task owns ALL rendering + UI updates. Periodic work runs in an
 *    lv_timer (inside the LVGL task, lock already held). No outside loop pokes
 *    the UI, so nothing can deadlock against the render task.
 *  - No blocking calls (audio/LED) in the UI path — those live in fx_task,
 *    fed by a queue / shared state.
 *  - Input callbacks only set flags; the lv_timer applies them.
 *  - Navigation = LVGL tileview (native touch swipe). Knob = in-app adjust.
 *    Button = action. Long-press = back to first tile.
 *  - Task watchdog stays ENABLED: a hang reboots (and is visible) instead of
 *    silently freezing.
 *
 * Add a function = write one app_t and register it in APPS[]. (Step 1: Home + Timer.)
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sensecap-watcher.h"
#include "esp_lvgl_port.h"
#include "iot_knob.h"
#include "iot_button.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

static const char *TAG = "WATCHER_OS";

/* ================= app framework ================= */
typedef struct {
    const char *name;
    void (*build)(lv_obj_t *tile);   /* build UI into the tile (once) */
    void (*on_show)(void);           /* became the active tile */
    void (*tick)(void);              /* periodic, runs in LVGL task */
    void (*on_knob)(int dir);        /* knob turned while active (+1/-1) */
    void (*on_button)(void);         /* short press while active */
} app_t;

#define MAX_APPS 6
static app_t   APPS[MAX_APPS];
static lv_obj_t *TILES[MAX_APPS];
static int      n_apps = 0;
static int      g_active = 0;
static int      g_wifi_idx = -1;   /* tile index of the WiFi app */
static lv_obj_t *tileview = NULL;

/* WiFi state — declared early because the Home app displays it */
enum { W_IDLE = 0, W_CONNECTING, W_CONNECTED, W_FAILED };
static volatile int  wifi_st = W_IDLE;
static char          wifi_ssid[33] = "";
static char          wifi_ip[16] = "";

static int app_register(app_t a)
{
    int i = n_apps++;
    APPS[i] = a;
    return i;
}

/* ================= input (flags only; applied in ui_tick) ================= */
/* burst-gated knob: a count registers only if it has neighbours within the
 * window (a real turn is a burst; phantom counts are isolated). */
static volatile int      knob_steps = 0;
static int               burst_n = 0;
static int64_t           knob_last_ev = 0;
#define KNOB_BURST_US    450000
#define KNOB_CONFIRM     3
static volatile uint32_t click_cnt = 0;
static volatile uint32_t long_cnt = 0;

static void knob_ev(int dir)
{
    int64_t now = esp_timer_get_time();
    if (now - knob_last_ev <= KNOB_BURST_US) burst_n++;
    else burst_n = 1;
    knob_last_ev = now;
    if (burst_n >= KNOB_CONFIRM) knob_steps += dir;
}
static void knob_left_cb(void *a, void *d)  { knob_ev(-1); }
static void knob_right_cb(void *a, void *d) { knob_ev(+1); }
static void btn_click_cb(void *a, void *d)  { click_cnt++; }
static void btn_long_cb(void *a, void *d)   { long_cnt++; }

static void input_init(void)
{
    knob_config_t kcfg = { .default_direction = 0, .gpio_encoder_a = BSP_KNOB_A, .gpio_encoder_b = BSP_KNOB_B };
    knob_handle_t knob = iot_knob_create(&kcfg);
    assert(knob);
    iot_knob_register_cb(knob, KNOB_LEFT, knob_left_cb, NULL);
    iot_knob_register_cb(knob, KNOB_RIGHT, knob_right_cb, NULL);
    button_config_t bcfg = {
        .type = BUTTON_TYPE_CUSTOM, .long_press_time = 1000, .short_press_time = 200,
        .custom_button_config = {
            .active_level = 0,
            .button_custom_init = bsp_knob_btn_init,
            .button_custom_deinit = bsp_knob_btn_deinit,
            .button_custom_get_key_value = bsp_knob_btn_get_key_value,
        },
    };
    button_handle_t btn = iot_button_create(&bcfg);
    assert(btn);
    iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, btn_click_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, btn_long_cb, NULL);
}

/* ================= fx: sound + LED (own task, never blocks the UI) ========= */
#define SR 16000
enum { SND_NAV, SND_TICK_UP, SND_TICK_DN, SND_START, SND_CANCEL, SND_ALARM };
static QueueHandle_t snd_q;
static volatile uint8_t led_r = 0, led_g = 0, led_b = 0;
static volatile bool    led_flash = false;

static void play_tone(float f, int ms, int ampl)
{
    int n = (SR * ms) / 1000;
    int16_t *b = heap_caps_malloc(n * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!b) return;
    int env = SR / 100;
    for (int i = 0; i < n; i++) {
        float g = 1.0f;
        if (i < env) g = (float)i / env;
        else if (i > n - env) g = (float)(n - i) / env;
        b[i] = (int16_t)(ampl * g * sinf(2.0f * (float)M_PI * f * i / SR));
    }
    size_t w = 0; bsp_i2s_write(b, n * sizeof(int16_t), &w, 500); free(b);
}
static void snd(int cue) { if (snd_q) xQueueSend(snd_q, &cue, 0); }

static void fx_task(void *arg)
{
    /* Audio is intentionally NOT initialized: the always-on I2S codec starves
     * the WiFi radio on this board (auth handshake fails), and the BSP has no
     * way to release I2S while keeping WiFi. So feedback is visual (LED +
     * on-screen). Revisit audio later as a WiFi-coexisting feature. */
    int flash = 0;
    while (1) {
        int cue;
        xQueueReceive(snd_q, &cue, pdMS_TO_TICKS(50));   /* drain cues, no playback */
        if (led_flash) { flash ^= 1; bsp_rgb_set(flash ? 60 : 0, 0, 0); }  /* alarm only */
        else bsp_rgb_set(0, 0, 0);   /* LED off during normal use */
    }
}

/* ================= Timer app ================= */
enum { T_SET = 0, T_RUN, T_DONE };
static volatile int   t_state = T_SET;
static int            t_dur = 300;
static int64_t        t_end = 0;
static bool           t_dirty = true;
static lv_obj_t *t_arc, *t_time, *t_status;

static void mmss(int s, char *o, int c) { snprintf(o, c, "%02d:%02d", s/60, s%60); }

static void timer_build(lv_obj_t *tile)
{
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x080a05), 0);
    t_arc = lv_arc_create(tile);
    lv_obj_set_size(t_arc, 360, 360); lv_obj_center(t_arc);
    lv_arc_set_rotation(t_arc, 270);
    lv_arc_set_bg_angles(t_arc, 0, 360);
    lv_arc_set_range(t_arc, 0, 1000);
    lv_arc_set_value(t_arc, 1000);
    lv_obj_remove_style(t_arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(t_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(t_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(t_arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(t_arc, lv_color_hex(0x203010), LV_PART_MAIN);
    lv_obj_set_style_arc_color(t_arc, lv_color_hex(0x8ce63a), LV_PART_INDICATOR);
    t_time = lv_label_create(tile);
    lv_label_set_text(t_time, "05:00");
    lv_obj_set_style_text_color(t_time, lv_color_white(), 0);
    lv_obj_set_style_text_font(t_time, &lv_font_montserrat_48, 0);
    lv_obj_align(t_time, LV_ALIGN_CENTER, 0, -8);
    t_status = lv_label_create(tile);
    lv_label_set_text(t_status, "turn: set   press: start");
    lv_obj_set_style_text_color(t_status, lv_color_hex(0x8ce63a), 0);
    lv_obj_set_style_text_font(t_status, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(t_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t_status, LV_ALIGN_CENTER, 0, 52);
}
static void timer_on_knob(int dir)
{
    if (t_state != T_SET) return;
    int nd = t_dur + dir * 30;
    if (nd < 30) nd = 30;
    if (nd > 99*60) nd = 99*60;
    t_dur = nd; t_dirty = true;
    snd(dir > 0 ? SND_TICK_UP : SND_TICK_DN);
}
static void timer_on_button(void)
{
    if (t_state == T_SET) { t_end = esp_timer_get_time() + (int64_t)t_dur*1000000; t_state = T_RUN; t_dirty = true; snd(SND_START); }
    else { t_state = T_SET; t_dirty = true; led_flash = false; snd(SND_CANCEL); }
}
static void timer_tick(void)
{
    static int shown = -1;
    char b[8];
    if (t_state == T_SET) {
        led_flash = false; led_r = 4; led_g = 16; led_b = 0;
        if (t_dirty) {
            mmss(t_dur, b, sizeof(b));
            lv_label_set_text(t_time, b);
            lv_arc_set_value(t_arc, 1000);
            lv_label_set_text(t_status, "turn: set   press: start");
            t_dirty = false; shown = -1;
        }
    } else if (t_state == T_RUN) {
        led_flash = false; led_r = 0; led_g = 40; led_b = 8;
        int64_t now = esp_timer_get_time();
        int rem = (int)((t_end - now + 999999) / 1000000);
        if (rem < 0) rem = 0;
        if (rem != shown) {
            shown = rem; mmss(rem, b, sizeof(b));
            lv_label_set_text(t_time, b);
            lv_arc_set_value(t_arc, t_dur > 0 ? (rem*1000)/t_dur : 0);
            lv_label_set_text(t_status, "running   press: cancel");
        }
        if (now >= t_end) { t_state = T_DONE; snd(SND_ALARM); }
    } else { /* T_DONE */
        led_flash = true;
        static int64_t last = 0;
        int64_t now = esp_timer_get_time();
        lv_label_set_text(t_time, "TIME!");
        lv_label_set_text(t_status, "press to reset");
        if (now - last > 1000000) { last = now; snd(SND_ALARM); }
    }
}

/* ================= Home app ================= */
static lv_obj_t *h_wifi;
static int       h_wifi_last = -2;

static void home_wifi_click_cb(lv_event_t *e)   /* tap the WiFi status -> WiFi settings */
{
    if (g_wifi_idx >= 0 && g_wifi_idx < n_apps)
        lv_obj_set_tile(tileview, TILES[g_wifi_idx], LV_ANIM_ON);
}
static void home_build(lv_obj_t *tile)
{
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x05060a), 0);
    lv_obj_t *sp = lv_spinner_create(tile, 2600, 70);
    lv_obj_set_size(sp, 300, 300); lv_obj_center(sp);
    lv_obj_clear_flag(sp, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(sp, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(sp, 6, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(sp, lv_color_hex(0x14213a), LV_PART_MAIN);
    lv_obj_set_style_arc_color(sp, lv_color_hex(0x39d0ff), LV_PART_INDICATOR);
    lv_obj_t *t = lv_label_create(tile);
    lv_label_set_text(t, "WATCHER");
    lv_obj_set_style_text_color(t, lv_color_white(), 0);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_28, 0);
    lv_obj_align(t, LV_ALIGN_CENTER, 0, -18);

    /* tappable WiFi status pill -> jumps to WiFi settings */
    h_wifi = lv_label_create(tile);
    lv_label_set_text(h_wifi, LV_SYMBOL_WIFI "  set up");
    lv_obj_set_style_text_color(h_wifi, lv_color_hex(0x39d0ff), 0);
    lv_obj_set_style_text_font(h_wifi, &lv_font_montserrat_16, 0);
    lv_obj_set_style_bg_color(h_wifi, lv_color_hex(0x14213a), 0);
    lv_obj_set_style_bg_opa(h_wifi, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(h_wifi, 7, 0);
    lv_obj_set_style_radius(h_wifi, 8, 0);
    lv_obj_set_style_text_align(h_wifi, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(h_wifi, LV_ALIGN_CENTER, 0, 26);
    lv_obj_add_flag(h_wifi, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(h_wifi, home_wifi_click_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *hint = lv_label_create(tile);
    lv_label_set_text(hint, "swipe for apps");
    lv_obj_set_style_text_color(hint, lv_color_hex(0x7a8699), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_16, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -34);
}
static void home_tick(void)
{
    led_flash = false; led_r = 2; led_g = 6; led_b = 10;
    if (wifi_st != h_wifi_last) {
        h_wifi_last = wifi_st;
        lv_color_t fg, bg;
        if (wifi_st == W_CONNECTED) {
            lv_label_set_text_fmt(h_wifi, LV_SYMBOL_WIFI "  %s", wifi_ssid);
            fg = lv_color_hex(0x8ce63a); bg = lv_color_hex(0x14301a);      /* green = connected */
        } else if (wifi_st == W_CONNECTING) {
            lv_label_set_text(h_wifi, LV_SYMBOL_WIFI "  connecting...");
            fg = lv_color_hex(0xffd166); bg = lv_color_hex(0x30240a);      /* amber */
        } else {  /* idle / failed = not connected */
            lv_label_set_text(h_wifi, LV_SYMBOL_WIFI " " LV_SYMBOL_CLOSE "  off - tap");
            fg = lv_color_hex(0xff6b6b); bg = lv_color_hex(0x351515);      /* red = off */
        }
        lv_obj_set_style_text_color(h_wifi, fg, 0);
        lv_obj_set_style_bg_color(h_wifi, bg, 0);
    }
}

/* ================= WiFi driver ================= */
static int           wifi_retry = 0;
static bool          have_creds = false;
/* scan results, filled by the event task, consumed by the wifi app tick */
#define MAX_SCAN 16
static char          scan_ssid[MAX_SCAN][33];
static int8_t        scan_rssi[MAX_SCAN];
static volatile int  scan_count = 0;
static volatile bool scan_ready = false;
static volatile bool scan_busy = false;

static void wifi_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "ssid", ssid); nvs_set_str(h, "pass", pass);
        nvs_commit(h); nvs_close(h);
    }
}
static bool wifi_load(char *ssid, char *pass, size_t cap)
{
    nvs_handle_t h;
    if (nvs_open("wifi", NVS_READONLY, &h) != ESP_OK) return false;
    size_t ls = cap, lp = cap;
    bool ok = (nvs_get_str(h, "ssid", ssid, &ls) == ESP_OK) && ls > 1;
    if (nvs_get_str(h, "pass", pass, &lp) != ESP_OK) pass[0] = 0;
    nvs_close(h);
    return ok;
}
static void wifi_connect(const char *ssid, const char *pass)
{
    esp_wifi_scan_stop();          /* release the driver if a scan is running */
    scan_busy = false;
    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;   /* don't over-filter; use the AP's real auth */
    cfg.sta.pmf_cfg.capable = true;                /* WPA2/WPA3-transition friendly */
    cfg.sta.pmf_cfg.required = false;   /* PMF optional (works with WPA2 and WPA3-transition) */
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;       /* enable WPA3-SAE (fixes auth-stage reason=2) */
    strncpy(wifi_ssid, ssid, sizeof(wifi_ssid) - 1);
    have_creds = true; wifi_retry = 0; wifi_st = W_CONNECTING; wifi_ip[0] = 0;
    ESP_LOGI(TAG, "connecting '%s' (pw %d chars)", ssid, (int)strlen(pass));
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_disconnect();
    esp_wifi_connect();
}
static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (have_creds) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE) {
        uint16_t n = 0; esp_wifi_scan_get_ap_num(&n);
        if (n > MAX_SCAN) n = MAX_SCAN;
        wifi_ap_record_t *recs = calloc(n, sizeof(wifi_ap_record_t));
        if (recs) {
            uint16_t got = n; esp_wifi_scan_get_ap_records(&got, recs);
            int out = 0;
            for (int i = 0; i < got && out < MAX_SCAN; i++) {
                if (recs[i].ssid[0] == 0) continue;
                bool dup = false;
                for (int j = 0; j < out; j++) if (strcmp(scan_ssid[j], (char *)recs[i].ssid) == 0) { dup = true; break; }
                if (dup) continue;
                strncpy(scan_ssid[out], (char *)recs[i].ssid, 32);
                scan_rssi[out] = recs[i].rssi; out++;
            }
            scan_count = out; free(recs);
        } else scan_count = 0;
        scan_busy = false; scan_ready = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "disconnected, reason=%d, retry=%d", d ? d->reason : -1, wifi_retry);
        if (have_creds && wifi_retry < 30) { wifi_retry++; wifi_st = W_CONNECTING; esp_wifi_connect(); }  /* keep trying ~1min so a sleepy phone hotspot can wake */
        else if (have_creds) { wifi_st = W_FAILED; }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(wifi_ip, sizeof(wifi_ip), IPSTR, IP2STR(&e->ip_info.ip));
        wifi_st = W_CONNECTED; wifi_retry = 0;
        ESP_LOGI(TAG, "got IP %s", wifi_ip);
    }
}
static void wifi_scan_start(void)
{
    if (scan_busy) return;
    scan_busy = true; scan_ready = false;
    wifi_scan_config_t sc = { .show_hidden = false };
    if (esp_wifi_scan_start(&sc, false) != ESP_OK) scan_busy = false;
}
static void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    /* NO power-save: modem-sleep gates the APB clock during idle and stalls
     * the LCD SPI/DMA flush -> LVGL flush hangs -> UI freeze (~28s in). */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    char ss[33], pw[65];
    if (wifi_load(ss, pw, sizeof(ss))) { ESP_LOGI(TAG, "saved creds '%s'", ss); wifi_connect(ss, pw); }
}

/* ================= WiFi settings app ================= */
static lv_obj_t *w_status, *w_list, *w_pwbox, *w_ta, *w_kb, *w_ssid_lbl, *w_note;
static char      w_sel_ssid[33] = "";
static int       w_last_shown_scan = -1;
static int       w_last_st = -1;

static void wifi_show_password(bool show)
{
    if (show) { lv_obj_clear_flag(w_pwbox, LV_OBJ_FLAG_HIDDEN); lv_obj_add_flag(w_list, LV_OBJ_FLAG_HIDDEN); }
    else      { lv_obj_add_flag(w_pwbox, LV_OBJ_FLAG_HIDDEN); lv_obj_clear_flag(w_list, LV_OBJ_FLAG_HIDDEN); }
}
static void wifi_ap_click_cb(lv_event_t *e)
{
    lv_obj_t *btn = lv_event_get_target(e);
    const char *txt = lv_list_get_btn_text(w_list, btn);
    if (!txt) return;
    strncpy(w_sel_ssid, txt, sizeof(w_sel_ssid) - 1);
    lv_label_set_text_fmt(w_ssid_lbl, "%s", w_sel_ssid);
    lv_textarea_set_text(w_ta, "");
    wifi_show_password(true);
    lv_keyboard_set_textarea(w_kb, w_ta);
}
static void wifi_kb_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {          /* checkmark = connect */
        const char *pw = lv_textarea_get_text(w_ta);
        wifi_save(w_sel_ssid, pw);
        wifi_connect(w_sel_ssid, pw);
        wifi_show_password(false);
    } else if (code == LV_EVENT_CANCEL) {  /* X = back to list */
        wifi_show_password(false);
    }
}
static void wifi_ok_cb(lv_event_t *e)     /* connect with typed password */
{
    const char *pw = lv_textarea_get_text(w_ta);
    wifi_save(w_sel_ssid, pw);
    wifi_connect(w_sel_ssid, pw);
    wifi_show_password(false);
}
static void wifi_cancel_cb(lv_event_t *e) { wifi_show_password(false); }   /* back to list */
static void wifi_qr_cb(lv_event_t *e)     { lv_label_set_text(w_note, LV_SYMBOL_IMAGE "  QR scan: coming next"); }
static void wifi_build(lv_obj_t *tile)
{
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x0a0a12), 0);

    w_status = lv_label_create(tile);
    lv_label_set_text(w_status, "WiFi: starting...");
    lv_obj_set_style_text_color(w_status, lv_color_hex(0x39d0ff), 0);
    lv_obj_set_style_text_font(w_status, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(w_status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(w_status, LV_ALIGN_TOP_MID, 0, 30);

    /* scrollable list of networks */
    w_list = lv_list_create(tile);
    lv_obj_set_size(w_list, 300, 250);
    lv_obj_align(w_list, LV_ALIGN_CENTER, 0, 30);
    lv_obj_set_style_bg_color(w_list, lv_color_hex(0x14141c), 0);
    lv_obj_set_style_border_width(w_list, 0, 0);

    /* password entry (hidden until an AP is tapped) */
    w_pwbox = lv_obj_create(tile);
    lv_obj_set_size(w_pwbox, 412, 412); lv_obj_center(w_pwbox);
    lv_obj_set_style_bg_color(w_pwbox, lv_color_hex(0x0a0a12), 0);
    lv_obj_set_style_border_width(w_pwbox, 0, 0);
    lv_obj_clear_flag(w_pwbox, LV_OBJ_FLAG_SCROLLABLE);
    w_ssid_lbl = lv_label_create(w_pwbox);
    lv_label_set_text(w_ssid_lbl, "");
    lv_obj_set_style_text_color(w_ssid_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(w_ssid_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(w_ssid_lbl, LV_ALIGN_TOP_MID, 0, 40);
    w_ta = lv_textarea_create(w_pwbox);
    lv_textarea_set_one_line(w_ta, true);
    lv_textarea_set_password_mode(w_ta, false);   /* visible: easier to type right */
    lv_textarea_set_placeholder_text(w_ta, "password");
    lv_obj_set_width(w_ta, 300);
    lv_obj_align(w_ta, LV_ALIGN_TOP_MID, 0, 70);
    /* action row below password: OK | Cancel | Scan QR */
    lv_obj_t *w_row = lv_obj_create(w_pwbox);
    lv_obj_remove_style_all(w_row);
    lv_obj_set_size(w_row, 384, 48);
    lv_obj_align(w_row, LV_ALIGN_TOP_MID, 0, 104);
    lv_obj_set_flex_flow(w_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(w_row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(w_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *bl;
    lv_obj_t *b_ok = lv_btn_create(w_row);
    lv_obj_set_style_bg_color(b_ok, lv_color_hex(0x1f7a33), 0);
    bl = lv_label_create(b_ok); lv_label_set_text(bl, LV_SYMBOL_OK " OK");
    lv_obj_add_event_cb(b_ok, wifi_ok_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *b_ca = lv_btn_create(w_row);
    lv_obj_set_style_bg_color(b_ca, lv_color_hex(0x5a1f1f), 0);
    bl = lv_label_create(b_ca); lv_label_set_text(bl, LV_SYMBOL_CLOSE " Cancel");
    lv_obj_add_event_cb(b_ca, wifi_cancel_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *b_qr = lv_btn_create(w_row);
    lv_obj_set_style_bg_color(b_qr, lv_color_hex(0x3a2f5a), 0);
    bl = lv_label_create(b_qr); lv_label_set_text(bl, LV_SYMBOL_IMAGE " QR");
    lv_obj_add_event_cb(b_qr, wifi_qr_cb, LV_EVENT_CLICKED, NULL);
    w_note = lv_label_create(w_pwbox);
    lv_label_set_text(w_note, "");
    lv_obj_set_style_text_color(w_note, lv_color_hex(0x7a8699), 0);
    lv_obj_set_style_text_font(w_note, &lv_font_montserrat_16, 0);
    lv_obj_align(w_note, LV_ALIGN_TOP_MID, 0, 158);
    w_kb = lv_keyboard_create(w_pwbox);
    lv_keyboard_set_textarea(w_kb, w_ta);
    lv_obj_add_event_cb(w_kb, wifi_kb_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(w_kb, wifi_kb_cb, LV_EVENT_CANCEL, NULL);
    lv_obj_add_flag(w_pwbox, LV_OBJ_FLAG_HIDDEN);
}
static void wifi_on_show(void)
{
    if (wifi_st != W_CONNECTED) wifi_scan_start();
}
static void wifi_tick(void)
{
    led_flash = false;
    if (wifi_st == W_CONNECTED) { led_r = 0; led_g = 30; led_b = 6; }
    else { led_r = 0; led_g = 8; led_b = 20; }

    /* status line */
    if (wifi_st != w_last_st) {
        w_last_st = wifi_st;
        switch (wifi_st) {
            case W_CONNECTED:  lv_label_set_text_fmt(w_status, "%s  %s", wifi_ssid, wifi_ip); break;
            case W_CONNECTING: lv_label_set_text_fmt(w_status, "connecting %s...", wifi_ssid); break;
            case W_FAILED:     lv_label_set_text(w_status, "connect failed - tap a network"); break;
            default:           lv_label_set_text(w_status, scan_busy ? "scanning..." : "tap a network"); break;
        }
    } else if (wifi_st == W_IDLE) {
        lv_label_set_text(w_status, scan_busy ? "scanning..." : "tap a network");
    }

    /* populate list when a scan completes */
    if (scan_ready && scan_count != w_last_shown_scan) {
        w_last_shown_scan = scan_count;
        scan_ready = false;
        lv_obj_clean(w_list);
        for (int i = 0; i < scan_count; i++) {
            lv_obj_t *b = lv_list_add_btn(w_list, LV_SYMBOL_WIFI, scan_ssid[i]);
            lv_obj_set_style_bg_color(b, lv_color_hex(0x1c1c28), 0);
            lv_obj_set_style_text_color(b, lv_color_white(), 0);
            lv_obj_add_event_cb(b, wifi_ap_click_cb, LV_EVENT_CLICKED, NULL);
        }
        if (scan_count == 0) lv_list_add_text(w_list, "no networks - swipe away & back to rescan");
    }
}

/* ================= Radar app (nearby flights) ================= */
#define RADAR_CX 206
#define RADAR_CY 206
#define RADAR_R  190
static const int RANGES[] = {10, 20, 50, 100, 200};
static int range_idx = 1;                 /* default 20km */
static volatile double g_range_km = 20.0;
static volatile bool g_refetch = false;
#define MAX_FLIGHTS 24
typedef struct { double lat, lon; float track; int alt; char cs[10]; } flight_t;
static lv_obj_t  *radar_status, *radar_sweep;
static lv_point_t sweep_pts[2];
static float      sweep_ang = 0.0f;
static lv_obj_t  *blip_dot[MAX_FLIGHTS], *blip_lbl[MAX_FLIGHTS];
static flight_t   g_flights[MAX_FLIGHTS];
static volatile int g_nfl = 0;
static volatile bool g_radar_dirty = false;
static double     my_lat = 0, my_lon = 0;
static volatile bool g_located = false;
static char       loc_city[24] = "";
static SemaphoreHandle_t fl_mux;
static int g_http_status = 0;
static volatile int64_t g_last_ok_us = 0;

static double hav_km(double la1,double lo1,double la2,double lo2){
    double R=6371.0, dla=(la2-la1)*M_PI/180.0, dlo=(lo2-lo1)*M_PI/180.0;
    double a=sin(dla/2)*sin(dla/2)+cos(la1*M_PI/180)*cos(la2*M_PI/180)*sin(dlo/2)*sin(dlo/2);
    return R*2*atan2(sqrt(a),sqrt(1-a));
}
static double bearing_deg(double la1,double lo1,double la2,double lo2){
    double p1=la1*M_PI/180,p2=la2*M_PI/180,dl=(lo2-lo1)*M_PI/180;
    double y=sin(dl)*cos(p2), x=cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dl);
    double b=atan2(y,x)*180.0/M_PI; if(b<0)b+=360; return b;
}

/* ---- HTTP ---- */
static int http_get(const char *url, char *out, int cap, bool tls){
    esp_http_client_config_t cfg = { .url=url, .timeout_ms=9000, .crt_bundle_attach = tls?esp_crt_bundle_attach:NULL };
    esp_http_client_handle_t h=esp_http_client_init(&cfg);
    int off=-1;
    if(esp_http_client_open(h,0)==ESP_OK){
        esp_http_client_fetch_headers(h);
        g_http_status = esp_http_client_get_status_code(h);
        off=0; int n;
        while((n=esp_http_client_read(h,out+off,cap-1-off))>0){ off+=n; if(off>=cap-1) break; }
        out[off>0?off:0]=0;
        esp_http_client_close(h);
    }
    esp_http_client_cleanup(h);
    return off;
}
static bool fetch_location(void){
    char *b=malloc(1024); if(!b) return false; bool ok=false;
    int n=http_get("http://ip-api.com/json/?fields=status,lat,lon,city",b,1024,false);
    ESP_LOGI(TAG,"iploc http=%d: %.90s",n,n>0?b:"(none)");
    if(n>0){ cJSON *r=cJSON_Parse(b);
        if(r){ cJSON *la=cJSON_GetObjectItem(r,"lat"),*lo=cJSON_GetObjectItem(r,"lon"),*ci=cJSON_GetObjectItem(r,"city");
            if(cJSON_IsNumber(la)&&cJSON_IsNumber(lo)){ my_lat=la->valuedouble; my_lon=lo->valuedouble;
                if(cJSON_IsString(ci)) strncpy(loc_city,ci->valuestring,sizeof(loc_city)-1);
                g_located=true; ok=true; }
            cJSON_Delete(r); } }
    free(b); return ok;
}
static void fetch_flights(void){
    double rk=g_range_km, dla=rk/111.0, dlo=rk/(111.0*cos(my_lat*M_PI/180.0));
    char url[240];
    snprintf(url,sizeof(url),"https://opensky-network.org/api/states/all?lamin=%.4f&lomin=%.4f&lamax=%.4f&lomax=%.4f",
             my_lat-dla,my_lon-dlo,my_lat+dla,my_lon+dlo);
    int cap=64*1024; char *b=heap_caps_malloc(cap,MALLOC_CAP_SPIRAM); if(!b)b=malloc(cap); if(!b)return;
    g_http_status=0;
    int n=http_get(url,b,cap,true);
    if(n>0 && g_http_status==200){ cJSON *r=cJSON_Parse(b);
        if(r){ cJSON *states=cJSON_GetObjectItem(r,"states");
            flight_t tmp[MAX_FLIGHTS]; int cnt=0;
            if(cJSON_IsArray(states)){ cJSON *st;
                cJSON_ArrayForEach(st,states){ if(cnt>=MAX_FLIGHTS)break; if(!cJSON_IsArray(st))continue;
                    cJSON *lo=cJSON_GetArrayItem(st,5),*la=cJSON_GetArrayItem(st,6),*cs=cJSON_GetArrayItem(st,1),
                          *tr=cJSON_GetArrayItem(st,10),*al=cJSON_GetArrayItem(st,7);
                    if(!cJSON_IsNumber(lo)||!cJSON_IsNumber(la))continue;
                    tmp[cnt].lat=la->valuedouble; tmp[cnt].lon=lo->valuedouble;
                    tmp[cnt].track=cJSON_IsNumber(tr)?tr->valuedouble:0;
                    tmp[cnt].alt=cJSON_IsNumber(al)?(int)al->valuedouble:0;
                    tmp[cnt].cs[0]=0;
                    if(cJSON_IsString(cs)){ int j=0; for(char *p=cs->valuestring;*p&&j<9;p++) if(*p!=' ') tmp[cnt].cs[j++]=*p; tmp[cnt].cs[j]=0; }
                    cnt++; }
            }
            xSemaphoreTake(fl_mux,portMAX_DELAY);
            for(int i=0;i<cnt;i++) g_flights[i]=tmp[i];
            g_nfl=cnt;
            xSemaphoreGive(fl_mux);
            g_last_ok_us = esp_timer_get_time();
            g_radar_dirty=true;
            ESP_LOGI(TAG,"flights=%d (%.0fkm box)",cnt,g_range_km);
            cJSON_Delete(r);
        } else { ESP_LOGW(TAG,"flights: parse fail status=%d",g_http_status); g_radar_dirty=true; }
    } else { ESP_LOGW(TAG,"flights: http n=%d status=%d (rate-limited?)",n,g_http_status); g_radar_dirty=true; }
    free(b);
}
static void radar_task(void *arg){
    ESP_LOGI(TAG,"radar_task started");
    while(wifi_st!=W_CONNECTED) vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG,"radar_task: wifi up, locating");
    while(!g_located){ if(!fetch_location()) vTaskDelay(pdMS_TO_TICKS(3000)); }
    ESP_LOGI(TAG,"location %.3f,%.3f (%s)",my_lat,my_lon,loc_city);
    int64_t last=0;
    while(1){
        int64_t now=esp_timer_get_time();
        if(wifi_st==W_CONNECTED && (g_refetch || last==0 || now-last>60000000LL)){ g_refetch=false; last=now; fetch_flights(); }
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

static void radar_on_knob(int dir){
    range_idx += (dir > 0) ? 1 : -1;
    if (range_idx < 0) range_idx = 0;
    if (range_idx > (int)(sizeof(RANGES)/sizeof(RANGES[0]))-1) range_idx = (int)(sizeof(RANGES)/sizeof(RANGES[0]))-1;
    g_range_km = RANGES[range_idx];
    g_radar_dirty = true;    /* rescale existing blips now */
    g_refetch = true;        /* pull a box matching the new range */
}
static void radar_ring(lv_obj_t *par, int d){
    lv_obj_t *r=lv_obj_create(par); lv_obj_set_size(r,d,d); lv_obj_center(r);
    lv_obj_set_style_radius(r,LV_RADIUS_CIRCLE,0); lv_obj_set_style_bg_opa(r,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_color(r,lv_color_hex(0x1f6b2f),0); lv_obj_set_style_border_width(r,2,0);
    lv_obj_clear_flag(r,LV_OBJ_FLAG_CLICKABLE); lv_obj_clear_flag(r,LV_OBJ_FLAG_SCROLLABLE);
}
static void radar_build(lv_obj_t *tile){
    lv_obj_set_style_bg_color(tile,lv_color_hex(0x02160a),0);
    radar_ring(tile,380); radar_ring(tile,260); radar_ring(tile,140);
    lv_obj_t *c=lv_obj_create(tile); lv_obj_set_size(c,8,8); lv_obj_center(c);
    lv_obj_set_style_radius(c,LV_RADIUS_CIRCLE,0); lv_obj_set_style_bg_color(c,lv_color_hex(0x8ce63a),0);
    lv_obj_set_style_border_width(c,0,0); lv_obj_clear_flag(c,LV_OBJ_FLAG_CLICKABLE);
    radar_sweep=lv_line_create(tile);
    static lv_style_t sst; lv_style_init(&sst);
    lv_style_set_line_color(&sst,lv_color_hex(0x39ff6a)); lv_style_set_line_width(&sst,3);
    lv_obj_add_style(radar_sweep,&sst,0);
    sweep_pts[0].x=RADAR_CX; sweep_pts[0].y=RADAR_CY; sweep_pts[1].x=RADAR_CX; sweep_pts[1].y=RADAR_CY-RADAR_R;
    lv_line_set_points(radar_sweep,sweep_pts,2);
    /* blip pool */
    for(int i=0;i<MAX_FLIGHTS;i++){
        blip_dot[i]=lv_obj_create(tile); lv_obj_set_size(blip_dot[i],7,7);
        lv_obj_set_style_radius(blip_dot[i],LV_RADIUS_CIRCLE,0);
        lv_obj_set_style_bg_color(blip_dot[i],lv_color_hex(0xffee55),0);
        lv_obj_set_style_border_width(blip_dot[i],0,0);
        lv_obj_clear_flag(blip_dot[i],LV_OBJ_FLAG_CLICKABLE|LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(blip_dot[i],LV_OBJ_FLAG_HIDDEN);
        blip_lbl[i]=lv_label_create(tile);
        lv_obj_set_style_text_color(blip_lbl[i],lv_color_hex(0xcfeecf),0);
        lv_obj_set_style_text_font(blip_lbl[i],&lv_font_montserrat_16,0);
        lv_obj_add_flag(blip_lbl[i],LV_OBJ_FLAG_HIDDEN);
    }
    radar_status=lv_label_create(tile);
    lv_label_set_text(radar_status,"RADAR  connecting...");
    lv_obj_set_style_text_color(radar_status,lv_color_hex(0x8ce63a),0);
    lv_obj_set_style_text_font(radar_status,&lv_font_montserrat_16,0);
    lv_obj_set_style_bg_color(radar_status,lv_color_black(),0); lv_obj_set_style_bg_opa(radar_status,LV_OPA_50,0);
    lv_obj_set_style_pad_all(radar_status,3,0); lv_obj_align(radar_status,LV_ALIGN_BOTTOM_MID,0,-6);
}
static void radar_tick(void){
    led_flash=false; led_r=0; led_g=18; led_b=6;
    sweep_ang+=0.12f; if(sweep_ang>6.2832f) sweep_ang-=6.2832f;
    sweep_pts[1].x=RADAR_CX+(int)(RADAR_R*sinf(sweep_ang));
    sweep_pts[1].y=RADAR_CY-(int)(RADAR_R*cosf(sweep_ang));
    lv_line_set_points(radar_sweep,sweep_pts,2);

    int64_t now_t = esp_timer_get_time();
    bool stale = (g_last_ok_us == 0) || (now_t - g_last_ok_us > 90000000LL);
    if(g_radar_dirty){
        g_radar_dirty=false;
        lv_color_t bc = stale ? lv_color_hex(0x557755) : lv_color_hex(0xffee55);   /* dim when stale */
        xSemaphoreTake(fl_mux,portMAX_DELAY);
        int shown=0;
        for(int i=0;i<g_nfl && shown<MAX_FLIGHTS;i++){
            double d=hav_km(my_lat,my_lon,g_flights[i].lat,g_flights[i].lon);
            if(d>g_range_km) continue;
            double br=bearing_deg(my_lat,my_lon,g_flights[i].lat,g_flights[i].lon);
            int rp=(int)((d/g_range_km)*RADAR_R);
            int x=RADAR_CX+(int)(rp*sin(br*M_PI/180.0));
            int y=RADAR_CY-(int)(rp*cos(br*M_PI/180.0));
            lv_obj_set_style_bg_color(blip_dot[shown],bc,0);
            lv_obj_set_pos(blip_dot[shown],x-3,y-3); lv_obj_clear_flag(blip_dot[shown],LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(blip_lbl[shown],g_flights[i].cs);
            lv_obj_set_pos(blip_lbl[shown],x+5,y-8); lv_obj_clear_flag(blip_lbl[shown],LV_OBJ_FLAG_HIDDEN);
            shown++;
        }
        xSemaphoreGive(fl_mux);
        for(int i=shown;i<MAX_FLIGHTS;i++){ lv_obj_add_flag(blip_dot[i],LV_OBJ_FLAG_HIDDEN); lv_obj_add_flag(blip_lbl[i],LV_OBJ_FLAG_HIDDEN); }
    }
    /* status refreshes every tick so the STALE age stays live */
    if(wifi_st==W_CONNECTED && g_located){
        char b[96];
        if(g_last_ok_us==0) snprintf(b,sizeof(b),"%s  waiting for flights  %dkm", loc_city[0]?loc_city:"?", (int)g_range_km);
        else if(stale)      snprintf(b,sizeof(b),"%s  STALE %ds  %dkm (turn=zoom)", loc_city[0]?loc_city:"?", (int)((now_t-g_last_ok_us)/1000000LL), (int)g_range_km);
        else                snprintf(b,sizeof(b),"%s  %d fl  %dkm (turn=zoom)", loc_city[0]?loc_city:"?", g_nfl, (int)g_range_km);
        lv_label_set_text(radar_status,b);
    } else if(wifi_st!=W_CONNECTED){
        lv_label_set_text(radar_status, wifi_st==W_CONNECTING?"RADAR  connecting wifi...":"RADAR  no wifi (swipe to set up)");
    } else if(!g_located){
        lv_label_set_text(radar_status,"RADAR  locating...");
    }
}

/* ================= navigation + tick ================= */
static void tv_changed_cb(lv_event_t *e)
{
    lv_obj_t *act = lv_tileview_get_tile_act(tileview);
    for (int i = 0; i < n_apps; i++) if (TILES[i] == act) {
        if (i != g_active) { g_active = i; snd(SND_NAV); if (APPS[i].on_show) APPS[i].on_show(); }
        break;
    }
}

static void ui_tick(lv_timer_t *t)
{
    /* apply knob (one detent per step) to the active app */
    int guard = 0;
    while (knob_steps != 0 && guard++ < 8) {
        int dir = knob_steps > 0 ? 1 : -1;
        knob_steps -= dir;
        if (APPS[g_active].on_knob) APPS[g_active].on_knob(dir);
    }
    /* button: short = app action, long = back to first tile */
    static uint32_t seen_click = 0, seen_long = 0;
    if (click_cnt != seen_click) { seen_click = click_cnt; if (APPS[g_active].on_button) APPS[g_active].on_button(); }
    if (long_cnt != seen_long)  { seen_long = long_cnt; lv_obj_set_tile(tileview, TILES[0], LV_ANIM_ON); }
    /* periodic UI for the active app */
    if (APPS[g_active].tick) APPS[g_active].tick();
}

static void ui_build(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_set_style_bg_color(tileview, lv_color_black(), 0);
    lv_obj_add_event_cb(tileview, tv_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    for (int i = 0; i < n_apps; i++) {
        lv_dir_t dir = LV_DIR_HOR;
        if (i == 0) dir = LV_DIR_RIGHT;
        else if (i == n_apps - 1) dir = LV_DIR_LEFT;
        TILES[i] = lv_tileview_add_tile(tileview, i, 0, dir);
        lv_obj_clear_flag(TILES[i], LV_OBJ_FLAG_SCROLLABLE);
        APPS[i].build(TILES[i]);
    }
    lv_obj_set_tile(tileview, TILES[0], LV_ANIM_OFF);
}

/* ================= main ================= */
void app_main(void)
{
    esp_io_expander_handle_t io = bsp_io_expander_init();
    assert(io != NULL);
    bsp_rgb_init();
    /* LVGL draw buffer in INTERNAL DMA RAM (not PSRAM): a PSRAM DMA buffer
     * feeding the QSPI LCD stalls the flush (hang ~28s in). Partial buffers
     * (48 lines, double) keep it small enough to fit internal RAM. */
    bsp_display_cfg_t dcfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = 412 * 20,          /* smaller internal buffer: frees RAM for WiFi/TLS + tasks */
        .double_buffer = false,
        .flags = { .buff_dma = true, .buff_spiram = false },
    };
    dcfg.lvgl_port_cfg.task_affinity = 1;   /* pin LVGL to core 1; leave core 0 for WiFi (auth handshake is timing-critical) */
    lv_disp_t *disp = bsp_lvgl_init_with_cfg(&dcfg);
    assert(disp != NULL);
    bsp_lcd_brightness_set(100);

    /* register apps (order = tile order) */
    app_register((app_t){ .name = "Radar", .build = radar_build, .tick = radar_tick, .on_knob = radar_on_knob });
    app_register((app_t){ .name = "Timer", .build = timer_build, .tick = timer_tick,
                          .on_knob = timer_on_knob, .on_button = timer_on_button });
    g_wifi_idx = app_register((app_t){ .name = "WiFi", .build = wifi_build, .on_show = wifi_on_show, .tick = wifi_tick });

    if (lvgl_port_lock(0)) {
        ui_build();
        lv_timer_create(ui_tick, 80, NULL);   /* runs inside the LVGL task */
        lvgl_port_unlock();
    }

    snd_q = xQueueCreate(8, sizeof(int));
    xTaskCreatePinnedToCore(fx_task, "fx", 4096, NULL, 5, NULL, 1);
    fl_mux = xSemaphoreCreateMutex();
    wifi_init();
    input_init();
    BaseType_t rc = xTaskCreate(radar_task, "radar", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "radar task create rc=%d free_internal=%d", (int)rc, (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    ESP_LOGI(TAG, "watcher_os running: %d apps (Home, Timer, WiFi). swipe to navigate.", n_apps);
    /* nothing else here — the LVGL task + fx_task + input drive everything */
    while (1) vTaskDelay(pdMS_TO_TICKS(5000));
}
