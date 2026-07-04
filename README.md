# WatcherOS

A small, extensible app framework for the **SenseCAP Watcher** (ESP32-S3 + round
412×412 touch LCD, rotary knob, mic/speaker, RGB LED, Himax AI camera, WiFi).
Built on ESP-IDF v5.2.1.

## Apps
- **Home** — status screen; the WiFi pill is tappable (green = connected, red = off) and jumps to WiFi settings.
- **Timer** — knob sets the countdown (±30 s), press starts/cancels; alarm + red flash when it elapses.
- **WiFi** — scan nearby networks, pick one, enter the password, connect; credentials saved to NVS and auto-reconnect on boot. (QR-scan provisioning planned.)

## Controls
- **Swipe / tap** the touchscreen → switch between apps (LVGL `tileview`).
- **Knob** → in-app adjustment only (e.g. timer duration).
- **Button (press)** → app action (start/select). **Long-press** → back to Home.

## Architecture (why it's stable)
- The **LVGL task owns all UI**. Periodic work runs in an `lv_timer` (inside the
  LVGL task) — never from an external loop grabbing the LVGL mutex. This avoids
  the render-loop deadlock that plagued earlier attempts.
- **Audio + LED live in `fx_task`** (fed by a queue / shared state) — never called
  from the UI path, so nothing blocking stalls rendering.
- **Input callbacks only set flags**; the `lv_timer` applies them. The knob is
  burst-gated (rejects electrical phantom counts).
- Add a function = write one `app_t { build, on_show, tick, on_knob, on_button }`
  and register it in `APPS[]`.

## The critical fix: LVGL draw buffer must be in INTERNAL RAM
The vendor `bsp_lvgl_init()` allocates the LVGL draw buffer in **PSRAM**. DMA-ing
that PSRAM buffer to the QSPI LCD **stalls the flush**, hanging the UI ~28 s in
(watchdog logs it; with panic off it just freezes). Fix — initialise with an
internal DMA buffer:

```c
bsp_display_cfg_t dcfg = {
    .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
    .buffer_size = 412 * 48,           /* partial buffers, fit internal RAM */
    .double_buffer = true,
    .flags = { .buff_dma = true, .buff_spiram = false },
};
lv_disp_t *disp = bsp_lvgl_init_with_cfg(&dcfg);   /* NOT bsp_lvgl_init() */
```

## Build & flash
```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py -p /dev/ttyACM1 -b 460800 flash    # ttyACM1 is the console+flash port
```
`main/idf_component.yml` references the SenseCAP Watcher BSP by absolute path
(`/home/im/lab/SenseCAP-Watcher-Firmware/components/sensecap-watcher`).
