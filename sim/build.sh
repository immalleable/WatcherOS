#!/usr/bin/env bash
# Build the headless WatcherOS UI simulator against the LVGL 8.4 source
# that ships with the SenseCAP firmware repo. Produces ./sim.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
LVGL="/home/im/lab/SenseCAP-Watcher-Firmware/components/lvgl"
cd "$HERE"
mapfile -t SRCS < <(find "$LVGL/src" -name '*.c' ! -path '*rlottie*' ! -path '*/gpu/*' ! -path '*sdl*' ! -path '*/libs/ffmpeg/*')
gcc -O1 -w -o sim sim_main.c "${SRCS[@]}" \
    -I"$HERE" -I"$LVGL" -I"$LVGL/src" \
    -DLV_CONF_INCLUDE_SIMPLE -lm
echo "built ./sim"
