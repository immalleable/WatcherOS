/* Headless LVGL renderer for WatcherOS README screenshots.
 * Reproduces each app screen's exact LVGL drawing (colors/radii/fonts/arrow
 * geometry copied from main/watcher_os.c) with representative data, renders
 * one frame into a 412x412 RGB565 framebuffer, and dumps it to a .565 file.
 * render.py then applies the round mask and writes PNGs. */
#include "lvgl.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define W 412
#define H 412
#define RADAR_CX 206
#define RADAR_CY 206
#define RADAR_R  190

static uint16_t fb[W*H];

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *px){
    for(int y=area->y1; y<=area->y2; y++)
        for(int x=area->x1; x<=area->x2; x++)
            fb[y*W+x] = px[(y-area->y1)*(area->x2-area->x1+1)+(x-area->x1)].full;
    lv_disp_flush_ready(drv);
}

static lv_disp_t *disp;
static void dump(const char *path){
    lv_refr_now(disp);
    FILE *f=fopen(path,"wb"); fwrite(fb,2,W*H,f); fclose(f);
    printf("wrote %s\n", path);
}
static lv_obj_t *fresh(void){
    lv_obj_t *s=lv_obj_create(NULL);
    lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
    lv_scr_load(s);
    return s;
}

/* ---- faithful geometry helpers (same math as watcher_os.c) ---- */
static double hav_km(double la1,double lo1,double la2,double lo2){
    double R=6371.0, dla=(la2-la1)*M_PI/180.0, dlo=(lo2-lo1)*M_PI/180.0;
    double a=sin(dla/2)*sin(dla/2)+cos(la1*M_PI/180)*cos(la2*M_PI/180)*sin(dlo/2)*sin(dlo/2);
    return 2*R*atan2(sqrt(a),sqrt(1-a));
}
static double bearing_deg(double la1,double lo1,double la2,double lo2){
    double y=sin((lo2-lo1)*M_PI/180)*cos(la2*M_PI/180);
    double x=cos(la1*M_PI/180)*sin(la2*M_PI/180)-sin(la1*M_PI/180)*cos(la2*M_PI/180)*cos((lo2-lo1)*M_PI/180);
    double b=atan2(y,x)*180.0/M_PI; if(b<0)b+=360; return b;
}
static void radar_ring(lv_obj_t *par,int d){
    lv_obj_t *r=lv_obj_create(par); lv_obj_set_size(r,d,d); lv_obj_center(r);
    lv_obj_set_style_radius(r,LV_RADIUS_CIRCLE,0); lv_obj_set_style_bg_opa(r,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_color(r,lv_color_hex(0x1f6b2f),0); lv_obj_set_style_border_width(r,2,0);
    lv_obj_clear_flag(r,LV_OBJ_FLAG_CLICKABLE); lv_obj_clear_flag(r,LV_OBJ_FLAG_SCROLLABLE);
}

/* ---- RADAR ---- */
static void build_radar(void){
    lv_obj_t *t=fresh();
    lv_obj_set_style_bg_color(t,lv_color_hex(0x02160a),0);
    radar_ring(t,380); radar_ring(t,260); radar_ring(t,140);
    lv_obj_t *c=lv_obj_create(t); lv_obj_set_size(c,8,8); lv_obj_center(c);
    lv_obj_set_style_radius(c,LV_RADIUS_CIRCLE,0); lv_obj_set_style_bg_color(c,lv_color_hex(0x8ce63a),0);
    lv_obj_set_style_border_width(c,0,0);
    /* sweep */
    static lv_point_t sp[2]; static lv_style_t sst; lv_style_init(&sst);
    lv_style_set_line_color(&sst,lv_color_hex(0x39ff6a)); lv_style_set_line_width(&sst,3);
    lv_obj_t *sw=lv_line_create(t); lv_obj_add_style(sw,&sst,0);
    float ang=0.7f; sp[0].x=RADAR_CX; sp[0].y=RADAR_CY;
    sp[1].x=RADAR_CX+(int)(RADAR_R*sinf(ang)); sp[1].y=RADAR_CY-(int)(RADAR_R*cosf(ang));
    lv_line_set_points(sw,sp,2);

    double my_lat=43.6532, my_lon=-79.3832, range=20.0;   /* Toronto, 20km */
    struct { double lat,lon; float track; const char*cs; } fl[] = {
        {43.72,-79.34,205,"ACA123"}, {43.60,-79.45,88,"WJA486"},
        {43.68,-79.30,310,"DAL229"}, {43.58,-79.38,15,"POE201"},
        {43.70,-79.42,145,"JZA8901"},{43.63,-79.29,260,"UAL55"},
    };
    static lv_point_t ap[8][4]; static lv_style_t arrow_st; lv_style_init(&arrow_st);
    lv_style_set_line_width(&arrow_st,2); lv_style_set_line_rounded(&arrow_st,true);
    int n=sizeof(fl)/sizeof(fl[0]);
    for(int i=0;i<n;i++){
        double d=hav_km(my_lat,my_lon,fl[i].lat,fl[i].lon); if(d>range)continue;
        double br=bearing_deg(my_lat,my_lon,fl[i].lat,fl[i].lon);
        int rp=(int)((d/range)*RADAR_R);
        int x=RADAR_CX+(int)(rp*sin(br*M_PI/180.0));
        int y=RADAR_CY-(int)(rp*cos(br*M_PI/180.0));
        double a=fl[i].track*M_PI/180.0; double fx=sin(a),fy=-cos(a);
        int bx=x-(int)(fx*4), by=y-(int)(fy*4);
        ap[i][0].x=x+(int)(fx*9);   ap[i][0].y=y+(int)(fy*9);
        ap[i][1].x=bx+(int)(-fy*5); ap[i][1].y=by+(int)(fx*5);
        ap[i][2].x=bx-(int)(-fy*5); ap[i][2].y=by-(int)(fx*5);
        ap[i][3]=ap[i][0];
        lv_obj_t *ln=lv_line_create(t); lv_obj_add_style(ln,&arrow_st,0);
        lv_line_set_points(ln,ap[i],4);
        lv_obj_set_style_line_color(ln,lv_color_hex(0xffee55),0);
        lv_obj_t *lb=lv_label_create(t); lv_label_set_text(lb,fl[i].cs);
        lv_obj_set_style_text_color(lb,lv_color_hex(0xcfeecf),0);
        lv_obj_set_style_text_font(lb,&lv_font_montserrat_16,0);
        lv_obj_set_pos(lb,x+7,y-8);
    }
    /* top range readout */
    lv_obj_t *rg=lv_label_create(t); lv_label_set_text(rg,"20km  (turn=zoom)");
    lv_obj_set_style_text_color(rg,lv_color_hex(0xffee55),0);
    lv_obj_set_style_text_font(rg,&lv_font_montserrat_16,0);
    lv_obj_set_style_bg_color(rg,lv_color_black(),0); lv_obj_set_style_bg_opa(rg,LV_OPA_50,0);
    lv_obj_set_style_pad_all(rg,3,0); lv_obj_align(rg,LV_ALIGN_TOP_MID,0,8);
    /* bottom status */
    lv_obj_t *st=lv_label_create(t); lv_label_set_text(st,"Toronto  6 fl");
    lv_obj_set_style_text_color(st,lv_color_hex(0x8ce63a),0);
    lv_obj_set_style_text_font(st,&lv_font_montserrat_16,0);
    lv_obj_set_style_bg_color(st,lv_color_black(),0); lv_obj_set_style_bg_opa(st,LV_OPA_50,0);
    lv_obj_set_style_pad_all(st,3,0); lv_obj_align(st,LV_ALIGN_BOTTOM_MID,0,-6);
    dump("out_radar.565");
}

/* ---- TIMER (running) ---- */
static void build_timer(void){
    lv_obj_t *t=fresh();
    lv_obj_set_style_bg_color(t,lv_color_hex(0x080a05),0);
    lv_obj_t *arc=lv_arc_create(t);
    lv_obj_set_size(arc,360,360); lv_obj_center(arc);
    lv_arc_set_rotation(arc,270); lv_arc_set_bg_angles(arc,0,360);
    lv_arc_set_range(arc,0,1000); lv_arc_set_value(arc,623);
    lv_obj_remove_style(arc,NULL,LV_PART_KNOB);
    lv_obj_set_style_arc_width(arc,10,LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc,10,LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc,lv_color_hex(0x203010),LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc,lv_color_hex(0x8ce63a),LV_PART_INDICATOR);
    lv_obj_t *tm=lv_label_create(t); lv_label_set_text(tm,"03:07");
    lv_obj_set_style_text_color(tm,lv_color_white(),0);
    lv_obj_set_style_text_font(tm,&lv_font_montserrat_48,0);
    lv_obj_align(tm,LV_ALIGN_CENTER,0,-8);
    lv_obj_t *stt=lv_label_create(t); lv_label_set_text(stt,"running   press: cancel");
    lv_obj_set_style_text_color(stt,lv_color_hex(0x8ce63a),0);
    lv_obj_set_style_text_font(stt,&lv_font_montserrat_16,0);
    lv_obj_set_style_text_align(stt,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(stt,LV_ALIGN_CENTER,0,52);
    dump("out_timer.565");
}

/* ---- WIFI (network list) ---- */
static void build_wifi(void){
    lv_obj_t *t=fresh();
    lv_obj_set_style_bg_color(t,lv_color_hex(0x0a0a12),0);
    lv_obj_t *ws=lv_label_create(t); lv_label_set_text(ws,"tap a network");
    lv_obj_set_style_text_color(ws,lv_color_hex(0x39d0ff),0);
    lv_obj_set_style_text_font(ws,&lv_font_montserrat_16,0);
    lv_obj_set_style_text_align(ws,LV_TEXT_ALIGN_CENTER,0);
    lv_obj_align(ws,LV_ALIGN_TOP_MID,0,30);
    lv_obj_t *ls=lv_list_create(t);
    lv_obj_set_size(ls,300,250); lv_obj_align(ls,LV_ALIGN_CENTER,0,30);
    lv_obj_set_style_bg_color(ls,lv_color_hex(0x14141c),0);
    lv_obj_set_style_border_width(ls,0,0);
    const char *ssids[]={"BELL741","TELUS-8829","eero-home","Pixel_hotspot","VM3344112"};
    for(int i=0;i<5;i++) lv_list_add_btn(ls,LV_SYMBOL_WIFI,ssids[i]);
    dump("out_wifi.565");
}

int main(void){
    lv_init();
    static lv_disp_draw_buf_t db; static lv_color_t buf[W*H];
    lv_disp_draw_buf_init(&db,buf,NULL,W*H);
    static lv_disp_drv_t drv; lv_disp_drv_init(&drv);
    drv.draw_buf=&db; drv.flush_cb=flush_cb; drv.hor_res=W; drv.ver_res=H;
    disp=lv_disp_drv_register(&drv);
    lv_theme_t *th=lv_theme_default_init(disp,lv_palette_main(LV_PALETTE_GREEN),
        lv_palette_main(LV_PALETTE_LIME),true,&lv_font_montserrat_16);
    lv_disp_set_theme(disp,th);
    build_radar();
    build_timer();
    build_wifi();
    return 0;
}
