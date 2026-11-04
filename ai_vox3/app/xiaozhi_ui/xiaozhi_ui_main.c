/****************************************************************************
 * xiaozhi_ui - XiaoZhi CompanionDisplay official 5-in-1 layout port
 *
 * 1:1 port of the xiaozhi companion_display.cc official weather-clock page
 * (nulllab-ai-vox-v3, DISPLAY_DESK_LAYOUT=1) to openvela LVGL 9.1 on fb0.
 * All coordinates copied verbatim from the measured official firmware
 * photos (kOff* constants). Zero image assets: everything drawn with
 * LVGL primitives. Fonts are the original subset C fonts.
 *
 * Data sources:
 *   time/date : system clock (NTP client sets it once WiFi is up)
 *   weather   : /tmp/weather.txt  "<text> <temp>"  (written by ai_agent)
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <nuttx/clock.h>   /* [V945] clock_systime_ticks() */
/* [V789] Stack watermark probe.  */
#ifdef CONFIG_SCHED
#include <sched.h>
#endif
#include <syslog.h>
#include <sys/stat.h>
#include <fcntl.h>      /* [V678] open()/lseek()/read() on /dev/fb0 */
#include <errno.h>      /* [V764] errno for the /dev/fb0 open() diagnostic */

#include <lvgl/lvgl.h>
#include "astronaut_map.h"   /* [V1227] AST_W/AST_H/astronaut_bits */

/* [V824b] fb_area_s / FBIO_UPDATE for the flush callback, and ioctl() itself.
 * The first build of this failed on all three, which is what reading the headers
 * first would have avoided. */
#include <sys/ioctl.h>
#include <nuttx/video/fb.h>
#include <sys/mman.h>   /* [V830] mmap() of /dev/fb0 */

/* [V1024] Probe gate.  V1024_QUIET is a compile-time constant, so
 * the guarded printfs disappear at compile time instead of being
 * skipped at run time.  Set it to 0 to bring all diagnostics back. */
#ifndef V1024_QUIET
#define V1024_QUIET 1
#endif

#define V1024P(...) do { if (V1024_QUIET == 0) printf(__VA_ARGS__); } while (0)
#define V1024S(lvl, ...) do { if (V1024_QUIET == 0) syslog(lvl, __VA_ARGS__); } while (0)


/* ---------------------------------------------------------------------------
 * Fonts (original xiaozhi subset fonts, LVGL 9 format)
 * ------------------------------------------------------------------------- */

LV_FONT_DECLARE(font_desk_20);
LV_FONT_DECLARE(font_desk_14);
LV_FONT_DECLARE(font_clock_70);   /* big clock, glyph height 52 */
LV_FONT_DECLARE(font_num_20);
LV_FONT_DECLARE(font_sec_30);     /* seconds, glyph height 21 */

#define F_DESK    (&font_desk_20)
#define F_SMALL   (&font_desk_14)
#define F_CLOCK   (&font_clock_70)
#define F_NUM     (&font_num_20)
#define F_SEC     (&font_sec_30)

/* [V672] Real Chinese for the status bar and the chat subtitle.
 *
 * The five fonts above are the official firmware's own subsets: ASCII plus
 * 442 CJK glyphs, all of them place names (see the note in
 * apps/graphics/lvgl/Makefile).  LVGL ships lv_font_simsun_16_cjk, whose
 * object file is already sitting in liblvgl.a, so referencing it here is all
 * it takes to pull it back into the image. */
LV_FONT_DECLARE(font_cn_16);   /* [V675] GB2312 7469-glyph CJK font (built with lv_font_conv from simhei.ttf) */
#define F_CJK     (&font_cn_16)

/* [V1222] bottom-right line-art astronaut, A8 alpha map (astronaut_map.c) */

/* ---------------------------------------------------------------------------
 * Show / hide helpers (LVGL 9)
 * ------------------------------------------------------------------------- */

#define LCLR_SHOW(o) lv_obj_remove_flag((o), LV_OBJ_FLAG_HIDDEN)
#define LCLR_HIDE(o) lv_obj_add_flag((o), LV_OBJ_FLAG_HIDDEN)
#define SCREEN_W 240
#define SCREEN_H 240

/* ---------------------------------------------------------------------------
 * Palette - official 5-in-1 firmware colors (measured from photos)
 * ------------------------------------------------------------------------- */

/* [V1222] reference photo: pure black background */
static const uint32_t kCardTop    = 0x000000;
static const uint32_t kCardBot    = 0x000000;

static const uint32_t kRain       = 0x4EA8FF;

static const uint32_t kOffTextHi  = 0xFFFFFF;   /* city */
static const uint32_t kOffText    = 0xFFFFFF;   /* wind / condition */
static const uint32_t kOffDate    = 0xFFFFFF;   /* date */
static const uint32_t kOffValue   = 0xFFFFFF;   /* 31C / 56% */
static const uint32_t kOffClockH  = 0xFFFFFF;   /* hour = white */
static const uint32_t kOffClockM  = 0xF2E19C;   /* min/sec = pale yellow */
static const uint32_t kOffPillBg  = 0x2FC44E;   /* air quality pill */
static const uint32_t kOffPillFg  = 0xFFFFFF;
static const uint32_t kOffSun     = 0xF7D91E;
static const uint32_t kOffCloud   = 0xA8DCD2;
static const uint32_t kOffTempIcon= 0xFFFFFF;
static const uint32_t kOffHumIcon = 0xFFFFFF;
static const uint32_t kOffBarTeal = 0x5FC6F2;   /* [V1222] bar gradient left */
static const uint32_t kOffBarMid  = 0x3FADE8;   /* [V1222] bar gradient mid */
static const uint32_t kOffBarSalmon = 0x1F7FD6; /* [V1222] bar gradient right */
static const uint32_t kOffHumFill = 0x5FC6F2;
static const uint32_t kOffHumTrack= 0x14304A;
static const uint32_t kOffDeco    = 0xFFFFFF;
static const uint32_t kOffMercury = 0xF05A5A;   /* [V1222] thermometer bulb */
static const uint32_t kOffDropFill= 0x49BCF2;   /* [V1222] droplet fill */

/* ---------------------------------------------------------------------------
 * Layout constants - measured from official firmware photos
 * ------------------------------------------------------------------------- */

static const int kOffCityX = 37;
static const int kOffCityY = 27;
static const int kOffCityW = 60;
static const int kOffPillX = 99;
static const int kOffPillY = 25;
static const int kOffPillW = 53;
static const int kOffPillH = 26;
static const int kOffWxX = 166;
static const int kOffWxY = 36;
static const int kOffWxW = 66;
static const int kOffWxH = 54;
static const int kOffWindX = 16;
static const int kOffWindY = 63;
static const int kOffWindW = 72;
static const int kOffCondX = 37;
static const int kOffCondY = 55;
static const int kOffCondW = 150;
static const int kOffClockX = 36;
static const int kOffClockY = 86;
static const int kOffClockLS = 3;
static const int kOffClockGap = 0;
static const int kOffSecX = 179;
static const int kOffSecY = 107;
static const int kOffDateX = 19;
static const int kOffDateY = 146;
static const int kOffBarX = 51;
static const int kOffTempBarY = 174;
static const int kOffBarW = 48;
static const int kOffBarH = 13;
static const int kOffTempValX = 105;
static const int kOffTempValY = 171;
static const int kOffHumBarY = 197;
static const int kOffBarW2 = 48;
static const int kOffHumValX = 105;
static const int kOffHumValY = 194;
static const int kOffDecoX = 155;
static const int kOffDecoY = 134;
static const int kOffDecoW = 83;
static const int kOffDecoH = 99;

/* weather icon group masks */
enum
  {
    G_SUN = 0x01,
    G_CLOUD = 0x02,
    G_RAIN = 0x04,
    G_FOG = 0x08,
    G_WIND = 0x10,
    G_SNOW = 0x20,
  };

/* [V1226] was 24; the 8 sun rays pushed the runtime count to 28, so the
 * old size overflowed g_icon_part[] into g_icon_grp[]/g_icon_parts and
 * apply_weather_icon() walked off into 0x10101010.  40 leaves headroom. */
#define K_ICON_PARTS_MAX 40
#define K_DECO_PARTS_MAX 16

static const char *kWeekday[] =
  {
    "周日", "周一", "周二", "周三", "周四", "周五", "周六"
  };

/* ---------------------------------------------------------------------------
 * Widget pointers
 * ------------------------------------------------------------------------- */

static lv_obj_t *g_page;
static lv_obj_t *g_city;
static lv_obj_t *g_air_pill;
static lv_obj_t *g_air_label;
static lv_obj_t *g_wind;
static lv_obj_t *g_cond;
static lv_obj_t *g_clock_h;
static lv_obj_t *g_clock_m;
static lv_obj_t *g_clock_s;
static lv_obj_t *g_date;
static lv_obj_t *g_wx_icon;
static lv_obj_t *g_temp_track;
static lv_obj_t *g_temp_fill;
static lv_obj_t *g_temp_fill_hi;
static lv_obj_t *g_temp_label;
static lv_obj_t *g_humid_track;
static lv_obj_t *g_humid_fill;
static lv_obj_t *g_humid_label;

static lv_obj_t *g_icon_part[K_ICON_PARTS_MAX];
static unsigned char g_icon_grp[K_ICON_PARTS_MAX];
static int g_icon_parts;

static lv_obj_t *g_deco_part[K_DECO_PARTS_MAX];
static int g_deco_parts;

/* weather cache */
static char g_wtext[32];   /* condition text, e.g. "晴" */
static int  g_wtemp;       /* degrees C */
static int  g_whumid = -1; /* [V1280] relative humidity %, -1 = unknown */
static int  g_wvalid;

/* ---------------------------------------------------------------------------
 * Companion mood / vision - provided by the ai_agent package.
 * The whole image is a flat build, so these are plain cross-task symbols.
 * ------------------------------------------------------------------------- */

extern int companion_mood_get(void);
extern const char *companion_mood_name(int mood);
extern void companion_face_stats(int *lines, int *faces, int *pan,
                                 int *tilt, int *have_peer);

/* mood style table - ported 1:1 from companion_display.cc kMoods[].
 * eye  : 0 dot  1 up-arc  2 down-arc  3 bar
 * mouth: 0 line 1 smile   2 frown    3 small O   4 big U */
typedef struct
{
  const char *name;
  uint32_t color;
  int eye;
  int mouth;
  int eye_w;
  int eye_h;
  int eye_dx;
  int mouth_w;
} mood_style_t;

static const mood_style_t kMoods[] =
{
  {"neutral",    0xF2C14E, 0, 0,  0,  0,  0, 56},
  {"happy",      0xF2C14E, 1, 4,  0,  0,  0, 70},
  {"laughing",   0xF2C14E, 1, 4,  0,  0,  0, 76},
  {"funny",      0xF2C14E, 1, 1,  0,  0,  0, 62},
  {"sad",        0x6EA8FE, 2, 2,  0,  0,  0, 52},
  {"crying",     0x6EA8FE, 2, 2,  0,  0,  0, 52},
  {"angry",      0xEA5455, 2, 2,  0,  0,  0, 52},
  {"loving",     0xFF6B9D, 0, 1,  0,  0,  0, 62},
  {"kissy",      0xFF6B9D, 1, 3,  0,  0,  0, 26},
  {"embarrassed",0xFF9F7A, 3, 0,  0,  0,  0, 48},
  {"surprised",  0x35C2FF, 0, 3, 22, 22, 14, 30},
  {"shocked",    0x35C2FF, 0, 3, 24, 24, 16, 34},
  {"thinking",   0x9B8CFF, 3, 0, 22,  8,  0, 40},
  {"winking",    0xF2C14E, 3, 1,  0,  0,  0, 62},
  {"cool",       0x35C2FF, 3, 1,  0,  0,  0, 62},
  {"relaxed",    0x28C76F, 1, 1,  0,  0,  0, 58},
  {"delicious",  0xF2C14E, 1, 4,  0,  0,  0, 70},
  {"confident",  0x35C2FF, 0, 4,  0,  0,  0, 66},
  {"sleepy",     0x9BA6B2, 3, 3,  0,  0,  0, 22},
  {"silly",      0xFF9F7A, 0, 1,  0,  0,  0, 60},
  {"confused",   0x9B8CFF, 0, 2,  0,  0,  0, 46},
};

#define K_MOODS (sizeof(kMoods) / sizeof(kMoods[0]))

/* face page widgets */
static lv_obj_t *g_face_page;
static lv_obj_t *g_eye_l;
static lv_obj_t *g_eye_r;
static lv_obj_t *g_mouth;
static lv_obj_t *g_sub_box;
static lv_obj_t *g_sub_label;
/* [V855] question bubble (the recognised utterance), drawn above the
 * reply bubble g_sub_box. */
static lv_obj_t *g_ask_box;
static lv_obj_t *g_ask_label;
static char      g_ask_text[192];
static char      g_ask_shown[192];
static lv_obj_t *g_vision_dot;

/* page / mood state */
static int g_desk_on = 1;         /* desk card visible (default) */
static int g_last_mood = 0;
static int g_face_hold_ticks;     /* counts down while the face is pinned */
#define FACE_HOLD_TICKS 25        /* 25 x 200 ms = 5 s */

/* subtitle (chat) queue - filled from /tmp/ui_state.txt */
static char g_sub_text[160];

/* [V672] *** Top status bar. ***
 *
 * Created last so it sits above both the weather card and the mood face, and
 * re-raised on every page switch.  It is driven by /tmp/ui_state.txt, which
 * ai_agent's voice_channel.c now writes on every BOOT press, release, reply
 * and end-of-reply -- so pressing BOOT has a visible effect within one poll
 * (read_ui_state() runs from the 200 ms mood timer, every fifth tick). */
/* [V824] LVGL flush callback: hand the dirty rectangle to the framebuffer.
 *
 * lv_disp_flush_ready() is not optional -- LVGL blocks the next refresh until it
 * is called, so omitting it stops the UI after one frame in a way that looks
 * identical to the original fault.
 */
static int v824_fb_fd = -1;
/* [V830] THE MISSING HALF OF THE PUSH PATH.
 *
 * v824 registered an LVGL display but its flush callback only called
 * ioctl(FBIO_UPDATE).  That ioctl means "the framebuffer MEMORY changed, please
 * blit this rectangle to the panel" -- it does not carry any pixels.  Nobody
 * ever wrote pixels into that memory, so it stayed at its kumm_zalloc() all-zero
 * state and the driver faithfully blitted a black rectangle forever.
 *
 * This is why the log looked healthy and the panel did not: SPI3 was pushing
 * ~1.8M words (see [V769]), fb_updates was incrementing, the backlight was held
 * HIGH ([V768]) -- and every one of those bytes was a zero.
 *
 * The other UI in this tree, info_screen_main.c, does it right and is the
 * template: mmap() the plane, write g_fbmem[...] = pixel, then FBIO_UPDATE.
 * v830 ports that copy step into the LVGL flush callback.
 */
static uint16_t *g_v830_fbmem     = NULL;
static uint32_t  g_v830_stride_px = 0;
static uint32_t  g_v830_copied    = 0;

/* [V973] How many rectangles LVGL actually asked us to blit.  Zero while
 * the picture is frozen points at the refresh side; non-zero points at the
 * panel / framebuffer path. */
static unsigned g_v973_flush;


static void v824_flush_cb(lv_display_t *disp,
                          const lv_area_t *area,
                          lv_color_t *color_p)
{
  struct fb_area_s a;

  a.x     = (uint16_t)area->x1;
  a.y     = (uint16_t)area->y1;
  a.w     = (uint16_t)(area->x2 - area->x1 + 1);
  a.h     = (uint16_t)(area->y2 - area->y1 + 1);

  /* [V830] Copy LVGL's rendered strip into the framebuffer memory BEFORE
   * asking the driver to blit it.  Without this the ioctl below has nothing
   * new to send and GRAM keeps the all-zero image from boot.
   *
   * PARTIAL mode hands us exactly rows area->y1..area->y2 and columns
   * area->x1..area->x2, packed (no row padding) inside color_p.  The
   * framebuffer's own row pitch is pinfo.stride bytes, so the copy is
   * per-row, not one memcpy.
   */
  if (g_v830_fbmem != NULL && g_v830_stride_px != 0)
    {
      uint32_t  wpx = (uint32_t)(area->x2 - area->x1 + 1);
      const uint16_t *src = (const uint16_t *)color_p;
      uint16_t *dst = g_v830_fbmem
                      + (uint32_t)area->y1 * g_v830_stride_px
                      + (uint32_t)area->x1;
      uint32_t  y;

      for (y = (uint32_t)area->y1; y <= (uint32_t)area->y2; y++)
        {
          memcpy(dst, src, wpx * sizeof(uint16_t));
          dst += g_v830_stride_px;
          src += wpx;
        }

      g_v830_copied++;
      g_v973_flush++;   /* [V973] */

      if (g_v830_copied <= 3)
        {
          V1024S(LOG_ERR, "[V830]] copied strip %u,%u %ux%u -> fbmem=%p "
                 "src[0]=0x%04x dst[0]=0x%04x (copy %u)\\n",
                 (unsigned)area->x1, (unsigned)area->y1,
                 (unsigned)wpx, (unsigned)(area->y2 - area->y1 + 1),
                 g_v830_fbmem,
                 (unsigned)((const uint16_t *)color_p)[0],
                 (unsigned)g_v830_fbmem[0],
                 (unsigned)g_v830_copied);
        }
    }

  if (v824_fb_fd >= 0)
    {
      /* Errors here are counted, not fatal: a failed push means the panel did
       * not get this rectangle, which is exactly what needs to be visible in
       * the log rather than swallowed. */
      int ret = ioctl(v824_fb_fd, FBIO_UPDATE, (unsigned long)&a);

      if (ret < 0)
        {
          static unsigned s_errs;

          if (++s_errs <= 8)
            {
              V1024S(LOG_ERR, "[V824]] FBIO_UPDATE %u,%u %ux%u failed: %d\n",
                     a.x, a.y, a.w, a.h, ret);
            }
        }
      else if (a.w > 100 && a.h > 100)
        {
          static unsigned s_big;

          if (++s_big <= 4)
            {
              V1024S(LOG_ERR, "[V824]] pushed %ux%u at %u,%u (push %u)\n",
                     a.w, a.h, a.x, a.y, s_big);
            }
        }
    }

  lv_display_flush_ready(disp);
}

static lv_obj_t *g_sta_bar;
static lv_obj_t *g_sta_label;
static char      g_sta_phase[24];   /* phase just read from the file  */
static char      g_sta_shown[24];   /* phase currently painted        */
static char      g_sub_shown[160];  /* subtitle currently painted     */

/* ---------------------------------------------------------------------------
 * Companion mood face page (ported from CompanionDisplay::BuildMoodFace)
 * ------------------------------------------------------------------------- */

/* primitive helpers are defined further down */
static lv_obj_t *make_box(lv_obj_t *parent, int x, int y, int w, int h,
                          uint32_t color, int radius);
static lv_obj_t *make_dot(lv_obj_t *parent, int x, int y, int d,
                          uint32_t color);

static const mood_style_t *find_mood(const char *mood)
{
  if (mood != NULL)
    {
      for (unsigned i = 0; i < K_MOODS; i++)
        {
          if (strcmp(kMoods[i].name, mood) == 0)
            {
              return &kMoods[i];
            }
        }
    }
  return &kMoods[0];
}

/* ---------------------------------------------------------------------------
 * Top status bar: BOOT press -> "正在聆听", release -> "正在思考",
 * reply starts -> "正在回答", everything done -> hidden.
 * ------------------------------------------------------------------------- */

static void build_status_bar(void)
{
  g_sta_bar = lv_obj_create(lv_screen_active());
  lv_obj_remove_style_all(g_sta_bar);
  lv_obj_set_size(g_sta_bar, SCREEN_W, 30);
  lv_obj_align(g_sta_bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(g_sta_bar, lv_color_hex(0x141618), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_sta_bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_sta_bar, 3, LV_PART_MAIN);
  lv_obj_set_style_border_side(g_sta_bar, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_sta_bar, lv_color_hex(0x4EA8FF),
                                LV_PART_MAIN);
  lv_obj_set_style_radius(g_sta_bar, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_sta_bar, 0, LV_PART_MAIN);
  lv_obj_remove_flag(g_sta_bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(g_sta_bar, LV_OBJ_FLAG_CLICKABLE);

  g_sta_label = lv_label_create(g_sta_bar);
  lv_obj_set_width(g_sta_label, SCREEN_W - 12);
  lv_label_set_long_mode(g_sta_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_align(g_sta_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(g_sta_label, lv_color_hex(0xF0F4EA),
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(g_sta_label, F_CJK, LV_PART_MAIN);
  lv_obj_center(g_sta_label);
  lv_label_set_text(g_sta_label, "");

  g_sta_phase[0] = '\0';
  g_sta_shown[0] = '\0';
  LCLR_HIDE(g_sta_bar);
}

/* Map the agent's phase word onto what the user should read. */
static void apply_status(const char *phase)
{
  const char *txt = NULL;
  uint32_t col = 0x4EA8FF;

  if (phase != NULL && strcmp(phase, "listening") == 0)
    {
      txt = "\xe6\xad\xa3\xe5\x9c\xa8\xe8\x81\x86\xe5\x90\xac";   /* 正在聆听 */
      col = 0x28C76F;
    }
  else if (phase != NULL && strcmp(phase, "thinking") == 0)
    {
      txt = "\xe6\xad\xa3\xe5\x9c\xa8\xe6\x80\x9d\xe8\x80\x83";   /* 正在思考 */
      col = 0xF7D91E;
    }
  else if (phase != NULL && strcmp(phase, "speaking") == 0)
    {
      txt = "\xe6\xad\xa3\xe5\x9c\xa8\xe5\x9b\x9e\xe7\xad\x94";   /* 正在回答 */
      col = 0x4EA8FF;
    }

  if (txt == NULL)
    {
      if (g_sta_shown[0] != '\0')
        {
          /* [V673] console-visible proof of the other half of the chain */
          syslog(LOG_INFO, "[XZUI] status bar -> (none)\n");
        }
      g_sta_shown[0] = '\0';
      LCLR_HIDE(g_sta_bar);
      return;
    }

  if (strcmp(txt, g_sta_shown) != 0)
    {
      snprintf(g_sta_shown, sizeof(g_sta_shown), "%s", txt);
      lv_label_set_text(g_sta_label, g_sta_shown);
      lv_obj_set_style_border_color(g_sta_bar, lv_color_hex(col), LV_PART_MAIN);
      LCLR_SHOW(g_sta_bar);
      syslog(LOG_INFO, "[XZUI] status bar -> %s\n", phase);
    }

  lv_obj_move_foreground(g_sta_bar);
}

static void build_face_page(void)
{
  g_face_page = lv_obj_create(lv_screen_active());
  lv_obj_set_size(g_face_page, SCREEN_W, SCREEN_H);
  lv_obj_center(g_face_page);
  lv_obj_set_style_bg_color(g_face_page, lv_color_hex(0x0A0B0C),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_face_page, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_face_page, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_face_page, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(g_face_page, 0, LV_PART_MAIN);
  lv_obj_remove_flag(g_face_page, LV_OBJ_FLAG_SCROLLABLE);
  LCLR_HIDE(g_face_page);

  g_eye_l = lv_obj_create(g_face_page);
  lv_obj_remove_style_all(g_eye_l);
  g_eye_r = lv_obj_create(g_face_page);
  lv_obj_remove_style_all(g_eye_r);

  g_mouth = lv_arc_create(g_face_page);
  lv_obj_set_style_bg_opa(g_mouth, LV_OPA_TRANSP, LV_PART_MAIN);

  /* subtitle bar at the bottom (xiaozhi bottom_bar look) */
  g_sub_box = lv_obj_create(g_face_page);
  lv_obj_remove_style_all(g_sub_box);
  lv_obj_set_size(g_sub_box, SCREEN_W - 16, 40);
  lv_obj_align(g_sub_box, LV_ALIGN_BOTTOM_MID, 0, -12);
  lv_obj_set_style_bg_color(g_sub_box, lv_color_hex(0x141414), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_sub_box, LV_OPA_80, LV_PART_MAIN);
  lv_obj_set_style_radius(g_sub_box, 12, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_sub_box, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_sub_box, lv_color_hex(0x4EA8FF),
                                LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_sub_box, 4, LV_PART_MAIN);
  lv_obj_remove_flag(g_sub_box, LV_OBJ_FLAG_SCROLLABLE);

  g_sub_label = lv_label_create(g_sub_box);
  lv_obj_set_width(g_sub_label, SCREEN_W - 32);
  lv_label_set_long_mode(g_sub_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(g_sub_label, LV_TEXT_ALIGN_CENTER,
                              LV_PART_MAIN);
  lv_obj_set_style_text_color(g_sub_label, lv_color_hex(0xF0F4EA),
                              LV_PART_MAIN);
  /* [V672] F_SMALL carries 442 place-name glyphs and no verbs, so the agent's
   * reply could never actually be read.  Use the real CJK font. */
  lv_obj_set_style_text_font(g_sub_label, F_CJK, LV_PART_MAIN);
  lv_obj_center(g_sub_label);
  lv_label_set_text(g_sub_label, "");
  LCLR_HIDE(g_sub_box);

  /* [V855] Question bubble.
   *
   * Geometry: the eyes sit at y=74 and the mouth at y=118 with a height
   * of at most 24, so everything below y=144 is free.  The reply bubble
   * (g_sub_box) is anchored to the bottom (y 188..228), so the question
   * bubble takes 146..184 -- a 4 px gap to either neighbour. */
  g_ask_box = lv_obj_create(g_face_page);
  lv_obj_remove_style_all(g_ask_box);
  lv_obj_set_size(g_ask_box, SCREEN_W - 16, 38);
  lv_obj_align(g_ask_box, LV_ALIGN_TOP_MID, 0, 146);
  lv_obj_set_style_bg_color(g_ask_box, lv_color_hex(0x101A24),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_ask_box, LV_OPA_80, LV_PART_MAIN);
  lv_obj_set_style_radius(g_ask_box, 12, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_ask_box, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_ask_box, lv_color_hex(0x4EA8FF),
                                LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_ask_box, 4, LV_PART_MAIN);
  lv_obj_remove_flag(g_ask_box, LV_OBJ_FLAG_SCROLLABLE);

  g_ask_label = lv_label_create(g_ask_box);
  lv_obj_set_width(g_ask_label, SCREEN_W - 32);
  lv_label_set_long_mode(g_ask_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(g_ask_label, LV_TEXT_ALIGN_CENTER,
                              LV_PART_MAIN);
  lv_obj_set_style_text_color(g_ask_label, lv_color_hex(0xBFD8FF),
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(g_ask_label, F_CJK, LV_PART_MAIN);
  lv_obj_center(g_ask_label);
  lv_label_set_text(g_ask_label, "");
  LCLR_HIDE(g_ask_box);

  /* vision indicator dot (companion_face / K230 tracking) */
  g_vision_dot = make_dot(g_face_page, 214, 12, 12, 0x28C76F);
  LCLR_HIDE(g_vision_dot);
}

static void apply_mood(const char *mood)
{
  const mood_style_t *s = find_mood(mood);

  const int eye_w = s->eye_w > 0 ? s->eye_w : 26;
  const int eye_h = s->eye_h > 0 ? s->eye_h : 26;
  const int dx = s->eye_dx > 0 ? s->eye_dx : 34;
  const int eye_y = 74;

  lv_obj_t *eyes[2];
  eyes[0] = g_eye_l;
  eyes[1] = g_eye_r;

  for (int i = 0; i < 2; i++)
    {
      lv_obj_t *e = eyes[i];
      const int sign = (i == 0) ? -1 : 1;
      lv_obj_set_size(e, eye_w, eye_h);
      lv_obj_align(e, LV_ALIGN_TOP_MID, sign * dx, eye_y);
      lv_obj_set_style_border_width(e, 0, LV_PART_MAIN);
      lv_obj_set_style_bg_opa(e, LV_OPA_COVER, LV_PART_MAIN);
      lv_obj_set_style_bg_color(e, lv_color_hex(s->color), LV_PART_MAIN);
      if (s->eye == 3)
        {
          lv_obj_set_style_radius(e, eye_h / 2, LV_PART_MAIN);
        }
      else
        {
          lv_obj_set_style_radius(e, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        }
    }

  /* mouth */
  int mh = s->mouth_w / 2 > 24 ? 24 : s->mouth_w / 2;
  lv_obj_set_size(g_mouth, s->mouth_w, mh);
  lv_obj_align(g_mouth, LV_ALIGN_TOP_MID, 0, 118);
  lv_obj_set_style_arc_width(g_mouth, 6, LV_PART_MAIN);
  lv_obj_set_style_arc_color(g_mouth, lv_color_hex(s->color),
                             LV_PART_MAIN);
  lv_obj_set_style_arc_opa(g_mouth, LV_OPA_TRANSP, LV_PART_INDICATOR);
  lv_obj_set_style_opa(g_mouth, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_border_width(g_mouth, 0, LV_PART_MAIN);
  lv_obj_remove_flag(g_mouth, LV_OBJ_FLAG_CLICKABLE);

  switch (s->mouth)
    {
      case 1:
      case 4:
        lv_arc_set_bg_angles(g_mouth, 0, 180);
        break;
      case 2:
        lv_arc_set_bg_angles(g_mouth, 180, 360);
        break;
      case 3:
        lv_arc_set_bg_angles(g_mouth, 0, 360);
        break;
      default:
        lv_arc_set_bg_angles(g_mouth, 0, 1);
        break;
    }
}

/* mood value (-2..+2) -> mood style name */
static const char *mood_value_to_name(int m)
{
  switch (m)
    {
      case -2: return "crying";
      case -1: return "sad";
      case  1: return "happy";
      case  2: return "laughing";
      default: return "neutral";
    }
}

static void show_desk_page(void)
{
  if (g_desk_on)
    {
      return;
    }
  g_desk_on = 1;
  LCLR_SHOW(g_page);
  lv_obj_move_foreground(g_page);
  LCLR_HIDE(g_face_page);
  if (g_sta_bar != NULL)
    {
      lv_obj_move_foreground(g_sta_bar);
    }
}

static void show_face_page(const char *mood)
{
  apply_mood(mood);
  if (!g_desk_on)
    {
      return;
    }
  g_desk_on = 0;
  LCLR_HIDE(g_page);
  LCLR_SHOW(g_face_page);
  lv_obj_move_foreground(g_face_page);
  if (g_sta_bar != NULL)
    {
      lv_obj_move_foreground(g_sta_bar);
    }
}

/* ---------------------------------------------------------------------------
 * Optional agent state file: /tmp/ui_state.txt
 *   state=idle|listening|speaking|thinking
 *   chat=<subtitle text>
 * Absent file => the face page is driven by the mood sensors alone.
 * ------------------------------------------------------------------------- */

/* [V857] Stale-content guard for /tmp/ui_state.txt.
 *
 * ui_state_flush() used to run only on a phase change, so anything that
 * was written into the file by hand -- the NSH render check that put
 * ask="Nihao, xianzai ji dian le?" on the panel -- stayed there for as
 * long as the board was up and read as "the dialog boxes are stuck and
 * in English".  The agent now rewrites the file once a second and bumps
 * beat=; this side declares the content dead 15 s after the last beat.
 *
 * [V857f] stat()/mtime is only consulted when the file carries no
 * beat= line, i.e. when the agent is not the one who wrote it. */
static unsigned long g_v857_beat;
static unsigned long g_v857_prev_beat;
static int           g_v857_have_beat;
static time_t        g_v857_beat_ts;   /* [V857e] last beat change  */
static int           g_v857_stale_rep;

static void read_ui_state(void)
{
  FILE *f = fopen("/tmp/ui_state.txt", "r");
  char line[160];
  int want_face = 0;

  g_ask_text[0]  = '\0';
  g_sub_text[0]  = '\0';
  g_sta_phase[0] = '\0';
  if (f == NULL)
    {
      /* [V672] no agent state file -> nothing to report upstairs */
      apply_status("");
      return;
    }

  while (fgets(line, sizeof(line), f) != NULL)
    {
      if (strncmp(line, "ask=", 4) == 0)
        {
          char *v = line + 4;
          size_t n = strlen(v);
          while (n > 0 && (v[n - 1] == '\n' || v[n - 1] == '\r'))
            {
              v[--n] = '\0';
            }
          snprintf(g_ask_text, sizeof(g_ask_text), "%s", v);
        }
      else if (strncmp(line, "chat=", 5) == 0)
        {
          char *v = line + 5;
          size_t n = strlen(v);
          while (n > 0 && (v[n - 1] == '\n' || v[n - 1] == '\r'))
            {
              v[--n] = '\0';
            }
          snprintf(g_sub_text, sizeof(g_sub_text), "%s", v);
        }
      else if (strncmp(line, "beat=", 5) == 0)
        {
          g_v857_beat      = strtoul(line + 5, NULL, 10);
          g_v857_have_beat = 1;
        }
      else if (strncmp(line, "state=", 6) == 0)
        {
          const char *v = line + 6;
          size_t n = strlen(v);
          while (n > 0 && (v[n - 1] == '\n' || v[n - 1] == '\r'))
            {
              n--;
            }
          if (n >= sizeof(g_sta_phase))
            {
              n = sizeof(g_sta_phase) - 1;
            }
          memcpy(g_sta_phase, v, n);
          g_sta_phase[n] = '\0';

          if (strcmp(g_sta_phase, "listening") == 0 ||
              strcmp(g_sta_phase, "speaking")  == 0 ||
              strcmp(g_sta_phase, "thinking")  == 0)
            {
              want_face = 1;
            }
        }
    }
  fclose(f);

  /* [V857] Declare the file dead if the agent has stopped rewriting it.
   * Everything below then treats it exactly like a missing file, so the
   * bubbles are hidden instead of showing somebody else's leftover. */

  {
    int         stale = 0;
    struct stat st;
    time_t      now;

    /* [V857f] beat= and mtime are ALTERNATIVES, not two votes.
     *
     * ORing them meant a filesystem that does not stamp mtime made
     * every file look old forever: V857e still logged 14 stale events
     * in 150 s while the NSH probe showed the beat advancing normally
     * (190 -> 196 -> 203).  The old "only trust mtime once it has been
     * seen to move" test was set by the very first read, so it never
     * guarded anything. */

    if (g_v857_have_beat)
      {
        time(&now);

        /* [V857e] REAL seconds, not read counts.
         *
         * This used to count read_ui_state() calls, and that function
         * runs about 4.5x per second -- so "45" was ~10 s, shorter
         * than one ASR+LLM+TTS turn, and the guard wiped the answer
         * the moment the board stopped speaking.  The agent side now
         * has its own heartbeat thread (V857e), so 20 real seconds of
         * silence really does mean the writer is gone. */

        if (g_v857_beat != g_v857_prev_beat)
          {
            g_v857_prev_beat = g_v857_beat;
            g_v857_beat_ts   = now;
          }

        if (g_v857_beat_ts != 0 && (now - g_v857_beat_ts) > 20)
          {
            stale = 1;
          }
      }
    else if (stat("/tmp/ui_state.txt", &st) == 0)
      {
        /* [V857f] No beat= line at all: the agent never wrote this
         * file.  That is exactly the reported fault -- an NSH render
         * check left ask="Nihao, xianzai ji dian le?" on the panel
         * for hours.  mtime is all there is, so use it. */

        time(&now);

        if (st.st_mtime <= now && (now - st.st_mtime) > 20)
          {
            stale = 1;
          }
      }

    /* [V857c] The agent says it is mid-recognition or mid-utterance: it
     * is alive and simply cannot beat (voice_channel_stop() blocks in a
     * synchronous TLS+HTTP round trip for 20 s or more).  Never clear on
     * that -- doing so blanked the status bar and the subtitle in the
     * middle of a reply.  Every other phase, including a missing one,
     * still goes through the window above. */

    if (stale &&
        !(g_sta_phase[0] != '\0' &&
          (strcmp(g_sta_phase, "thinking") == 0 ||
           strcmp(g_sta_phase, "speaking") == 0)))
      {
        g_ask_text[0]  = '\0';
        g_sub_text[0]  = '\0';
        g_sta_phase[0] = '\0';

        if (!g_v857_stale_rep)
          {
            g_v857_stale_rep = 1;
            syslog(LOG_INFO,
                   "[XZUI] [V857f] ui_state stale (>20 s) -> cleared\n");
          }
      }
    else if (g_v857_stale_rep)
      {
        g_v857_stale_rep = 0;
      }
  }

  /* [V886] Content-age TTL.
   *
   * Nothing in this build ever truncates /tmp/ui_state.txt: the agent's
   * ui_state_flush() is unreachable because voice_channel_init() returns
   * at line 3870 ([V872] "aivox is the only voice assistant"), and
   * ai_vox's ui_send() only ever APPENDS an "ask=" line.  So any text
   * that reaches this file stays there for the whole uptime, want_face
   * stays 1 forever, and the weather card never comes back -- the user
   * sees one lone face on a black page and nothing else.
   *
   * The mtime branch above cannot save us: it needs a beat= line or a
   * trustworthy tmpfs timestamp, and neither exists here.  Time the
   * CONTENT instead: if the visible ask/sub text has not changed for
   * V886_TTL_TICKS ticks, declare it done and drop it.  read_ui_state()
   * is called once per 200 ms tick, so 50 ticks is 10 s. */
  {
    static unsigned long s_v886_age;
    static char          s_v886_seen[320];
    char                 cur[320];
    static int           s_v886_rep;

    snprintf(cur, sizeof(cur), "%s|%s", g_ask_text, g_sub_text);

    if (strcmp(cur, s_v886_seen) == 0)
      {
        s_v886_age++;
      }
    else
      {
        snprintf(s_v886_seen, sizeof(s_v886_seen), "%s", cur);
        s_v886_age = 0;
        s_v886_rep = 0;
      }

    /* [V857c] same courtesy as the stale guard: never blank the screen
     * in the middle of a real exchange. */
    if (s_v886_age > 50 &&
        (g_ask_text[0] != '\0' || g_sub_text[0] != '\0') &&
        !(g_sta_phase[0] != '\0' &&
          (strcmp(g_sta_phase, "thinking") == 0 ||
           strcmp(g_sta_phase, "speaking") == 0)))
      {
        g_ask_text[0] = '\0';
        g_sub_text[0] = '\0';
        if (!s_v886_rep)
          {
            s_v886_rep = 1;
            syslog(LOG_INFO,
                   "[XZUI] [V886] ask/sub unchanged for 10 s -> cleared\n");
          }
      }
  }

  /* [V672] top status bar */
  apply_status(g_sta_phase);

  /* [V855] question bubble */
  if (g_ask_text[0] != '\0')
    {
      if (strcmp(g_ask_text, g_ask_shown) != 0)
        {
          snprintf(g_ask_shown, sizeof(g_ask_shown), "%s", g_ask_text);
          lv_label_set_text(g_ask_label, g_ask_text);
          LCLR_SHOW(g_ask_box);
          lv_obj_move_foreground(g_ask_box);
          syslog(LOG_INFO, "[XZUI] ask: \"%s\"\n", g_ask_text);
        }
    }
  else if (g_ask_shown[0] != '\0')
    {
      g_ask_shown[0] = '\0';
      lv_label_set_text(g_ask_label, "");
      LCLR_HIDE(g_ask_box);
    }

  /* [V855] Either bubble carrying text is enough to keep the face page
   * up: the user is reading the exchange, not the weather card. */
  if (g_ask_text[0] != '\0' || g_sub_text[0] != '\0')
    {
      want_face = 1;
    }

  if (want_face)
    {
      show_face_page(g_last_mood != 0 ? mood_value_to_name(g_last_mood)
                                      : "neutral");
      g_face_hold_ticks = FACE_HOLD_TICKS;
    }

  if (g_sub_text[0] != '\0')
    {
      if (strcmp(g_sub_text, g_sub_shown) != 0)
        {
          snprintf(g_sub_shown, sizeof(g_sub_shown), "%s", g_sub_text);
          lv_label_set_text(g_sub_label, g_sub_text);
          LCLR_SHOW(g_sub_box);
          syslog(LOG_INFO, "[XZUI] subtitle: \"%s\"\n", g_sub_text);
        }
    }
  else if (g_sub_shown[0] != '\0')
    {
      g_sub_shown[0] = '\0';
      lv_label_set_text(g_sub_label, "");
      LCLR_HIDE(g_sub_box);
      syslog(LOG_INFO, "[XZUI] subtitle: (cleared)\n");
    }
}

/* ---------------------------------------------------------------------------
 * 200 ms companion tick: mood face + vision indicator
 * ------------------------------------------------------------------------- */

/* [V678] fb heartbeat -- is the framebuffer still being repainted?
 *
 * The panel went dark while every software-side log stayed green, so sample
 * two rows of /dev/fb0 every ~30 s and print their checksum:
 *   - values keep changing / stay plausible -> rendering is alive, the
 *     fault is on the panel side (backlight pad or SPI);
 *   - values drop to 0 and stay there                        -> LVGL is
 *     painting black, the fault is in the render path.
 */
static int g_v678_div;

static void v678_fb_heartbeat(void)
{
  unsigned char rowb[64];
  unsigned sum1 = 0;
  unsigned sum2 = 0;
  int fd;
  int i;

  fd = open("/dev/fb0", O_RDONLY);
  if (fd < 0)
    {
      /* [V764] Print errno.  "open fb0 failed" is ambiguous across two
       * completely different faults that were observed one second apart on
       * the same cold boot:
       *
       *   87.31 [V760] panel wake re-assert #16 ..., fb=3   watchdog: O_RDWR ok
       *   88.80 [XZUI] [V678] open fb0 failed                 here
       *
       * The node exists, so ENOENT cannot be the answer - the open was
       * failing for want of memory, i.e. ENOMEM, which is the same
       * exhaustion that already produced "probe gw:80 = -12" and
       * "Failed to create agent_loop task".  Without the errno this looks
       * like a driver race and gets "fixed" by adding retries, which is the
       * wrong fix for a memory fault. */
      syslog(LOG_ERR, "[XZUI] [V678] open fb0 failed: %d\n", errno);

      return;
    }

  if (lseek(fd, (off_t)120 * SCREEN_W * 2, SEEK_SET) >= 0 &&
      read(fd, rowb, sizeof(rowb)) == (int)sizeof(rowb))
    {
      for (i = 0; i < (int)sizeof(rowb); i++)
        {
          sum1 += rowb[i];
        }
    }

  if (lseek(fd, (off_t)210 * SCREEN_W * 2, SEEK_SET) >= 0 &&
      read(fd, rowb, sizeof(rowb)) == (int)sizeof(rowb))
    {
      for (i = 0; i < (int)sizeof(rowb); i++)
        {
          sum2 += rowb[i];
        }
    }

  close(fd);
  {
    static unsigned s_v679_beats;
    syslog(LOG_INFO, "[XZUI] [V679] fb beat=%u row120=%u row210=%u\n",
           ++s_v679_beats, sum1, sum2);
  }
}

static void mood_cb(lv_timer_t *t)
{
  static int state_div;
  int mood = companion_mood_get();


  /* mood changed -> pop the face and pin it for a few seconds */
  /* [V885] Disabled.  This popped the mood face on every mood change
   * with no conversation behind it - weather -> a lone face -> weather.
   * The other show_face_page() caller (the want_face branch above) is
   * the conversation bubble and still works. */
  if (0 && mood != g_last_mood)
    {
      g_last_mood = mood;
      show_face_page(mood_value_to_name(mood));
      g_face_hold_ticks = FACE_HOLD_TICKS;
      syslog(LOG_INFO, "[XZUI] mood=%d (%s) -> face page\n",
             mood, companion_mood_name(mood));
    }
  else if (!g_desk_on)
    {
      if (g_face_hold_ticks > 0)
        {
          g_face_hold_ticks--;
        }
      else
        {
          /* no chat text pending -> go back to the weather card */
          if (g_sub_text[0] == '\0')
            {
              show_desk_page();
            }
        }
    }

  /* vision indicator: green while the K230 is streaming face boxes */
  {
    int lines = 0;
    int faces = -1;
    int pan = 0;
    int tilt = 0;
    int have_peer = 0;
    companion_face_stats(&lines, &faces, &pan, &tilt, &have_peer);
    if (!g_desk_on && have_peer)
      {
        lv_obj_set_style_bg_color(g_vision_dot,
                                  lv_color_hex(faces > 0 ? 0x28C76F
                                                         : 0xEA5455),
                                  LV_PART_MAIN);
        LCLR_SHOW(g_vision_dot);
      }
    else
      {
        LCLR_HIDE(g_vision_dot);
      }
  }

  /* [V672] Poll the agent state file twice as often as before (400 ms
   * instead of 1 s): the status bar is the only feedback the BOOT key has,
   * and a whole second of "nothing happened" reads as a dead button.  One
   * fopen/fgets pair per 400 ms costs nothing next to a 240x240 repaint. */
  /* [V855] Every 200 ms tick, not every second one: the two bubbles are
   * the only feedback a voice turn has, and 400 ms of lag on top of the
   * network round trip reads as "it did not hear me". */
  if (++state_div >= 1)
    {
      state_div = 0;
      read_ui_state();
    }

  /* [V678] framebuffer heartbeat: 150 x 200 ms = 30 s */
  if (++g_v678_div >= 150)
    {
      g_v678_div = 0;
      v678_fb_heartbeat();
    }
}


/* ---------------------------------------------------------------------------
 * Primitive helpers (all shapes drawn live, zero assets)
 * ------------------------------------------------------------------------- */

static lv_point_precise_t *seg_pool(int *used)
{
  static lv_point_precise_t pool[64][2];
  static int n = 0;
  if (n >= 64)
    {
      n = 0;
    }
  *used = n;
  return pool[n++];
}

static lv_obj_t *make_box(lv_obj_t *parent, int x, int y, int w, int h,
                          uint32_t color, int radius)
{
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_size(o, w, h);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_style_bg_color(o, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(o, radius, LV_PART_MAIN);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static lv_obj_t *make_dot(lv_obj_t *parent, int x, int y, int d,
                          uint32_t color)
{
  return make_box(parent, x, y, d, d, color, LV_RADIUS_CIRCLE);
}

static lv_obj_t *make_frame(lv_obj_t *parent, int x, int y, int w, int h,
                            uint32_t color, int radius, int bw)
{
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_remove_style_all(o);
  lv_obj_set_size(o, w, h);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(o, bw, LV_PART_MAIN);
  lv_obj_set_style_border_color(o, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_radius(o, radius, LV_PART_MAIN);
  lv_obj_set_style_pad_all(o, 0, LV_PART_MAIN);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static lv_obj_t *make_seg(lv_obj_t *parent, int x1, int y1, int x2, int y2,
                          uint32_t color, int lw)
{
  int idx = 0;
  lv_point_precise_t *pts = seg_pool(&idx);
  pts[0].x = x1;
  pts[0].y = y1;
  pts[1].x = x2;
  pts[1].y = y2;
  lv_obj_t *o = lv_line_create(parent);
  lv_obj_remove_style_all(o);
  lv_line_set_points(o, pts, 2);
  lv_obj_set_style_line_width(o, lw, LV_PART_MAIN);
  lv_obj_set_style_line_color(o, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_line_rounded(o, true, LV_PART_MAIN);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
  return o;
}

/* make a plain transparent label with given font/color */
static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font,
                            uint32_t color, const char *text)
{
  lv_obj_t *o = lv_label_create(parent);
  lv_label_set_text(o, text);
  lv_obj_set_style_text_font(o, font, LV_PART_MAIN);
  lv_obj_set_style_text_color(o, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, LV_PART_MAIN);
  return o;
}

/* single-line fitted text: shrink to the small font when too wide,
 * pin the height to the line height so it can never wrap */
static void set_one_line_fitted(lv_obj_t *label, const char *txt, int max_w,
                                const lv_font_t *fbig,
                                const lv_font_t *fsmall, int base_y)
{
  if (label == NULL || txt == NULL)
    {
      return;
    }
  const lv_font_t *use = fbig;
  lv_point_t sz;
  sz.x = 0;
  sz.y = 0;
  lv_text_get_size(&sz, txt, fbig, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  if (sz.x > max_w)
    {
      use = fsmall;
    }
  const int lh_big = lv_font_get_line_height(fbig);
  const int lh_use = lv_font_get_line_height(use);
  lv_obj_set_style_text_font(label, use, LV_PART_MAIN);
  lv_obj_set_width(label, max_w);
  lv_obj_set_height(label, lh_use);
  lv_obj_remove_flag(label, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_set_pos(label, lv_obj_get_x(label), base_y + (lh_big - lh_use) / 2);
  lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
  lv_label_set_text(label, txt);
}

/* ---------------------------------------------------------------------------
 * Weather icon (official: sun above, cloud below)
 * ------------------------------------------------------------------------- */

static void build_weather_icon(void)
{
  g_wx_icon = lv_obj_create(g_page);
  lv_obj_remove_style_all(g_wx_icon);
  lv_obj_set_size(g_wx_icon, kOffWxW, kOffWxH);
  lv_obj_set_pos(g_wx_icon, kOffWxX, kOffWxY);
  lv_obj_add_flag(g_wx_icon, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  /* [V1222] official: sun center abs(192,47) r15, 8 short rays */
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 9, 10, 30, kOffSun);
  g_icon_grp[g_icon_parts++] = G_SUN;
  {
    static const int srx1[8] = { 42, 37, 24, 11,  6, 11, 24, 37 };
    static const int sry1[8] = { 25, 12,  7, 12, 25, 38, 43, 38 };
    static const int srx2[8] = { 48, 42, 24,  6,  0,  6, 24, 42 };
    static const int sry2[8] = { 25,  7,  1,  7, 25, 43, 49, 43 };
    int r;
    for (r = 0; r < 8; r++)
      {
        g_icon_part[g_icon_parts] = make_seg(g_wx_icon, srx1[r], sry1[r],
                                             srx2[r], sry2[r], kOffSun, 3);
        g_icon_grp[g_icon_parts++] = G_SUN;
      }
  }
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 18, 20, 28, kOffCloud);
  g_icon_grp[g_icon_parts++] = G_CLOUD;
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 4, 28, 18, kOffCloud);
  g_icon_grp[g_icon_parts++] = G_CLOUD;
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 40, 27, 20, kOffCloud);
  g_icon_grp[g_icon_parts++] = G_CLOUD;
  g_icon_part[g_icon_parts] = make_box(g_wx_icon, 10, 40, 54, 14,
                                       kOffCloud, 7);
  g_icon_grp[g_icon_parts++] = G_CLOUD;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 22, 50, 19, 58, kRain, 2);
  g_icon_grp[g_icon_parts++] = G_RAIN;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 32, 50, 29, 58, kRain, 2);
  g_icon_grp[g_icon_parts++] = G_RAIN;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 42, 50, 39, 58, kRain, 2);
  g_icon_grp[g_icon_parts++] = G_RAIN;
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 17, 50, 5, 0xDCE9FF);
  g_icon_grp[g_icon_parts++] = G_SNOW;
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 29, 50, 5, 0xDCE9FF);
  g_icon_grp[g_icon_parts++] = G_SNOW;
  g_icon_part[g_icon_parts] = make_dot(g_wx_icon, 41, 50, 5, 0xDCE9FF);
  g_icon_grp[g_icon_parts++] = G_SNOW;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 14, 48, 56, 48,
                                       0x94A6BC, 2);
  g_icon_grp[g_icon_parts++] = G_FOG;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 8, 53, 52, 53,
                                       0x94A6BC, 2);
  g_icon_grp[g_icon_parts++] = G_FOG;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 16, 58, 58, 58,
                                       0x94A6BC, 2);
  g_icon_grp[g_icon_parts++] = G_FOG;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 4, 12, 30, 12,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 30, 12, 36, 8,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 4, 28, 38, 28,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 38, 28, 44, 24,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 4, 44, 26, 44,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  g_icon_part[g_icon_parts] = make_seg(g_wx_icon, 26, 44, 32, 40,
                                       kOffCloud, 2);
  g_icon_grp[g_icon_parts++] = G_WIND;
  syslog(LOG_ERR, "[V1226] icon parts=%d/%d\n", g_icon_parts, K_ICON_PARTS_MAX);
}

static void apply_weather_icon(const char *key)
{
  unsigned char mask = G_CLOUD | G_SUN;
  if (key == NULL)
    {
      key = "unknown";
    }
  if (strcmp(key, "qing") == 0)
    {
      mask = G_SUN;
    }
  else if (strcmp(key, "duoyun") == 0 || strcmp(key, "yun") == 0)
    {
      mask = G_SUN | G_CLOUD;
    }
  else if (strcmp(key, "yin") == 0)
    {
      mask = G_CLOUD;
    }
  else if (strcmp(key, "yu") == 0 || strcmp(key, "lei") == 0)
    {
      mask = G_CLOUD | G_RAIN;
    }
  else if (strcmp(key, "xue") == 0 || strcmp(key, "bingbao") == 0)
    {
      mask = G_CLOUD | G_SNOW;
    }
  else if (strcmp(key, "wu") == 0 || strcmp(key, "wumai") == 0 ||
           strcmp(key, "smog") == 0 || strcmp(key, "mai") == 0 ||
           strcmp(key, "shachen") == 0)
    {
      mask = G_CLOUD | G_FOG;
    }
  else if (strcmp(key, "feng") == 0)
    {
      mask = G_WIND;
    }
  int n = g_icon_parts;
  if (n < 0 || n > K_ICON_PARTS_MAX)
    {
      n = K_ICON_PARTS_MAX;   /* [V1226] never walk past the arrays */
    }
  for (int i = 0; i < n; i++)
    {
      if ((g_icon_grp[i] & mask) != 0)
        {
          LCLR_SHOW(g_icon_part[i]);
        }
      else
        {
          LCLR_HIDE(g_icon_part[i]);
        }
    }
}

/* map Chinese condition text to an icon key */
static const char *icon_key_from_text(const char *t)
{
  if (t == NULL || t[0] == '\0')
    {
      return NULL;
    }
  if (strstr(t, "雪") != NULL) return "xue";
  if (strstr(t, "雹") != NULL) return "bingbao";
  if (strstr(t, "雷") != NULL) return "lei";
  if (strstr(t, "雨") != NULL) return "yu";
  if (strstr(t, "雾") != NULL || strstr(t, "霾") != NULL ||
      strstr(t, "尘") != NULL) return "wu";
  if (strstr(t, "阴") != NULL) return "yin";
  if (strstr(t, "云") != NULL) return "yun";
  if (strstr(t, "晴") != NULL) return "qing";
  if (strstr(t, "风") != NULL) return "feng";
  return NULL;
}

/* ---------------------------------------------------------------------------
 * Bottom-right planet deco (line art, official position)
 * ------------------------------------------------------------------------- */

/* [V1227] photo-derived bottom-right artwork: moon + astronaut + stars in
 * one 83x99 A8 alpha map (carved from the official 5-in-1 reference photo),
 * rendered white, with a slow float animation so it is alive. */
static const lv_image_dsc_t ast_dsc =
{
  .header =
  {
    .magic  = LV_IMAGE_HEADER_MAGIC,
    .cf     = LV_COLOR_FORMAT_A8,
    .flags  = 0,
    .w      = AST_W,
    .h      = AST_H,
    .stride = AST_W,          /* A8: 1 byte per pixel */
  },
  .data_size = AST_W * AST_H,
  .data      = astronaut_bits,
};

static lv_obj_t *g_ast_img;   /* [V1227] animated astronaut */

/* 16-step bob cycle, ~4 s at 250 ms: +/-4 px vertical, +/-2 px sway */
static const int8_t kBobY[16] = {0, 2, 3, 4, 3, 2, 0, -2, -3, -4, -3, -2, 0, 1, 0, -1};
static const int8_t kBobX[16] = {0, -1, -2, -1, 0, 1, 2, 1, 0, -1, 0, 1, 2, 1, 0, -1};

static void ast_float_cb(lv_timer_t *t)
{
  static int ph = 0;
  if (g_ast_img == NULL) return;
  ph = (ph + 1) & 15;
  lv_obj_set_pos(g_ast_img, kBobX[ph], kBobY[ph]);   /* base pos is (0,0) */
}

static void build_deco(void)
{
  /* [V1227] the photo artwork (moon + astronaut + stars + speed lines) is
   * ONE A8 alpha map now.  The earlier "crash in lv_image_set_src" was the
   * g_icon_part[] overflow (fixed in v1226), not the bitmap. */
  lv_obj_t *d = lv_obj_create(g_page);
  lv_obj_remove_style_all(d);
  lv_obj_set_size(d, kOffDecoW, kOffDecoH);
  lv_obj_set_pos(d, kOffDecoX, kOffDecoY);
  lv_obj_add_flag(d, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

  g_ast_img = lv_image_create(d);
  lv_image_set_src(g_ast_img, &ast_dsc);
  lv_obj_set_pos(g_ast_img, 0, 0);
  /* A8 map is shape only -> recolor it to pure white like the rest of the art */
  lv_obj_set_style_image_recolor(g_ast_img, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_image_recolor_opa(g_ast_img, LV_OPA_COVER, 0);

  lv_timer_create(ast_float_cb, 250, NULL);   /* 16 steps x 250 ms = 4 s loop */

  syslog(LOG_ERR, "[V1227] deco: astr bitmap %dx%d A8 + float\n", AST_W, AST_H);
  g_deco_parts = 0;
}

/* ---------------------------------------------------------------------------
 * Build the whole page
 * ------------------------------------------------------------------------- */

static void build_page(void)
{
  g_page = lv_obj_create(lv_screen_active());
  lv_obj_set_size(g_page, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(g_page, 0, 0);
  lv_obj_add_flag(g_page, LV_OBJ_FLAG_FLOATING);
  lv_obj_set_style_radius(g_page, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_page, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_page, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_page, lv_color_hex(kCardTop), LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(g_page, lv_color_hex(kCardBot),
                                 LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(g_page, LV_GRAD_DIR_VER, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_page, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_remove_flag(g_page, LV_OBJ_FLAG_SCROLLABLE);

  /* row 1: city */
  /* [V1222] official boots with the literal placeholder 城市 */
  g_city = make_label(g_page, F_CJK, kOffTextHi, "城市");
  lv_obj_set_pos(g_city, kOffCityX, kOffCityY);
  lv_obj_set_width(g_city, kOffCityW);
  lv_obj_set_height(g_city, lv_font_get_line_height(F_CJK));
  lv_label_set_long_mode(g_city, LV_LABEL_LONG_DOT);

  /* row 1: air quality pill (hidden until data exists) */
  g_air_pill = lv_obj_create(g_page);
  lv_obj_remove_style_all(g_air_pill);
  lv_obj_set_size(g_air_pill, kOffPillW, kOffPillH);
  lv_obj_set_pos(g_air_pill, kOffPillX, kOffPillY);
  lv_obj_set_style_radius(g_air_pill, kOffPillH / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_air_pill, lv_color_hex(kOffPillBg),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_air_pill, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_flex_flow(g_air_pill, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(g_air_pill, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  g_air_label = make_label(g_air_pill, F_CJK, kOffPillFg, "优");
  /* [V1222] official boots with the pill visible */
  LCLR_SHOW(g_air_pill);

  /* row 2: wind (left) / condition (right) */
  g_wind = make_label(g_page, F_CJK, kOffText, "--");
  lv_obj_set_pos(g_wind, kOffWindX, kOffWindY);
  lv_obj_set_width(g_wind, kOffWindW);
  lv_obj_set_height(g_wind, lv_font_get_line_height(F_CJK));
  lv_label_set_long_mode(g_wind, LV_LABEL_LONG_DOT);
  /* [V1222] reference photo has no wind row */
  LCLR_HIDE(g_wind);

  g_cond = make_label(g_page, F_CJK, kOffText, "正在获取中...");
  lv_obj_set_pos(g_cond, kOffCondX, kOffCondY);
  lv_obj_set_width(g_cond, kOffCondW);
  lv_obj_set_height(g_cond, lv_font_get_line_height(F_CJK));
  lv_label_set_long_mode(g_cond, LV_LABEL_LONG_DOT);

  /* row 3: big clock - hour white, minute pale yellow, no colon */
  lv_obj_t *clock_row = lv_obj_create(g_page);
  lv_obj_remove_style_all(clock_row);
  lv_obj_set_size(clock_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_pos(clock_row, kOffClockX, kOffClockY);
  lv_obj_set_flex_flow(clock_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(clock_row, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(clock_row, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_column(clock_row, 0, LV_PART_MAIN);

  g_clock_h = make_label(clock_row, F_CLOCK, kOffClockH, "00");
  lv_obj_set_style_text_letter_space(g_clock_h, kOffClockLS, LV_PART_MAIN);
  g_clock_m = make_label(clock_row, F_CLOCK, kOffClockM, "00");
  lv_obj_set_style_text_letter_space(g_clock_m, kOffClockLS, LV_PART_MAIN);
  lv_obj_set_style_margin_left(g_clock_m, kOffClockGap, LV_PART_MAIN);

  /* seconds - absolute position */
  g_clock_s = make_label(g_page, F_SEC, kOffClockM, "00");
  lv_obj_set_pos(g_clock_s, kOffSecX, kOffSecY);

  /* row 4: date + weekday */
  g_date = make_label(g_page, F_CJK, kOffDate, "--月--日 --");
  lv_obj_set_pos(g_date, kOffDateX, kOffDateY);

  /* weather icon top-right */
  build_weather_icon();

  /* temp row: thermometer + 3-stage gradient bar + value */
  lv_obj_t *temp_row = lv_obj_create(g_page);
  lv_obj_remove_style_all(temp_row);
  lv_obj_set_size(temp_row, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(temp_row, 0, 0);
  lv_obj_add_flag(temp_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_remove_flag(temp_row, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(temp_row, LV_OBJ_FLAG_SCROLLABLE);

  /* [V1222] official thermometer: white outline capsule + red bulb */
  make_frame(temp_row, 26, 170, 12, 22, kOffTempIcon, 6, 2);
  make_dot(temp_row, 29, 184, 6, kOffMercury);

  g_temp_track = make_box(temp_row, kOffBarX, kOffTempBarY, kOffBarW,
                          kOffBarH, kOffHumTrack, kOffBarH / 2);
  g_temp_fill = lv_obj_create(g_temp_track);
  lv_obj_remove_style_all(g_temp_fill);
  lv_obj_set_size(g_temp_fill, kOffBarW, kOffBarH);
  lv_obj_set_pos(g_temp_fill, 0, 0);
  lv_obj_set_style_radius(g_temp_fill, kOffBarH / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_temp_fill, lv_color_hex(kOffBarTeal),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(g_temp_fill, lv_color_hex(kOffBarMid),
                                 LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(g_temp_fill, LV_GRAD_DIR_HOR, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_temp_fill, LV_OPA_COVER, LV_PART_MAIN);

  g_temp_fill_hi = lv_obj_create(g_temp_track);
  lv_obj_remove_style_all(g_temp_fill_hi);
  lv_obj_set_size(g_temp_fill_hi, kOffBarW / 2, kOffBarH);
  lv_obj_set_pos(g_temp_fill_hi, kOffBarW / 2, 0);
  lv_obj_set_style_radius(g_temp_fill_hi, kOffBarH / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_temp_fill_hi, lv_color_hex(kOffBarMid),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(g_temp_fill_hi, lv_color_hex(kOffBarSalmon),
                                 LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(g_temp_fill_hi, LV_GRAD_DIR_HOR,
                               LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_temp_fill_hi, LV_OPA_COVER, LV_PART_MAIN);

  g_temp_label = make_label(temp_row, F_NUM, kOffValue, "--°C");
  lv_obj_set_pos(g_temp_label, kOffTempValX, kOffTempValY);

  /* humidity row: water drop + bar + value */
  lv_obj_t *humid_row = lv_obj_create(g_page);
  lv_obj_remove_style_all(humid_row);
  lv_obj_set_size(humid_row, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(humid_row, 0, 0);
  lv_obj_add_flag(humid_row, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_remove_flag(humid_row, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(humid_row, LV_OBJ_FLAG_SCROLLABLE);

  /* [V1222] official droplet: white outline + tip + cyan fill */
  make_frame(humid_row, 29, 200, 14, 14, kOffHumIcon, 7, 2);
  make_seg(humid_row, 32, 200, 36, 193, kOffHumIcon, 2);
  make_seg(humid_row, 40, 200, 36, 193, kOffHumIcon, 2);
  make_dot(humid_row, 32, 203, 8, kOffDropFill);

  g_humid_track = make_box(humid_row, kOffBarX, kOffHumBarY, kOffBarW2,
                           kOffBarH, kOffHumTrack, kOffBarH / 2);
  g_humid_fill = lv_obj_create(g_humid_track);
  lv_obj_remove_style_all(g_humid_fill);
  lv_obj_set_size(g_humid_fill, kOffBarW2, kOffBarH);
  lv_obj_set_pos(g_humid_fill, 0, 0);
  lv_obj_set_style_radius(g_humid_fill, kOffBarH / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_humid_fill, lv_color_hex(kOffHumFill),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_grad_color(g_humid_fill,
                                 lv_color_hex(kOffBarSalmon),
                                 LV_PART_MAIN);
  lv_obj_set_style_bg_grad_dir(g_humid_fill, LV_GRAD_DIR_HOR,
                               LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_humid_fill, LV_OPA_COVER, LV_PART_MAIN);

  g_humid_label = make_label(humid_row, F_NUM, kOffValue, "--%");
  lv_obj_set_pos(g_humid_label, kOffHumValX, kOffHumValY);

  /* bottom-right planet deco */
  build_deco();

  apply_weather_icon("qing");   /* [V1222] boot shows sun only, like official */
}

/* ---------------------------------------------------------------------------
 * Weather file: /tmp/weather.txt  "<text> <temp>"
 * ------------------------------------------------------------------------- */

static void read_weather(void)
{
  const char *path = "/tmp/weather.txt";
  struct stat st;
  g_wvalid = 0;
  g_wtext[0] = '\0';
  g_wtemp = 0;
  g_whumid = -1;

  if (stat(path, &st) != 0)
    {
      return;
    }
  time_t now;
  time(&now);
  if (now - st.st_mtime > 1800)
    {
      return;   /* stale > 30 min */
    }

  FILE *f = fopen(path, "r");
  if (f == NULL)
    {
      return;
    }
  char buf[64];
  if (fgets(buf, sizeof(buf), f) != NULL)
    {
      char text[32] = "";
      int temp = 0;
      int hum = -1;
      /* [V1280] 第三字段是湿度；老格式（两字段）也能读，hum 保持 -1 */
      if (sscanf(buf, "%31s %d %d", text, &temp, &hum) >= 1 && text[0] != '\0')
        {
          snprintf(g_wtext, sizeof(g_wtext), "%s", text);
          g_wtemp = temp;
          g_whumid = hum;
          g_wvalid = 1;
        }
    }
  fclose(f);
}

/* ---------------------------------------------------------------------------
 * 1s refresh: clock / date / weather
 * ------------------------------------------------------------------------- */

/* [V945] LVGL 9.1 needs a tick source and this tree never installed one.
 *
 * apps/graphics/lvgl/lvgl/src/tick/lv_tick.c:
 *
 *     uint32_t lv_tick_get(void)
 *     {
 *       if (state_p->tick_get_cb)
 *         return state_p->tick_get_cb();
 *       ... return state_p->sys_time;      <- only lv_tick_inc() moves this
 *     }
 *
 * All 32 occurrences of lv_tick_inc / lv_tick_set_cb in the tree live under
 * apps/graphics/lvgl/lvgl/tests/, which is not part of the firmware.  So
 * sys_time stayed 0, no lv_timer_t ever became due, and lv_timer_handler()
 * ran nothing for the whole uptime.
 *
 * tick_cb() is created with a 1000 ms period and also invoked once directly
 * right after creation.  That one direct call is the only frame the clock
 * ever produced -- which is exactly what "the picture is frozen and the
 * seconds do not move" looks like from the outside.
 *
 * CONFIG_USEC_PER_TICK is 10000 in this build, i.e. one system tick is
 * 10 ms, so ticks * (USEC_PER_TICK / 1000) is milliseconds.
 */

static uint32_t v945_tick_ms(void)
{
  return (uint32_t)(clock_systime_ticks()
                    * (clock_t)(CONFIG_USEC_PER_TICK / 1000));
}

static void tick_cb(lv_timer_t *t)
{
  static int weather_div = 9;   /* first tick refreshes weather */
  char buf[64];

  /* ---- clock / date ---- */
  time_t now = time(NULL);
  struct tm tm_now;

  /* [V758] *** localtime_r() here was UTC -- the clock was 8 h slow. ***
   *
   * It was NOT a TZ problem, and setenv("TZ", ...) could never fix it.
   * CONFIG_LIBC_LOCALTIME is not set in this build, so
   * libs/libc/time/Make.defs compiles lib_gmtimer.c instead of
   * lib_localtime.c -- and lib_localtime.c is the only file in the
   * whole tree that reads TZ (lib_localtime.c:2792 getenv("TZ")).
   * The localtime_r() we actually link is lib_gmtimer.c:345:
   *
   *     FAR struct tm *localtime_r(const time_t *t, struct tm *r)
   *       { return gmtime_r(t, r); }      -- plain UTC, forever
   *
   * There is no CONFIG_LIBC_TZ_OFFSET fallback either.  Hence the house
   * convention, used verbatim by the five other time formatters in this
   * tree (tool_get_time.c:93, tool_get_time.c:173, memory_store.c:42,
   * agent_loop.c:612, context_builder.c:153): shift the epoch by +08:00
   * and then call gmtime_r().  This was the only localtime_r() left.
   *
   * `now` is deliberately left un-shifted: it also feeds time_ok. */

  time_t local_epoch = now + 8 * 3600;   /* UTC+8 = CST, no DST */
  gmtime_r(&local_epoch, &tm_now);

  bool time_ok = (now > 1600000000);

  /* [V920] clock observation: is time() still advancing, and does the UI
   * consider it valid?  One line per 10 s -- the clock on the panel is
   * reported stuck and nothing else in this file can distinguish "time() is
   * frozen" from "the label is updated but the panel is not repainted". */
  {
    static int v920_clk_div;
    if (++v920_clk_div >= 10) {
      v920_clk_div = 0;
      V1024S(LOG_INFO, "[V920]] clk now=%ld ok=%d hh=%02d mm=%02d ss=%02d\n",
             (long)now, (int)time_ok, tm_now.tm_hour, tm_now.tm_min,
             tm_now.tm_sec);
    }
  }

  if (time_ok)
    {
      snprintf(buf, sizeof(buf), "%02d", tm_now.tm_hour);
      lv_label_set_text(g_clock_h, buf);
      snprintf(buf, sizeof(buf), "%02d", tm_now.tm_min);
      lv_label_set_text(g_clock_m, buf);
      snprintf(buf, sizeof(buf), "%02d", tm_now.tm_sec);
      lv_label_set_text(g_clock_s, buf);

      int wd = tm_now.tm_wday;
      if (wd < 0 || wd > 6)
        {
          wd = 0;
        }
      snprintf(buf, sizeof(buf), "%02d月%02d日 %s", tm_now.tm_mon + 1,
               tm_now.tm_mday, kWeekday[wd]);
      lv_label_set_text(g_date, buf);
    }
  else
    {
      lv_label_set_text(g_clock_h, "--");
      lv_label_set_text(g_clock_m, "--");
      lv_label_set_text(g_clock_s, "--");
      lv_label_set_text(g_date, "正在同步时间");
    }

  /* ---- weather every 10s ---- */
  if (weather_div++ >= 10)
    {
      weather_div = 0;
      read_weather();
    }

  if (g_wvalid)
    {
      char line[48];
      snprintf(line, sizeof(line), "今日%s", g_wtext);
      set_one_line_fitted(g_cond, line, kOffCondW, F_DESK, F_SMALL,
                          kOffCondY);
      if (g_wind != NULL)
        {
          lv_label_set_text(g_wind, "");
        }

      snprintf(buf, sizeof(buf), "%d°C", g_wtemp);
      lv_label_set_text(g_temp_label, buf);

      /* temp bar: map -10..40 degC to 0..1 */
      float pos = ((float)g_wtemp + 10.0f) / 50.0f;
      if (pos < 0.0f) pos = 0.0f;
      if (pos > 1.0f) pos = 1.0f;
      int fillw = (int)(pos * (float)kOffBarW + 0.5f);
      if (fillw < 6) fillw = 6;
      if (fillw > kOffBarW) fillw = kOffBarW;
      int half = fillw / 2;
      lv_obj_set_pos(g_temp_fill, 0, 0);
      lv_obj_set_size(g_temp_fill, fillw, kOffBarH);
      lv_obj_set_pos(g_temp_fill_hi, half, 0);
      lv_obj_set_size(g_temp_fill_hi, fillw - half, kOffBarH);

      /* [V1280] 湿度有真值了：条和数字都跟着走 */
      if (g_whumid >= 0)
        {
          char hbuf[16];
          int hp = g_whumid > 100 ? 100 : g_whumid;
          snprintf(hbuf, sizeof(hbuf), "%d%%", hp);
          lv_label_set_text(g_humid_label, hbuf);
          int hw = kOffBarW2 * hp / 100;
          if (hw < 4) hw = 4;
          lv_obj_set_size(g_humid_fill, hw, kOffBarH);
        }
      else
        {
          lv_label_set_text(g_humid_label, "--%");
          lv_obj_set_size(g_humid_fill, kOffBarW2 * 55 / 100, kOffBarH);
        }

      apply_weather_icon(icon_key_from_text(g_wtext));
    }
  else
    {
      lv_label_set_text(g_cond, "正在获取中...");
      if (g_wind != NULL)
        {
          lv_label_set_text(g_wind, "");
        }
    }
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[])
{
  lv_nuttx_dsc_t info;
  lv_nuttx_result_t result;

  syslog(LOG_INFO,
         "[XZUI] xiaozhi_ui (official 5-in-1 port) starting [V1226 iconfix]\n");

  /* [V758] Version marker.  The on-screen clock now goes through
   * gmtime_r(epoch + 8*3600) rather than localtime_r(), which in this
   * build IS gmtime_r() (CONFIG_LIBC_LOCALTIME off) and therefore
   * rendered plain UTC -- a permanent 8-hour lag.  The substantive fix
   * is one line and the rest is comments, and comments do not survive
   * preprocessing, so this line is what lets us prove the running
   * image carries the fix (see ironclad rule 1). */
  V1024S(LOG_INFO, "[V758]] clock: gmtime_r(epoch+8h) (localtime_r() was UTC)\n");

  V1024S(LOG_ERR, "[V789]] main entered, before lv_init\n");
  lv_init();
  V1024S(LOG_ERR, "[V789]] lv_init returned\n");
  lv_init();

  /* [V945] Bind the tick BEFORE any timer is created, so the
   * first lv_timer_handler() already sees a live clock. */

  lv_tick_set_cb(v945_tick_ms);
  V1024S(LOG_ERR, "[V945]] lvgl tick bound: systicks * %d ms\n",
         (int)(CONFIG_USEC_PER_TICK / 1000));

  /* [V805] Wait for the framebuffer instead of giving up on the first miss.
   *
   * lv_nuttx_init() returned NULL here, and the old code went straight to
   * `return 1`, ending the UI thread -- which is why the panel stayed dark even
   * though nothing was wrong with the panel.  The timeline from the capture that
   * exposed it:
   *
   *   @ 4.9%  ui stack allocated
   *   @ 5.9%  xiaozhi_ui starting
   *   @ 9.1%  lv_nuttx_init failed: no display   <- gave up here
   *   @16.7%  panel settled after 5 wakes
   *
   * /dev/fb0 is registered late by design: its own comment says so ("framebuffer
   * deferred until the network is up"), and on a cold boot that is around 30
   * seconds.  The UI thread was asking at ~5 s and treating the answer as final.
   *
   * So keep asking.  The panel is already awake by then -- the waker owns it --
   * and lv_nuttx_init() is idempotent enough to be retried: nothing has been
   * allocated yet at this point in the function, so a failed attempt leaves
   * nothing to unwind.
   *
   * The bound is generous on purpose.  A miss here costs nothing but a sleep,
   * while giving up costs the whole interface; and this thread owns no resources
   * that another thread is waiting on, so waiting cannot deadlock anything. */
  lv_nuttx_dsc_init(&info);

  {
    int v805_waits = 0;
    int v805_ok    = 0;

    for (;;)
      {
        lv_nuttx_init(&info, &result);
        V1024S(LOG_ERR, "[V805]] lv_nuttx_init attempt %d -> disp=%p\n",
               v805_waits + 1, (void *)result.disp);

        if (result.disp != NULL)
          {
            v805_ok = 1;
            break;
          }

        if (++v805_waits >= 30)
          {
            syslog(LOG_ERR,
                   "[XZUI] lv_nuttx_init failed: no display after %d "
                   "attempts over %d s -- giving up\n", v805_waits,
                   v805_waits * 2);
            return 1;
          }

        sleep(2);
      }

    V1024S(LOG_ERR, "[V805]] display acquired after %d wait(s)\n",
           v805_waits);
  }
  syslog(LOG_INFO, "[XZUI] display ok (%dx%d)\n",
         lv_display_get_horizontal_resolution(result.disp),
         lv_display_get_vertical_resolution(result.disp));

  V1024S(LOG_ERR, "[V788]] enter weather\n");
  read_weather();
  V1024S(LOG_ERR, "[V788]] enter page\n");
  build_page();
  V1024S(LOG_ERR, "[V788]] enter face\n");
  build_face_page();
  V1024S(LOG_ERR, "[V788]] enter statusbar\n");
  build_status_bar();   /* [V672] last, so it starts on top of both pages */
  V1024S(LOG_ERR, "[V788]] enter timer_tick\n");
  lv_timer_create(tick_cb, 1000, NULL);
  V1024S(LOG_ERR, "[V788]] enter timer_mood\n");
  lv_timer_create(mood_cb, 200, NULL);
  V1024S(LOG_ERR, "[V788]] enter tick_run\n");
  tick_cb(NULL);
  V1024S(LOG_ERR, "[V788]] enter mood_run\n");
  mood_cb(NULL);

  V1024S(LOG_ERR, "[V788]] done -- startup window complete\n");
  /* [V824] Register the LVGL display driver.
   *
   * The whole UI was built and then never reached the panel.  Everything in this
   * file draws through lv_obj_create() into an object tree that lives in RAM, and
   * the run loop below is nothing but:
   *
   *     for (;;) {
   *       uint32_t idle = lv_timer_handler();
   *       usleep((idle ? idle : 1) * 1000);
   *     }
   *
   * Without a display driver registered, lv_timer_handler() has nothing to
   * refresh: no draw buffers, no flush callback, no ioctl.  Grepping this file
   * for lv_display_create / lv_disp_drv_register / flush_cb / draw_buf returns
   * nothing at all.  The symptom was completely misleading from the outside --
   * "display ok (240x240)", "page built", fb_err=0 -- and the frame counters
   * stopped at exactly two frames, which is what lcdinitialize() sends as its
   * power-on placeholder plus one update.  The UI never contributed a pixel.
   *
   * This is the piece that connects the object tree to /dev/fb0.  It follows the
   * LVGL 9.1 API this tree ships (lv_display_create / lv_display_set_draw_buffers
   * / lv_display_set_flush_cb), not the v7 lv_disp_drv_* spelling.
   *
   * /dev/fb0 is opened read-only above, which is enough: the pixels are already
   * in the mapped memory and the update is carried by the ioctl, not by write().
   */
  /* [V824d] Open its own handle rather than borrowing one.  The open() further
   * up lives in a different function, so its fd is not in scope here -- that is
   * what the compiler kept reporting.  Opening where it is used is the fix. */
  {
      /* [V888] Halved from SCREEN_W * 40 rows to SCREEN_W * 20 rows to free
       * 14400 bytes of internal DRAM for the es8311 worker stacks (see the
       * [V888] block in nuttx/drivers/audio/es8311.c).  This is LVGL's
       * partial-render line buffer, so fewer rows per buffer only means the
       * display is refreshed in more, smaller passes - no functional change.
       * The row count in the lv_draw_buf_init() call below MUST stay equal
       * to this one or LVGL will write past the end of the buffer.
       * Put 40 back here and there once internal DRAM is not tight any more.
       */

      static lv_color_t s_v824_buf[SCREEN_W * 20];
      static lv_draw_buf_t s_v824_draw;

    v824_fb_fd = open("/dev/fb0", O_RDWR);

    if (v824_fb_fd < 0)
      {
        V1024S(LOG_ERR, "[V824d]] open(/dev/fb0) for flush failed: %d\n",
               errno);
      }
    else
      {
        /* [V830] Map the plane so the flush callback has somewhere to put the
         * pixels.  Same three steps info_screen_main.c uses:
         *   FBIOGET_PLANEINFO -> mmap() -> write -> FBIO_UPDATE.
         */
        struct fb_planeinfo_s pinfo;

        memset(&pinfo, 0, sizeof(pinfo));

        if (ioctl(v824_fb_fd, FBIOGET_PLANEINFO,
                  (unsigned long)&pinfo) == 0 && pinfo.fblen > 0)
          {
            void *m = mmap(NULL, (size_t)pinfo.fblen,
                           PROT_READ | PROT_WRITE,
                           MAP_SHARED | MAP_FILE, v824_fb_fd, 0);

            if (m != NULL && m != MAP_FAILED)
              {
                g_v830_fbmem     = (uint16_t *)m;
                g_v830_stride_px = (uint32_t)pinfo.stride / 2u;  /* B -> px */
              }
          }

        V1024S(LOG_ERR, "[V830]] fb plane mapped: fbmem=%p fblen=%u stride=%u "
               "stride_px=%u (flush now COPIES pixels before FBIO_UPDATE)\\n",
               g_v830_fbmem,
               (unsigned)pinfo.fblen,
               (unsigned)pinfo.stride,
               (unsigned)g_v830_stride_px);
      }

    lv_display_t *disp = lv_display_create(SCREEN_W, SCREEN_H);

    if (disp == NULL)
      {
        V1024S(LOG_ERR, "[V824]] lv_display_create failed\n");
      }
    else if (v824_fb_fd < 0)
      {
        V1024S(LOG_ERR, "[V824c]] no fb0 handle; flush callback will drop "
               "every rectangle -- this is the original fault, so say so\n");
      }
    else
      {
        /* Partial mode: LVGL hands us the dirty rectangle, so only the changed
         * rows go out.  A full 240x240 push is 115200 bytes, which is more than
         * half the kernel heap at one shot. */
        /* [V824b] LVGL 9.1 signatures, read from lv_display.h rather than
         * assumed from the v7 docs: lv_display_set_draw_buffers() takes exactly
         * (disp, buf1, buf2), and the partial mode is selected by
         * lv_display_set_render_mode() with LV_DISPLAY_RENDER_MODE_PARTIAL. */
        lv_draw_buf_init(&s_v824_draw, SCREEN_W, 20,
                         LV_COLOR_FORMAT_RGB565, 0, s_v824_buf,
                         sizeof(s_v824_buf));

        lv_display_set_draw_buffers(disp, &s_v824_draw, NULL);
        lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_PARTIAL);
        lv_display_set_flush_cb(disp, v824_flush_cb);

        V1024S(LOG_ERR, "[V824]] LVGL display %dx%d registered, partial 40-line buffer, "
               "flush -> FBIO_UPDATE on fd=%d\n",
               SCREEN_W, SCREEN_H, v824_fb_fd);
      }
  }

  syslog(LOG_INFO, "[XZUI] page built, entering loop\n");

  for (;;)
    {
      /* [V789] first pass through the main loop. */
      V1024S(LOG_ERR, "[V789]] main loop entered\n");
      uint32_t idle = lv_timer_handler();

      /* [V973] Once a second: is the LVGL clock alive, is the label being
       * rewritten, and is LVGL pushing rectangles to us? */
      {
        static uint32_t s_v973_t0;
        static unsigned s_v973_n;
        uint32_t        v973_tk = lv_tick_get();

        if (s_v973_t0 == 0) {
          s_v973_t0 = v973_tk;
        } else if ((uint32_t)(v973_tk - s_v973_t0) >= 1000u) {
          s_v973_t0 = v973_tk;
          V1024P("[V973]] loop n=%u tick=%u flush=%u idle=%u lbl=%d\n",
                 ++s_v973_n, (unsigned)v973_tk, g_v973_flush, idle,
                 (int)(g_clock_s != NULL));

          /* [V975] Which display owns the widgets, is that the active screen,
           * and does the framebuffer memory actually change?
           *
           * flush frozen at 8 and 8 x 30 rows = one 240-row screen says LVGL
           * rendered exactly one frame and then stopped asking us to blit.
           * _lv_display_refr_timer() PAUSES its own timer and is only resumed
           * by lv_obj_invalidate() -> _lv_inv_area() -> LV_EVENT_REFR_REQUEST,
           * so a frozen flush means invalidation is being dropped.  There are
           * exactly three ways to drop it:
           *
           *   d     != the display we set the flush cb on  -> two displays
           *   act   != scr(lbl)                            -> orphan screen,
           *                                                   never the active one
           *   vis   == 0   or   inven == 0                 -> hidden / disabled
           *
           * px is the framebuffer pixel under the seconds label: if it keeps
           * changing while the panel does not, the pixels reach /dev/fb0 and
           * the fault is downstream of LVGL. */
          {
            lv_area_t     v975_a;
            lv_display_t *v975_d = lv_display_get_default();
            long          v975_px = -1;

            v975_a.x1 = 0;
            v975_a.y1 = 0;
            v975_a.x2 = 0;
            v975_a.y2 = 0;

            if (g_clock_s != NULL) {
              lv_obj_get_coords(g_clock_s, &v975_a);
            }

            if (g_v830_fbmem != NULL && g_v830_stride_px != 0) {
              v975_px = (long)g_v830_fbmem[
                  (uint32_t)v975_a.y1 * g_v830_stride_px
                  + (uint32_t)v975_a.x1];
            }

            V1024P("[V975]] d=%p act=%p scr=%p vis=%d inven=%d cxy=%d,%d px=%ld\n",
                   (void *)v975_d,
                   (void *)lv_screen_active(),
                   (void *)(g_clock_s != NULL ? lv_obj_get_screen(g_clock_s)
                                              : NULL),
                   (int)(g_clock_s != NULL ? lv_obj_is_visible(g_clock_s) : -1),
                   (int)(v975_d != NULL
                             ? lv_display_is_invalidation_enabled(v975_d)
                             : -1),
                   (int)v975_a.x1, (int)v975_a.y1, v975_px);
          }
        }
      }
      usleep((idle ? idle : 1) * 1000);
    }

  return 0;
}
