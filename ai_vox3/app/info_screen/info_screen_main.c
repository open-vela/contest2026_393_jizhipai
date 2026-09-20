/****************************************************************************
 * info_screen_main.c - time + weather info page on /dev/fb0
 *
 * Framebuffer direct-draw (RGB565, 240x240 ST7789). Draws a big HH:MM clock
 * on the first line and "weekday weather temp" on the second line, reading
 * the weather from /tmp/weather.txt (written by the ai_agent weather tool).
 * No LVGL / FreeType - uses the embedded 16x16 bitmap font (font16.c).
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>

/* [V79] Declared here rather than included: the probe lives in the agent's
 * voice module and this app must not have to know about it. */

extern int diag_report(const char *msg);
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <nuttx/video/fb.h>

#include "font16.h"

#define WEATHER_PATH      "/tmp/weather.txt"
#define WEATHER_MAX_AGE   1800   /* seconds: file older than 30 min -> "--" */
#define WEATHER_REFRESH   600    /* seconds: re-read the file every 10 min */
#define NET_PATH          "/tmp/net.txt"
#define NET_MAX_AGE       30     /* seconds: state file older -> NO NET */
#define EMO_PATH          "/tmp/emotion.txt"   /* [V315] voice state badge */

#define COLOR_BG   0xFFE7       /* dark navy background (RGB565) */
#define COLOR_FG   0x0000       /* clock - white */
#define COLOR_SUB  0x3186       /* weather line - light blue-grey */
#define COLOR_LISTEN  0xF800    /* [V315] listening - red */
#define COLOR_THINK   0xFD20    /* [V315] thinking - orange */
#define COLOR_SPEAK   0x07E0    /* [V315] speaking - green */

static int          g_fb_fd = -1;
static uint16_t    *g_fbmem = NULL;
static uint32_t     g_stride = 0;  /* bytes */
static uint32_t     g_xres   = 0;
static uint32_t     g_yres   = 0;

/****************************************************************************
 * UTF-8 -> codepoint (handles 1/2/3 byte sequences)
 ****************************************************************************/

static uint16_t utf8_next(const char **pp)
{
  const unsigned char *s = (const unsigned char *)*pp;
  uint16_t c;

  if ((*s & 0x80) == 0)
    {
      c = s[0]; *pp += 1;
    }
  else if ((*s & 0xE0) == 0xC0)
    {
      c = (uint16_t)(((s[0] & 0x1F) << 6) | (s[1] & 0x3F)); *pp += 2;
    }
  else if ((*s & 0xF0) == 0xE0)
    {
      c = (uint16_t)(((s[0] & 0x0F) << 12) | ((s[1] & 0x3F) << 6) |
                     (s[2] & 0x3F)); *pp += 3;
    }
  else
    {
      c = (uint16_t)'?'; *pp += 1;
    }

  return c;
}

/****************************************************************************
 * Draw one glyph (scaled by 'scale') in fg colour. Background is left
 * untouched (the screen is cleared to COLOR_BG each refresh).
 ****************************************************************************/

static void draw_glyph(int x, int y, int scale, const uint8_t *data,
                       uint16_t fg)
{
  int stride_px = (int)(g_stride / 2);
  int ry, rx, sy, sx;

  for (ry = 0; ry < FONT16_H; ry++)
    {
      const uint8_t *row = &data[ry * FONT16_BPR];
      for (rx = 0; rx < FONT16_W; rx++)
        {
          int bit = (row[rx >> 3] >> (7 - (rx & 7))) & 1;
          if (!bit) continue;
          for (sy = 0; sy < scale; sy++)
            for (sx = 0; sx < scale; sx++)
              {
                int px = x + rx * scale + sx;
                int py = y + ry * scale + sy;
                if (px >= 0 && px < (int)g_xres &&
                    py >= 0 && py < (int)g_yres)
                  g_fbmem[py * stride_px + px] = fg;
              }
        }
    }
}

static void draw_text(int x, int y, int scale, const char *s, uint16_t fg)
{
  const char *p = s;
  int cx = x;
  while (*p)
    {
      uint16_t code = utf8_next(&p);
      const uint8_t *data = font16_lookup(code);
      if (data) draw_glyph(cx, y, scale, data, fg);
      cx += FONT16_W * scale;
    }
}

static int text_width(const char *s, int scale)
{
  int n = 0;
  const char *p = s;
  while (*p) { utf8_next(&p); n++; }
  return n * FONT16_W * scale;
}

static void draw_text_center(int y, int scale, const char *s, uint16_t fg)
{
  int w = text_width(s, scale);
  int x = ((int)g_xres - w) / 2;
  if (x < 0) x = 0;
  draw_text(x, y, scale, s, fg);
}

/****************************************************************************
 * [V315] Voice state badge: voice_channel writes one word to
 * /tmp/emotion.txt on every state change (listening/thinking/speaking/
 * happy/sad/idle/sleeping).  faced used to render it; faced is off in this
 * build, so the info screen owns the badge now.  Map the word to a short
 * Chinese line + colour; empty string means "show nothing".
 ****************************************************************************/

static void read_emotion(char *buf, size_t cap)
{
  int fd = open(EMO_PATH, O_RDONLY);
  ssize_t n = 0;
  if (fd >= 0)
    {
      n = read(fd, buf, cap - 1);
      close(fd);
    }
  if (n <= 0) n = 0;
  buf[n] = '\0';
}

static int emotion_badge(const char *emo, const char **text, uint16_t *color)
{
  if (!strcmp(emo, "listening")) { *text = "开始聆听..."; *color = COLOR_LISTEN; return 1; }
  if (!strcmp(emo, "thinking"))  { *text = "正在思考..."; *color = COLOR_THINK;  return 1; }
  if (!strcmp(emo, "speaking"))  { *text = "正在说话..."; *color = COLOR_SPEAK;  return 1; }
  if (!strcmp(emo, "sad"))       { *text = "出错了";      *color = COLOR_LISTEN; return 1; }
  return 0;   /* happy / idle / sleeping / unknown: show nothing */
}

/****************************************************************************
 * Framebuffer init
 ****************************************************************************/

static int fb_init(void)
{
  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;

  g_fb_fd = open("/dev/fb0", O_RDWR);
  if (g_fb_fd < 0)
    {
      printf("ERROR: open /dev/fb0 failed\n");
      return -1;
    }

  if (ioctl(g_fb_fd, FBIOGET_VIDEOINFO, (unsigned long)&vinfo) < 0 ||
      ioctl(g_fb_fd, FBIOGET_PLANEINFO, (unsigned long)&pinfo) < 0)
    {
      printf("ERROR: FBIOGET failed\n");
      close(g_fb_fd);
      g_fb_fd = -1;
      return -1;
    }

  g_xres   = vinfo.xres;
  g_yres   = vinfo.yres;
  g_stride = pinfo.stride;

  g_fbmem = (uint16_t *)mmap(NULL, pinfo.fblen,
                             PROT_READ | PROT_WRITE,
                             MAP_SHARED | MAP_FILE, g_fb_fd, 0);
  if (g_fbmem == MAP_FAILED)
    {
      printf("ERROR: mmap failed\n");
      close(g_fb_fd);
      g_fb_fd = -1;
      g_fbmem = NULL;
      return -1;
    }

  printf("info_screen: fb %lux%lu stride=%lu\n",
         (unsigned long)g_xres, (unsigned long)g_yres,
         (unsigned long)g_stride);
  return 0;
}

static void fb_update(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
#ifdef CONFIG_FB_UPDATE
  struct fb_area_s area = { x, y, w, h };
  if (g_fb_fd >= 0)
    ioctl(g_fb_fd, FBIO_UPDATE, (unsigned long)&area);
#endif
}

static void fb_clear(void)
{
  if (!g_fbmem) return;
  uint32_t total = (g_stride / 2) * g_yres;
  for (uint32_t i = 0; i < total; i++)
    g_fbmem[i] = COLOR_BG;
  fb_update(0, 0, (uint16_t)g_xres, (uint16_t)g_yres);
}

/****************************************************************************
 * Time (UTC+8 manually, avoids zoneinfo lookups on NuttX)
 ****************************************************************************/

static void get_now(struct tm *tm, long *t8)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  *t8 = ts.tv_sec + 8L * 3600L;
  gmtime_r(t8, tm);
}

static const char *weekday_cn(int wday)
{
  static const char *names[7] = { "日", "一", "二", "三",
                                  "四", "五", "六" };
  if (wday < 0 || wday > 6) return "?";
  return names[wday];
}

/****************************************************************************
 * Weather: read /tmp/weather.txt
 *   Format (single line):  <text> <temp> [HHMM]
 *   e.g.  "晴 25"  or  "无网 0"
 * Stale (>30 min) or missing -> not valid (UI shows "--").
 ****************************************************************************/

static void read_weather(char *text, size_t text_n, int *temp, int *valid)
{
  *valid = 0;
  *temp  = 0;
  text[0] = '\0';

  struct stat st;
  if (stat(WEATHER_PATH, &st) != 0)
    return;

  time_t now;
  time(&now);
  if (now - st.st_mtime > WEATHER_MAX_AGE)
    return;   /* stale -> treat as unavailable */

  FILE *f = fopen(WEATHER_PATH, "r");
  if (!f) return;

  char line[64];
  if (!fgets(line, sizeof(line), f))
    {
      fclose(f);
      return;
    }
  fclose(f);

  char *tok = strtok(line, " \t\r\n");
  if (!tok) return;
  strncpy(text, tok, text_n - 1);
  text[text_n - 1] = '\0';

  tok = strtok(NULL, " \t\r\n");
  if (tok) *temp = atoi(tok);

  *valid = 1;
}

/****************************************************************************
 * Network state: read /tmp/net.txt (written by ai_agent net_watch)
 *   Content: the IP string when connected, "NO NET" otherwise.
 * Stale (>30 s) or missing -> not valid (UI shows "NO NET").
 ****************************************************************************/

/* [V80] The address, straight from the stack.
 *
 * One ioctl, no filesystem, no writers to race with.  This is the fix for a
 * panel that painted "NO NET" for entire uptimes while the board was reaching
 * the bridge on every request. */

static int read_ifaddr(char *out, size_t n)
{
  struct ifreq ifr;
  struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
  int fd;

  out[0] = '\0';

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    {
      return 0;
    }

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ - 1);

  if (ioctl(fd, SIOCGIFADDR, (unsigned long)(uintptr_t)&ifr) < 0 ||
      sin->sin_addr.s_addr == 0)
    {
      close(fd);
      return 0;
    }

  close(fd);

  if (inet_ntop(AF_INET, &sin->sin_addr, out, n) == NULL)
    {
      out[0] = '\0';
      return 0;
    }

  return 1;
}

static void read_net(char *l1, size_t l1n, char *l2, size_t l2n, int *valid)
{
  *valid = 0;
  l1[0] = '\0';
  l2[0] = '\0';

  if (read_ifaddr(l1, l1n))
    {
      *valid = 1;
      return;
    }

  struct stat st;
  if (stat(NET_PATH, &st) != 0)
    return;

  /* [V63] The mtime gate is gone.
   *
   * It required the file to be rewritten every NET_MAX_AGE seconds, but on
   * this build the mtime of /tmp/net.txt never advances - the keepalive in
   * the log shipper rewrote the file 60 times in 20 s and stat() still
   * reported an old timestamp.  That made the panel treat a perfectly good
   * address as stale and paint "NO NET" forever.  The file's content is now
   * maintained continuously, so only the content matters. */

  (void)st;

  FILE *f = fopen(NET_PATH, "r");
  if (!f) return;

  if (!fgets(l1, l1n, f))
    {
      fclose(f);
      return;
    }

  if (!fgets(l2, l2n, f))
    {
      l2[0] = '\0';
    }
  fclose(f);

  char *nl = strchr(l1, '\n');
  if (nl) *nl = '\0';
  nl = strchr(l2, '\n');
  if (nl) *nl = '\0';
  *valid = (l1[0] != '\0');
}

/****************************************************************************
 * Main loop
 ****************************************************************************/


/* ══ [V398] 屏幕交互：页面状态机（按键 + 语音双输入）══════════════════
 *
 * 页面：0 = 主界面（时间 / 天气 / 网络 / 语音字幕）
 *       1 = 网络详情     2 = 系统状态
 *
 * 两个输入源都只调用 ui_apply()，UI 状态只有一份，不会互相打架：
 *   按键  VOL-(GPIO45) = 上一页    VOL+(GPIO46) = 下一页
 *   语音  ai_agent 的 ui_control 工具写 /tmp/ui_cmd，本任务消费后删除
 *
 * info_screen 是 apps 应用，不直接包含 arch 头文件，按 voice_channel.c 的
 * 用法 extern 声明这两个板级 GPIO 函数即可。
 */

#define UI_PAGE_MAX     3
#define UI_KEY_PREV     45    /* VOL- */
#define UI_KEY_NEXT     46    /* VOL+ */
#define UI_GPIO_IN_PU   0x09  /* INPUT | PULLUP */

static int  g_page = 0;

extern bool esp32s3_gpioread(int pin);
extern void esp32s3_configgpio(int pin, unsigned int cfg);

static void ui_goto(int p)
{
  if (p < 0) p = UI_PAGE_MAX - 1;
  if (p >= UI_PAGE_MAX) p = 0;
  if (p != g_page)
    {
      g_page = p;
      printf("[UI] page -> %d\n", g_page);
    }
}

static void ui_apply(const char *cmd)
{
  if (!cmd || cmd[0] == '\0') return;
  if (!strncmp(cmd, "next", 4))      ui_goto(g_page + 1);
  else if (!strncmp(cmd, "prev", 4)) ui_goto(g_page - 1);
  else if (!strncmp(cmd, "home", 4)) ui_goto(0);
  else if (!strncmp(cmd, "goto", 4))
    {
      int n = atoi(cmd + 4);
      if (n >= 0 && n < UI_PAGE_MAX) ui_goto(n);
    }
}

/* 语音侧 */
static void ui_poll_cmd(void)
{
  int fd = open("/tmp/ui_cmd", O_RDONLY);
  if (fd < 0) return;
  char b[32];
  ssize_t n = read(fd, b, sizeof(b) - 1);
  close(fd);
  unlink("/tmp/ui_cmd");
  if (n > 0)
    {
      b[n] = '\0';
      ui_apply(b);
    }
}

/* 按键侧：按下沿检测 + 周期性重配 pad 自愈（同 GPIO0 的做法） */
static void ui_poll_keys(void)
{
  static int prev_prev = 1;
  static int prev_next = 1;
  static int tick = 0;

  if ((tick % 5) == 0)
    {
      esp32s3_configgpio(UI_KEY_PREV, UI_GPIO_IN_PU);
      esp32s3_configgpio(UI_KEY_NEXT, UI_GPIO_IN_PU);
    }
  tick++;

  int kp = esp32s3_gpioread(UI_KEY_PREV) ? 1 : 0;   /* 低有效 */
  int kn = esp32s3_gpioread(UI_KEY_NEXT) ? 1 : 0;

  if (prev_prev == 1 && kp == 0) ui_goto(g_page - 1);
  if (prev_next == 1 && kn == 0) ui_goto(g_page + 1);

  prev_prev = kp;
  prev_next = kn;
}

int main(int argc, FAR char *argv[])
{
  /* Product build: discard stdout/stderr (USB CDC blocks when no host
   * reader is attached, which would freeze the app at its first print).
   * Diagnostics go to syslog (/dev/kmsg) instead. */
  freopen("/dev/null", "w", stdout);
  freopen("/dev/null", "w", stderr);

  (void)argc;
  (void)argv;

  printf("=== Info Screen (Team 307) ===\n");

  if (fb_init() < 0)
    {
      printf("ERROR: framebuffer init failed\n");
      return 1;
    }

  time_t  last_w  = 0;
  char    wtext[32];
  int     wtemp   = 0;
  int     wvalid  = 0;
  time_t  last_n  = 0;
  char    ntext[48];
  char    ntext2[48];
  int     nvalid  = 0;

  while (1)
    {
      time_t now;
      time(&now);
      if (now - last_w >= WEATHER_REFRESH || last_w == 0)
        {
          read_weather(wtext, sizeof(wtext), &wtemp, &wvalid);
          last_w = now;
        }

      if (now - last_n >= 2 || last_n == 0)
        {
          read_net(ntext, sizeof(ntext), ntext2, sizeof(ntext2), &nvalid);
          last_n = now;
        }

      fb_clear();

      /* [V398] 两个输入源：按键轮询 + 语音命令文件 */
      ui_poll_keys();
      ui_poll_cmd();

      struct tm tm;
      long t8;
      get_now(&tm, &t8);

      char tbuf[8];
      if (tm.tm_year + 1900 >= 2020)
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d", tm.tm_hour, tm.tm_min);
      else
        snprintf(tbuf, sizeof(tbuf), "--:--");

      if (g_page == 0) draw_text_center(56, 2, tbuf, COLOR_FG);

      char wd[16];
      snprintf(wd, sizeof(wd), "星期%s", weekday_cn(tm.tm_wday));

      char line2[64];
      if (wvalid)
        snprintf(line2, sizeof(line2), "%s %s %d°C", wd, wtext, wtemp);
      else
        snprintf(line2, sizeof(line2), "%s --", wd);

      if (g_page == 0) draw_text_center(150, 1, line2, COLOR_SUB);

      /* [UI-FIX-R35] The address stands on its own.
       *
       * "NET " plus a dotted quad is 18 glyphs and the 16 px font fits only
       * 15 across 240 px - the prefix is what pushed the tail of the address
       * off the screen.  A bare address is 14 glyphs, fits, and needs no
       * label.
       *
       * Line 1 of net.txt is netinit's contract: the address, or "NO NET".
       * It used to hold netinit's association counters, which is why this
       * line read "NET a0" / "L0". */

      char line3[64];
      if (nvalid)
        snprintf(line3, sizeof(line3), "%s", ntext);
      else
        snprintf(line3, sizeof(line3), "NO NET");
      if (g_page == 0) draw_text_center(190, 1, line3, COLOR_SUB);

      /* [V315] Voice state badge owns the bottom line whenever the voice
       * channel has a live state (BOOT PTT: 按下=开始聆听, 松开=正在思考,
       * 播报=正在说话); otherwise fall back to the association diagnostics
       * while there is no address. */

      char emo[24];
      const char *vtext = NULL;
      uint16_t vcolor = COLOR_SUB;
      read_emotion(emo, sizeof(emo));
      if (emotion_badge(emo, &vtext, &vcolor))
        {
          draw_text_center(214, 1, vtext, vcolor);
        }
      else if (!nvalid && ntext2[0] != '\0')
        {
          draw_text_center(214, 1, ntext2, COLOR_SUB);
        }

      /* [V398] 非主页面：纯文字，不画任何色块。
       * 底部语音字幕（vtext / ntext2）在所有页面都保留。 */

      if (g_page == 1)
        {
          draw_text_center(70, 1, "NETWORK", COLOR_FG);
          draw_text_center(120, 1, nvalid ? ntext : "NO NET", COLOR_SUB);
          draw_text_center(150, 1,
                           ntext2[0] != '\0' ? ntext2 : "-", COLOR_SUB);
        }
      else if (g_page == 2)
        {
          char sb[48];

          draw_text_center(70, 1, "SYSTEM", COLOR_FG);
          snprintf(sb, sizeof(sb), "UP %lds", (long)now);
          draw_text_center(120, 1, sb, COLOR_SUB);
          draw_text_center(150, 1, "AI-VOX3 / openvela", COLOR_SUB);
        }

      {
        char pg[8];

        snprintf(pg, sizeof(pg), "%d/%d", g_page + 1, UI_PAGE_MAX);
        draw_text(6, 6, 1, pg, COLOR_SUB);
      }

      fb_update(0, 0, (uint16_t)g_xres, (uint16_t)g_yres);

      /* [V79] Heartbeat carrying the exact string this panel just painted for
       * the address.  If the heartbeats stop, the panel task is wedged and the
       * screen is showing a stale frame; if they keep coming with the wrong
       * text, the writer is wrong.  Those two look identical from the sofa and
       * need opposite fixes. */

      {
        static int s_hb;

        if (++s_hb >= 20)
          {
            char hb[80];

            s_hb = 0;
            struct stat ns;

            snprintf(hb, sizeof(hb), "IS_HB_A%s_S%d_E%d",
                     nvalid ? ntext : "INVALID",
                     stat(NET_PATH, &ns), errno);
            diag_report(hb);
          }
      }

      sleep(1);
    }

  return 0;
}
