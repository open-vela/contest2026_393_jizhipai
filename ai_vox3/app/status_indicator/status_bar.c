/****************************************************************************
 * status_bar.c — 顶部 4px 状态色带（framebuffer 直绘版）
 *
 * 实现：
 *   - 直接写 framebuffer 内存 + FBIO_UPDATE 刷新
 *   - 闪烁/呼吸动画用后台线程 + usleep 驱动
 *   - 状态切换时终止旧线程再启动新线程
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <nuttx/video/fb.h>

#include "status_bar.h"

/****************************************************************************
 * 颜色定义（RGB565）
 ****************************************************************************/

#define RGB565(r, g, b) \
  (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b) >> 3))

#define COLOR_GREEN   RGB565(0x00, 0xE6, 0x76)  /* #00E676 */
#define COLOR_BLUE    RGB565(0x29, 0x79, 0xFF)  /* #2979FF */
#define COLOR_RED     RGB565(0xFF, 0x17, 0x44)  /* #FF1744 */
#define COLOR_BLACK   0x0000

#define BAR_HEIGHT    4

/****************************************************************************
 * 私有数据
 ****************************************************************************/

static uint16_t        *g_fbmem  = NULL;
static uint32_t         g_xres   = 0;
static uint32_t         g_stride = 0;  /* 字节数 */
static int              g_fb_fd  = -1;

static enum device_status g_status = STATUS_NORMAL;
static volatile int     g_running = 0;   /* 动画线程运行标志 */
static pthread_t        g_anim_thread;
static int              g_visible = 1;

/****************************************************************************
 * 绘制色带区域
 ****************************************************************************/

static void draw_bar(uint16_t color)
{
  if (!g_fbmem || !g_visible) return;

  uint32_t stride_px = g_stride / 2;
  for (int y = 0; y < BAR_HEIGHT; y++)
    {
      uint16_t *row = g_fbmem + y * stride_px;
      for (uint32_t x = 0; x < g_xres; x++)
        {
          row[x] = color;
        }
    }

  /* 刷新色带区域 */

#ifdef CONFIG_FB_UPDATE
  if (g_fb_fd >= 0)
    {
      struct fb_area_s area = { 0, 0, g_xres, BAR_HEIGHT };
      ioctl(g_fb_fd, FBIO_UPDATE, (unsigned long)&area);
    }
#endif
}

/****************************************************************************
 * 设置透明度模拟（通过混合黑色实现）
 ****************************************************************************/

static uint16_t blend_color(uint16_t color, uint8_t opacity)
{
  if (opacity >= 255) return color;
  if (opacity == 0)   return COLOR_BLACK;

  /* 分离 RGB565 通道 */

  uint8_t r = ((color >> 11) & 0x1F) * 8;
  uint8_t g = ((color >> 5)  & 0x3F) * 4;
  uint8_t b = (color & 0x1F) * 8;

  /* 按 opacity 混合 */

  r = (uint8_t)((r * opacity) / 255);
  g = (uint8_t)((g * opacity) / 255);
  b = (uint8_t)((b * opacity) / 255);

  return RGB565(r, g, b);
}

/****************************************************************************
 * 动画线程函数
 ****************************************************************************/

static void *blink_thread(void *arg)
{
  (void)(arg);
  int count = 0;

  while (g_running && count < 6)  /* 3 次闪烁 = 6 次切换 */
    {
      if (count % 2 == 0)
        draw_bar(COLOR_BLACK);
      else
        draw_bar(COLOR_BLUE);

      count++;
      usleep(250000);  /* 250ms */
    }

  /* 闪烁结束，保持蓝色 */

  if (g_running)
    draw_bar(COLOR_BLUE);

  g_running = 0;
  return NULL;
}

static void *breath_thread(void *arg)
{
  (void)(arg);
  int opa = 30;
  int direction = 1;  /* 1=渐亮, -1=渐暗 */

  while (g_running)
    {
      uint16_t blended = blend_color(COLOR_RED, (uint8_t)opa);
      draw_bar(blended);

      opa += direction * 5;
      if (opa >= 255) { opa = 255; direction = -1; }
      if (opa <= 30)  { opa = 30;  direction = 1;  }

      usleep(20000);  /* 20ms 步进，100 步 = 2s 周期 */
    }

  g_running = 0;
  return NULL;
}

/****************************************************************************
 * 停止当前动画
 ****************************************************************************/

static void anim_stop(void)
{
  if (g_running)
    {
      g_running = 0;
      pthread_join(g_anim_thread, NULL);
    }
}

/****************************************************************************
 * 公开接口
 ****************************************************************************/

void status_bar_init_fb(uint16_t *fbmem, uint32_t xres, uint32_t yres,
                        uint32_t stride, int fb_fd)
{
  g_fbmem  = fbmem;
  g_xres   = xres;
  g_stride = stride;
  g_fb_fd  = fb_fd;
  g_status = STATUS_NORMAL;
  g_visible = 1;

  draw_bar(COLOR_GREEN);
}

void status_bar_set(enum device_status s)
{
  if (s == g_status) return;

  anim_stop();
  g_status = s;

  switch (s)
    {
      case STATUS_NORMAL:
        draw_bar(COLOR_GREEN);
        break;

      case STATUS_NOTIFY:
        draw_bar(COLOR_BLUE);
        g_running = 1;
        pthread_create(&g_anim_thread, NULL, blink_thread, NULL);
        break;

      case STATUS_ALERT:
        g_running = 1;
        pthread_create(&g_anim_thread, NULL, breath_thread, NULL);
        break;

      case STATUS_SLEEP:
        anim_stop();
        g_visible = 0;
        draw_bar(COLOR_BLACK);
        break;
    }
}

void status_bar_acknowledge(void)
{
  if (g_status == STATUS_ALERT)
    {
      anim_stop();
      g_status = STATUS_NORMAL;
      draw_bar(COLOR_GREEN);
    }
}

void status_bar_hide(void)
{
  anim_stop();
  g_visible = 0;
  draw_bar(COLOR_BLACK);
}

void status_bar_show(void)
{
  g_visible = 1;
  draw_bar(COLOR_GREEN);
  g_status = STATUS_NORMAL;
}

enum device_status status_bar_get(void)
{
  return g_status;
}
