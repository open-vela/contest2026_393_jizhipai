/****************************************************************************
 * status_indicator_main.c — 状态指示系统 NSH 入口
 *
 * 使用 framebuffer API 直接操作屏幕，不依赖 LVGL 初始化
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <nuttx/video/fb.h>

#include "status_bar.h"
#include "motor.h"
#include "device_status.h"

/****************************************************************************
 * 全局 framebuffer 状态
 ****************************************************************************/

static int          g_fb_fd   = -1;
static FAR uint16_t *g_fbmem  = NULL;
static uint32_t     g_stride  = 0;
static uint32_t     g_xres    = 0;
static uint32_t     g_yres    = 0;

/****************************************************************************
 * framebuffer 初始化
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

  g_fbmem = (FAR uint16_t *)mmap(NULL, pinfo.fblen,
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

  printf("FB: %lux%lu bpp=%u stride=%lu\n",
         (unsigned long)g_xres, (unsigned long)g_yres,
         pinfo.bpp, (unsigned long)g_stride);

  return 0;
}

/****************************************************************************
 * 刷新屏幕指定区域
 ****************************************************************************/

static void fb_update(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
#ifdef CONFIG_FB_UPDATE
  struct fb_area_s area = { x, y, w, h };
  if (g_fb_fd >= 0)
    ioctl(g_fb_fd, FBIO_UPDATE, (unsigned long)&area);
#endif
}

/****************************************************************************
 * 清屏为黑色
 ****************************************************************************/

static void fb_clear(void)
{
  if (!g_fbmem) return;
  uint32_t total = (g_stride / 2) * g_yres;
  for (uint32_t i = 0; i < total; i++)
    g_fbmem[i] = 0x0000;
  fb_update(0, 0, g_xres, g_yres);
}

/****************************************************************************
 * 主函数
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  printf("=== Status Indicator (Team 307) ===\n");
  printf("Frame buffer mode (no LVGL)\n\n");

  /* 初始化 framebuffer */

  if (fb_init() < 0)
    {
      printf("ERROR: framebuffer init failed\n");
      return -1;
    }

  /* 清屏 */

  fb_clear();

  /* 初始化状态系统（传入 fb 参数） */

  status_bar_init_fb(g_fbmem, g_xres, g_yres, g_stride, g_fb_fd);
  motor_init();
  device_status_set(STATUS_NORMAL);

  printf("\nStatus indicator started.\n");
  printf("Framebuffer: %lux%lu\n", (unsigned long)g_xres, (unsigned long)g_yres);
  printf("Use 'status_bar_set N/O/A/S' to test (N=Normal O=Notify A=Alert S=Sleep)\n");

  /* If stdin is not a TTY (auto-launched at boot without a console),
   * keep the status bar on screen instead of blocking on serial input. */
  if (!isatty(0))
    {
      printf("status_indicator: non-interactive, holding display\n");
      fflush(stdout);
      for (;;)
        {
          sleep(1);
        }
    }

  /* 简单交互循环 */

  char cmd[32];
  while (1)
    {
      printf("status> ");
      fflush(stdout);

      if (fgets(cmd, sizeof(cmd), stdin) == NULL)
        break;

      switch (cmd[0])
        {
          case 'n': case 'N':
            device_status_set(STATUS_NORMAL);
            printf("-> Normal\n");
            break;
          case 'o': case 'O':
            device_status_set(STATUS_NOTIFY);
            printf("-> Notify\n");
            break;
          case 'a': case 'A':
            device_status_set(STATUS_ALERT);
            printf("-> Alert\n");
            break;
          case 's': case 'S':
            status_bar_acknowledge();
            device_status_set(STATUS_SLEEP);
            printf("-> Sleep\n");
            break;
          case 'w': case 'W':
            status_bar_show();
            device_status_set(STATUS_NORMAL);
            motor_wakeup();
            printf("-> Wakeup\n");
            break;
          case 'q': case 'Q':
            goto out;
          default:
            printf("Commands: N=Normal O=Notify A=Alert S=Sleep W=Wakeup Q=Quit\n");
            break;
        }
    }

out:
  fb_clear();
  if (g_fbmem) munmap(g_fbmem, g_stride * g_yres);
  if (g_fb_fd >= 0) close(g_fb_fd);
  motor_deinit();
  printf("Bye.\n");
  return 0;
}
