/****************************************************************************
 * touch_diag.c — 触摸屏诊断（仅测试 /dev/input0）
 *
 * 用法：nsh> touch_diag
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <nuttx/input/touchscreen.h>

#define TIMEOUT_MS  10000

int main(int argc, char *argv[])
{
  struct touch_sample_s sample;
  struct pollfd pfd;
  int fd, ret;

  printf("=== Touch Diagnostic v3 ===\n\n");

  /* 打开触摸设备 */

  fd = open("/dev/input0", O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("[FAIL] open /dev/input0: %d\n", fd);
      return -1;
    }

  printf("[OK] /dev/input0 opened\n");

  /* 尝试非阻塞读取 */

  ret = read(fd, &sample, sizeof(sample));
  if (ret >= (int)sizeof(sample))
    {
      printf("[OK] Pending event: flags=0x%02X x=%ld y=%ld\n",
             sample.point[0].flags, (long)sample.point[0].x,
             (long)sample.point[0].y);
    }
  else
    {
      printf("[INFO] No pending event (errno=%d)\n", errno);
    }

  /* 等待触摸事件 */

  printf("[INFO] Please touch screen within %ds...\n", TIMEOUT_MS / 1000);

  pfd.fd     = fd;
  pfd.events = POLLIN;

  ret = poll(&pfd, 1, TIMEOUT_MS);
  if (ret <= 0)
    {
      printf("[FAIL] poll timeout (%dms) — no touch event\n", TIMEOUT_MS);
      printf("  → Touch controller may not be responding\n");
      printf("  → Check: power, reset, I2C connection\n");
      close(fd);
      return -1;
    }

  ret = read(fd, &sample, sizeof(sample));
  if (ret < (int)sizeof(sample))
    {
      printf("[FAIL] read after poll: %d\n", ret);
      close(fd);
      return -1;
    }

  printf("[PASS] Touch event!\n");
  printf("  flags: 0x%02X\n", sample.point[0].flags);
  printf("  x:     %ld\n", (long)sample.point[0].x);
  printf("  y:     %ld\n", (long)sample.point[0].y);
  printf("  points: %ld\n", (long)sample.npoints);

  close(fd);
  printf("\n=== Done ===\n");
  return 0;
}
