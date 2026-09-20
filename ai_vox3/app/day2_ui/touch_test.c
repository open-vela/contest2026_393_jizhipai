/****************************************************************************
 * touch_test.c — 触摸屏最小测试
 *
 * 用法：nsh> touch_test
 *
 * 功能：
 *   - 打开 /dev/input0
 *   - 读取触摸事件
 *   - 打印坐标和状态
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <nuttx/input/touchscreen.h>

/****************************************************************************
 * 主函数
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  struct touch_sample_s sample;
  int count = 0;

  printf("=== Touch Screen Test ===\n");
  printf("Open /dev/input0...\n");

  fd = open("/dev/input0", O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: open /dev/input0 failed: %d (errno=%d)\n", fd, errno);
      return -1;
    }

  printf("Touch device opened successfully.\n");
  printf("Touch the screen to see coordinates (Ctrl+C to exit):\n\n");

  while (1)
    {
      ret = read(fd, &sample, sizeof(sample));
      if (ret < 0)
        {
          printf("ERROR: read failed: %d (errno=%d)\n", ret, errno);
          break;
        }

      if (ret == sizeof(sample))
        {
          count++;
          printf("[%d] Touch %s: x=%d, y=%d, pressure=%d\n",
                 count,
                 (sample.point[0].flags & TOUCH_DOWN) ? "DOWN" : "UP",
                 sample.point[0].x,
                 sample.point[0].y,
                 sample.point[0].pressure);
        }
    }

  close(fd);
  printf("Test ended. Total events: %d\n", count);

  return 0;
}
