/*
 * servo.c — AI-VOX3 舵机控制模块 (v102)
 *
 * 依赖 NuttX PWM 接口 (include/nuttx/timers/pwm.h):
 *   PWMIOC_SETCHARACTERISTICS _PWMIOC(1)  -> 设置 frequency + duty(ub16)
 *   PWMIOC_START            _PWMIOC(3)    -> 启动输出
 *   PWMIOC_STOP             _PWMIOC(4)    -> 停止
 * struct pwm_info_s { uint32_t frequency; ub16_t duty; }  (单通道)
 *   duty 为 ub16 小数: 65535 ≈ 100% 占空比
 *
 * 编译: 在 packages/ai_agent/Make.defs 的 CSRCS 中加入 servo.c
 */
#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <syslog.h>

#include <fixedmath.h>           /* ub16_t (also pulled in by pwm.h) */
#include <nuttx/timers/pwm.h>

#include "servo.h"

/* ---- 内部状态 ---- */
struct servo_ch_s
{
  int         fd;
  const char *path;
  const char *name;
  int         invert;
  int         cur;               /* 当前角度 (缓存，用于动画) */
};
static struct servo_ch_s g_pan  = { -1, SERVO_PAN_DEV,  "PAN",  SERVO_PAN_INVERT, 90 };
static struct servo_ch_s g_tilt = { -1, SERVO_TILT_DEV, "TILT", SERVO_TILT_INVERT, 90 };

/* 角度 -> 脉宽(us): 0°=1000us, 90°=1500us, 180°=2000us */
static uint32_t angle_to_pulse_us(int angle)
{
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return (uint32_t)1000 + (uint32_t)angle * 1000u / 180u;
}

/* 脉宽 -> ub16 占空比 (period=20000us, ub16: 65536=100%) */
static ub16_t pulse_to_duty_ub16(uint32_t pulse_us)
{
  return (ub16_t)((pulse_us * 65536ull) / (uint64_t)SERVO_PERIOD_US);
}

/* 角度 -> 实际下发角度 (考虑 INVERT) */
static int apply_invert(struct servo_ch_s *ch, int angle)
{
  return ch->invert ? (180 - angle) : angle;
}

/* 设置单通道角度 */
static int servo_set_ch(struct servo_ch_s *ch, int angle)
{
  struct pwm_info_s info;
  int ret;
  int real = apply_invert(ch, angle);

  if (ch->fd < 0)
    {
      syslog(LOG_ERR, "servo: %s not opened, skip angle=%d\n", ch->name, angle);
      return -ENODEV;
    }

  info.frequency = SERVO_FREQ_HZ;
  info.duty = pulse_to_duty_ub16(angle_to_pulse_us(real));

  ret = ioctl(ch->fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      int e = errno;
      syslog(LOG_ERR, "servo: %s set angle=%d (real=%d) failed errno=%d\n",
             ch->name, angle, real, e);
      return -e;
    }

  ch->cur = angle;
  return OK;
}

int servo_init(void)
{
  int opened = 0;

  if (g_pan.fd < 0)
    {
      g_pan.fd = open(SERVO_PAN_DEV, O_WRONLY);
      if (g_pan.fd < 0)
        syslog(LOG_ERR, "servo: open %s failed errno=%d\n", SERVO_PAN_DEV, errno);
      else
        opened++;
    }
  if (g_tilt.fd < 0)
    {
      g_tilt.fd = open(SERVO_TILT_DEV, O_WRONLY);
      if (g_tilt.fd < 0)
        syslog(LOG_ERR, "servo: open %s failed errno=%d\n", SERVO_TILT_DEV, errno);
      else
        opened++;
    }

  if (opened == 0)
    {
      syslog(LOG_ERR, "servo: BOTH channels failed to open -> 舵机不可用(不影响语音)\n");
      return -ENODEV;
    }

  /* START 两个通道 */
  if (g_pan.fd >= 0)  ioctl(g_pan.fd, PWMIOC_START, 0);
  if (g_tilt.fd >= 0) ioctl(g_tilt.fd, PWMIOC_START, 0);

  /* 上电回中位，避免舵机猛甩 */
  servo_center();

  syslog(LOG_INFO, "servo: init ok pan_fd=%d tilt_fd=%d (PAN=GPIO3 TILT=GPIO4)\n",
         g_pan.fd, g_tilt.fd);
  return OK;
}

void servo_deinit(void)
{
  if (g_pan.fd >= 0)  { ioctl(g_pan.fd, PWMIOC_STOP, 0); close(g_pan.fd);  g_pan.fd = -1; }
  if (g_tilt.fd >= 0) { ioctl(g_tilt.fd, PWMIOC_STOP, 0); close(g_tilt.fd); g_tilt.fd = -1; }
}

int servo_pan(int angle)
{
  return servo_set_ch(&g_pan, angle);
}

int servo_tilt(int angle)
{
  return servo_set_ch(&g_tilt, angle);
}

int servo_center(void)
{
  int r1 = servo_set_ch(&g_pan, 90);
  int r2 = servo_set_ch(&g_tilt, 90);
  return (r1 < 0) ? r1 : r2;
}

/* 点头: TILT 90->115->65->90 (小幅度快速) */
int servo_nod(void)
{
  servo_set_ch(&g_tilt, 90);
  usleep(120000);
  servo_set_ch(&g_tilt, 115);
  usleep(180000);
  servo_set_ch(&g_tilt, 65);
  usleep(180000);
  servo_set_ch(&g_tilt, 90);
  return OK;
}

/* 摇头: PAN 90->120->60->90 (小幅度快速) */
int servo_shake(void)
{
  servo_set_ch(&g_pan, 90);
  usleep(120000);
  servo_set_ch(&g_pan, 120);
  usleep(180000);
  servo_set_ch(&g_pan, 60);
  usleep(180000);
  servo_set_ch(&g_pan, 90);
  return OK;
}

/* 上电自检: 两轴各扫 0->180->90，肉眼即可确认舵机在线 */
int servo_self_test(void)
{
  syslog(LOG_INFO, "servo: self-test start\n");
  servo_set_ch(&g_pan, 0);   usleep(400000);
  servo_set_ch(&g_pan, 180); usleep(400000);
  servo_set_ch(&g_pan, 90);  usleep(300000);
  servo_set_ch(&g_tilt, 0);   usleep(400000);
  servo_set_ch(&g_tilt, 180); usleep(400000);
  servo_set_ch(&g_tilt, 90);  usleep(300000);
  syslog(LOG_INFO, "servo: self-test done\n");
  return OK;
}
