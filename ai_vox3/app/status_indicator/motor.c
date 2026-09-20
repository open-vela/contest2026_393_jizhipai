/****************************************************************************
 * motor.c — 马达震动模式封装（无 LVGL 依赖版）
 *
 * 驱动接口：/dev/pwm0 (GPTIM1_CH3)
 * 非阻塞：pthread 驱动震动序列
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>
#include <fixedmath.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "motor.h"

#define MOTOR_PWM_DEV   "/dev/pwm0"
#define MOTOR_PWM_FREQ  200
#define PCT_TO_UB16(p)  ((p) >= 100 ? ub16MAX : b16divi(uitoub16(p) - 1, 100))

static int       g_pwm_fd   = -1;
static pthread_t g_seq_thread;
static volatile int g_running = 0;

/* ── PWM 底层 ────────────────────────────────────────────────────── */

static void pwm_set(uint8_t pct)
{
  struct pwm_info_s info;
  if (g_pwm_fd < 0) return;
  memset(&info, 0, sizeof(info));
  info.frequency = MOTOR_PWM_FREQ;
  info.duty = PCT_TO_UB16(pct);
  ioctl(g_pwm_fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);
}

static void pwm_on(void)  { if (g_pwm_fd >= 0) ioctl(g_pwm_fd, PWMIOC_START, 0); }
static void pwm_off(void) { if (g_pwm_fd >= 0) ioctl(g_pwm_fd, PWMIOC_STOP, 0); }

static void motor_vibrate(int ms)
{
  pwm_set(100); pwm_on();
  usleep(ms * 1000);
  pwm_off(); pwm_set(0);
}

static void seq_stop(void)
{
  if (g_running)
    {
      g_running = 0;
      pthread_join(g_seq_thread, NULL);
    }
  pwm_off(); pwm_set(0);
}

/* ── 震动序列线程 ────────────────────────────────────────────────── */

static void *notify_thread(void *arg)
{
  (void)(arg);
  motor_vibrate(200);
  g_running = 0;
  return NULL;
}

static void *alert_thread(void *arg)
{
  (void)(arg);
  for (int i = 0; i < 3 && g_running; i++)
    {
      motor_vibrate(200);
      if (i < 2 && g_running) usleep(150000);
    }
  g_running = 0;
  return NULL;
}

static void *wakeup_thread(void *arg)
{
  (void)(arg);
  motor_vibrate(100);
  g_running = 0;
  return NULL;
}

static void seq_start(void *(*func)(void *))
{
  seq_stop();
  g_running = 1;
  pthread_create(&g_seq_thread, NULL, func, NULL);
}

/* ── 公开接口 ────────────────────────────────────────────────────── */

void motor_init(void)
{
  if (g_pwm_fd >= 0) return;
  g_pwm_fd = open(MOTOR_PWM_DEV, O_RDONLY);
  if (g_pwm_fd < 0)
    printf("[motor] open %s failed: %d\n", MOTOR_PWM_DEV, errno);
  else
    { pwm_set(0); printf("[motor] OK on %s\n", MOTOR_PWM_DEV); }
}

void motor_on(void)     { seq_stop(); pwm_set(100); pwm_on(); }
void motor_off(void)    { seq_stop(); }
void motor_notify(void) { seq_start(notify_thread); }
void motor_alert(void)  { seq_start(alert_thread); }
void motor_wakeup(void) { seq_start(wakeup_thread); }

void motor_deinit(void)
{
  seq_stop();
  if (g_pwm_fd >= 0) { close(g_pwm_fd); g_pwm_fd = -1; }
}
