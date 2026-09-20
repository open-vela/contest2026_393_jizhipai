/****************************************************************************
 * device_status.c — 统一状态入口（无 LVGL 依赖版）
 * Contest 2026 Team 307
 ****************************************************************************/

#include "device_status.h"
#include "status_bar.h"
#include "motor.h"

static enum device_status g_current = STATUS_NORMAL;

void device_status_init(void *unused)
{
  (void)unused;
  motor_init();
  g_current = STATUS_NORMAL;
}

void device_status_set(enum device_status s)
{
  if (s == g_current) return;
  g_current = s;

  status_bar_set(s);

  switch (s)
    {
      case STATUS_NORMAL: motor_off();     break;
      case STATUS_NOTIFY: motor_notify();  break;
      case STATUS_ALERT:  motor_alert();   break;
      case STATUS_SLEEP:  motor_off();     break;
    }
}

enum device_status device_status_get(void)
{
  return g_current;
}
