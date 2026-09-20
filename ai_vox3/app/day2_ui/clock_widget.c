/****************************************************************************
 * clock_widget.c — 时钟组件（RTC 实时更新）
 *
 * 功能：
 *   - 读取 /dev/rtc0 获取当前时间
 *   - 大字显示时间 HH:MM:SS
 *   - 显示日期 YYYY-MM-DD
 *   - 显示星期
 *   - 每秒自动刷新
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/timers/rtc.h>

#include "ui_common.h"
#include "clock_widget.h"

/****************************************************************************
 * 私有数据
 ****************************************************************************/

static lv_obj_t *g_time_label  = NULL;
static lv_obj_t *g_date_label  = NULL;
static lv_obj_t *g_week_label  = NULL;
static lv_timer_t *g_timer     = NULL;

static const char *g_weekdays[] =
{
  "周日", "周一", "周二", "周三", "周四", "周五", "周六"
};

/****************************************************************************
 * 读取 RTC 时间
 ****************************************************************************/

static int rtc_read(struct rtc_time *rt)
{
  int fd = open("/dev/rtc0", O_RDONLY);
  if (fd < 0) return -1;

  int ret = ioctl(fd, RTC_RD_TIME, (unsigned long)rt);
  close(fd);
  return ret;
}

/****************************************************************************
 * 更新显示
 ****************************************************************************/

static void clock_update_cb(lv_timer_t *timer)
{
  struct rtc_time rt;
  char buf[32];

  if (rtc_read(&rt) < 0)
    return;

  /* 时间 HH:MM:SS */

  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           rt.tm_hour, rt.tm_min, rt.tm_sec);
  lv_label_set_text(g_time_label, buf);

  /* 日期 YYYY-MM-DD */

  snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
           rt.tm_year + 1900, rt.tm_mon + 1, rt.tm_mday);
  lv_label_set_text(g_date_label, buf);

  /* 星期 */

  if (rt.tm_wday >= 0 && rt.tm_wday <= 6)
    lv_label_set_text(g_week_label, g_weekdays[rt.tm_wday]);
}

/****************************************************************************
 * 公开接口
 ****************************************************************************/

lv_obj_t *clock_widget_create(lv_obj_t *parent)
{
  /* 容器 */

  lv_obj_t *cont = lv_obj_create(parent);
  lv_obj_set_size(cont, 350, 160);
  lv_obj_set_style_bg_color(cont, COLOR_BG_CARD, 0);
  lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(cont, 16, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_set_style_pad_all(cont, 20, 0);
  lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(cont, LV_ALIGN_TOP_MID, 0, 30);

  /* Flex 布局：垂直居中 */

  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(cont, 8, 0);

  /* 时间（大字） */

  g_time_label = ui_create_label(cont, "00:00:00",
                                 FONT_LARGE, COLOR_TEXT_WHITE);
  lv_obj_set_style_text_letter_space(g_time_label, 4, 0);

  /* 日期 */

  g_date_label = ui_create_label(cont, "2026-01-01",
                                 FONT_MEDIUM, COLOR_TEXT_GRAY);

  /* 星期 */

  g_week_label = ui_create_label(cont, "周一",
                                 FONT_SMALL, COLOR_TEXT_GREEN);

  /* 启动定时器：每秒刷新 */

  g_timer = lv_timer_create(clock_update_cb, 1000, NULL);
  lv_timer_ready(g_timer);  /* 立即触发首次更新 */

  return cont;
}
