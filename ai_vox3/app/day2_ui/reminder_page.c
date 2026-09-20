/****************************************************************************
 * reminder_page.c — 全屏课程提醒页面
 *
 * 大字显示课程名 + 地点 + 时间，带确认按钮
 *
 * Contest 2026 Team 307
 ****************************************************************************/

#include "ui_common.h"
#include "reminder_page.h"
#include "popup.h"

/****************************************************************************
 * 按钮回调：关闭提醒页
 ****************************************************************************/

static void btn_ok_cb(lv_event_t *e)
{
  lv_obj_t *page = (lv_obj_t *)lv_event_get_user_data(e);
  if (page)
    lv_obj_del(page);
}

/****************************************************************************
 * 公开接口
 ****************************************************************************/

lv_obj_t *reminder_page_create(lv_obj_t *parent, const char *course,
                                const char *location, const char *time_str)
{
  /* 全屏半透明背景 */

  lv_obj_t *page = lv_obj_create(parent);
  lv_obj_set_size(page, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_style_bg_color(page, lv_color_hex(0x0D1B2A), 0);
  lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(page, 0, 0);
  lv_obj_set_style_radius(page, 0, 0);
  lv_obj_remove_flag(page, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(page, LV_ALIGN_TOP_LEFT, 0, 0);

  /* Flex 布局：垂直居中 */

  lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(page, LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER,
                         LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(page, 16, 0);

  /* 顶部装饰条 */

  lv_obj_t *bar = lv_obj_create(page);
  lv_obj_set_size(bar, 60, 4);
  lv_obj_set_style_bg_color(bar, COLOR_TEXT_RED, 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(bar, 2, 0);
  lv_obj_set_style_border_width(bar, 0, 0);

  /* 提示文字 */

  ui_create_label(page, "📢 课程提醒", FONT_SMALL, COLOR_TEXT_RED);

  /* 课程名（大字） */

  lv_obj_t *course_label = ui_create_label(page, course,
                                           FONT_LARGE, COLOR_TEXT_WHITE);
  lv_obj_set_style_text_letter_space(course_label, 2, 0);

  /* 分隔线 */

  lv_obj_t *sep = lv_obj_create(page);
  lv_obj_set_size(sep, 200, 1);
  lv_obj_set_style_bg_color(sep, COLOR_TEXT_GRAY, 0);
  lv_obj_set_style_bg_opa(sep, LV_OPA_30, 0);
  lv_obj_set_style_border_width(sep, 0, 0);

  /* 地点 */

  char loc_buf[64];
  snprintf(loc_buf, sizeof(loc_buf), "📍 %s", location);
  ui_create_label(page, loc_buf, FONT_MEDIUM, COLOR_TEXT_GREEN);

  /* 时间 */

  char time_buf[64];
  snprintf(time_buf, sizeof(time_buf), "🕐 %s", time_str);
  ui_create_label(page, time_buf, FONT_MEDIUM, COLOR_TEXT_BLUE);

  /* 间距 */

  lv_obj_t *spacer = lv_obj_create(page);
  lv_obj_set_size(spacer, 1, 20);
  lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(spacer, 0, 0);

  /* 确认按钮 */

  lv_obj_t *btn = ui_create_button(page, 240, 50, "知道了", NULL);
  lv_obj_add_event_cb(btn, btn_ok_cb, LV_EVENT_CLICKED, page);

  return page;
}
